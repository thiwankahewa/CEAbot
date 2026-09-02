#!/usr/bin/env python3
"""Jetson-side capture acknowledgement and resumable Pi spool downloader."""

import hashlib
import json
import os
import shutil
import tarfile
import tempfile
import threading
import urllib.error
import urllib.parse
import urllib.request

import rclpy
import yaml
from arm_interfaces.srv import CaptureView
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


ARCHIVE_MEMBERS = { "color.png", "depth.npy", "meta.yaml"}


class RemoteOrbbecCapture(Node):
    def __init__(self):
        super().__init__("remote_orbbec_capture")
        self.declare_parameter("rpi_url", "http://10.20.0.200:8080")
        self.declare_parameter("request_timeout_sec", 120.0)
        self.declare_parameter("chunk_bytes", 1024 * 1024)
        self.declare_parameter("auth_token", "c9bd51af1d743131fe34b962b691400736498c99c767c7061087c948bee6dcac")
        self.rpi_url = str(self.get_parameter("rpi_url").value).rstrip("/")
        self.timeout = float(self.get_parameter("request_timeout_sec").value)
        self.chunk_bytes = int(self.get_parameter("chunk_bytes").value)
        self.auth_token = str(self.get_parameter("auth_token").value)

        self.transfer_enabled = threading.Event()  # Set only after row completion.
        self.transfer_lock = threading.Lock()
        self.transfer_thread = None
        self.run_dirs = {}
        self.last_status = "waiting for captures"

        self.create_service(CaptureView, "/orbbec_test_scan/capture_view", self.capture)
        self.create_service(Trigger, "/remote_orbbec_transfer/start", self.start_transfer)
        self.create_service(Trigger, "/remote_orbbec_transfer/pause", self.pause_transfer)
        self.create_service(Trigger, "/remote_orbbec_transfer/status", self.transfer_status)
        self.create_subscription(Bool, "/individual_scan_done", self.row_done, 10)
        self.get_logger().info(f"Pi capture spool: {self.rpi_url}")

    def _headers(self, content_type=None):
        headers = {}
        if content_type:
            headers["Content-Type"] = content_type
        if self.auth_token:
            headers["Authorization"] = f"Bearer {self.auth_token}"
        return headers

    def _post_json(self, endpoint, value):
        request = urllib.request.Request(
            f"{self.rpi_url}{endpoint}",
            data=json.dumps(value).encode("utf-8"),
            headers=self._headers("application/json"),
            method="POST",
        )
        with urllib.request.urlopen(request, timeout=self.timeout) as response:
            return json.load(response)

    def _get_json(self, endpoint, query):
        url = f"{self.rpi_url}{endpoint}?{urllib.parse.urlencode(query)}"
        request = urllib.request.Request(url, headers=self._headers())
        with urllib.request.urlopen(request, timeout=self.timeout) as response:
            return json.load(response)

    @staticmethod
    def _run_id(run_dir):
        name = os.path.basename(os.path.normpath(run_dir))
        safe = "".join(character if character.isalnum() or character in "_.-" else "_" for character in name)
        if not safe:
            raise ValueError("run_dir has no usable final directory name")
        suffix = hashlib.sha256(os.path.abspath(run_dir).encode("utf-8")).hexdigest()[:10]
        return f"{safe[:117]}-{suffix}"

    def capture(self, request, response):
        # A new capture row always pauses any old-row transfer. A download may
        # finish its current chunk (at most chunk_bytes) before observing this.
        self.transfer_enabled.clear()
        try:
            if os.path.basename(request.view_label) != request.view_label:
                raise ValueError("view_label must be a single directory name")
            if int(request.plant_id) < 0:
                raise ValueError("plant_id must be non-negative")
            run_id = self._run_id(request.run_dir)
            self.run_dirs[run_id] = os.path.abspath(request.run_dir)
            acknowledgement = self._post_json(
                "/capture",
                {
                    "run_id": run_id,
                    "plant_id": int(request.plant_id),
                    "view_label": str(request.view_label),
                },
            )
            destination = os.path.join(
                request.run_dir, f"plant_{int(request.plant_id):02d}", request.view_label
            )
            os.makedirs(destination, exist_ok=True)
            metadata = dict(acknowledgement["metadata"])
            metadata["remote_archive_pending"] = True
            self._write_yaml_atomic(os.path.join(destination, "meta.yaml"), metadata)
            self.last_status = f"spooled {run_id}/{acknowledgement['archive_id']}"
            response.success = True
            response.message = f"Capture compressed on Pi: {acknowledgement['archive_id']}"
            self._resume_transfer("capture complete")
        except urllib.error.HTTPError as exc:
            detail = exc.read(2048).decode("utf-8", errors="replace")
            response.success = False
            response.message = f"Pi capture HTTP {exc.code}: {detail}"
        except Exception as exc:
            response.success = False
            response.message = f"Remote Orbbec capture failed: {exc}"
        return response

    def row_done(self, message):
        if message.data:
            self._resume_transfer("row-complete topic")

    def start_transfer(self, request, response):
        self._resume_transfer("start service")
        response.success = True
        response.message = "Pi archive transfer started/resumed"
        return response

    def pause_transfer(self, request, response):
        self.transfer_enabled.clear()
        self.last_status = "transfer paused by command"
        response.success = True
        response.message = self.last_status
        return response

    def transfer_status(self, request, response):
        response.success = self.transfer_enabled.is_set()
        response.message = self.last_status
        return response

    def _resume_transfer(self, source):
        self.transfer_enabled.set()
        self.last_status = f"transfer enabled by {source}"
        with self.transfer_lock:
            if self.transfer_thread is None or not self.transfer_thread.is_alive():
                self.transfer_thread = threading.Thread(target=self._transfer_all, daemon=True)
                self.transfer_thread.start()

    def _transfer_all(self):
        try:
            for run_id, run_dir in list(self.run_dirs.items()):
                self._wait_until_transfer_enabled()
                manifest = self._get_json("/manifest", {"run_id": run_id})["archives"]
                for entry in manifest:
                    self._wait_until_transfer_enabled()
                    self._download_entry(run_dir, entry)
            self.last_status = "all available Pi archives transferred"
        except Exception as exc:
            self.transfer_enabled.clear()
            self.last_status = f"transfer paused after error: {exc}"
            self.get_logger().error(self.last_status)

    def _download_entry(self, run_dir, entry):
        run_id = entry["run_id"]
        archive_id = entry["archive_id"]
        destination = os.path.join(
            run_dir, f"plant_{int(entry['plant_id']):02d}", str(entry["view_label"])
        )
        os.makedirs(destination, exist_ok=True)
        part_path = os.path.join(destination, f".{archive_id}.tar.gz.part")
        offset = os.path.getsize(part_path) if os.path.exists(part_path) else 0
        expected_size = int(entry["bytes"])
        if offset > expected_size:
            os.replace(part_path, part_path + ".invalid")
            offset = 0

        while offset < expected_size:
            if not self.transfer_enabled.is_set():
                self.last_status = f"paused {archive_id} at {offset}/{expected_size} bytes"
                self._wait_until_transfer_enabled()
            query = urllib.parse.urlencode(
                {"run_id": run_id, "archive_id": archive_id, "offset": offset, "limit": self.chunk_bytes}
            )
            request = urllib.request.Request(
                f"{self.rpi_url}/archive/chunk?{query}", headers=self._headers()
            )
            with urllib.request.urlopen(request, timeout=self.timeout) as remote:
                chunk = remote.read()
                remote_offset = int(remote.headers["X-Chunk-Offset"])
                remote_size = int(remote.headers["X-Archive-Size"])
            if remote_offset != offset or remote_size != expected_size or not chunk:
                raise IOError("Pi returned an invalid archive chunk")
            with open(part_path, "ab") as output:
                output.write(chunk)
                output.flush()
                os.fsync(output.fileno())
            offset += len(chunk)
            self.last_status = f"downloading {archive_id}: {offset}/{expected_size} bytes"

        if self._sha256(part_path) != entry["sha256"]:
            raise IOError(f"Archive checksum mismatch: {archive_id}")
        self._install_archive(part_path, destination)
        self._post_json("/archive/complete", {"run_id": run_id, "archive_id": archive_id})
        os.unlink(part_path)
        self.last_status = f"installed {run_id}/{archive_id}"

    def _wait_until_transfer_enabled(self):
        # Keep the worker alive while paused so a resume command cannot race
        # with thread shutdown and strand a partially downloaded archive.
        self.transfer_enabled.wait()

    @staticmethod
    def _sha256(path):
        digest = hashlib.sha256()
        with open(path, "rb") as source:
            for chunk in iter(lambda: source.read(1024 * 1024), b""):
                digest.update(chunk)
        return digest.hexdigest()

    def _install_archive(self, archive_path, destination):
        temporary = tempfile.mkdtemp(prefix=".capture_extract_", dir=os.path.dirname(destination))
        try:
            with tarfile.open(archive_path, mode="r:gz") as tar:
                members = tar.getmembers()
                if {member.name for member in members} != ARCHIVE_MEMBERS:
                    raise ValueError("Unexpected view archive contents")
                for member in members:
                    if not member.isfile() or member.name != os.path.basename(member.name):
                        raise ValueError("Unsafe view archive")
                    source = tar.extractfile(member)
                    if source is None:
                        raise ValueError(f"Cannot read {member.name}")
                    with open(os.path.join(temporary, member.name), "wb") as output:
                        shutil.copyfileobj(source, output)

            remote_meta_path = os.path.join(temporary, "meta.yaml")
            with open(remote_meta_path, "r", encoding="utf-8") as source:
                merged_metadata = yaml.safe_load(source) or {}
            local_meta_path = os.path.join(destination, "meta.yaml")
            if os.path.exists(local_meta_path):
                with open(local_meta_path, "r", encoding="utf-8") as source:
                    merged_metadata.update(yaml.safe_load(source) or {})
            merged_metadata["remote_archive_pending"] = False
            os.makedirs(destination, exist_ok=True)
            for name in sorted(ARCHIVE_MEMBERS - {"meta.yaml"}):
                os.replace(os.path.join(temporary, name), os.path.join(destination, name))
            self._write_yaml_atomic(local_meta_path, merged_metadata)
        finally:
            shutil.rmtree(temporary, ignore_errors=True)

    @staticmethod
    def _write_yaml_atomic(path, value):
        temporary = path + ".tmp"
        with open(temporary, "w", encoding="utf-8") as output:
            yaml.safe_dump(value, output, sort_keys=False)
        os.replace(temporary, path)


def main(args=None):
    rclpy.init(args=args)
    node = RemoteOrbbecCapture()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
