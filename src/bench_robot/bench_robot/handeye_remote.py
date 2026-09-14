#!/usr/bin/env python3
"""HTTP RGB transport for hand-eye capture; run this file directly on the Pi.

The Pi side subscribes to the camera driver already running there. No SDK,
second camera driver, shared DDS network, or archive capture is required.
"""

import argparse
import base64
import hmac
import json
import os
import threading
import time
import urllib.error
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np


MAX_RESPONSE_BYTES = 32 * 1024 * 1024


class FrameUnavailable(RuntimeError):
    """An actionable camera-input failure to return to the Jetson."""


def camera_status(message, info, now_ns):
    """Describe ROS inputs without converting or transferring an image."""
    result = {"pi_time_ns": now_ns, "image": None, "camera_info": None, "problems": []}
    if message is None:
        result["problems"].append("No color Image received")
    else:
        stamp = message.header.stamp
        timestamp = stamp.sec * 10**9 + stamp.nanosec
        age = (now_ns - timestamp) / 1e9
        result["image"] = {"frame_id": message.header.frame_id, "width": message.width,
                           "height": message.height, "stamp_ns": timestamp, "age_seconds": age}
        if timestamp == 0:
            result["problems"].append("Image has a zero timestamp")
        elif age < 0:
            result["problems"].append(f"Image timestamp is {-age:.3f} s ahead of Pi ROS clock")
        elif age > 1.:
            result["problems"].append(f"Image timestamp is {age:.3f} s behind Pi ROS clock "
                                      "(limit 1 s); check camera timestamps or stalled driver")
    if info is None:
        result["problems"].append("No color CameraInfo received")
    else:
        result["camera_info"] = {"frame_id": info.header.frame_id, "width": info.width,
                                 "height": info.height}
    if message is not None and info is not None:
        if message.header.frame_id != info.header.frame_id:
            result["problems"].append(f"Frame mismatch: Image={message.header.frame_id!r}, "
                                      f"CameraInfo={info.header.frame_id!r}")
        if message.width != info.width or message.height != info.height:
            result["problems"].append(f"Resolution mismatch: Image={message.width}x{message.height}, "
                                      f"CameraInfo={info.width}x{info.height}")
    return result


def decode_frame(payload):
    """Validate one atomic image/intrinsics/timestamp response."""
    if payload.get("version") != 1:
        raise ValueError("Unsupported hand-eye preview protocol")
    stamp = payload["stamp"]
    if (type(stamp["sec"]) is not int or type(stamp["nanosec"]) is not int
            or stamp["sec"] <= 0 or not 0 <= stamp["nanosec"] < 1_000_000_000):
        raise ValueError("Invalid camera timestamp")
    if not isinstance(payload["frame_id"], str) or not payload["frame_id"]:
        raise ValueError("Missing camera optical frame")
    info = payload["camera_info"]
    matrix = np.asarray(info["k"], dtype=float).reshape(3, 3)
    distortion = np.asarray(info["d"], dtype=float)
    if (not np.isfinite(matrix).all() or matrix[0, 0] <= 0 or matrix[1, 1] <= 0
            or not np.allclose(matrix[2], [0, 0, 1])
            or distortion.ndim != 1 or not np.isfinite(distortion).all()
            or info["distortion_model"] not in ("plumb_bob", "rational_polynomial")
            or len(distortion) not in (0, 4, 5, 8, 12, 14)):
        raise ValueError("Invalid/unsupported color camera intrinsics")
    if not (0 < info["width"] <= 8192 and 0 < info["height"] <= 8192):
        raise ValueError("Invalid image dimensions")
    encoded = base64.b64decode(payload["image_png"], validate=True)
    frame = cv2.imdecode(np.frombuffer(encoded, np.uint8), cv2.IMREAD_COLOR)
    if frame is None or frame.shape[:2] != (info["height"], info["width"]):
        raise ValueError("Preview image and intrinsics resolution mismatch")
    return frame, info, stamp, payload["frame_id"]


class RemoteCamera:
    """Poll outside the UI/ROS executor, retaining only the latest good frame."""

    def __init__(self, url, token="", rate=5., timeout=2., poll=True):
        self.url = url.rstrip("/") + "/frame"
        self.token, self.rate, self.timeout = token, rate, timeout
        self.latest = None
        self.error = "Waiting for Pi camera"
        self.stop = threading.Event()
        self.thread = None
        if poll:
            self.thread = threading.Thread(target=self._run, daemon=True)
            self.thread.start()

    def fetch(self, fresh=False):
        headers = {"Authorization": f"Bearer {self.token}"} if self.token else {}
        url = self.url.removesuffix("/frame") + "/capture" if fresh else self.url
        request = urllib.request.Request(url, headers=headers)
        try:
            with urllib.request.urlopen(request, timeout=self.timeout) as response:
                body = response.read(MAX_RESPONSE_BYTES + 1)
        except urllib.error.HTTPError as exc:
            if exc.code == 401:
                raise ValueError("Pi authentication failed; check CEABOT_CAPTURE_TOKEN") from exc
            if exc.code == 503:
                detail = "No fresh matching color Image/CameraInfo; update Pi bridge for detailed diagnostics"
                try:
                    error = json.loads(exc.read(8192)).get("error")
                    if isinstance(error, str):
                        detail = error[:2048]
                except (ValueError, AttributeError):
                    pass
                raise ValueError(f"Pi (503): {detail}") from exc
            if exc.code == 404 and fresh:
                raise ValueError("Pi bridge lacks /capture; copy the updated handeye_remote.py "
                                 "to the Pi and restart it") from exc
            raise
        if len(body) > MAX_RESPONSE_BYTES:
            raise ValueError("Pi preview response exceeds size limit")
        return decode_frame(json.loads(body))

    def _run(self):
        while not self.stop.is_set():
            started = time.monotonic()
            try:
                self.latest = self.fetch()
                self.error = None
            except Exception as exc:
                self.error = str(exc)
            self.stop.wait(max(0.01, 1. / self.rate - (time.monotonic() - started)))

    def close(self):
        self.stop.set()
        if self.thread is not None:
            self.thread.join(timeout=self.timeout + 1.)


def make_handler(snapshot, token, capture=None, status=None):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            expected = f"Bearer {token}"
            if token and not hmac.compare_digest(
                    self.headers.get("Authorization", "").encode(), expected.encode()):
                self.send_error(401, "Authentication required")
                return
            code = 200
            try:
                if self.path == "/capture" and capture is not None:
                    body = capture()
                elif self.path == "/frame":
                    body = snapshot()
                elif self.path == "/status" and status is not None:
                    body = json.dumps(status()).encode()
                else:
                    self.send_error(404)
                    return
                if body is None:
                    raise FrameUnavailable("No fresh matching image/CameraInfo available")
            except FrameUnavailable as exc:
                code, body = 503, json.dumps({"error": str(exc)}).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def log_message(self, *args):
            pass

    return Handler


def main():
    # ROS imports stay here so the HTTP client also works without server setup.
    import rclpy
    from cv_bridge import CvBridge
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8081)
    parser.add_argument("--image-topic", default="/gemini336/color/image_raw")
    parser.add_argument("--camera-info-topic", default="/gemini336/color/camera_info")
    parser.add_argument("--rate", type=float, default=5.,
                        help="Maximum preview encoding rate; encoding only occurs on HTTP requests")
    args, ros_args = parser.parse_known_args()
    if not np.isfinite(args.rate) or args.rate <= 0:
        parser.error("--rate must be finite and positive")
    rclpy.init(args=ros_args)

    class Preview(Node):
        def __init__(self):
            super().__init__("handeye_preview")
            self.bridge = CvBridge()
            self.info = self.image = self.record = None
            self.last_stamp = None
            self.last_encode_time = 0.
            self.encode_lock = threading.Lock()
            self.image_condition = threading.Condition()
            self.image_count = 0
            self.create_subscription(CameraInfo, args.camera_info_topic, self.on_info,
                                     qos_profile_sensor_data)
            self.create_subscription(Image, args.image_topic, self.on_image,
                                     qos_profile_sensor_data)

        def on_info(self, message):
            self.info = message

        def on_image(self, message):
            with self.image_condition:
                self.image = message
                self.image_count += 1
                self.image_condition.notify_all()

        def status(self):
            result = camera_status(self.image, self.info, self.get_clock().now().nanoseconds)
            result.update(image_topic=args.image_topic, camera_info_topic=args.camera_info_topic,
                          images_received=self.image_count, encoder_busy=self.encode_lock.locked())
            return result

        def encode(self):
            message, info = self.image, self.info
            problems = camera_status(message, info, self.get_clock().now().nanoseconds)["problems"]
            if problems:
                self.record = None
                raise FrameUnavailable("; ".join(problems))
            stamp = message.header.stamp
            key = (stamp.sec, stamp.nanosec)
            if key == self.last_stamp:
                return
            try:
                frame = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
                success, encoded = cv2.imencode(".png", frame, [cv2.IMWRITE_PNG_COMPRESSION, 1])
                if not success:
                    raise ValueError("PNG encoding failed")
                payload = {"version": 1, "frame_id": message.header.frame_id,
                           "stamp": {"sec": stamp.sec, "nanosec": stamp.nanosec},
                           "camera_info": {"width": info.width, "height": info.height,
                                           "k": list(info.k), "d": list(info.d),
                                           "distortion_model": info.distortion_model},
                           "image_png": base64.b64encode(encoded).decode("ascii")}
                # This immutable record is swapped atomically for HTTP threads.
                self.record = (json.dumps(payload).encode(), stamp.sec * 10**9 + stamp.nanosec)
                self.last_stamp = key
                self.last_encode_time = time.monotonic()
            except (ValueError, cv2.error) as exc:
                self.record = None
                raise FrameUnavailable(f"Image conversion/PNG encoding failed: {exc}") from exc

        def snapshot(self, fresh=False):
            # At most one request encodes at once. With no requests, no image
            # conversion, compression, or JSON serialization takes place.
            if not self.encode_lock.acquire(blocking=False):
                raise FrameUnavailable("Encoder is busy with another request; stop other preview clients")
            try:
                if fresh:
                    after = self.get_clock().now().nanoseconds
                    count_before = self.image_count
                    def received_new_image():
                        message = self.image
                        return (message is not None and message.header.stamp.sec * 10**9
                                + message.header.stamp.nanosec > after)
                    with self.image_condition:
                        if not self.image_condition.wait_for(received_new_image, timeout=1.):
                            status = self.status()
                            detail = "; ".join(status["problems"])
                            if status["image"] is not None:
                                detail += f"; latest image age={status['image']['age_seconds']:.3f} s"
                            raise FrameUnavailable(
                                "No image stamped after capture request within 1 s; "
                                f"{self.image_count - count_before} image callbacks during wait. " + detail)
                if fresh or time.monotonic() - self.last_encode_time >= 1. / args.rate:
                    self.encode()
                record = self.record
                if record is None:
                    raise FrameUnavailable("No encoded image available")
                age = (self.get_clock().now().nanoseconds - record[1]) / 1e9
                if not 0 <= age <= 1.:
                    raise FrameUnavailable(f"Image age after encoding={age:.3f} s (allowed 0 to 1 s)")
                if fresh and record[1] <= after:
                    raise FrameUnavailable("Encoded image predates capture request")
                return record[0]
            except FrameUnavailable as exc:
                self.get_logger().warning(str(exc))
                raise
            finally:
                self.encode_lock.release()

    node = Preview()
    server = ThreadingHTTPServer((args.host, args.port), make_handler(
        node.snapshot, os.environ.get("CEABOT_CAPTURE_TOKEN", ""),
        lambda: node.snapshot(fresh=True), node.status))
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    node.get_logger().info(f"Hand-eye HTTP camera on port {args.port}, input {args.image_topic}; "
                           "encoding on request only")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2.)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
