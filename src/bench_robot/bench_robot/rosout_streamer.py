#!/usr/bin/env python3
import asyncio
import json
import threading
from typing import Set
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rcl_interfaces.msg import Log
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
import uvicorn

LEVEL_MAP = {
    10: "DEBUG",
    20: "INFO",
    30: "WARN",
    40: "ERROR",
    50: "FATAL",
}

def log_to_dict(msg: Log):
    ts_ns = int(msg.stamp.sec) * 1_000_000_000 + int(msg.stamp.nanosec)
    return {
        "ts_ns": ts_ns,
        "level": LEVEL_MAP.get(int(msg.level), str(int(msg.level))),
        "name": msg.name,                 # node name
        "msg": msg.msg,                   # message text
        "file": msg.file,
        "function": msg.function,
        "line": int(msg.line),
    }


# --- websocket hub ---
class Hub:
    def __init__(self):
        self.clients: Set[WebSocket] = set()
        self.loop: asyncio.AbstractEventLoop | None = None

    async def register(self, ws: WebSocket):
        await ws.accept()
        self.clients.add(ws)

    def unregister(self, ws: WebSocket):
        self.clients.discard(ws)

    async def broadcast(self, payload: dict):
        if not self.clients:
            return
        dead = []
        data = json.dumps(payload)
        for ws in list(self.clients):
            try:
                await ws.send_text(data)
            except Exception:
                dead.append(ws)
        for ws in dead:
            self.unregister(ws)

hub = Hub()

# --- ROS node ---
class RosoutListener(Node):
    def __init__(self):
        super().__init__("rosout_listener")

        qos = QoSProfile(
            depth=200,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.sub = self.create_subscription(Log, "/rosout", self.on_log, qos)

    def on_log(self, msg: Log):
        payload = {"type": "rosout", "data": log_to_dict(msg)}
        # schedule broadcast on the FastAPI event loop
        if hub.loop:
            asyncio.run_coroutine_threadsafe(hub.broadcast(payload), hub.loop)

# --- FastAPI ---
app = FastAPI()

@app.on_event("startup")
async def on_startup():
    hub.loop = asyncio.get_running_loop()

@app.websocket("/ws/logs")
async def ws_logs(ws: WebSocket):
    await hub.register(ws)
    try:
        while True:
            await asyncio.sleep(60)
    except WebSocketDisconnect:
        hub.unregister(ws)

def spin_ros():
    rclpy.init()
    node = RosoutListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

def main():
    ros_thread = threading.Thread(target=spin_ros, daemon=True)
    ros_thread.start()
    uvicorn.run(app, host="0.0.0.0", port=8080, log_level="info")

if __name__ == "__main__":
    main()
