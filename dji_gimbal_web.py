#!/usr/bin/env python3
"""Web control panel for a DJI RS gimbal on a CANable adapter.

  python dji_gimbal_web.py
  python dji_gimbal_web.py --channel COM6
  python dji_gimbal_web.py --host 0.0.0.0 --port 8080

Then open http://127.0.0.1:8080
"""

from __future__ import annotations

import argparse
import asyncio
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Any

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field

from dji_can_session import GimbalCanSession, GimbalSnapshot, list_adapters

STATIC_DIR = Path(__file__).resolve().parent / "web" / "static"
session = GimbalCanSession()
_startup_channel: str | None = None
_startup_interface: str | None = None


def _snap_dict(snap: GimbalSnapshot) -> dict[str, Any]:
    return {
        "connected": snap.connected,
        "adapter": snap.adapter,
        "interface": snap.interface,
        "yaw": snap.yaw,
        "roll": snap.roll,
        "pitch": snap.pitch,
        "zoom": snap.zoom,
        "zoom_target": snap.zoom_target,
        "zoom_vmax": snap.zoom_vmax,
        "zoom_accel": snap.zoom_accel,
        "last_rx_age_s": snap.last_rx_age_s,
        "last_error": snap.last_error,
        "tx_ok": snap.tx_ok,
        "rx_ok": snap.rx_ok,
    }


@asynccontextmanager
async def lifespan(_app: FastAPI):
    if _startup_channel is not None or _startup_interface is not None:
        await asyncio.to_thread(session.connect, _startup_channel, _startup_interface)
    try:
        yield
    finally:
        await asyncio.to_thread(session.disconnect)


app = FastAPI(title="DJI gimbal control", lifespan=lifespan)
app.mount("/static", StaticFiles(directory=STATIC_DIR), name="static")


class ConnectBody(BaseModel):
    channel: str | None = None
    interface: str | None = None


class SpeedBody(BaseModel):
    yaw: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    hold: bool = True


class PositionBody(BaseModel):
    yaw: float
    roll: float = 0.0
    pitch: float
    time_s: float = Field(default=0.4, ge=0.0, le=25.5)


class ZoomBody(BaseModel):
    position: int = Field(ge=0, le=4096)


class ZoomProfileBody(BaseModel):
    vmax: float = Field(default=900.0, ge=50.0, le=8000.0)
    accel: float = Field(default=1800.0, ge=50.0, le=40000.0)


@app.get("/")
async def index() -> FileResponse:
    return FileResponse(STATIC_DIR / "index.html")


@app.get("/api/adapters")
async def adapters() -> dict[str, Any]:
    items = list_adapters()
    return {
        "adapters": [
            {
                "device": a.device,
                "description": a.description,
                "interface": a.interface,
            }
            for a in items
        ]
    }


@app.get("/api/state")
async def state() -> dict[str, Any]:
    return _snap_dict(session.snapshot())


@app.post("/api/connect")
async def connect(body: ConnectBody) -> dict[str, Any]:
    snap = await asyncio.to_thread(session.connect, body.channel, body.interface)
    return _snap_dict(snap)


@app.post("/api/disconnect")
async def disconnect() -> dict[str, Any]:
    await asyncio.to_thread(session.disconnect)
    return _snap_dict(session.snapshot())


@app.post("/api/speed")
async def speed(body: SpeedBody) -> dict[str, str]:
    if body.hold:
        session.hold_speed(body.yaw, body.roll, body.pitch)
    else:
        session.release_speed()
        session.enqueue({"op": "speed", "yaw": body.yaw, "roll": body.roll, "pitch": body.pitch})
    return {"ok": "speed"}


@app.post("/api/position")
async def position(body: PositionBody) -> dict[str, str]:
    session.enqueue(
        {
            "op": "position",
            "yaw": body.yaw,
            "roll": body.roll,
            "pitch": body.pitch,
            "time_s": body.time_s,
            "absolute": True,
        }
    )
    return {"ok": "position"}


@app.post("/api/zoom")
async def zoom(body: ZoomBody) -> dict[str, str]:
    session.set_zoom_target(body.position)
    return {"ok": "zoom"}


@app.post("/api/zoom/profile")
async def zoom_profile(body: ZoomProfileBody) -> dict[str, str]:
    session.set_zoom_profile(body.vmax, body.accel)
    return {"ok": "zoom-profile"}


@app.post("/api/command/{name}")
async def command(name: str) -> dict[str, str]:
    allowed = {
        "sleep",
        "wake",
        "recenter",
        "selfie",
        "calibrate",
        "motor-calib",
        "activetrack",
        "rec-start",
        "rec-stop",
        "focus-center-start",
        "focus-center-stop",
        "push-on",
        "push-off",
        "cam-cmd",
        "limit",
        "zoom_get",
        "stop",
    }
    if name not in allowed:
        raise HTTPException(status_code=400, detail=f"unknown command {name}")
    if name == "stop":
        session.release_speed()
        return {"ok": "stop"}
    session.enqueue({"op": name})
    return {"ok": name}


@app.websocket("/ws")
async def telemetry(ws: WebSocket) -> None:
    await ws.accept()
    try:
        while True:
            await ws.send_json(_snap_dict(session.snapshot()))
            await asyncio.sleep(0.05)
    except WebSocketDisconnect:
        return


def main() -> None:
    global _startup_channel, _startup_interface
    parser = argparse.ArgumentParser(description="DJI gimbal web control over CANable")
    parser.add_argument("--channel", help="slcan COM port, gs_usb:0, or auto")
    parser.add_argument("--interface", choices=["slcan", "gs_usb", "auto"], default=None)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8080)
    args = parser.parse_args()
    _startup_channel = args.channel
    _startup_interface = args.interface
    import uvicorn

    uvicorn.run(app, host=args.host, port=args.port, log_level="info")


if __name__ == "__main__":
    main()
