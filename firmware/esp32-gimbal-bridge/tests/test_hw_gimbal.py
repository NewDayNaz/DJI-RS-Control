"""Hardware-in-the-loop checks against a live ESP32 bridge.

Skipped unless GIMBAL_HW=1. Default URL is the LAN hostname used in this
project; override with GIMBAL_URL.

These tests move the focus / power-zoom motor at a modest vmax. They do not
send sleep, autotune, motor-calib, or gimbal body speed.
"""

from __future__ import annotations

import json
import os
import ssl
import time
import urllib.error
import urllib.request

import pytest

from tests.zoom_ramp_model import ZOOM_MAX, ZOOM_MIN, clamp_motor

pytestmark = pytest.mark.hardware

DEFAULT_URL = "https://djicontrol.lan.mgraham.me"
HW_VMAX = 500.0
HW_ACCEL = 1800.0
HW_VCUT = 80.0
HW_DECEL = 1800.0


def _require_hw() -> None:
    if os.environ.get("GIMBAL_HW") != "1":
        pytest.skip("set GIMBAL_HW=1 to talk to the live gimbal (moves the zoom motor)")


class GimbalClient:
    def __init__(self, base: str) -> None:
        self.base = base.rstrip("/")
        self._ssl = ssl.create_default_context()

    def _open(self, req: urllib.request.Request):
        ctx = self._ssl if self.base.startswith("https") else None
        try:
            return urllib.request.urlopen(req, timeout=10, context=ctx)
        except ssl.SSLError:
            self._ssl = ssl._create_unverified_context()
            return urllib.request.urlopen(req, timeout=10, context=self._ssl)

    def get(self, path: str) -> dict:
        req = urllib.request.Request(self.base + path, method="GET")
        with self._open(req) as resp:
            return json.loads(resp.read().decode())

    def post(self, path: str, body: dict | None = None) -> dict:
        data = None if body is None else json.dumps(body).encode()
        req = urllib.request.Request(self.base + path, data=data or b"", method="POST")
        if body is not None:
            req.add_header("Content-Type", "application/json")
        try:
            with self._open(req) as resp:
                return json.loads(resp.read().decode())
        except urllib.error.HTTPError as exc:
            raw = exc.read().decode()
            try:
                parsed = json.loads(raw)
            except json.JSONDecodeError:
                raise
            parsed.setdefault("http_status", exc.code)
            return parsed


@pytest.fixture(scope="module")
def client() -> GimbalClient:
    _require_hw()
    url = os.environ.get("GIMBAL_URL", DEFAULT_URL)
    c = GimbalClient(url)
    try:
        c.get("/api/state")
    except (urllib.error.URLError, TimeoutError, json.JSONDecodeError) as exc:
        pytest.skip(f"gimbal not reachable at {url}: {exc}")
    return c


@pytest.fixture(scope="module")
def restored(client: GimbalClient):
    start = client.get("/api/state")
    saved = {
        "vmax": start.get("zoom_vmax", HW_VMAX),
        "accel": start.get("zoom_accel", HW_ACCEL),
        "vcut": start.get("zoom_vcut", HW_VCUT),
        "decel": start.get("zoom_decel", HW_DECEL),
        "zoom": start.get("zoom"),
    }
    client.post(
        "/api/zoom/profile",
        {"vmax": HW_VMAX, "accel": HW_ACCEL, "vcut": HW_VCUT, "decel": HW_DECEL},
    )
    try:
        yield saved
    finally:
        try:
            client.post("/api/zoom/hold")
        except Exception:
            pass
        if saved["zoom"] is not None:
            try:
                client.post("/api/zoom", {"position": int(saved["zoom"])})
                _wait_idle(client, timeout=20.0)
            except Exception:
                pass
        try:
            client.post(
                "/api/zoom/profile",
                {
                    "vmax": saved["vmax"],
                    "accel": saved["accel"],
                    "vcut": saved["vcut"],
                    "decel": saved["decel"],
                },
            )
        except Exception:
            pass


def _wait_idle(client: GimbalClient, timeout: float = 18.0) -> dict:
    t0 = time.time()
    last = {}
    while time.time() - t0 < timeout:
        last = client.get("/api/state")
        if not last.get("zoom_moving"):
            return last
        time.sleep(0.12)
    raise AssertionError(f"zoom still moving after {timeout}s: {last}")


def _wait_zoom(client: GimbalClient, timeout: float = 4.0) -> int:
    t0 = time.time()
    while time.time() - t0 < timeout:
        st = client.get("/api/state")
        if st.get("zoom") is not None:
            return int(st["zoom"])
        time.sleep(0.15)
    raise AssertionError("zoom position never reported")


@pytest.fixture(scope="module")
def can_report_zoom(client: GimbalClient, restored):
    client.post("/api/command/zoom_get")
    try:
        return _wait_zoom(client)
    except AssertionError:
        return None


@pytest.fixture
def motor_pos(client: GimbalClient, can_report_zoom) -> int:
    if can_report_zoom is None:
        pytest.skip("focus motor position not reported — CAN RX looks silent; skip moves")
    client.post("/api/command/zoom_get")
    return _wait_zoom(client)


def test_state_and_can_are_healthy(client: GimbalClient):
    st = client.get("/api/state")
    assert st.get("connected") is True
    status = client.get("/api/status")
    can = status.get("can") or {}
    state = can.get("state") or st.get("can_state")
    assert state not in ("bus_off", "recovering")
    assert can.get("tx_ok", st.get("tx_ok", 0)) >= 0


def test_can_probe_is_acked(client: GimbalClient):
    out = client.post("/api/can/probe")
    assert out.get("ok") is True, f"CAN probe not ACKed: {out}"


def test_attitude_arrives_over_can(client: GimbalClient):
    st = client.get("/api/state")
    assert st.get("yaw") is not None, (
        f"no attitude on CAN (can_state={st.get('can_state')} rx_ok={st.get('rx_ok')})"
    )
    assert st.get("pitch") is not None


def test_zoom_get_reports_motor_counts(client: GimbalClient, restored, can_report_zoom):
    assert can_report_zoom is not None, "zoom_get produced no focus reply"
    assert ZOOM_MIN <= int(can_report_zoom) <= ZOOM_MAX


def test_motor_travels_mid_range(client: GimbalClient, restored, motor_pos):
    start = motor_pos
    delta = 450 if start < 2800 else -450
    target = clamp_motor(start + delta)
    if abs(target - start) < 200:
        target = clamp_motor(2000)
    client.post("/api/zoom", {"position": target})
    time.sleep(0.25)
    mid = client.get("/api/state")
    assert mid.get("zoom_moving") is True or abs(int(mid.get("zoom") or start) - start) > 30
    done = _wait_idle(client)
    landed = int(done["zoom"])
    assert abs(landed - start) > 120, f"motor barely moved {start} -> {landed} (target {target})"
    assert abs(landed - target) < 160


def test_interrupt_does_not_keep_charging_old_target(client: GimbalClient, restored, motor_pos):
    start = motor_pos
    far = clamp_motor(start + 900 if start < 2500 else start - 900)
    assert abs(far - start) > 400
    client.post("/api/zoom", {"position": far})
    time.sleep(0.35)
    mid = int(client.get("/api/state").get("zoom") or start)
    assert abs(mid - start) > 25, "long move never left the start before interrupt"
    client.post("/api/zoom", {"position": start})
    time.sleep(0.55)
    after = int(client.get("/api/state").get("zoom") or mid)
    # Must reverse (or at least stop advancing toward `far`), not keep cruise.
    if far > start:
        assert after < mid + 90, f"kept driving out: start={start} mid={mid} after={after} far={far}"
    else:
        assert after > mid - 90, f"kept driving in: start={start} mid={mid} after={after} far={far}"
    _wait_idle(client)


def test_jump_low_to_high_actually_travels(client: GimbalClient, restored, motor_pos):
    lo, hi = 900, 2800
    client.post("/api/zoom", {"position": lo})
    parked = _wait_idle(client, timeout=22.0)
    parked_pos = int(parked["zoom"])
    client.post("/api/zoom", {"position": hi})
    time.sleep(0.3)
    moving = client.get("/api/state")
    assert moving.get("zoom_moving") is True or abs(int(moving.get("zoom") or parked_pos) - parked_pos) > 40
    done = _wait_idle(client, timeout=22.0)
    landed = int(done["zoom"])
    assert landed - parked_pos > 800, f"jump {parked_pos} -> {hi} landed at {landed}"
    assert abs(landed - hi) < 200
