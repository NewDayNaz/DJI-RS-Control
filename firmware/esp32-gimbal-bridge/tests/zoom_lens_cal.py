"""Interactive zoom calibration: motor moves, you type the LCD.

After each settle, enter millimetres and optional digital × (default 1).

Usage:
  python -m tests.zoom_lens_cal --restart
  python -m tests.zoom_lens_cal 18
  python -m tests.zoom_lens_cal 105 1.2
  python -m tests.zoom_lens_cal --abort
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

from tests.test_hw_gimbal import DEFAULT_URL, GimbalClient, clamp_motor

HERE = Path(__file__).resolve().parent
STATE_PATH = HERE / "_captures" / "zoom_lens_cal" / "state.json"

WIDE_POS = 4095
TELE_POS = 1
PARK = 2048
TEST = 1024
JOG = 250
WIDE_MM = 18.0
TELE_MM = 105.0
DIGI = 2.0
TOL = 0.8
NAME = "18-105"
SETTLE_S = 1.5

MAP_IN = [3600, 3200, 2800, 2400, 2000, 1600, 1100, 700, 350, 1]
MAP_OUT = [350, 700, 1100, 1600, 2000, 2400, 2800, 3200, 3600, 4095]


def equiv(mm: float, dx: float) -> float:
    if dx > 1.05:
        return float(mm) * float(dx)
    return float(mm)


def band_of(mm: float, dx: float) -> int:
    if dx > 1.05:
        return 1
    if mm > TELE_MM + 0.35:
        return 1
    return 0


def store_mm(mm: float, dx: float) -> float:
    return equiv(mm, dx) if band_of(mm, dx) == 1 else float(mm)


class Cal:
    def __init__(self) -> None:
        self.client = GimbalClient(DEFAULT_URL)
        self.st: dict = {}

    def say(self, msg: str) -> None:
        print(msg, flush=True)

    def save(self) -> None:
        STATE_PATH.parent.mkdir(parents=True, exist_ok=True)
        STATE_PATH.write_text(json.dumps(self.st, indent=2) + "\n", encoding="utf-8")

    def load(self) -> bool:
        if not STATE_PATH.exists():
            return False
        self.st = json.loads(STATE_PATH.read_text(encoding="utf-8"))
        return True

    def profile(self, vmax: float, accel: float, vcut: float, decel: float | None = None) -> None:
        d = accel if decel is None else decel
        self.client.post(
            "/api/zoom/profile",
            {"vmax": vmax, "accel": accel, "vcut": vcut, "decel": d},
        )

    def wait_idle(self, timeout: float = 22.0) -> dict:
        t0 = time.time()
        last: dict = {}
        saw_move = False
        while time.time() - t0 < timeout:
            last = self.client.get("/api/state")
            if last.get("zoom_moving"):
                saw_move = True
            elif saw_move or time.time() - t0 > 0.6:
                if not last.get("zoom_moving"):
                    return last
            time.sleep(0.12)
        return last

    def wait_arrived(self, target: int, timeout: float = 24.0) -> int:
        t0 = time.time()
        last_pos = None
        while time.time() - t0 < timeout:
            st = self.client.get("/api/state")
            pos = st.get("zoom")
            if pos is not None:
                last_pos = int(pos)
                if abs(last_pos - target) <= 24 and not st.get("zoom_moving"):
                    return last_pos
            time.sleep(0.12)
        if last_pos is not None:
            return last_pos
        return self.hold()

    def wait_zoom(self, timeout: float = 5.0) -> int:
        try:
            self.client.post("/api/command/zoom_get", {})
        except Exception:
            try:
                self.client.post("/api/command/zoom_get")
            except Exception:
                pass
        t0 = time.time()
        while time.time() - t0 < timeout:
            st = self.client.get("/api/state")
            if st.get("zoom") is not None:
                return int(st["zoom"])
            time.sleep(0.12)
        return int(self.st.get("pos") or PARK)

    def hold(self) -> int:
        try:
            out = self.client.post("/api/zoom/hold", {})
            if isinstance(out, dict) and out.get("position") is not None:
                return int(out["position"])
        except Exception:
            pass
        return self.wait_zoom()

    def move(self, pos: int, label: str, prompt: str) -> None:
        pos = clamp_motor(pos)
        self.say(f"moving {label} -> {pos}")
        self.client.post("/api/zoom", {"position": pos})
        time.sleep(0.35)
        self.wait_idle()
        landed = self.wait_arrived(pos)
        time.sleep(SETTLE_S)
        landed = self.hold() or landed
        self.st["pos"] = landed
        self.st["awaiting"] = True
        self.st["prompt"] = (
            f"{prompt}\n"
            f"ring {landed}. Type millimetres and optional × (default 1).  "
            f"Examples: 18    or 105 1.2"
        )
        self.save()
        self.say(self.st["prompt"])

    def post_lens(self, body: dict) -> None:
        self.client.post("/api/zoom/lens", body)

    def post_sample(self, mm: float, dx: float, pos: int, dir_: int) -> None:
        vmax = float(self.st["vmax"])
        accel = float(self.st["accel"])
        stored = store_mm(mm, dx)
        b = band_of(mm, dx)
        self.client.post(
            "/api/zoom/lens/sample",
            {
                "mm": stored,
                "pos": pos,
                "dir": dir_,
                "vmax": vmax,
                "accel": accel,
                "band": b,
            },
        )
        self.st.setdefault("samples", []).append(
            {"mm": stored, "pos": pos, "dir": dir_, "band": b, "optical": mm, "x": dx}
        )
        self.say(f"  sample pos={pos} mm={stored:.1f} dir={dir_} band={b}")

    def stats(self, xs: list[float]) -> dict:
        xs = [x for x in xs if x and x > 0]
        if not xs:
            return {"n": 0, "range": 99.0, "mean": 0.0}
        return {"n": len(xs), "range": max(xs) - min(xs), "mean": sum(xs) / len(xs)}

    def start(self) -> None:
        st = self.client.get("/api/state")
        if not st.get("connected"):
            raise SystemExit("gimbal not connected")
        self.wait_zoom()
        saved = {
            "vmax": st.get("zoom_vmax"),
            "accel": st.get("zoom_accel"),
            "vcut": st.get("zoom_vcut"),
            "decel": st.get("zoom_decel"),
        }
        self.client.post("/api/zoom/lens/enable", {"enabled": False})
        self.st = {
            "phase": "home_wide",
            "awaiting": False,
            "saved": saved,
            "try_v": 80.0,
            "try_a": 700.0,
            "t_static": 0.0,
            "t_run": 0.0,
            "vmax": 0.0,
            "accel": 0.0,
            "vcut": 0.0,
            "last_eq": None,
            "speed_i": 0,
            "speed_r": 0,
            "speed_samples": [],
            "speed_rows": [],
            "accel_i": 0,
            "accel_r": 0,
            "accel_samples": [],
            "accel_rows": [],
            "engage_dir": -1,
            "engage_travel": 0,
            "engage_step": 40,
            "engage_in": 0,
            "engage_out": 0,
            "map_i": 0,
            "samples": [],
        }
        self.profile(80.0, 700.0, 0, 700.0)
        self.say("=== characterize speeds ===")
        self.move(WIDE_POS, "home wide", "Fully out. Optical millimetres on the LCD (want ~18).")

    def abort(self) -> None:
        saved = (self.st or {}).get("saved") or {}
        if saved:
            self.profile(
                float(saved.get("vmax") or 500),
                float(saved.get("accel") or 1800),
                float(saved.get("vcut") or 80),
                float(saved.get("decel") or 1800),
            )
        self.st["awaiting"] = False
        self.st["phase"] = "aborted"
        self.save()
        self.say("aborted; restored previous speed profile")

    def apply(self, mm: float, dx: float) -> None:
        if not self.st.get("awaiting"):
            raise SystemExit("not waiting for a reading — pass --restart to start")
        eq = equiv(mm, dx)
        phase = self.st["phase"]
        pos = int(self.st.get("pos") or PARK)
        self.st["awaiting"] = False
        self.st["last_mm"] = mm
        self.st["last_dx"] = dx
        self.st["last_eq"] = eq
        self.say(f"  lcd mm={mm:g} x={dx:g} eq={eq:g}")
        getattr(self, f"after_{phase}")(mm, dx, eq, pos)

    def after_home_wide(self, mm: float, dx: float, eq: float, pos: int) -> None:
        self.st["phase"] = "park"
        self.st["wide_reading"] = eq
        self.profile(self.st["try_v"], self.st["try_a"], 0, self.st["try_a"])
        self.move(PARK, "park for T_static", "Parked mid-ring. Type the LCD.")

    def after_park(self, mm: float, dx: float, eq: float, pos: int) -> None:
        self.st["base_eq"] = eq
        self.st["phase"] = "tstatic"
        dest = clamp_motor(PARK - JOG)
        self.profile(self.st["try_v"], self.st["try_a"], 0, self.st["try_a"])
        self.move(
            dest,
            f"T_static jog {self.st['try_v']:.0f}/s",
            f"From rest at {self.st['try_v']:.0f}/s. Was {self.st['base_eq']:.1f} mm-eq. Type the LCD.",
        )

    def after_tstatic(self, mm: float, dx: float, eq: float, pos: int) -> None:
        base = float(self.st["base_eq"])
        if abs(eq - base) >= 0.45:
            self.st["t_static"] = float(self.st["try_v"])
            self.st["t_run"] = float(self.st["try_v"])
            self.st["trun_streak"] = 0
            self.say(f"T_static {self.st['t_static']:.0f}/s")
            self.st["phase"] = "trun_park"
            t = float(self.st["t_run"])
            self.profile(t, self.st["try_a"], 0, self.st["try_a"])
            self.move(PARK, f"T_run park {t:.0f}/s", f"Min-run park at {t:.0f}/s. Type the LCD.")
            return
        nxt = min(1200.0, round(float(self.st["try_v"]) * 1.25))
        if nxt == self.st["try_v"]:
            raise SystemExit("ring never moved the LCD — check F mode / motor calib")
        self.st["try_v"] = nxt
        self.st["phase"] = "tstatic_repark"
        self.move(PARK, "no move, repark", "No millimetre change. Parked. Type the LCD.")

    def after_tstatic_repark(self, mm: float, dx: float, eq: float, pos: int) -> None:
        self.st["base_eq"] = eq
        self.st["phase"] = "tstatic"
        dest = clamp_motor(PARK - JOG)
        self.profile(self.st["try_v"], self.st["try_a"], 0, self.st["try_a"])
        self.move(
            dest,
            f"T_static jog {self.st['try_v']:.0f}/s",
            f"From rest at {self.st['try_v']:.0f}/s. Was {self.st['base_eq']:.1f} mm-eq. Type the LCD.",
        )

    def after_trun_park(self, mm: float, dx: float, eq: float, pos: int) -> None:
        self.st["base_eq"] = eq
        self.st["phase"] = "trun_jog"
        t = float(self.st["t_run"])
        self.profile(t, self.st["try_a"], 0, self.st["try_a"])
        self.move(
            clamp_motor(PARK - JOG),
            f"T_run jog {t:.0f}/s",
            f"Repeat jog at {t:.0f}/s. Type the LCD.",
        )

    def after_trun_jog(self, mm: float, dx: float, eq: float, pos: int) -> None:
        base = float(self.st["base_eq"])
        if abs(eq - base) >= 0.45:
            self.st["trun_streak"] = int(self.st.get("trun_streak") or 0) + 1
        else:
            self.st["trun_streak"] = 0
            self.st["t_run"] = min(1600.0, round(float(self.st["t_run"]) * 1.2))
        if int(self.st["trun_streak"]) >= 2:
            self.say(f"T_running {self.st['t_run']:.0f}/s")
            self._begin_cruise()
            return
        self.st["phase"] = "trun_park"
        t = float(self.st["t_run"])
        self.profile(t, self.st["try_a"], 0, self.st["try_a"])
        self.move(PARK, f"T_run park {t:.0f}/s", f"Min-run park at {t:.0f}/s. Type the LCD.")

    def _begin_cruise(self) -> None:
        t_run = float(self.st["t_run"])
        lo = max(400.0, round(t_run * 4))
        self.st["ladder"] = [
            {"name": "slow", "vmax": lo},
            {"name": "medium", "vmax": max(900.0, round(lo * 2))},
            {"name": "fast", "vmax": 1800.0},
        ]
        self.st["med_a"] = max(400.0, round(t_run * 2))
        self.st["speed_i"] = 0
        self.st["speed_r"] = 0
        self.st["speed_samples"] = []
        self.st["speed_rows"] = []
        self._cruise_home()

    def _cruise_home(self) -> None:
        step = self.st["ladder"][int(self.st["speed_i"])]
        self.st["phase"] = "cruise_home"
        self.profile(step["vmax"], self.st["med_a"], self.st["t_run"], self.st["med_a"])
        r = int(self.st["speed_r"]) + 1
        self.move(
            WIDE_POS,
            f"{step['name']} home",
            f"{step['name']} cruise {step['vmax']:.0f}/s home (repeat {r}/2). Type the LCD.",
        )

    def after_cruise_home(self, mm: float, dx: float, eq: float, pos: int) -> None:
        step = self.st["ladder"][int(self.st["speed_i"])]
        self.st["phase"] = "cruise_test"
        r = int(self.st["speed_r"]) + 1
        self.move(
            TEST,
            f"{step['name']} to {TEST} at {step['vmax']:.0f}/s ({r}/2)",
            f"{step['name']} {step['vmax']:.0f}/s landed. Type the LCD.",
        )

    def after_cruise_test(self, mm: float, dx: float, eq: float, pos: int) -> None:
        self.st["speed_samples"].append(eq)
        self.st["speed_r"] = int(self.st["speed_r"]) + 1
        if int(self.st["speed_r"]) < 2:
            self._cruise_home()
            return
        step = self.st["ladder"][int(self.st["speed_i"])]
        st = self.stats(list(self.st["speed_samples"]))
        row = {**step, "samples": list(self.st["speed_samples"]), "st": st, "pass": st["range"] <= TOL}
        self.st["speed_rows"].append(row)
        self.say(f"  cruise {step['name']} {step['vmax']:.0f}/s samples={row['samples']} spread={st['range']:.2f}")
        if step["name"] != "slow" and not row["pass"]:
            self._finish_cruise()
            return
        self.st["speed_i"] = int(self.st["speed_i"]) + 1
        self.st["speed_r"] = 0
        self.st["speed_samples"] = []
        if int(self.st["speed_i"]) >= len(self.st["ladder"]):
            self._finish_cruise()
            return
        self._cruise_home()

    def _finish_cruise(self) -> None:
        rows = self.st["speed_rows"]
        chosen = next((row for row in reversed(rows) if row["pass"]), None)
        if chosen is None:
            chosen = sorted(rows, key=lambda r: r["st"]["range"])[0]
        self.st["vmax"] = chosen["vmax"]
        self.st["chosen_spread"] = chosen["st"]["range"]
        t_max = float(self.st["vmax"])
        self.st["accels"] = [
            {"name": "gentle", "a": max(400.0, round(t_max * 2))},
            {"name": "snappy", "a": max(800.0, round(t_max * 5))},
        ]
        self.st["accel_i"] = 0
        self.st["accel_r"] = 0
        self.st["accel_samples"] = []
        self.st["accel_rows"] = []
        self._accel_home()

    def _accel_home(self) -> None:
        step = self.st["accels"][int(self.st["accel_i"])]
        self.st["phase"] = "accel_home"
        self.profile(self.st["vmax"], step["a"], self.st["t_run"], step["a"])
        r = int(self.st["accel_r"]) + 1
        self.move(
            WIDE_POS,
            f"{step['name']} home",
            f"{step['name']} accel {step['a']:.0f}/s² home (repeat {r}/2). Type the LCD.",
        )

    def after_accel_home(self, mm: float, dx: float, eq: float, pos: int) -> None:
        step = self.st["accels"][int(self.st["accel_i"])]
        self.st["phase"] = "accel_test"
        r = int(self.st["accel_r"]) + 1
        self.move(
            TEST,
            f"{step['name']} {step['a']:.0f}/s2 ({r}/2)",
            f"{step['name']} {step['a']:.0f}/s² landed. Type the LCD.",
        )

    def after_accel_test(self, mm: float, dx: float, eq: float, pos: int) -> None:
        self.st["accel_samples"].append(eq)
        self.st["accel_r"] = int(self.st["accel_r"]) + 1
        if int(self.st["accel_r"]) < 2:
            self._accel_home()
            return
        step = self.st["accels"][int(self.st["accel_i"])]
        st = self.stats(list(self.st["accel_samples"]))
        row = {**step, "samples": list(self.st["accel_samples"]), "st": st, "pass": st["range"] <= TOL}
        self.st["accel_rows"].append(row)
        self.say(f"  accel {step['name']} {step['a']:.0f}/s2 samples={row['samples']} spread={st['range']:.2f}")
        self.st["accel_i"] = int(self.st["accel_i"]) + 1
        self.st["accel_r"] = 0
        self.st["accel_samples"] = []
        if int(self.st["accel_i"]) >= len(self.st["accels"]):
            self._finish_accel()
            return
        self._accel_home()

    def _finish_accel(self) -> None:
        rows = self.st["accel_rows"]
        passed = [r for r in rows if r["pass"]] or rows
        passed = sorted(passed, key=lambda r: (r["st"]["range"], -r["a"]))
        self.st["accel"] = passed[0]["a"]
        self.st["repeat_mm"] = max(float(self.st["chosen_spread"]), passed[0]["st"]["range"])
        self.st["vcut"] = float(self.st["t_run"])
        self.st["phase"] = "engage_park"
        self.st["engage_dir"] = -1
        self.profile(self.st["vmax"], self.st["accel"], self.st["vcut"], self.st["accel"])
        self.move(PARK, "engage in park", "Trigger in (toward tele): parked. Type the LCD.")

    def after_engage_park(self, mm: float, dx: float, eq: float, pos: int) -> None:
        self.st["engage_base"] = eq
        self.st["engage_origin"] = pos
        self.st["engage_travel"] = 0
        self.st["engage_step"] = 40
        self._engage_step()

    def _engage_step(self) -> None:
        self.st["engage_travel"] = int(self.st["engage_travel"]) + int(self.st["engage_step"])
        origin = int(self.st["engage_origin"])
        dir_ = int(self.st["engage_dir"])
        dest = clamp_motor(origin + dir_ * int(self.st["engage_travel"]))
        toward = "in" if dir_ < 0 else "out"
        self.st["phase"] = "engage_step"
        self.move(
            dest,
            f"engage {toward} {abs(dest - origin)}",
            f"Trigger {toward}: {abs(dest - origin)} counts from park. Type the LCD.",
        )

    def after_engage_step(self, mm: float, dx: float, eq: float, pos: int) -> None:
        base = float(self.st["engage_base"])
        origin = int(self.st["engage_origin"])
        dir_ = int(self.st["engage_dir"])
        if abs(eq - base) >= 0.45:
            counts = max(16, int(round(abs(pos - origin) * 1.08)))
            if dir_ < 0:
                self.st["engage_in"] = counts
                self.say(f"engage in {counts}")
                self.st["engage_dir"] = 1
                self.st["phase"] = "engage_park"
                self.move(PARK, "engage out park", "Trigger out (toward wide): parked. Type the LCD.")
            else:
                self.st["engage_out"] = counts
                self.say(f"engage out {counts}")
                self._begin_map()
            return
        if int(self.st["engage_travel"]) >= 900:
            raise SystemExit("millimetres never changed during engage steps")
        self.st["engage_step"] = min(int(round(int(self.st["engage_step"]) * 1.35)), 120)
        self._engage_step()

    def _begin_map(self) -> None:
        meta = {
            "name": NAME,
            "wide_mm": WIDE_MM,
            "tele_mm": TELE_MM,
            "coupling": True,
            "engage_in": int(self.st["engage_in"]),
            "engage_out": int(self.st["engage_out"]),
            "settle_s": SETTLE_S,
            "vmax": float(self.st["vmax"]),
            "accel": float(self.st["accel"]),
            "decel": float(self.st["accel"]),
            "vcut": float(self.st["vcut"]),
            "vstatic": float(self.st["t_static"]),
            "repeat_mm": float(self.st["repeat_mm"]),
            "enabled": False,
        }
        self.st["meta"] = meta
        self.client.post("/api/zoom/lens/clear", {})
        try:
            self.client.post("/api/zoom/lens/clear")
        except Exception:
            pass
        self.post_lens(meta)
        self.profile(meta["vmax"], meta["accel"], meta["vcut"], meta["accel"])
        self.say("=== map millimetres ===")
        self.st["phase"] = "map_wide"
        self.move(WIDE_POS, "map wide", "Map wide stop. Type the LCD (want ~18 mm).")

    def after_map_wide(self, mm: float, dx: float, eq: float, pos: int) -> None:
        self.post_sample(mm, dx, pos, 1)
        self.st["phase"] = "map_tele"
        self.move(TELE_POS, "map tele", "Map tele stop. Type optical mm and × if digital (want 105 ×2.0).")

    def after_map_tele(self, mm: float, dx: float, eq: float, pos: int) -> None:
        self.post_sample(mm, dx, pos, -1)
        self.st["map_i"] = 0
        self.st["phase"] = "map_in_home"
        self.say("inbound (motor decreasing / toward tele)")
        self.move(WIDE_POS, "inbound home", "Inbound home (wide). Type the LCD.")

    def after_map_in_home(self, mm: float, dx: float, eq: float, pos: int) -> None:
        self._map_in_point()

    def _map_in_point(self) -> None:
        i = int(self.st["map_i"])
        tgt = MAP_IN[i]
        self.st["phase"] = "map_in"
        self.move(tgt, f"in {tgt}", f"Inbound point {i + 1}/{len(MAP_IN)} ring {tgt}. Type the LCD.")

    def after_map_in(self, mm: float, dx: float, eq: float, pos: int) -> None:
        self.post_sample(mm, dx, pos, -1)
        self.st["map_i"] = int(self.st["map_i"]) + 1
        if int(self.st["map_i"]) < len(MAP_IN):
            self._map_in_point()
            return
        self.st["map_i"] = 0
        self.st["phase"] = "map_out_home"
        self.say("outbound (motor increasing / toward wide)")
        self.move(TELE_POS, "outbound home", "Outbound home (tele). Type optical mm and × if showing.")

    def after_map_out_home(self, mm: float, dx: float, eq: float, pos: int) -> None:
        self._map_out_point()

    def _map_out_point(self) -> None:
        i = int(self.st["map_i"])
        tgt = MAP_OUT[i]
        self.st["phase"] = "map_out"
        self.move(tgt, f"out {tgt}", f"Outbound point {i + 1}/{len(MAP_OUT)} ring {tgt}. Type the LCD.")

    def after_map_out(self, mm: float, dx: float, eq: float, pos: int) -> None:
        self.post_sample(mm, dx, pos, 1)
        self.st["map_i"] = int(self.st["map_i"]) + 1
        if int(self.st["map_i"]) < len(MAP_OUT):
            self._map_out_point()
            return
        vmax = float(self.st["vmax"])
        accel = float(self.st["accel"])
        vcut = float(self.st["vcut"])
        self.client.post("/api/zoom/lens/enable", {"enabled": True})
        self.post_lens(
            {
                "name": NAME,
                "wide_mm": WIDE_MM,
                "tele_mm": TELE_MM,
                "enabled": True,
                "vmax": vmax,
                "accel": accel,
                "decel": accel,
                "vcut": vcut,
            }
        )
        self.profile(vmax, accel, vcut, accel)
        lens = self.client.get("/api/zoom/lens")
        self.st["phase"] = "done"
        self.st["awaiting"] = False
        self.st["prompt"] = ""
        self.save()
        self.say(f"done samples={lens.get('count')} enabled={lens.get('enabled')}")
        self.say(
            f"vmax={vmax} accel={accel} vcut={vcut} "
            f"engage {self.st['engage_in']}/{self.st['engage_out']}"
        )


def parse_reading(mm_s: str, x_s: str | None) -> tuple[float, float]:
    mm = float(mm_s.replace("mm", "").strip())
    dx = 1.0 if not x_s else float(x_s.replace("x", "").replace("×", "").strip())
    if dx <= 0:
        dx = 1.0
    return mm, dx


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("mm", nargs="?", help="optical millimetres from the LCD")
    parser.add_argument("x", nargs="?", help="digital ×, default 1")
    parser.add_argument("--restart", action="store_true")
    parser.add_argument("--abort", action="store_true")
    args = parser.parse_args()

    cal = Cal()
    if args.abort:
        if cal.load():
            cal.abort()
        return
    if args.restart or not cal.load() or cal.st.get("phase") in (None, "done", "aborted"):
        cal.start()
        return
    if args.mm is None:
        if cal.st.get("awaiting"):
            cal.say(cal.st.get("prompt") or "waiting for millimetres")
            return
        raise SystemExit("nothing to do — pass millimetres or --restart")
    mm, dx = parse_reading(args.mm, args.x)
    cal.apply(mm, dx)


if __name__ == "__main__":
    main()
