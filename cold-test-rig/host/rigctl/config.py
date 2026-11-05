"""rig.yaml (how the rig is wired) and rig_cal.json (pressure zeros), with defaults."""
from __future__ import annotations

import copy
import json
from pathlib import Path

import yaml

DEFAULTS = {
    "port": "COM5",
    "data_dir": "data",
    "bus_voltage": 24.0,          # measured at the base board with a meter; used for feed-forward and R estimates
    "pressure": [
        {"name": "accumulator", "adc": 0, "range_psi": 100.0, "supply": 3.3, "divider": 1.0},
        {"name": "nozzle", "adc": 1, "range_psi": 100.0, "supply": 3.3, "divider": 1.0},
    ],
    "accumulator_sensor": "accumulator",   # used for fill checks and consumption
    "nozzle_sensor": "nozzle",             # used for valve opening / closing timing
    "valve": {"pair": 1, "adc": 3, "r_path": 0.15, "v_free": 0.8, "max_on_ms": 3000,
              "peak_a": 2.7, "peak_ms": 100, "hold_a": 1.0, "hold_duty_max": 0.5, "r_coil": 8.5, "l_coil": 0.1,
              "t_max_s": None},           # measured hardware limiter T_max (solenoid spec §12); shots refuse > T_max / 2
    "led": {"pair": 2, "adc": 2, "current_a": 2.0, "max_on_s": 20},
    "thermocouple": {"type": "K", "label": "gas"},
    "camera": {
        "backend": "ffmpeg",                  # ffmpeg | fake | none
        "device": "Global Shutter Camera",
        "size": "1280x720",
        "fps": 200,                           # requested rate
        "fps_real": None,                     # measured real rate if it differs (Arducam fast modes)
        "pixel_format": "yuyv422",            # yuyv422 or mjpeg
        "ffmpeg": "ffmpeg",
        "rtbufsize": "512M",
        "start_timeout_s": 10.0,
        "container": "avi",
        "fake": {"truth": None, "size": "320x180", "offset_ms": 1234.5, "period_error": 0.002},
    },
    "video": {
        "sync_roi": None,         # [x0, y0, x1, y1] full-resolution pixels around the signal LED; None = auto
        "jet_roi": None,          # [x0, y0, x1, y1] region the jet crosses; None = whole frame
        "nozzle_px": None,        # [x, y] nozzle exit; None = left edge, mid height
        "direction": "+x",        # jet direction in the image: +x, -x, +y, -y
        "mm_per_px": 1.7,         # full-resolution scale at the jet plane
        "analysis_scale": 0.5,    # downscale for the jet analysis
        "contact_frames": 12,
    },
    "shot": {"fire_ms": 800, "post_ms": 1500, "tc_ms": None, "dip": False},   # tc_ms None = auto from fps
}


def _merge(base, over):
    out = copy.deepcopy(base)
    for k, v in (over or {}).items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = _merge(out[k], v)
        else:
            out[k] = v
    return out


class Rig:
    def __init__(self, path: str | Path | None):
        self.path = Path(path) if path else None
        user = {}
        if self.path and self.path.exists():
            user = yaml.safe_load(self.path.read_text()) or {}
        self.cfg = _merge(DEFAULTS, user)
        base = self.path.parent if self.path else Path(".")
        self.base_dir = base
        self.cal_path = base / "rig_cal.json"
        self.cal = json.loads(self.cal_path.read_text()) if self.cal_path.exists() else {}
        dd = Path(self.cfg["data_dir"])
        self.data_dir = dd if dd.is_absolute() else base / dd
        self._check()

    def _check(self):
        lines = [p["adc"] for p in self.cfg["pressure"]]
        if self.cfg["valve"]:
            lines.append(self.cfg["valve"]["adc"])
        if self.cfg["led"]:
            lines.append(self.cfg["led"]["adc"])
        if len(lines) != len(set(lines)):
            raise ValueError(f"rig.yaml: two things on one ADC line: {lines}")
        if not 1 <= len(self.cfg["pressure"]) <= 3:
            raise ValueError("rig.yaml: 1 to 3 pressure sensors")
        names = [p["name"] for p in self.cfg["pressure"]]
        for key in ("accumulator_sensor", "nozzle_sensor"):
            if self.cfg[key] and self.cfg[key] not in names:
                raise ValueError(f"rig.yaml: {key} '{self.cfg[key]}' is not one of {names}")

    def __getitem__(self, k):
        return self.cfg[k]

    def get(self, k, d=None):
        return self.cfg.get(k, d)

    # ------------------------------------------------------------ pressure scaling
    def sensor(self, name: str) -> dict:
        for p in self.cfg["pressure"]:
            if p["name"] == name:
                return p
        raise KeyError(name)

    def counts_per_psi(self, p: dict) -> float:
        # ratiometric 0.5-4.5 V at 5 V = 10 %..90 % of supply; ADC reference 3.3 V
        if "counts_per_psi" in p:
            return float(p["counts_per_psi"])
        span_v = 0.8 * p.get("supply", 3.3) * p.get("divider", 1.0)
        return span_v / 3.3 * 4095.0 / p["range_psi"]

    def zero_counts(self, p: dict) -> float:
        z = self.cal.get("zero", {}).get(p["name"])
        if z is not None:
            return float(z)
        return 0.1 * p.get("supply", 3.3) * p.get("divider", 1.0) / 3.3 * 4095.0

    def psi(self, name: str, counts):
        """counts: mean ADC counts (0..4095), scalar or array."""
        p = self.sensor(name)
        return (counts - self.zero_counts(p)) / self.counts_per_psi(p)

    def save_zero(self, zeros: dict):
        self.cal.setdefault("zero", {}).update(zeros)
        self.cal_path.write_text(json.dumps(self.cal, indent=2))

    # ------------------------------------------------------------ device setup
    def valve_keys(self, valve: dict | None = None) -> dict:
        v = dict(self.cfg["valve"] or {})
        v.update(valve or {})
        return {"v.vbus": self.cfg["bus_voltage"], "v.r": v["r_coil"], "v.l": v["l_coil"], "v.peak": v["peak_a"],
                "v.peakms": v["peak_ms"], "v.hold": v["hold_a"], "v.holdmax": v["hold_duty_max"], "v.maxon": v["max_on_ms"]}

    def led_keys(self) -> dict:
        l = self.cfg["led"]
        if not l:
            return {}
        return {"l.vbus": self.cfg["bus_voltage"], "l.maxon": l["max_on_s"], "l.i": l["current_a"]}


def configure_device(dev, rig: Rig, valve: dict | None = None):
    """Wire map + valve + LED settings. The drives must be off. The LED is detached first so a
    pair or ADC swap between the two drives never collides with the firmware's checks."""
    v, l = rig["valve"], rig["led"]
    dev.set({"l.pair": 0})
    dev.set({"v.pair": v["pair"], "v.adc": v["adc"]} if v else {"v.pair": 0})
    if l:
        dev.set({"l.adc": l["adc"], "l.pair": l["pair"]})
    if v:
        dev.set(rig.valve_keys(valve))
    if l:
        dev.set(rig.led_keys())
