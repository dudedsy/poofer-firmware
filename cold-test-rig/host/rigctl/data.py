"""Load a run folder into physical units on one time base: ms relative to 'go'.

Sample frame f of block b covers (b-1 + f/5, b-1 + (f+1)/5] ms; it is stamped at its centre.
Telemetry of block b is the drive state after the control update at the end of block b.
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from . import thermo
from .config import Rig
from .link import LED_STATES, VALVE_STATES


class Run:
    def __init__(self, path: str | Path):
        self.dir = Path(path)
        self.meta = json.loads((self.dir / "run.json").read_text())
        self.params = self.meta["params"]
        self.rig = Rig(None)
        self.rig.cfg = self.meta["rig"]
        self.rig.cal = self.meta.get("cal", {})
        self.go = self.meta["go_ms"] or 0
        self.events = [json.loads(l) for l in (self.dir / "events.jsonl").read_text().splitlines() if l.strip()]
        self.status = [json.loads(l) for l in (self.dir / "status.jsonl").read_text().splitlines() if l.strip()]
        f = self.dir / "samples.npz"
        self.has_samples = f.exists()
        if self.has_samples:
            z = np.load(f)
            self.raw = z["samples"].astype(np.float64) / 4.0      # mean counts 0..4095
            self.tele = z["tele"]
            self.valid = z["valid"]
            self.t0 = int(z["t0_ms"])
            self.fpm = int(z["fpm"])
            n = len(self.raw)
            j = np.arange(n)
            self.t = (self.t0 - 1 + (j // self.fpm) + ((j % self.fpm) + 0.5) / self.fpm) - self.go
            self.tt = (self.t0 + np.arange(len(self.tele))) - self.go          # telemetry times
            self.valid_s = np.repeat(self.valid, self.fpm)

    # ------------------------------------------------------------ events
    def ev(self, name: str, nth: int = 0):
        hits = [e for e in self.events if e["ev"] == name]
        return hits[nth] if len(hits) > nth else None

    def ev_t(self, name: str, nth: int = 0):
        e = self.ev(name, nth)
        return None if e is None else e["t"] - self.go

    @property
    def t_fire(self):
        e = self.ev("fire")
        return None if e is None or e.get("skipped") else e["t"] - self.go

    @property
    def t_release(self):
        return self.ev_t("release")

    @property
    def t_end(self):
        return self.ev_t("end") if self.ev("end") else self.ev_t("abort")

    # ------------------------------------------------------------ signals
    def adc_counts(self, line: int):
        return self.raw[:, line]

    def psi(self, name: str):
        p = self.rig.sensor(name)
        return self.rig.psi(name, self.raw[:, p["adc"]])

    def pressure_names(self):
        return [p["name"] for p in self.rig["pressure"]]

    def sense_amps(self, line: int):
        """Q2 current from the board's current sense (0.5 V/A, 1 ms RC): duty x load current."""
        return self.raw[:, line] * 3.3 / 4095.0 / 0.5

    @property
    def valve_i(self):
        return self.tele["v_ma"] / 1000.0

    @property
    def valve_d(self):
        return self.tele["v_duty"] / 10000.0

    @property
    def valve_state(self):
        return np.array([VALVE_STATES[s & 0x0F] if (s & 0x0F) < len(VALVE_STATES) else "?" for s in self.tele["v_st"]])

    @property
    def led_i(self):
        return self.tele["l_ma"] / 1000.0

    @property
    def led_d(self):
        return self.tele["l_duty"] / 10000.0

    @property
    def led_state(self):
        return np.array([LED_STATES[s & 0x0F] if (s & 0x0F) < len(LED_STATES) else "?" for s in self.tele["l_st"]])

    def temperatures(self):
        """Status-rate (10 Hz) temperatures: chip, cold junction, thermocouple (degC, NaN if absent)."""
        kind = (self.rig["thermocouple"] or {}).get("type", "K")
        t = np.array([s["t"] for s in self.status], float) - self.go
        chip = np.array([s.get("chip", np.nan) for s in self.status], float)
        cj = np.array([np.nan if s.get("cj") is None else s["cj"] for s in self.status], float)
        tc = []
        for s in self.status:
            v = thermo.from_ads1115(s.get("tc"), s.get("cj"), kind) if self.rig["thermocouple"] else None
            tc.append(np.nan if v is None else v)
        return {"t": t, "chip": chip, "cj": cj, "tc": np.array(tc, float)}

    def mean(self, sig, t, t0, t1):
        m = (t >= t0) & (t < t1)
        return float(np.mean(sig[m])) if m.any() else float("nan")
