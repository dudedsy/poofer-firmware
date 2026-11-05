"""Valve characterization: coil R and L, armature motion, opening / closing delays at the
nozzle sensor, and dropout current. Writes valves/<name>.json, which experiments use for
the coil model (loop gains) and the hold current (2 x dropout, solenoid spec §9).

No camera, LED off. Gas on at the test pressure (the valve is semi-direct, so its timing
depends on pressure), nozzle sensor fitted.
"""
from __future__ import annotations

import datetime as dt
import json
import math
from pathlib import Path

import numpy as np

from .config import Rig
from .experiment import Quit, ask, wait_pressure
from .link import Device
from .report import run_report
from .shot import ShotError, run_shot

TESTS = ("coil", "response", "dropout")


def _stats(vals):
    v = np.array([x for x in vals if x is not None and isinstance(x, (int, float)) and math.isfinite(x)], float)
    if not len(v):
        return None, None, 0
    return float(v.mean()), (float(v.std(ddof=1)) if len(v) > 1 else None), int(len(v))


def characterize(rig: Rig, dev: Device, name: str, pressure: float | None, tests=TESTS, repeats: int = 3,
                 auto: bool = False, ramp: float = -0.5, start_hold: float | None = None, out=print) -> Path:
    if not rig["valve"]:
        raise SystemExit("rig.yaml has no valve board")
    stamp = f"{dt.datetime.now():%Y%m%d-%H%M%S}"
    base = rig.data_dir / "valves" / name / stamp
    base.mkdir(parents=True, exist_ok=True)
    vfile_path = rig.data_dir / "valves" / f"{name}.json"
    prev = json.loads(vfile_path.read_text()) if vfile_path.exists() else {}
    tol = 2.0
    found: dict = {"name": name, "date": stamp, "pressure_psi": pressure, "bus_voltage": rig["bus_voltage"], "runs": {}}
    common = {"camera": False, "led": False, "pressure_psi": pressure, "pressure_tol": tol}
    vf = {k: prev.get("recommended", {}).get(k) for k in ("r_coil", "l_coil")} if prev else {}
    vf = {k: v for k, v in vf.items() if v}

    def shot(tag, params, close90=None):
        run_dir = base / tag
        psi = wait_pressure(dev, rig, pressure, tol, auto, out)
        if psi is not None and psi != psi:
            raise Quit()
        if not auto and ask(f"  {tag}: Enter = FIRE, q = quit: ", ",q") == "q":
            raise Quit()
        p = dict(common)
        p.update(params)
        run_shot(dev, rig, run_dir, p, {"valve": name, "test": tag, "pressure_at_fire": psi}, vf, log=out)
        res = run_report(run_dir, close90_ms=close90, video=False, log=out)
        found["runs"][tag] = {k: v for k, v in res.items() if isinstance(v, (int, float)) and not isinstance(v, bool)}
        return res

    try:
        if "coil" in tests:
            out("\ncoil test: one 300 ms fire")
            r = shot("coil_1", {"open_ms": 300})
            for k in ("r_coil_est", "l_coil_est", "l_closed_est", "tau_rise_ms", "armature_ms", "armature_end_ms", "armature_i_a",
                      "i_peak_max", "t_peak_90_ms"):
                found[k] = r.get(k)
            if r.get("r_coil_est"):
                vf["r_coil"] = round(float(r["r_coil_est"]), 2)
            if r.get("l_coil_est"):
                vf["l_coil"] = round(float(r["l_coil_est"]), 3)
        if "response" in tests:
            out(f"\nresponse test: {repeats} x 300 ms fires")
            rs = [shot(f"response_{k + 1}", {"open_ms": 300}) for k in range(repeats)]
            for k in ("t_open_10_ms", "t_open_50_ms", "t_open_90_ms", "t_close_90_ms", "t_close_50_ms", "t_close_10_ms",
                      "p_noz_plateau", "armature_ms"):
                m, s, n = _stats([r.get(k) for r in rs])
                found[k] = m
                found[k + "_sd"] = s
        if "dropout" in tests:
            hold0 = start_hold or float(rig["valve"]["hold_a"])
            ramp_at = 300
            open_ms = int(ramp_at + (hold0 - 0.1) / abs(ramp) * 1000 + 300)
            t_max = rig["valve"].get("t_max_s")
            out(f"\ndropout test: hold {hold0:.2f} A, ramp {ramp} A/s from {ramp_at} ms, {open_ms} ms open"
                f" (hardware limiter T_max must be > {2 * (open_ms + 50) / 1000:.1f} s"
                f"{'' if t_max else '; set valve.t_max_s in rig.yaml once measured'})")
            close90 = found.get("t_close_90_ms")
            rs = [shot(f"dropout_{k + 1}", {"open_ms": open_ms, "hold_a": hold0, "ramp": ramp, "ramp_at": ramp_at,
                                            "hold_min": 0.1, "max_on_ms": open_ms + 50}, close90) for k in range(2)]
            for k in ("i_dropout_est", "i_at_pressure_fall"):
                m, s, n = _stats([r.get(k) for r in rs])
                found[k] = m
                found[k + "_sd"] = s
    except Quit:
        out("stopped early; saving what was measured")
    except ShotError as e:
        out(f"shot failed: {e}; saving what was measured")

    rec = dict(prev.get("recommended", {}))
    rec.update({"r_coil": vf.get("r_coil"), "l_coil": vf.get("l_coil")})
    drop = found.get("i_dropout_est") or found.get("i_at_pressure_fall")
    if drop:
        # spec §9: hold >= 2 x measured dropout. Rounded up to 0.05 A. Check against the coil rating.
        rec["hold_a"] = math.ceil(2 * drop / 0.05) * 0.05
    found["recommended"] = {k: v for k, v in rec.items() if v is not None}
    for k in ("t_close_90_ms",):
        if found.get(k) is None and prev.get(k) is not None:
            found[k] = prev[k]
    vfile_path.parent.mkdir(parents=True, exist_ok=True)
    vfile_path.write_text(json.dumps(found, indent=2, default=float))
    (base / "valve.json").write_text(json.dumps(found, indent=2, default=float))
    out(f"\nvalve file: {vfile_path}")
    for k in ("r_coil_est", "l_coil_est", "l_closed_est", "armature_ms", "armature_end_ms", "t_open_50_ms", "t_open_90_ms", "t_close_90_ms",
              "t_close_10_ms", "i_dropout_est", "i_at_pressure_fall"):
        if found.get(k) is not None:
            sd = found.get(k + "_sd")
            out(f"  {k:20s} {found[k]:.4g}" + (f" ± {sd:.2g}" if sd else ""))
    out(f"  recommended: {found['recommended']}")
    return vfile_path


def load_valve(rig: Rig, name: str | None) -> dict | None:
    """valves/<name>.json 'recommended' block as valve overrides (r_coil, l_coil, hold_a)."""
    if not name:
        return None
    p = rig.data_dir / "valves" / f"{name}.json"
    if not p.exists():
        raise SystemExit(f"no valve file {p}; run `rigctl characterize {name}` first, or drop 'valve:' from the experiment")
    v = json.loads(p.read_text())
    rec = dict(v.get("recommended", {}))
    rec["t_close_90_ms"] = v.get("t_close_90_ms")
    return rec
