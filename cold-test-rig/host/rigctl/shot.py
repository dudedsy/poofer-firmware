"""One shot: configure, start the camera, 'go', collect, save raw data to a run folder."""
from __future__ import annotations

import datetime as dt
import json
import time
from pathlib import Path

import numpy as np

from .camera import CameraError, make_camera
from .config import Rig
from .link import Device, DeviceError

# Parameters a shot or an experiment may set, with what they map to.
PARAM_DEFAULTS = {
    "open_ms": 300,          # fire -> release
    "fire_ms": None,         # go -> fire (rig.yaml shot.fire_ms)
    "post_ms": None,         # release -> end of recording (rig.yaml shot.post_ms)
    "pressure_psi": None,    # accumulator fill target before firing, None = don't check
    "pressure_tol": 2.0,
    "peak_a": None, "peak_ms": None, "hold_a": None,   # None = valve file / rig.yaml
    "led_a": None,           # None = rig.yaml led.current_a
    "camera": True,
    "led": None,             # None = on when a camera records and an LED board exists
    "valve": True,
    "tc_ms": None,
    "dip": None,
    "ramp": 0.0, "ramp_at": 300, "hold_min": 0.1,      # hold current ramp (dropout test)
    "max_on_ms": None,
}


class ShotError(RuntimeError):
    pass


def resolve(params: dict, rig: Rig) -> dict:
    p = dict(PARAM_DEFAULTS)
    unknown = set(params) - set(PARAM_DEFAULTS)
    if unknown:
        raise ShotError(f"unknown shot parameter(s): {sorted(unknown)}; known: {sorted(PARAM_DEFAULTS)}")
    p.update({k: v for k, v in params.items() if v is not None or k in ("pressure_psi",)})
    s = rig["shot"]
    p["fire_ms"] = p["fire_ms"] if p["fire_ms"] is not None else s["fire_ms"]
    p["post_ms"] = p["post_ms"] if p["post_ms"] is not None else s["post_ms"]
    p["tc_ms"] = p["tc_ms"] if p["tc_ms"] is not None else (s["tc_ms"] or auto_tc_ms(rig["camera"]))
    p["dip"] = bool(p["dip"] if p["dip"] is not None else s["dip"])
    has_led = bool(rig["led"])
    if p["led"] is None:
        p["led"] = has_led and bool(p["camera"]) and rig["camera"]["backend"] != "none"
    if p["led"] and not has_led:
        raise ShotError("shot wants the LED but rig.yaml has no LED board (led: null)")
    if p["led_a"] is None and has_led:
        p["led_a"] = rig["led"]["current_a"]
    return p


def auto_tc_ms(cam: dict) -> int:
    """Timecode cell in ms: the smallest integer >= 1.35 frame periods whose ratio to the frame
    period has a denominator of at least 5. The frames then sample the blink edges at 5 or more
    evenly spread phases, so the edge-time rounding averages out in the sync fit (a 10 ms cell at
    exactly 200 fps samples one phase only), and a cell longer than a frame is never missed."""
    from fractions import Fraction
    fps = Fraction(str(cam.get("fps_real") or cam["fps"])).limit_denominator(1000)
    n = max(2, int(np.ceil(1.35 * 1000 / float(fps))))
    while (n * fps / 1000).denominator < 5:
        n += 1
    return n


def firmware_shot_keys(p: dict) -> dict:
    return {"led": int(p["led"]), "led_a": p["led_a"] or 0, "led_req": int(p["led"]), "valve": int(p["valve"]),
            "fire": int(p["fire_ms"]), "open": int(p["open_ms"]), "post": int(p["post_ms"]), "tc": 1,
            "tc_ms": int(p["tc_ms"]), "dip": int(p["dip"]), "ramp": p["ramp"], "ramp_at": int(p["ramp_at"]),
            "hold_min": p["hold_min"]}


def valve_overrides(p: dict, valve_file: dict | None) -> dict:
    v = dict(valve_file or {})
    for k in ("peak_a", "peak_ms", "hold_a", "max_on_ms"):
        if p.get(k) is not None:
            v[k] = p[k]
    return v


def run_shot(dev: Device, rig: Rig, run_dir: Path, params: dict, meta: dict | None = None,
             valve_file: dict | None = None, log=print) -> Path:
    """Fire one shot and save it. Returns the run folder. Raises ShotError on failure; the
    folder then holds whatever was recorded, with status 'error' or 'aborted'."""
    run_dir = Path(run_dir)
    run_dir.mkdir(parents=True, exist_ok=True)
    p = resolve(params, rig)
    st = dev.wait_status()
    if st["shot"] != "idle":
        raise ShotError("device busy (a shot is running)")
    for drive in ("v", "l"):
        if st[drive]["st"] == "FAULT":
            raise ShotError(f"{'valve' if drive == 'v' else 'LED'} fault latched: {st[drive]['f']} (rigctl clear)")
    if st["v"]["st"] != "OFF":
        raise ShotError("valve not off")
    if rig["valve"]:
        vk = rig.valve_keys(valve_overrides(p, valve_file))
        if p["valve"] and vk["v.maxon"] < p["open_ms"] + 50:
            vk["v.maxon"] = p["open_ms"] + 50
        t_max = rig["valve"].get("t_max_s")
        if p["valve"] and t_max and vk["v.maxon"] > t_max * 1000 / 2:
            raise ShotError(f"open {p['open_ms']} ms needs firmware max-on {vk['v.maxon']} ms, over half the "
                            f"hardware limiter T_max ({t_max} s, rig.yaml valve.t_max_s). Turn RV1 up and re-measure.")
        dev.set(vk)
    dev.shot(firmware_shot_keys(p))
    info = dev.info()

    cam = make_camera(rig["camera"], enabled=bool(p["camera"]))
    video = run_dir / f"video.{'mkv' if cam.name == 'fake' else rig['camera']['container']}"
    status = "ok"
    why = ""
    cam_info = {"backend": "none"}
    rec = None
    t_go_host = None
    try:
        cam.start(video, dev)
        rec = dev.start_recording()
        mark = dev.event_mark()
        t_go_host = time.time()
        dev.command("go")
        total_s = (p["fire_ms"] + p["open_ms"] + p["post_ms"]) / 1000.0
        try:
            ev = dev.wait_event({"end", "abort"}, mark, timeout=total_s + 5.0)
        except KeyboardInterrupt:
            dev.send("abort")
            ev = dev.wait_event({"end", "abort"}, mark, timeout=2.0)
            status, why = "aborted", "keyboard"
        if ev is None:
            dev.send("abort")
            status, why = "error", "no end event (device stopped responding?)"
        elif ev["ev"] == "abort" and status == "ok":
            status, why = "aborted", ev.get("why", "")
        time.sleep(0.3)                      # trailing packet after the end event
    except (CameraError, DeviceError) as e:
        status, why = "error", str(e)
    finally:
        if rec is not None:
            dev.stop_recording()
        try:
            cam_info = cam.stop()
        except CameraError as e:
            cam_info = {"backend": cam.name, "error": str(e)}
            if status == "ok":
                status, why = "error", f"camera: {e}"

    arrays = rec.arrays() if rec else None
    events = rec.events if rec else []
    go = next((e for e in events if e["ev"] == "go"), None)
    if arrays is not None:
        np.savez_compressed(run_dir / "samples.npz", samples=arrays["samples"], tele=arrays["tele"],
                            valid=arrays["valid"], t0_ms=arrays["t0_ms"], fpm=arrays["fpm"])
        if not arrays["valid"].all():
            log(f"  warning: {int((~arrays['valid']).sum())} ms of samples missing (USB drops)")
    with open(run_dir / "events.jsonl", "w") as f:
        for e in events:
            f.write(json.dumps(e) + "\n")
    with open(run_dir / "status.jsonl", "w") as f:
        for s in (rec.status if rec else []):
            f.write(json.dumps(s) + "\n")
    meta_out = {
        "created": dt.datetime.now().isoformat(timespec="seconds"),
        "status": status, "why": why,
        "params": p, "meta": meta or {},
        "go_ms": go["t"] if go else None, "go_host_time": t_go_host,
        "device": {"port": dev.port, "info": info},
        "rig": rig.cfg, "cal": rig.cal, "valve_file": valve_file,
        "camera": cam_info, "video": video.name if video.exists() else None,
        "crc_errors": rec.crc_errors if rec else 0,
        "device_log": rec.log if rec else [],
    }
    (run_dir / "run.json").write_text(json.dumps(meta_out, indent=2, default=str))
    if status != "ok":
        raise ShotError(f"{status}: {why}")
    return run_dir
