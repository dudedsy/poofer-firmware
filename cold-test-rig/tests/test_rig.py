"""End-to-end test of rig_control + rigctl against the simulator.

Builds tools/sim, starts it on a free port, then runs valve characterization and a small
experiment through the real rigctl code (serial link over socket://, fake camera rendered
from the simulator's truth log), and checks the results against the simulator's own
parameters. Also unit-checks the thermocouple tables, the timecode LFSR and the packet CRC.

    python cold-test-rig/tests/test_rig.py          (needs g++, make, ffmpeg; Linux / WSL)
"""
from __future__ import annotations

import json
import math
import os
import socket
import subprocess
import sys
import tempfile
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT / "host"))

from rigctl import thermo  # noqa: E402
from rigctl.analysis import lfsr_bits  # noqa: E402
from rigctl.link import crc16  # noqa: E402

fails = 0
passes = 0


def check(cond, msg):
    global fails, passes
    if cond:
        passes += 1
    else:
        fails += 1
        print("  FAIL:", msg)


def near(v, want, tol, what):
    ok = v is not None and isinstance(v, (int, float)) and math.isfinite(v) and abs(v - want) <= tol
    check(ok, f"{what} = {v}, want {want} ± {tol}")
    print(f"  {what:34s} {v if v is None else f'{v:.4g}':>10}   (want {want} ± {tol})")


def unit():
    print("unit checks")
    near(thermo.emf_mv(100, "K"), 4.096, 0.001, "type K emf at 100 C (NIST 4.096 mV)")
    near(thermo.emf_mv(100, "N"), 2.774, 0.001, "type N emf at 100 C (NIST 2.774 mV)")
    near(thermo.temperature_c(thermo.emf_mv(300, "K") - thermo.emf_mv(25, "K"), 25, "K"), 300, 0.001, "type K round trip 300 C")
    check(thermo.from_ads1115(32767, 22) is None, "ADS1115 full scale reads as open probe")
    b = lfsr_bits(1100)
    check(b[:511].tolist() == b[511:1022].tolist() and b[:255].tolist() != b[255:510].tolist(), "LFSR period 511")
    check(crc16(b"123456789") == 0x29B1, "CRC-16/CCITT-FALSE check value")


def free_port():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


def main():
    unit()
    subprocess.run(["make", "-s", "-C", str(ROOT / "tools" / "sim")], check=True)
    work = Path(tempfile.mkdtemp(prefix="rigtest_"))
    port = free_port()
    truth = work / "truth.csv"
    sim = subprocess.Popen([str(ROOT / "tools" / "sim" / "rig_sim"), "--port", str(port), "--truth", str(truth)],
                           stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    time.sleep(0.4)
    rig_yaml = (ROOT / "examples" / "rig_sim.yaml").read_text()
    rig_yaml = rig_yaml.replace("socket://127.0.0.1:5555", f"socket://127.0.0.1:{port}").replace("sim_truth.csv", str(truth))
    (work / "rig.yaml").write_text(rig_yaml)
    exp = work / "exp.yaml"
    exp.write_text("""name: t
valve: simvalve
defaults: {pressure_psi: 60, pressure_tol: 2}
conditions:
  - {nozzle: A}
  - {nozzle: B}
sweep: {open_ms: [400]}
repeats: 1
""")
    env = dict(os.environ, PYTHONPATH=str(ROOT / "host"))
    rigctl = [sys.executable, "-m", "rigctl.cli", "--rig", str(work / "rig.yaml")]
    try:
        print("\nvalve characterization (simulator: R 8.7 ohm, L 0.075 -> 0.120 H, dropout 0.32 A)")
        r = subprocess.run(rigctl + ["characterize", "simvalve", "--pressure", "60", "--repeats", "1", "--auto"],
                           env=env, capture_output=True, text=True, timeout=300)
        if r.returncode:
            print(r.stdout[-3000:], r.stderr[-3000:])
        check(r.returncode == 0, "characterize exit code")
        v = json.loads((work / "sim_data" / "valves" / "simvalve.json").read_text())
        near(v.get("r_coil_est"), 8.7, 0.15, "coil R")
        near(v.get("l_coil_est"), 0.075, 0.008, "coil L, armature open")
        near(v.get("l_closed_est"), 0.120, 0.012, "coil L, armature closed")
        near(v.get("armature_ms"), 3.8, 1.5, "armature starts (ms)")
        near(v.get("i_dropout_est"), 0.32, 0.03, "dropout current (A)")
        near(v["recommended"].get("hold_a"), 0.65, 0.051, "recommended hold (2 x dropout)")
        near(v.get("t_open_50_ms"), 22, 4, "valve open 50 % (ms)")
        near(v.get("t_close_10_ms"), 53, 6, "valve closed to 10 % (ms)")

        print("\nexperiment (2 conditions x 1 run, fake camera)")
        r = subprocess.run(rigctl + ["run", str(exp), "--name", "e1", "--auto"], env=env, capture_output=True, text=True,
                           timeout=300)
        if r.returncode:
            print(r.stdout[-3000:], r.stderr[-3000:])
        check(r.returncode == 0, "run exit code")
        ed = work / "sim_data" / "e1"
        check((ed / "summary.html").exists(), "summary.html written")
        runs = sorted(p for p in ed.iterdir() if (p / "analysis.json").exists())
        check(len(runs) == 2, f"2 runs analysed, got {len(runs)}")
        for rd in runs:
            a = json.loads((rd / "analysis.json").read_text())
            sy = a["video"]["sync"]
            check(sy.get("ok"), f"{rd.name}: video sync ok")
            end_err = (sy["a_ms"] + sy["period_ms"] * sy["n_frames"]) - (sy["truth_a_ms"] + sy["truth_period_ms"] * sy["n_frames"])
            near(sy["a_ms"] - sy["truth_a_ms"], 0, 1.0, f"{rd.name[:14]} frame 0 time error (ms)")
            near(end_err, 0, 1.0, f"{rd.name[:14]} last frame time error (ms)")
            near(a["i_hold"], 0.65, 0.03, f"{rd.name[:14]} hold current from valve file (A)")
            near(a["video"]["jet"].get("front_speed_m_s"), 20.1, 2.0, f"{rd.name[:14]} jet front speed (m/s)")
            near(a.get("temp_cj_start"), 22.5, 0.1, f"{rd.name[:14]} cold junction (C)")
            check(a.get("temp_tc_min") is not None and a["temp_tc_min"] < a["temp_tc_start"], f"{rd.name}: gas cools while flowing")
            check(a["missing_ms"] == 0, f"{rd.name}: no missing samples")
    finally:
        sim.terminate()
    print(f"\n{passes} checks passed, {fails} failed")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
