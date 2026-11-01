"""End-to-end tests of the firmware in the host simulator (tools/sim).

Build the simulator first:   make -C tools/sim
Run:                         python tests/test_sim.py      (or pytest tests/test_sim.py)

Each test drives the real firmware sources: buttons, a modelled pressure rig, the
emulated UI board / FRAM, flash storage, and the real burst_logger dump code.
"""
import json
import os
import re
import subprocess
import sys
import tempfile
import time
import types

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SIM = os.path.join(ROOT, "tools", "sim", "burst_sim")
sys.path.insert(0, os.path.join(ROOT, "host"))
sys.path.insert(0, os.path.join(ROOT, "tools", "sim"))
sys.path.insert(0, os.path.join(ROOT, "analysis"))
from burst_logger import StreamParser, Device, run_dump   # noqa: E402
from simport import SimPort                               # noqa: E402

# Menu positions (MI_* in the sketch)
MI_COLLECT, MI_LIVE, MI_DUMP, MI_CH, MI_TAG, MI_ARM, MI_TRIG, MI_ZERO, MI_CAL, MI_CAPS, MI_DELALL = range(11)
N_ITEMS = 11


def run(lines, *args, state=None):
    """Runs a batch script; returns (parsed serial items, stderr text)."""
    argv = [SIM, *args] + (["--state", state] if state else [])
    script = "\n".join(lines + ["!quit"]) + "\n"
    p = subprocess.run(argv, input=script.encode(), capture_output=True, timeout=600)
    err = p.stderr.decode()
    assert "SIMERR" not in err, err
    items = list(StreamParser().feed(p.stdout))
    return items, err


def infos(items):
    return [i[1] for i in items if i[0] == "info"]


def leds(err):
    return [l.split()[2:] for l in err.splitlines() if l.startswith("SIM leds")]


def screens_lit(err):
    return [l.split("lit=")[1][0] == "1" for l in err.splitlines() if l.startswith("SIM screen")]


def menu_to(item, cur=0):
    """Button presses to move the main-menu cursor from `cur` to `item` (DN only)."""
    return ["!btn DN 50"] * ((item - cur) % N_ITEMS)


ENTER_COLLECT = ["!wait 400", "!btn SET 80"]
TO_MENU = ["!btn SET 1000"]


def burst_cycle(psi=30, keep=True):
    return [f"!p {psi} 400", "!wait 700", "!burst", "!wait 300", "!btn SET 80" if keep else "!btn DN 80",
            "!p 0 1", "!wait 300"]


def dump(state, tmp, erase=False, tags="1=mylar_base"):
    port = SimPort(["--state", state], stderr_path=os.path.join(tmp, "dump_err.txt"))
    try:
        time.sleep(0.3)
        args = types.SimpleNamespace(outdir=os.path.join(tmp, "runs"), tags=tags, label="run", notes="sim",
                                     port="sim", erase=erase)
        rc = run_dump(port, args)
        dev = Device(port, verbose=False)
        _, end = dev.list()
    finally:
        port.close()
    return rc, end


# ---------------------------------------------------------------- tests
def test_collect_auto_arm_keep_and_dump():
    with tempfile.TemporaryDirectory() as tmp:
        st = os.path.join(tmp, "state.bin")
        items, err = run(ENTER_COLLECT + ["!wait 200", "!leds"] + ["!p 30 1500", "!wait 1800", "!leds", "!burst",
                         "!wait 300", "!leds", "!btn SET 80", "!wait 200", "!leds", "L"], state=st)
        l = leds(err)
        assert l[0][0] == "green" and l[1][:2] == ["red", "amber"] and l[2][0] == "blue" and l[3][0] == "green", l
        assert any(i.startswith("#saved id=1") for i in infos(items))
        errs = [i[1] for i in items if i[0] == "json" and i[1].get("t") == "error"]
        assert errs and "Collect" in errs[0]["msg"]           # dump refused while collecting

        rc, end = dump(st, tmp, erase=True)
        assert rc == 0 and end["count"] == 0 and end["free"] == end["slots"]
        folder = [d for d in os.listdir(os.path.join(tmp, "runs"))]
        assert len(folder) == 1 and folder[0].endswith("_mylar_base"), folder
        base = os.path.join(tmp, "runs", folder[0], "capture_0001")
        counts = np.load(base + ".npz")["counts"]
        meta = json.load(open(base + ".json"))
        assert counts.shape == (15000, 2) and meta["crc_ok"] and meta["framing_ok"] and not meta["problems"]
        assert abs(meta["p0_psi"][0] - 30) < 0.3, meta["p0_psi"]
        assert abs(meta["chip_temp_c"] - 25) < 0.5, meta["chip_temp_c"]

        # the analysis loads it with the 20/80 window settings
        import blowdown_fit
        cfg = json.load(open(os.path.join(ROOT, "analysis", "rig_config.example.json")))
        cfg.update(_dir=tmp, sensors=[{"name": "A", "ch": 0}, {"name": "B", "ch": 1}],
                   groups=[{"captures": "runs/*_mylar_base/capture_*.npz", "volume_cc": 20, "bore_mm": 15.8}])
        caps = blowdown_fit.load_captures(cfg)
        # nominal gain in the config; sim sensor B reads 3% low, so the mean is ~29.6
        assert len(caps) == 1 and abs(caps[0]["p0g_psi"] - 30) < 1.0, caps[0]["p0g_psi"]
        y = caps[0]["y"]
        t = caps[0]["t"]
        assert abs(y[:, t < -0.005].mean() - 1) < 0.01 and abs(y[:, t > 0.05].mean()) < 0.01

        # dumping again is a no-op (erased), and re-dump without erase skips copies
        rc, end = dump(st, tmp)
        assert rc == 0 and end["count"] == 0


def test_manual_arm_force_and_drop():
    items, err = run(ENTER_COLLECT + ["!wait 300", "!btn SET 80", "!leds", "!btn UP 80", "!wait 300", "!leds",
                                      "!btn DN 80", "!wait 300", "!leds", "MM", "L"])
    l = leds(err)
    assert l[0][0] == "red" and l[1][0] == "blue" and l[2][0] == "green", l
    assert any("capture dropped" in i for i in infos(items))
    end = [i[1] for i in items if i[0] == "json" and i[1].get("t") == "list_end"][0]
    assert end["count"] == 0


def test_slow_vent_disarms_without_trigger():
    items, err = run(ENTER_COLLECT + ["!p 20 1000", "!wait 1500", "!leds", "!p 0 4000", "!wait 4500", "!leds"])
    l = leds(err)
    assert l[0][0] == "red" and l[1][0] == "green", l
    assert any("vented without a trigger" in i for i in infos(items))
    assert not any(i.startswith("#captured") for i in infos(items))


def test_menu_delete_one_and_all():
    with tempfile.TemporaryDirectory() as tmp:
        st = os.path.join(tmp, "state.bin")
        run(ENTER_COLLECT + burst_cycle() * 3, state=st)
        # menu -> Captures -> first entry (newest, #3) -> delete
        items, err = run(["!wait 400"] + menu_to(MI_CAPS) + ["!btn SET 80", "!btn DN 80", "!btn SET 80",
                         "!screen " + os.path.join(tmp, "del.pbm"), "!btn SET 80", "!wait 1500", "L"], state=st)
        lst = [i[1] for i in items if i[0] == "json"]
        assert [e["id"] for e in lst if e["t"] == "entry"] == [1, 2], lst
        # back to main, Delete all needs a long press; a short press must not delete
        items, err = run(["!wait 400"] + menu_to(MI_DELALL) + ["!btn SET 80", "!btn SET 80", "L",
                                                                  "!btn SET 1000", "!wait 1700", "L"], state=st)
        ends = [i[1]["count"] for i in items if i[0] == "json" and i[1].get("t") == "list_end"]
        assert ends == [2, 0], ends


def test_zero_and_tag_persist_across_reboot():
    with tempfile.TemporaryDirectory() as tmp:
        st = os.path.join(tmp, "state.bin")
        # zero (sim sensors sit at 410 / 406 counts), set tag 3, one capture
        # Zero -> SET -> SET (measure), wait out the result message; cursor is on Zero (7):
        # UP x3 -> Tag, SET edit, UP x2 (1 -> 3), SET save, UP x4 -> Collect, SET
        run(["!wait 400"] + menu_to(MI_ZERO) + ["!btn SET 80", "!btn SET 80", "!wait 2600"]
            + ["!btn UP 50"] * 3 + ["!btn SET 80", "!btn UP 50", "!btn UP 50", "!btn SET 80"]
            + ["!btn UP 50"] * 4 + ["!btn SET 80"] + burst_cycle(), state=st)
        # reboot: second capture must be id 2 with the same tag and zeros
        items, err = run(ENTER_COLLECT + burst_cycle(45) + TO_MENU + ["!wait 300", "L"], state=st)
        entries = [i[1] for i in items if i[0] == "json" and i[1].get("t") == "entry"]
        assert [e["id"] for e in entries] == [1, 2], entries
        for e in entries:
            assert e["tag"] == 3 and e["zero"] == [410, 406], e
        assert abs(entries[1]["p0_psi"][0] - 45) < 0.2            # B has a 3% gain error until calibrated
        assert 42.9 < entries[1]["p0_psi"][1] < 43.9, entries[1]["p0_psi"]
        assert entries[1]["session"] == entries[0]["session"] + 1


def test_calibrate():
    with tempfile.TemporaryDirectory() as tmp:
        st = os.path.join(tmp, "state.bin")
        def point(psi, prev):
            return [f"!p {psi} 300", "!wait 800"] + ["!btn UP 40"] * (psi - prev) + ["!btn SET 80", "!wait 900"]
        # sim sensors: A gain 1.00, B 0.97 (counts per psi 32.76 / 31.78), zeros 410 / 406
        lines = ["!wait 400"] + menu_to(MI_CAL) + ["!btn SET 80"]
        lines += point(0, 0) + point(20, 0) + point(40, 20) + point(60, 40)
        lines += ["!btn SET 1000", "!wait 200", "!screen " + os.path.join(tmp, "cal.pbm"), "!btn SET 80", "!wait 2500",
                  "!p 0 300", "!wait 600"]
        lines += ["!btn UP 50"] * (MI_CAL - MI_COLLECT) + ["!btn SET 80"] + burst_cycle(45) + TO_MENU + ["!wait 300", "L"]
        items, err = run(lines, state=st)
        e = [i[1] for i in items if i[0] == "json" and i[1].get("t") == "entry"]
        assert len(e) == 1, items[-5:]
        e = e[0]
        assert abs(e["cpp"][0] - 32.76) < 0.1 and abs(e["cpp"][1] - 31.78) < 0.1, e["cpp"]
        assert abs(e["zero"][0] - 410) <= 1 and abs(e["zero"][1] - 406) <= 1, e["zero"]
        assert all(abs(p - 45) < 0.2 for p in e["p0_psi"]), e["p0_psi"]


def test_storage_full():
    with tempfile.TemporaryDirectory() as tmp:
        st = os.path.join(tmp, "state.bin")
        # the 38th SET (keep) does nothing when full; DN drops it
        items, err = run(ENTER_COLLECT + burst_cycle(20) * 38 + ["!wait 200", "!screen " + os.path.join(tmp, "f.pbm"),
                         "!btn DN 80"] + TO_MENU + ["!wait 300", "L"], state=st)
        saved = [i for i in infos(items) if i.startswith("#saved")]
        end = [i[1] for i in items if i[0] == "json" and i[1].get("t") == "list_end"][0]
        assert len(saved) == end["slots"] == end["count"] and end["free"] == 0, (len(saved), end)
        caps = [i for i in infos(items) if i.startswith("#captured")]
        assert len(caps) == 38


def test_live_mode_bare_board():
    items, err = run(["!wait 400", "?", "!p 30 500", "!wait 1000", "!burst", "!wait 500"], "--bare")
    hdr = [i[1] for i in items if i[0] == "header"][-1]
    assert hdr["mode"] == "live" and hdr["ui"] is False
    caps = [i for i in items if i[0] == "capture"]
    assert len(caps) == 1 and caps[0][1]["framing_ok"] and caps[0][2].shape == (15000, 2)
    lm = [i[1] for i in items if i[0] == "json" and i[1].get("t") == "live_meta"]
    assert len(lm) == 1 and abs(lm[0]["chip_temp_c"] - 25) < 0.5, lm
    c = caps[0][2].astype(float)
    assert c[:2000, 0].mean() > 1300 and c[-2000:, 0].mean() < 430
    assert sum(1 for i in items if i[0] == "slow") > 500


def test_live_switch_from_menu_and_back():
    items, err = run(["!wait 400", "ML", "N1", "!p 30 500", "!wait 1000", "!burst", "!wait 500", "MM", "!wait 200"])
    hdrs = [i[1] for i in items if i[0] == "header"]
    assert hdrs[0]["mode"] == "live" and hdrs[-1]["mode"] == "menu" and hdrs[1]["n_ch"] == 1
    caps = [i for i in items if i[0] == "capture"]
    assert len(caps) == 1 and caps[0][2].shape == (30000, 1)


def test_blank_and_wake():
    with tempfile.TemporaryDirectory() as tmp:
        s = lambda n: "!screen " + os.path.join(tmp, f"{n}.pbm")
        items, err = run(["!wait 400", s(1), "!wait 301000", s(2), "!btn DN 80", "!wait 300", s(3),
                          "!btn SET 80", "!wait 400", "!wait 301000", s(4), "!p 10 200", "!wait 1500", s(5)])
        # 1 lit at boot, 2 dark after 5 min idle in the menu, 3 lit after DN (the waking
        # press is swallowed, so SET still enters Collect), 4 dark after 5 min idle in
        # Collect at 0 psi, 5 lit again once the rig is pressurized
        assert screens_lit(err) == [True, False, True, False, True], screens_lit(err)
        assert not any(i.startswith("#captured") for i in infos(items))


def test_trigger_ignores_glitches():
    # 1-2 ms blips of several psi, up or down, on either channel, plus heavy noise:
    # all must leave Collect armed (LED1 red) with no capture
    lines = ENTER_COLLECT + ["!p 30 400", "!wait 1100"]
    for g in ["!glitch 0 100 1", "!glitch 0 -100 1", "!glitch 1 100 2", "!glitch 1 -100 2", "!glitch 0 300 1",
              "!glitch 0 -100 5", "!glitch 1 -200 8", "!noise 40"]:
        lines += [g, "!wait 300"]
    lines += ["!leds"]
    items, err = run(lines)
    assert leds(err)[-1][0] == "red", leds(err)
    assert not any(i.startswith("#captured") for i in infos(items))


def test_trigger_timing_and_diagnostics():
    # low-pressure burst, then a normal one; the trigger point must sit at the start
    # of the drop and the capture must name the channel that tripped
    with tempfile.TemporaryDirectory() as tmp:
        st = os.path.join(tmp, "state.bin")
        run(ENTER_COLLECT + burst_cycle(6) + burst_cycle(40), state=st)
        port = SimPort(["--state", st])
        try:
            time.sleep(0.3)
            dev = Device(port, verbose=False)
            entries, _ = dev.list()
            assert [e["id"] for e in entries] == [1, 2], entries
            for e in entries:
                meta, hdr, counts = dev.read(e["id"])
                assert meta["trig_ch"] == 0 and 0.9 < meta["trig_drop_psi"] < 25.6, meta
                c = counts[:, 0].astype(float)
                fs = meta["fs_ch"] / 1000.0                     # frames per ms
                p0 = c[: meta["trig_frame"] - int(5 * fs)].mean()
                p0_psi = (p0 - meta["zero"][0]) / meta["cpp"][0]
                thresh = p0 - 0.5 * meta["cpp"][0]              # 0.5 psi below p0
                first = int(np.argmax(c < thresh))
                dt_ms = (first - meta["trig_frame"]) / fs
                assert -1.5 < dt_ms < 1.0, (e["id"], p0_psi, dt_ms)
        finally:
            port.close()


def test_display_recovery():
    with tempfile.TemporaryDirectory() as tmp:
        s = lambda n: "!screen " + os.path.join(tmp, f"{n}.pbm")
        # expander reset by a supply dip: panel goes dark, firmware notices and restarts it
        # holding SET restarts the panel from any state
        items, err = run(["!wait 400", s(1), "!tca_por", "!wait 10", s(2), "!wait 1500", s(3),
                          "!btn SET 1000", "!wait 400", s(4)])
        assert screens_lit(err) == [True, False, True, True], screens_lit(err)


if __name__ == "__main__":
    if not os.path.exists(SIM):
        sys.exit("build the simulator first: make -C tools/sim")
    fails = 0
    for name, fn in list(globals().items()):
        if name.startswith("test_") and callable(fn):
            t0 = time.time()
            try:
                fn()
                print(f"PASS {name} ({time.time() - t0:.1f}s)")
            except AssertionError as e:
                fails += 1
                print(f"FAIL {name}: {str(e)[:2000]}")
    sys.exit(1 if fails else 0)
