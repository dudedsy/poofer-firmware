"""rigctl: run cold-test experiments from the laptop.

\b
    rigctl ports                         list serial ports
    rigctl info | monitor | zero | clear
    rigctl led on 2.0 | led off          shadowgraph LED
    rigctl sync blink|on|off             signal LED (aiming the camera)
    rigctl camera test|dialog|snapshot
    rigctl shot --open-ms 300 [--pressure 60] [--no-camera] [--label nozzle=N1]
    rigctl characterize VALVE --pressure 60
    rigctl run EXPERIMENT.yaml [--name NAME] [--resume]
    rigctl report RUN_OR_EXPERIMENT_DIR
    rigctl sim                           simulator (Linux / WSL) on socket://127.0.0.1:5555
"""
from __future__ import annotations

import datetime as dt
import json
import os
import subprocess
import sys
import time
from pathlib import Path

import click

from .config import Rig, configure_device
from .link import Device, DeviceError


class Ctx:
    def __init__(self, rig_path, port):
        self.rig_path = rig_path
        self.port = port
        self._rig = None
        self._dev = None

    @property
    def rig(self) -> Rig:
        if self._rig is None:
            if not Path(self.rig_path).exists():
                raise click.ClickException(f"{self.rig_path} not found (copy examples/rig.yaml and edit it)")
            self._rig = Rig(self.rig_path)
            if self.port:
                self._rig.cfg["port"] = self.port
        return self._rig

    def device(self, configure=True) -> Device:
        if self._dev is None:
            try:
                self._dev = Device(self.rig["port"])
                self._dev.info()
            except Exception as e:
                raise click.ClickException(f"cannot talk to rig_control on {self.rig['port']}: {e}")
            if configure:
                try:
                    configure_device(self._dev, self.rig)
                except DeviceError as e:
                    raise click.ClickException(f"configuring the board: {e} (drives must be off; `rigctl clear`?)")
        return self._dev


@click.group(help=__doc__)
@click.option("--rig", "rig_path", default="rig.yaml", show_default=True, help="rig description")
@click.option("--port", default=None, help="override rig.yaml port (COM5, socket://127.0.0.1:5555)")
@click.pass_context
def main(ctx, rig_path, port):
    ctx.obj = Ctx(rig_path, port)


@main.command()
def ports():
    """List serial ports (the RP2040-Zero shows as a USB serial device)."""
    from serial.tools import list_ports
    for p in list_ports.comports():
        click.echo(f"{p.device:10s} {p.description}  {p.hwid}")


@main.command()
@click.pass_obj
def info(o: Ctx):
    """Firmware version and every setting."""
    click.echo(json.dumps(o.device(configure=False).info(), indent=2))


@main.command()
@click.option("--seconds", type=float, default=0, help="stop after N s (0 = until Ctrl-C)")
@click.pass_obj
def monitor(o: Ctx, seconds):
    """Live pressures, drive states and temperatures."""
    from . import thermo
    dev, rig = o.device(), o.rig
    t0 = time.time()
    kind = (rig["thermocouple"] or {}).get("type", "K")
    try:
        while not seconds or time.time() - t0 < seconds:
            st = dev.wait_status()
            ps = "  ".join(f"{p['name']} {float(rig.psi(p['name'], st['adc'][p['adc']])):6.1f}" for p in rig["pressure"])
            tc = thermo.from_ads1115(st.get("tc"), st.get("cj"), kind) if rig["thermocouple"] else None
            temps = f"chip {st['chip']:.1f}" + (f"  cj {st['cj']:.1f}" if st.get("cj") is not None else "") + (
                f"  tc {tc:.1f}" if tc is not None else ("  tc open/absent" if rig["thermocouple"] else ""))
            v, l = st["v"], st["l"]
            sys.stdout.write(f"\r{ps} psi | valve {v['st']} {v['i']:.2f} A | LED {l['st']} {l['i']:.2f} A | {temps} °C   ")
            sys.stdout.flush()
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    click.echo()


@main.command()
@click.option("--seconds", type=float, default=2.0, show_default=True)
@click.option("--yes", is_flag=True, help="don't ask")
@click.pass_obj
def zero(o: Ctx, seconds, yes):
    """Record vented pressure-sensor zeros into rig_cal.json."""
    dev, rig = o.device(), o.rig
    if not yes and not click.confirm("All pressure sensors vented to atmosphere?"):
        return
    acc = {p["name"]: [] for p in rig["pressure"]}
    t0 = time.time()
    while time.time() - t0 < seconds:
        st = dev.wait_status()
        for p in rig["pressure"]:
            acc[p["name"]].append(st["adc"][p["adc"]])
    zeros = {}
    for p in rig["pressure"]:
        z = sum(acc[p["name"]]) / max(1, len(acc[p["name"]]))
        nominal = rig.zero_counts({**p, "name": "__nominal__"})
        off_psi = (z - nominal) / rig.counts_per_psi(p)
        flag = "  <-- more than 4 psi from nominal: check wiring / supply" if abs(off_psi) > 4 else ""
        click.echo(f"{p['name']:12s} zero {z:7.1f} counts (nominal {nominal:.1f}, {off_psi:+.2f} psi){flag}")
        zeros[p["name"]] = round(z, 2)
    rig.save_zero(zeros)
    click.echo(f"saved {rig.cal_path}")


@main.command()
@click.pass_obj
def clear(o: Ctx):
    """Clear latched drive faults."""
    o.device(configure=False).command("clear")
    click.echo("cleared")


@main.command()
@click.argument("action", type=click.Choice(["on", "off"]))
@click.argument("amps", type=float, required=False)
@click.pass_obj
def led(o: Ctx, action, amps):
    """Shadowgraph LED on [A] / off (firmware on-time limit applies)."""
    dev = o.device()
    dev.command(f"led on {amps}" if action == "on" and amps else f"led {action}")
    time.sleep(0.5)
    st = dev.wait_status()["l"]
    click.echo(f"LED {st['st']} {st['i']:.2f} A (target {st['tgt']:.2f})")


@main.command()
@click.argument("mode", type=click.Choice(["blink", "on", "off"]))
@click.pass_obj
def sync(o: Ctx, mode):
    """Signal LED: blink (1 Hz, for aiming the camera), on, off."""
    o.device(configure=False).command(f"sync {mode}")


@main.group()
def camera():
    """Camera helpers (ffmpeg DirectShow)."""


@camera.command("test")
@click.option("--seconds", type=float, default=3.0)
@click.pass_obj
def camera_test(o: Ctx, seconds):
    """Record a few seconds with the rig.yaml settings and report the frame count."""
    from .camera import FfmpegCamera
    cfg = o.rig["camera"]
    cam = FfmpegCamera(cfg)
    out = Path("camera_test." + cfg["container"])
    cam.start(out)
    time.sleep(seconds)
    r = cam.stop()
    click.echo(f"{r['frames_reported']} frames in about {seconds:.1f} s after the first frame -> {out}")


@camera.command("dialog")
@click.pass_obj
def camera_dialog(o: Ctx):
    """Open the driver's property dialog (exposure, gain, white balance)."""
    c = o.rig["camera"]
    subprocess.run([c["ffmpeg"], "-hide_banner", "-f", "dshow", "-show_video_device_dialog", "true",
                    "-i", f"video={c['device']}", "-frames:v", "1", "-f", "null", "-"])


@camera.command("snapshot")
@click.option("--out", default="snapshot.png", show_default=True)
@click.pass_obj
def camera_snapshot(o: Ctx, out):
    """One full-resolution frame, for picking sync_roi / jet_roi / nozzle_px."""
    c = o.rig["camera"]
    cmd = [c["ffmpeg"], "-hide_banner", "-loglevel", "error", "-f", "dshow", "-video_size", c["size"], "-framerate", str(c["fps"])]
    cmd += ["-vcodec", "mjpeg"] if c["pixel_format"] == "mjpeg" else ["-pixel_format", c["pixel_format"]]
    cmd += ["-i", f"video={c['device']}", "-vf", "select=gte(n\\,30)", "-frames:v", "1", "-y", out]
    subprocess.run(cmd, check=False)
    click.echo(out)


def _labels(pairs):
    out = {}
    for p in pairs:
        if "=" not in p:
            raise click.BadParameter(f"--label wants key=value, got {p!r}")
        k, v = p.split("=", 1)
        out[k] = v
    return out


@main.command()
@click.option("--open-ms", type=int, default=300, show_default=True)
@click.option("--pressure", type=float, default=None, help="wait for this accumulator psi first")
@click.option("--tol", type=float, default=2.0, show_default=True)
@click.option("--hold-a", type=float, default=None)
@click.option("--peak-a", type=float, default=None)
@click.option("--peak-ms", type=int, default=None)
@click.option("--no-camera", is_flag=True)
@click.option("--no-valve", is_flag=True, help="LED, camera and timecode only (sync test)")
@click.option("--valve", "valve_name", default=None, help="valve file from `characterize`")
@click.option("--name", default=None, help="run folder name (default shot_<time>)")
@click.option("--label", multiple=True, help="key=value recorded with the run, e.g. nozzle=N1")
@click.option("--yes", is_flag=True, help="fire without asking")
@click.pass_obj
def shot(o: Ctx, open_ms, pressure, tol, hold_a, peak_a, peak_ms, no_camera, no_valve, valve_name, name, label, yes):
    """One shot, saved under data_dir/shots/ with its report."""
    from .characterize import load_valve
    from .experiment import ask, wait_pressure
    from .report import run_report
    from .shot import ShotError, run_shot
    dev, rig = o.device(), o.rig
    vf = load_valve(rig, valve_name)
    psi = wait_pressure(dev, rig, pressure, tol, False)
    if not yes and ask("Enter = FIRE, q = quit: ", ",q") == "q":
        return
    run_dir = rig.data_dir / "shots" / (name or f"shot_{dt.datetime.now():%Y%m%d-%H%M%S}")
    params = {"open_ms": open_ms, "pressure_psi": pressure, "pressure_tol": tol, "hold_a": hold_a, "peak_a": peak_a,
              "peak_ms": peak_ms, "camera": not no_camera, "valve": not no_valve}
    meta = _labels(label)
    meta["pressure_at_fire"] = psi
    try:
        run_shot(dev, rig, run_dir, params, meta, vf)
    except ShotError as e:
        click.echo(f"shot failed: {e}")
    if (run_dir / "run.json").exists():
        from .experiment import quick_line
        res = run_report(run_dir, close90_ms=(vf or {}).get("t_close_90_ms"))
        click.echo(quick_line(res))
        click.echo(f"report: {run_dir / 'report.html'}")


@main.command()
@click.argument("valve_name")
@click.option("--pressure", type=float, required=True, help="accumulator psi for the tests")
@click.option("--tests", default="coil,response,dropout", show_default=True)
@click.option("--repeats", type=int, default=3, show_default=True)
@click.option("--start-hold", type=float, default=None, help="dropout ramp start (A), default rig.yaml valve.hold_a")
@click.option("--ramp", type=float, default=-0.5, show_default=True, help="dropout ramp, A/s")
@click.option("--auto", is_flag=True, help="no prompts (simulator / unattended); fires without asking")
@click.pass_obj
def characterize(o: Ctx, valve_name, pressure, tests, repeats, start_hold, ramp, auto):
    """Coil R/L, armature motion, valve opening/closing delays, dropout -> valves/<name>.json."""
    from .characterize import characterize as run_char
    run_char(o.rig, o.device(), valve_name, pressure, [t.strip() for t in tests.split(",")], repeats, auto, ramp, start_hold)


@main.command()
@click.argument("experiment", type=click.Path(exists=True))
@click.option("--name", default=None, help="name of this experiment run (folder under data_dir)")
@click.option("--resume", is_flag=True, help="continue an interrupted run: skip runs already ok")
@click.option("--auto", is_flag=True, help="no prompts (simulator / unattended); fires without asking")
@click.pass_obj
def run(o: Ctx, experiment, name, resume, auto):
    """Run an experiment file: conditions x sweep x repeats, with reports."""
    from .characterize import load_valve
    from .experiment import load, run_experiment
    e = load(experiment)
    vf = load_valve(o.rig, e.get("valve"))
    d = run_experiment(o.rig, o.device(), experiment, name, resume, auto, vf)
    click.echo(f"done: {d / 'summary.html'}")


@main.command()
@click.argument("path", type=click.Path(exists=True, file_okay=False))
@click.option("--no-video", is_flag=True)
def report(path, no_video):
    """(Re)analyse a run folder, or every run in an experiment folder, and write the reports."""
    from .report import run_report, summary_report
    p = Path(path)
    if (p / "run.json").exists():
        run_report(p, video=not no_video)
        click.echo(p / "report.html")
        return
    results, conds, sweep = [], set(), []
    exp = p / "experiment.yaml"
    if exp.exists():
        from .experiment import load
        e = load(exp)
        conds = sorted({k for c in e["conditions"] for k in c})
        sweep = list(e["sweep"])
    vf = None
    for d in sorted(x for x in p.iterdir() if (x / "run.json").exists()):
        click.echo(d.name)
        meta = json.loads((d / "run.json").read_text())
        vf = vf or meta.get("valve_file")
        results.append(run_report(d, close90_ms=(vf or {}).get("t_close_90_ms"), video=not no_video))
    if results:
        click.echo(summary_report(p, results, sweep, conds))


@main.command()
@click.option("--port", "tcp", type=int, default=5555, show_default=True)
@click.option("--truth", default="sim_truth.csv", show_default=True)
def sim(tcp, truth):
    """Start the simulator (needs `make -C cold-test-rig/tools/sim`, Linux or WSL). Use
    port socket://127.0.0.1:<port> and camera backend fake with fake.truth = the truth file."""
    exe = Path(__file__).resolve().parents[2] / "tools" / "sim" / "rig_sim"
    if not exe.exists():
        raise click.ClickException(f"{exe} not built: make -C {exe.parent}")
    click.echo(f"socket://127.0.0.1:{tcp}  (truth log {os.path.abspath(truth)})  Ctrl-C to stop")
    try:
        subprocess.run([str(exe), "--port", str(tcp), "--truth", truth])
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
