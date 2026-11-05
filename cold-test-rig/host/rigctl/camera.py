"""Cameras: ffmpeg DirectShow recording on the laptop, a fake for the simulator, or none.

The camera free-runs; frames are timed afterwards against the signal-LED timecode
(analysis.sync_video), so nothing here needs to be precise.
"""
from __future__ import annotations

import csv
import json
import subprocess
import threading
import time
from pathlib import Path

import numpy as np


class CameraError(RuntimeError):
    pass


class NoCamera:
    name = "none"

    def start(self, path: Path, dev=None):
        return None

    def stop(self) -> dict:
        return {"backend": "none"}


class FfmpegCamera:
    """ffmpeg -f dshow ... -c:v copy. Settings such as exposure live in the camera driver
    (set them with `rigctl camera dialog`); ffmpeg cannot set exposure itself."""
    name = "ffmpeg"

    def __init__(self, cfg: dict):
        self.cfg = cfg
        self.proc = None
        self.frames = 0
        self.t_cmd = self.t_first = None
        self.log: list[str] = []

    def command(self, out: Path) -> list[str]:
        c = self.cfg
        cmd = [c["ffmpeg"], "-hide_banner", "-loglevel", "info", "-f", "dshow", "-rtbufsize", str(c["rtbufsize"]),
               "-video_size", c["size"], "-framerate", str(c["fps"])]
        if c["pixel_format"] == "mjpeg":
            cmd += ["-vcodec", "mjpeg"]
        else:
            cmd += ["-pixel_format", c["pixel_format"]]
        cmd += ["-i", f"video={c['device']}", "-c:v", "copy", "-progress", "pipe:1", "-nostats", "-y", str(out)]
        return cmd

    def _pump_stdout(self):
        for raw in self.proc.stdout:
            line = raw.decode(errors="replace").strip()
            if line.startswith("frame="):
                try:
                    n = int(line.split("=", 1)[1])
                except ValueError:
                    continue
                if n > 0 and self.t_first is None:
                    self.t_first = time.time()
                self.frames = n

    def _pump_stderr(self):
        for raw in self.proc.stderr:
            self.log.append(raw.decode(errors="replace").rstrip())

    def start(self, path: Path, dev=None):
        self.path = Path(path)
        self.t_cmd = time.time()
        self.cmd = self.command(self.path)
        try:
            self.proc = subprocess.Popen(self.cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        except FileNotFoundError as e:
            raise CameraError(f"ffmpeg not found ({self.cfg['ffmpeg']}): {e}")
        threading.Thread(target=self._pump_stdout, daemon=True).start()
        threading.Thread(target=self._pump_stderr, daemon=True).start()
        deadline = time.time() + float(self.cfg["start_timeout_s"])
        while time.time() < deadline:
            if self.proc.poll() is not None:
                raise CameraError("ffmpeg exited before the first frame:\n" + "\n".join(self.log[-15:]))
            if self.frames > 2:
                return
            time.sleep(0.05)
        self.stop()
        raise CameraError(f"no frames from '{self.cfg['device']}' within {self.cfg['start_timeout_s']} s")

    def stop(self) -> dict:
        t_stop = time.time()
        if self.proc and self.proc.poll() is None:
            try:
                self.proc.stdin.write(b"q")
                self.proc.stdin.flush()
            except OSError:
                pass
            try:
                self.proc.wait(timeout=15)
            except subprocess.TimeoutExpired:
                self.proc.kill()
        return {"backend": "ffmpeg", "file": self.path.name, "cmd": self.cmd, "t_cmd": self.t_cmd, "t_first": self.t_first,
                "t_stop": t_stop, "frames_reported": self.frames, "returncode": self.proc.returncode if self.proc else None,
                "log_tail": self.log[-40:], "fps_requested": self.cfg["fps"], "fps_real": self.cfg.get("fps_real")}


class FakeCamera:
    """For the simulator: after the shot, renders a video from the simulator's truth log
    (signal LED, LED current, nozzle pressure) at a deliberately wrong offset and frame period,
    so the timecode sync is tested for real. Writes video_truth.json with the true mapping."""
    name = "fake"

    def __init__(self, cfg: dict):
        self.cfg = cfg
        self.fk = cfg["fake"]

    def _sim_now(self, dev) -> float:
        st = dev.wait_status(timeout=2.0)
        return st["t"] + (time.time() - st["host_time"]) * 1000.0

    def start(self, path: Path, dev=None):
        if not self.fk.get("truth"):
            raise CameraError("camera.backend fake needs camera.fake.truth (rig_sim --truth file)")
        self.path = Path(path)
        self.dev = dev
        self.t_start = self._sim_now(dev) + float(self.fk["offset_ms"]) % 300.0

    def stop(self) -> dict:
        t_end = self._sim_now(self.dev)
        truth = Path(self.fk["truth"])
        rows = []
        deadline = time.time() + 3.0
        while time.time() < deadline:
            rows = _read_truth(truth)
            if rows is not None and len(rows["t_ms"]) and rows["t_ms"][-1] >= t_end:
                break
            time.sleep(0.1)
        fps = float(self.cfg.get("fps_real") or self.cfg["fps"])
        period = 1000.0 / fps * (1.0 + float(self.fk["period_error"]))
        n = int((t_end - self.t_start) / period)
        w, h = (int(x) for x in self.fk["size"].split("x"))
        t_frames = self.t_start + period * np.arange(n)
        frames = render_fake(rows, t_frames, w, h)
        cmd = ["ffmpeg", "-hide_banner", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "gray", "-s", f"{w}x{h}",
               "-r", f"{fps:g}", "-i", "-", "-c:v", "ffv1", "-y", str(self.path)]
        p = subprocess.run(cmd, input=frames.tobytes(), capture_output=True)
        if p.returncode:
            raise CameraError(p.stderr.decode())
        (self.path.parent / "video_truth.json").write_text(json.dumps({"t_frame0_ms": t_frames[0], "period_ms": period}))
        return {"backend": "fake", "file": self.path.name, "frames": n, "fps_requested": self.cfg["fps"],
                "fps_real": self.cfg.get("fps_real")}


def _read_truth(path: Path):
    try:
        with open(path, newline="") as f:
            r = list(csv.DictReader(f))
    except OSError:
        return None
    if not r:
        return None
    r = r[:-1] if len(r) > 1 else r       # last line may be partial
    cols = {k: np.array([float(x[k]) for x in r]) for k in r[0]}
    return cols


def render_fake(rows, t_frames, w, h):
    """Shadowgraph-like frames: uniform screen lit by the LED, a dark card with the signal LED
    in the top-right corner, and a speckled plume growing from the left edge while the valve
    is open (front speed ~ sqrt(nozzle pressure)), detaching after it closes."""
    rng = np.random.default_rng(7)
    keep = (rows["t_ms"] >= t_frames[0] - 50) & (rows["t_ms"] <= t_frames[-1] + 50)   # this recording only
    rows = {k: v[keep] for k, v in rows.items()}
    t = rows["t_ms"]
    led = np.interp(t_frames, t, rows["led_a"])
    sync = rows["sync"][np.clip(np.searchsorted(t, t_frames, side="right") - 1, 0, len(t) - 1)]   # state set at block end
    pn = np.interp(t_frames, t, rows["p_noz"])
    # front and tail positions in px, integrated on the 1 ms truth grid
    u = 3.0 * np.sqrt(np.clip(rows["p_noz"], 0, None) / 30.0)      # px per ms
    front = np.cumsum(u)
    pmax = max(rows["p_noz"].max(), 1e-6)
    closed = rows["p_noz"] < 0.1 * pmax
    started = np.cumsum(u > 0) > 0
    tail = np.cumsum(np.where(closed & started, 3.0, 0.0))
    tail = np.minimum(tail, front)
    f_front = np.interp(t_frames, t, front)
    f_tail = np.interp(t_frames, t, tail)
    yy, xx = np.mgrid[0:h, 0:w].astype(np.float32)
    tex = 1.0 + 0.03 * rng.standard_normal((h, w)).astype(np.float32)
    y0, x0 = h / 2.0, 10.0
    out = np.empty((len(t_frames), h, w), np.uint8)
    for k in range(len(t_frames)):
        base = 190.0 * (max(led[k], 0.0) / 2.0) ** 0.8
        img = base * tex
        d = xx - x0
        inside = (d > f_tail[k]) & (d < f_front[k]) & (np.abs(yy - y0) < 0.12 * np.maximum(d, 0) + 3)
        if inside.any():
            img = np.where(inside, img * (1.0 + 0.25 * rng.standard_normal((h, w)).astype(np.float32)), img)
        img += 2.0 * rng.standard_normal((h, w)).astype(np.float32)
        img[4:16, w - 16:w - 4] = 240.0 if sync[k] > 0.5 else 12.0
        out[k] = np.clip(img, 0, 255).astype(np.uint8)
    return out


def make_camera(cfg: dict, enabled: bool = True):
    if not enabled or cfg["backend"] == "none":
        return NoCamera()
    if cfg["backend"] == "fake":
        return FakeCamera(cfg)
    if cfg["backend"] == "ffmpeg":
        return FfmpegCamera(cfg)
    raise ValueError(f"camera.backend {cfg['backend']!r}")


def read_gray(path: Path, scale: float = 1.0, ffmpeg: str = "ffmpeg"):
    """Decode a video to luma frames (n, h, w) uint8, optionally downscaled."""
    probe = subprocess.run([ffmpeg.replace("ffmpeg", "ffprobe") if ffmpeg.endswith("ffmpeg") else "ffprobe",
                            "-v", "error", "-select_streams", "v:0", "-show_entries", "stream=width,height",
                            "-of", "json", str(path)], capture_output=True)
    if probe.returncode:
        raise CameraError(probe.stderr.decode())
    s = json.loads(probe.stdout)["streams"][0]
    w, h = int(s["width"]), int(s["height"])
    if scale != 1.0:
        w2, h2 = max(2, int(w * scale) // 2 * 2), max(2, int(h * scale) // 2 * 2)
        vf = ["-vf", f"scale={w2}:{h2}:flags=area"]
    else:
        w2, h2, vf = w, h, []
    p = subprocess.run([ffmpeg, "-v", "error", "-i", str(path), *vf, "-f", "rawvideo", "-pix_fmt", "gray", "-"],
                       capture_output=True)
    if p.returncode:
        raise CameraError(p.stderr.decode())
    n = len(p.stdout) // (w2 * h2)
    return np.frombuffer(p.stdout, np.uint8, count=n * w2 * h2).reshape(n, h2, w2), (w, h)
