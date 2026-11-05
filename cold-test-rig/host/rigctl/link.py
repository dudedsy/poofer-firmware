"""Serial link to rig_control (USB serial, or socket:// for the simulator).

Device -> host stream: text lines ('I,', 'S,', 'E,', 'OK', 'ERR', '#') and binary sample
packets (0xA5 0x5A 'D' len16 payload crc16), see firmware/rig_control/rig_core.h.
"""
from __future__ import annotations

import json
import queue
import struct
import threading
import time
from dataclasses import dataclass, field

import numpy as np
import serial

MAGIC = b"\xA5\x5A"
TELE_DTYPE = np.dtype([("v_duty", "<u2"), ("v_ma", "<i2"), ("v_st", "u1"),
                       ("l_duty", "<u2"), ("l_ma", "<i2"), ("l_st", "u1")])
VALVE_STATES = ["OFF", "PEAK", "HOLD", "RELEASE", "FAULT"]
LED_STATES = ["OFF", "RAMP", "RUN", "STOPPING", "REARM", "FAULT"]


class DeviceError(RuntimeError):
    pass


def crc16(data: bytes, crc: int = 0xFFFF) -> int:
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


@dataclass
class Packet:
    t0: int            # ms (block index) of the first block
    n_ms: int
    samples: np.ndarray    # (n_ms * fpm, n_ch) uint16, sums of 4 ADC samples
    tele: np.ndarray       # (n_ms,) TELE_DTYPE


def parse_packet(payload: bytes) -> Packet:
    t0, n_ms, n_ch, fpm, _flags = struct.unpack_from("<IBBBB", payload, 0)
    ns = n_ms * fpm * n_ch
    samples = np.frombuffer(payload, dtype="<u2", count=ns, offset=8).reshape(n_ms * fpm, n_ch).copy()
    tele = np.frombuffer(payload, dtype=TELE_DTYPE, count=n_ms, offset=8 + 2 * ns).copy()
    return Packet(t0, n_ms, samples, tele)


@dataclass
class Recording:
    """Everything received between start() and stop(): packets, events, status."""
    packets: list = field(default_factory=list)
    events: list = field(default_factory=list)
    status: list = field(default_factory=list)
    log: list = field(default_factory=list)
    crc_errors: int = 0

    def arrays(self):
        """Concatenate packets into contiguous arrays, filling gaps (dropped packets) with 0
        and marking them in `valid`. Returns dict, or None if nothing was recorded."""
        if not self.packets:
            return None
        pk = sorted(self.packets, key=lambda p: p.t0)
        fpm = pk[0].samples.shape[0] // pk[0].n_ms
        n_ch = pk[0].samples.shape[1]
        t0 = pk[0].t0
        n_ms = pk[-1].t0 + pk[-1].n_ms - t0
        samples = np.zeros((n_ms * fpm, n_ch), np.uint16)
        tele = np.zeros(n_ms, TELE_DTYPE)
        valid = np.zeros(n_ms, bool)
        for p in pk:
            o = p.t0 - t0
            samples[o * fpm:(o + p.n_ms) * fpm] = p.samples
            tele[o:o + p.n_ms] = p.tele
            valid[o:o + p.n_ms] = True
        return {"t0_ms": t0, "fpm": fpm, "samples": samples, "tele": tele, "valid": valid}


class Device:
    def __init__(self, port: str, baud: int = 115200):
        self.port = port
        self.ser = serial.serial_for_url(port, baudrate=baud, timeout=0.05)
        self.status: dict | None = None
        self.status_time = 0.0
        self.info_json: dict | None = None
        self._replies: queue.Queue = queue.Queue()
        self._events: list = []
        self._cv = threading.Condition()
        self._rec: Recording | None = None
        self._stop = False
        self.log_lines: list[str] = []
        self.crc_errors = 0
        self._th = threading.Thread(target=self._reader, daemon=True)
        self._th.start()

    # ------------------------------------------------------------ reader
    def _reader(self):
        buf = bytearray()
        while not self._stop:
            try:
                d = self.ser.read(65536)
            except (serial.SerialException, OSError):
                time.sleep(0.1)
                continue
            if d:
                buf += d
            while True:
                if not buf:
                    break
                if buf[0] == 0xA5:
                    if len(buf) < 5:
                        break
                    if buf[1] != 0x5A:
                        del buf[0]
                        continue
                    ln = buf[3] | (buf[4] << 8)
                    total = 5 + ln + 2
                    if len(buf) < total:
                        break
                    body = bytes(buf[2:5 + ln])
                    crc = buf[5 + ln] | (buf[6 + ln] << 8)
                    del buf[:total]
                    if crc16(body) != crc:
                        self.crc_errors += 1
                        if self._rec:
                            self._rec.crc_errors += 1
                        continue
                    if body[0] == ord("D") and self._rec is not None:
                        self._rec.packets.append(parse_packet(body[3:]))
                    continue
                nl = buf.find(b"\n")
                if nl < 0:
                    if len(buf) > 4096:
                        del buf[:]
                    break
                line = bytes(buf[:nl]).decode("ascii", "replace").strip()
                del buf[:nl + 1]
                if line:
                    self._line(line)

    def _line(self, line: str):
        now = time.time()
        if line.startswith("S,"):
            try:
                st = json.loads(line[2:])
            except json.JSONDecodeError:
                return
            st["host_time"] = now
            self.status, self.status_time = st, now
            if self._rec is not None:
                self._rec.status.append(st)
        elif line.startswith("E,"):
            try:
                ev = json.loads(line[2:])
            except json.JSONDecodeError:
                return
            ev["host_time"] = now
            with self._cv:
                self._events.append(ev)
                if self._rec is not None:
                    self._rec.events.append(ev)
                self._cv.notify_all()
        elif line.startswith("I,"):
            try:
                self.info_json = json.loads(line[2:])
            except json.JSONDecodeError:
                pass
            self._replies.put(("I", line))
        elif line.startswith("OK") or line.startswith("ERR"):
            self._replies.put(("R", line))
        else:
            self.log_lines.append(line)
            if self._rec is not None:
                self._rec.log.append(line)

    # ------------------------------------------------------------ commands
    def send(self, line: str):
        self.ser.write((line + "\n").encode("ascii"))
        self.ser.flush()

    def command(self, line: str, timeout: float = 2.0) -> str:
        """Send a command and wait for its OK/ERR. Raises DeviceError on ERR or timeout."""
        while not self._replies.empty():
            self._replies.get_nowait()
        self.send(line)
        word = line.split()[0].lower()
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                kind, r = self._replies.get(timeout=0.05)
            except queue.Empty:
                continue
            if kind != "R":
                continue
            if r.startswith("ERR"):
                raise DeviceError(f"{line!r}: {r}")
            if r.split()[1:2] == [word] or word in ("?",):
                return r
        raise DeviceError(f"{line!r}: no reply from {self.port}")

    def info(self, timeout: float = 2.0) -> dict:
        self.info_json = None
        self.send("?")
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.info_json is not None:
                return self.info_json
            time.sleep(0.02)
        raise DeviceError(f"no reply to '?' from {self.port} (is rig_control flashed?)")

    def set(self, kv: dict):
        """'set' keys such as {"v.peak": 2.7, "l.i": 2.0}. Drive keys need that drive off."""
        if kv:
            self.command("set " + " ".join(f"{k}={_fmt(v)}" for k, v in kv.items()))

    def shot(self, kv: dict):
        if kv:
            self.command("shot " + " ".join(f"{k}={_fmt(v)}" for k, v in kv.items()))

    def wait_status(self, timeout: float = 2.0) -> dict:
        t = time.time()
        while time.time() - t < timeout:
            if self.status and self.status_time > t:
                return self.status
            time.sleep(0.02)
        raise DeviceError("no status from the device")

    # ------------------------------------------------------------ events and recording
    def event_mark(self) -> int:
        with self._cv:
            return len(self._events)

    def wait_event(self, names, since: int, timeout: float) -> dict | None:
        names = {names} if isinstance(names, str) else set(names)
        deadline = time.time() + timeout
        with self._cv:
            while True:
                for ev in self._events[since:]:
                    if ev.get("ev") in names:
                        return ev
                left = deadline - time.time()
                if left <= 0:
                    return None
                self._cv.wait(left)

    def start_recording(self) -> Recording:
        self._rec = Recording()
        return self._rec

    def stop_recording(self) -> Recording | None:
        r, self._rec = self._rec, None
        return r

    def close(self):
        self._stop = True
        self._th.join(timeout=1)
        try:
            self.ser.close()
        except Exception:
            pass


def _fmt(v) -> str:
    if isinstance(v, bool):
        return "1" if v else "0"
    if isinstance(v, float):
        return f"{v:.6g}"
    return str(v)
