#!/usr/bin/env python3
"""
Host tests for the DMX receive test: the firmware parser (PooferCommon poofer_dmx_rx.h, compiled with g++
into a small harness) fed with synthetic UART streams built from the real sender's
pattern function (host/artnet_send.py), plus the Art-Net packet format.

  python3 board-test/tests/test_dmx.py        (needs g++)
"""
import os
import shutil
import struct
import subprocess
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "host"))
import artnet_send as an  # noqa: E402

BE, FE, OE = 1 << 10, 1 << 8, 1 << 11
SLOT_US = 44                       # 11 bits at 250 kbaud


class Stream:
    """Builds UARTDR words with timestamps, as the PL011 would deliver them."""

    def __init__(self):
        self.t = 0
        self.words = []

    def gap(self, us):
        self.t += us
        return self

    def brk(self):
        self.t += 100                  # 92 us break + 12 us MAB, roughly
        self.words.append((self.t, BE))
        return self

    def byte(self, b, flags=0):
        self.t += SLOT_US
        self.words.append((self.t, (b & 0xFF) | flags))
        return self

    def frame(self, slots, sc=0, mtbf_us=200):
        self.brk().byte(sc)
        for b in slots:
            self.byte(b)
        self.t += mtbf_us
        return self

    def end(self):
        return self.brk()              # a frame only finishes at the next break


def run(stream):
    exe = run.exe
    inp = "".join("%d %x\n" % (t, w) for t, w in stream.words)
    out = subprocess.run([exe], input=inp, capture_output=True, text=True, check=True).stdout
    return {k: int(v) for k, v in (kv.split("=") for kv in out.split())}


def setUpModule():
    if not shutil.which("g++"):
        raise unittest.SkipTest("g++ not found")
    run.tmp = tempfile.mkdtemp()
    run.exe = os.path.join(run.tmp, "dmx_harness")
    subprocess.run(["g++", "-std=c++17", "-O1", "-Wall", "-Wextra", "-Werror", "-o", run.exe,
                    os.path.join(HERE, "dmx_harness.cpp")], check=True)


def tearDownModule():
    shutil.rmtree(getattr(run, "tmp", ""), ignore_errors=True)


class TestPattern(unittest.TestCase):
    def test_h_matches_firmware(self):
        # The harness frame below is built in Python and must verify in C, slot for slot.
        r = run(Stream().frame(an.pattern_frame(0x5A)).end())
        self.assertEqual((r["patOk"], r["lastSeq"]), (1, 0x5A))
        diffs = {(an.h(i + 1) - an.h(i)) & 0xFF for i in range(1, 512)}
        self.assertGreater(len(diffs), 200)       # nonlinear: no dominant slot-to-slot step

    def test_clean_frames(self):
        s = Stream()
        for k in range(20):
            s.frame(an.pattern_frame(k // 2))      # each packet sent twice by the node
        r = run(s.end())
        self.assertEqual((r["frames"], r["patOk"], r["patBad"], r["patTear"]), (20, 20, 0, 0))
        self.assertEqual((r["slotsMin"], r["slotsMax"], r["errFrames"], r["fe"]), (512, 512, 0, 0))
        self.assertEqual(r["seqChanges"], 9)
        self.assertEqual(r["lastSeq"], 9)

    def test_short_frames_and_wrap(self):
        s = Stream()
        for k in range(250, 262):
            s.frame(an.pattern_frame(k & 0xFF, 24))
        r = run(s.end())
        self.assertEqual((r["frames"], r["patOk"], r["slotsMin"], r["seqChanges"]), (12, 12, 24, 11))

    def test_corrupt_byte(self):
        f = bytearray(an.pattern_frame(7))
        f[99] ^= 0x10                              # slot 100
        r = run(Stream().frame(an.pattern_frame(6)).frame(bytes(f)).end())
        self.assertEqual((r["patOk"], r["patBad"]), (1, 1))
        self.assertEqual((r["badSlot"], r["badGot"], r["badExp"], r["badSeq"]), (100, f[99], f[99] ^ 0x10, 7))

    def test_corrupt_slot1_still_recognised(self):
        f = bytearray(an.pattern_frame(3)); f[0] ^= 0xFF
        r = run(Stream().frame(bytes(f)).end())
        self.assertEqual((r["patBad"], r["badSlot"], r["patNone"]), (1, 1, 0))

    def test_dropped_byte_is_corruption_not_tear(self):
        f = an.pattern_frame(40)
        f = f[:200] + f[201:]                      # slot 201 lost, rest shifted
        r = run(Stream().frame(f).end())
        self.assertEqual((r["patBad"], r["patTear"], r["badSlot"]), (1, 0, 201))

    def test_torn_frame(self):
        old, new = an.pattern_frame(10), an.pattern_frame(11)
        r = run(Stream().frame(old[:300] + new[300:]).frame(new[:5] + old[5:]).end())
        self.assertEqual((r["patTear"], r["patBad"], r["frames"]), (2, 0, 2))

    def test_tear_far_seq_is_corruption(self):
        a, b = an.pattern_frame(10), an.pattern_frame(100)
        r = run(Stream().frame(a[:400] + b[400:]).end())
        self.assertEqual((r["patTear"], r["patBad"]), (0, 1))

    def test_dropped_byte_early(self):
        f = an.pattern_frame(40)
        f = f[:3] + f[4:]                          # slot 4 lost: only 3 slots line up
        r = run(Stream().frame(f).end())
        self.assertEqual((r["patBad"], r["patNone"], r["patTear"]), (1, 0, 0))

    def test_extra_and_double_drop_early(self):
        f = an.pattern_frame(41)
        r = run(Stream().frame(f[:2] + b"\x00" + f[2:511]).frame(f[:5] + f[7:]).end())
        self.assertEqual((r["patBad"], r["patNone"], r["badSlot"]), (2, 0, 3))

    def test_node_zero_pads_short_packet(self):
        r = run(Stream().frame(an.pattern_frame(8, 24) + bytes(488)).end())
        self.assertEqual((r["patPad"], r["patBad"], r["patOk"]), (1, 0, 0))

    def test_not_pattern(self):
        r = run(Stream().frame(bytes(512)).frame(bytes([255]) * 512).frame(bytes([1, 2, 3])).end())
        self.assertEqual((r["frames"], r["patNone"], r["patBad"], r["patOk"]), (3, 3, 0, 0))

    def test_framing_error_drops_frame(self):
        s = Stream().frame(an.pattern_frame(1))
        s.brk().byte(0)
        for i, b in enumerate(an.pattern_frame(2)):
            s.byte(b, FE if i == 50 else 0)
        s.frame(an.pattern_frame(3))
        r = run(s.end())
        self.assertEqual((r["frames"], r["errFrames"], r["fe"], r["patOk"], r["patBad"]), (2, 1, 1, 2, 0))

    def test_overrun_counted(self):
        s = Stream().brk().byte(0)
        for i, b in enumerate(an.pattern_frame(5)):
            s.byte(b, OE if i == 10 else 0)
        r = run(s.end())
        self.assertEqual((r["oe"], r["errFrames"], r["frames"]), (1, 1, 0))

    def test_alt_start_code_and_stray(self):
        s = Stream().byte(0x55).byte(0x66)          # mid-frame when we started listening
        s.frame(b"\x01\x02\x03", sc=0xCC).frame(an.pattern_frame(9))
        r = run(s.end())
        self.assertEqual((r["presync"], r["stray"], r["altSc"], r["lastAltSc"], r["frames"], r["patOk"]), (2, 0, 1, 0xCC, 1, 1))

    def test_started_mid_frame_errors_ignored(self):
        # Listening starts mid-frame: the UART is misaligned until the first break and
        # logs framing errors. Seen on hardware 2026-10-27 (8 FE, 0 frames dropped).
        s = Stream()
        for b in range(8):
            s.byte(b, FE)
        s.byte(0x42)
        for k in range(3):
            s.frame(an.pattern_frame(k))
        r = run(s.end())
        self.assertEqual((r["presync"], r["fe"], r["errFrames"], r["frames"], r["patOk"]), (9, 0, 0, 3, 3))

    def test_framing_error_between_frames_still_counted(self):
        s = Stream().frame(an.pattern_frame(1))
        s.brk().byte(0)
        for b in an.pattern_frame(2):
            s.byte(b)
        s.byte(0x00, FE)                           # junk after slot 512, after sync
        r = run(s.end())
        self.assertEqual((r["fe"], r["errFrames"], r["frames"], r["presync"]), (1, 0, 2, 0))

    def test_513_slots_ends_frame(self):
        s = Stream().brk().byte(0)
        for b in an.pattern_frame(4) + b"\x77\x88":   # two extra bytes past slot 512
            s.byte(b)
        r = run(s.end())
        self.assertEqual((r["frames"], r["patOk"], r["slotsMax"], r["stray"]), (1, 1, 512, 2))

    def test_timing_and_loss(self):
        s = Stream()
        for k in range(5):
            s.frame(an.pattern_frame(k), mtbf_us=1000)
        s.gap(1_500_000)
        s.frame(an.pattern_frame(5), mtbf_us=1000)
        r = run(s.end())
        per = 100 + 513 * SLOT_US + 1000
        self.assertEqual(r["perMinUs"], per)
        self.assertGreater(r["perMaxUs"], 1_500_000)
        self.assertEqual((r["losses"], r["frames"]), (1, 6))

    def test_break_only_and_garbage(self):
        s = Stream()
        for _ in range(10):
            s.brk()
            for b in range(20):
                s.byte(b, FE)                    # A/B reversed looks like this
        r = run(s.end())
        self.assertEqual((r["frames"], r["fe"], r["breaks"]), (0, 200, 11))


class TestArtNet(unittest.TestCase):
    def test_artdmx_layout(self):
        p = an.artdmx(bytes(range(5)), universe=0x1234, sequence=9)
        self.assertEqual(p[:8], b"Art-Net\x00")
        self.assertEqual(struct.unpack("<H", p[8:10])[0], 0x5000)
        self.assertEqual(struct.unpack(">H", p[10:12])[0], 14)
        self.assertEqual((p[12], p[13], p[14], p[15]), (9, 0, 0x34, 0x12))
        self.assertEqual(struct.unpack(">H", p[16:18])[0], 6)            # padded to even
        self.assertEqual(p[18:], bytes(range(5)) + b"\x00")

    def test_poll_reply_parse(self):
        r = bytearray(239)
        r[:8] = b"Art-Net\x00"; r[8:10] = struct.pack("<H", 0x2100)
        r[10:14] = bytes([10, 0, 0, 10]); r[16:18] = bytes([2, 5]); r[18] = 0; r[19] = 0
        r[26:26 + 6] = b"CR011R"; r[44:44 + 4] = b"node"; r[172:174] = struct.pack(">H", 1)
        r[174] = 0x80; r[190] = 0
        d = an.parse_poll_reply(bytes(r))
        self.assertEqual((d["ip"], d["firmware"], d["short"], d["ports"], d["sw_out"][0]), ("10.0.0.10", "2.5", "CR011R", 1, 0))

    def test_cli_parses(self):
        out = subprocess.run([sys.executable, os.path.join(HERE, "..", "host", "artnet_send.py"), "set", "1=300"],
                             capture_output=True, text=True)
        self.assertNotEqual(out.returncode, 0)
        self.assertIn("out of range", out.stderr)


if __name__ == "__main__":
    unittest.main(verbosity=1)
