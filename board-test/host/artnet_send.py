#!/usr/bin/env python3
"""
artnet_send.py - Art-Net (ArtDmx) test source for the poofer board bring-up test.

Runs on the DMX controller Pi (or any machine on the node's network). Sends one
universe to the PKnight CR011R, which puts it out on XLR as DMX512; the base
board's 'd' test and 'e' monitor read it back.

Stdlib only, Python 3.8+.

  python3 artnet_send.py                     # verification pattern, 30 packets/s, until Ctrl-C
  python3 artnet_send.py pattern -d 20       # pattern for 20 s, then blackout
  python3 artnet_send.py set 1=255 2=128     # fixed values, everything else 0
  python3 artnet_send.py set --all 0         # hold a blackout
  python3 artnet_send.py ramp                # every slot the same value, rising 1 per packet
  python3 artnet_send.py pattern -d 10 --no-blackout   # Art-Net loss test: stop dead after 10 s
  python3 artnet_send.py poll                # ArtPoll: list nodes and their port settings

Pattern: slot i (1-512) = (seq + h(i)) mod 256, h(i) a fixed 8-bit hash of i (see h()
below). seq goes up by one per packet. The board recovers seq from the frame and
checks every slot, so a dropped, extra or corrupted byte anywhere in the frame is
caught, and a frame that is half one packet and half the next (node updating its
buffer mid-frame) is told apart from corruption.

BENCH ONLY. This drives every slot of the universe, including values that would
arm and fire a poofer running production firmware. Never point it at a chain
with gas connected.

Only one Art-Net source per universe: quit QLC+ (its own menu, so the launcher
leaves it closed) or unpatch its output before running this.
"""

import argparse
import errno
import signal
import socket
import struct
import sys
import time

ARTNET_PORT = 6454
OP_POLL = 0x2000
OP_POLL_REPLY = 0x2100
OP_DMX = 0x5000
PROT_VER = 14


def h(i):
    """Per-slot pattern offset. Must match dmx_pat_h() in libraries/PooferCommon/src/poofer_dmx_rx.h.
    Nonlinear in i, so a frame shifted by a dropped byte cannot pass as another seq."""
    x = (i * 0x9E3779B1) & 0xFFFFFFFF
    x ^= x >> 15
    x = (x * 0x85EBCA6B) & 0xFFFFFFFF
    x ^= x >> 13
    return x >> 24


def pattern_frame(seq, n=512):
    return bytes(((seq + h(i)) & 0xFF) for i in range(1, n + 1))


def artdmx(data, universe=0, sequence=0, physical=0):
    """ArtDmx packet. universe is the 15-bit Port-Address (Net:SubNet:Universe)."""
    n = len(data)
    if n % 2:
        data = data + b"\x00"
        n += 1
    return (b"Art-Net\x00" + struct.pack("<H", OP_DMX) + struct.pack(">H", PROT_VER)
            + bytes([sequence & 0xFF, physical & 0xFF, universe & 0xFF, (universe >> 8) & 0x7F])
            + struct.pack(">H", n) + data)


def artpoll():
    # Flags 0x02: send ArtPollReply whenever node conditions change. DiagPriority 0.
    return b"Art-Net\x00" + struct.pack("<H", OP_POLL) + struct.pack(">H", PROT_VER) + b"\x02\x00"


def parse_poll_reply(pkt):
    if len(pkt) < 194 or pkt[:8] != b"Art-Net\x00" or struct.unpack("<H", pkt[8:10])[0] != OP_POLL_REPLY:
        return None
    cstr = lambda b: b.split(b"\x00", 1)[0].decode("ascii", "replace")
    nports = struct.unpack(">H", pkt[172:174])[0]
    return {
        "ip": ".".join(str(b) for b in pkt[10:14]),
        "firmware": "%d.%d" % (pkt[16], pkt[17]),
        "net": pkt[18] & 0x7F,
        "subnet": pkt[19] & 0x0F,
        "short": cstr(pkt[26:44]),
        "long": cstr(pkt[44:108]),
        "report": cstr(pkt[108:172]),
        "ports": nports,
        "port_types": pkt[174:178].hex(" "),
        "sw_out": [b & 0x0F for b in pkt[190:194]],
    }


def open_socket(bind_ip, broadcast):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    if broadcast:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    try:
        s.bind((bind_ip, ARTNET_PORT))   # some nodes only accept packets from source port 6454
    except OSError as e:
        if e.errno == errno.EADDRNOTAVAIL:
            sys.exit("Cannot bind %s: that address is not on this machine right now.\n"
                     "On the Pi, eth0 only has 10.0.0.1 while the cable to the CR011R has link "
                     "(node powered). Check: ip addr show eth0\n"
                     "Elsewhere, pass --bind with this machine's address on the node's network, "
                     "or --bind '' for any." % (bind_ip or "*"))
        s.bind((bind_ip, 0))         # port in use and not shareable: fall back to any source port
        print("note: source port 6454 busy, using %d" % s.getsockname()[1])
    return s


def cmd_poll(args):
    # Bound to any address so broadcast replies arrive too. Poll both ways: some nodes
    # only answer a broadcast poll, and a unicast one works even with a wrong netmask.
    s = open_socket("" if args.bind is None else args.bind, True)
    for d in ("10.255.255.255", args.dest):
        try:
            s.sendto(artpoll(), (d, ARTNET_PORT))
        except OSError as e:
            print("poll to %s failed: %s" % (d, e))
    s.settimeout(0.2)
    seen = {}
    t_end = time.monotonic() + args.wait
    while time.monotonic() < t_end:
        try:
            pkt, addr = s.recvfrom(1024)
        except socket.timeout:
            continue
        r = parse_poll_reply(pkt)
        if r and r["ip"] not in seen:
            seen[r["ip"]] = r
            print("%-15s fw %s  %r / %r" % (r["ip"], r["firmware"], r["short"], r["long"]))
            print("    Net %d  SubNet %d  ports %d  types [%s]  output universe nibbles %s"
                  % (r["net"], r["subnet"], r["ports"], r["port_types"], r["sw_out"]))
            if r["report"]:
                print("    report: %s" % r["report"])
    if not seen:
        print("No ArtPollReply in %.1f s. Link up (ping %s)? Node in Art-Net mode? Some nodes "
              "do not answer polls at all; DMX output can still work." % (args.wait, args.dest))
        return 1
    return 0


def run_sender(args):
    frame_len = args.length
    if args.mode == "set":
        vals = bytearray([args.all] * frame_len)
        for kv in args.values:
            try:
                ch, v = kv.split("=")
                ch, v = int(ch), int(v)
            except ValueError:
                sys.exit("bad value %r, want CH=VAL like 1=255" % kv)
            if not (1 <= ch <= frame_len and 0 <= v <= 255):
                sys.exit("out of range: %r (channel 1-%d, value 0-255)" % (kv, frame_len))
            vals[ch - 1] = v
        fixed = bytes(vals)

    bind = "10.0.0.1" if args.bind is None else args.bind
    s = open_socket(bind, args.broadcast)
    dest = ("10.255.255.255" if args.broadcast else args.dest, ARTNET_PORT)

    stop = {"flag": False}
    signal.signal(signal.SIGINT, lambda *a: stop.__setitem__("flag", True))
    signal.signal(signal.SIGTERM, lambda *a: stop.__setitem__("flag", True))

    period = 1.0 / args.rate
    seq = 0
    artseq = 1
    sent = 0
    t0 = time.monotonic()
    next_t = t0
    last_print = t0
    print("Sending %s to %s:%d universe %d, %d slots, %.0f packets/s%s. Ctrl-C stops."
          % (args.mode, dest[0], dest[1], args.universe, frame_len, args.rate,
             (" for %.0f s" % args.duration) if args.duration else ""))
    if args.mode == "set":
        nz = [(i + 1, b) for i, b in enumerate(fixed) if b]
        print("Non-zero slots: %s" % (", ".join("%d=%d" % p for p in nz[:24]) + (" ..." if len(nz) > 24 else "") if nz else "none"))
    try:
        while not stop["flag"]:
            now = time.monotonic()
            if args.duration and now - t0 >= args.duration:
                break
            if args.mode == "pattern":
                data = pattern_frame(seq, frame_len)
            elif args.mode == "ramp":
                data = bytes([seq & 0xFF]) * frame_len
            else:
                data = fixed
            s.sendto(artdmx(data, args.universe, artseq), dest)
            sent += 1
            seq = (seq + 1) & 0xFF
            artseq = artseq + 1 if artseq < 255 else 1     # 0 means "sequencing off"
            if now - last_print >= 2.0:
                extra = (" seq 0x%02X" % ((seq - 1) & 0xFF)) if args.mode != "set" else ""
                print("  %6.1f s  %d packets%s" % (now - t0, sent, extra), flush=True)
                last_print = now
            next_t += period
            delay = next_t - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_t = time.monotonic()
    finally:
        if args.no_blackout:
            print("Stopped after %d packets, NO blackout sent (last frame left on the node)." % sent)
        else:
            z = bytes(frame_len)
            for _ in range(3):
                s.sendto(artdmx(z, args.universe, artseq), dest)
                artseq = artseq + 1 if artseq < 255 else 1
                time.sleep(0.03)
            print("Stopped after %d packets, blackout sent." % sent)
    return 0


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0],
                                epilog="modes: pattern (default) | set CH=VAL ... [--all V] | ramp | poll [--wait S]",
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("mode", nargs="?", default="pattern", choices=["pattern", "set", "ramp", "poll"])
    p.add_argument("values", nargs="*", help="for set: CH=VAL pairs, e.g. 1=255 2=128")
    p.add_argument("--all", type=int, default=0, help="for set: value for every slot not listed (default 0)")
    p.add_argument("--wait", type=float, default=2.0, help="for poll: seconds to listen (default 2)")
    p.add_argument("--dest", default="10.0.0.10", help="node IP for unicast (default 10.0.0.10, the CR011R)")
    p.add_argument("--broadcast", action="store_true", help="send to 10.255.255.255 instead of unicast")
    p.add_argument("--bind", default=None,
                   help="local source address (default 10.0.0.1, the Pi's eth0; '' = any)")
    p.add_argument("-u", "--universe", type=int, default=0,
                   help="15-bit Art-Net Port-Address; 0 = Net 0 SubNet 0 Universe 0 = QLC+ universe 1")
    p.add_argument("-r", "--rate", type=float, default=30.0, help="packets per second (default 30)")
    p.add_argument("-n", "--length", type=int, default=512, help="slots per packet, 2-512 (default 512)")
    p.add_argument("-d", "--duration", type=float, default=0, help="stop after this many seconds (default: run until Ctrl-C)")
    p.add_argument("--no-blackout", action="store_true", help="on exit just stop; do not send zeros (Art-Net loss test)")
    args = p.parse_intermixed_args(argv)
    if args.values and args.mode != "set":
        p.error("CH=VAL values only go with the set mode")
    if not 0 <= args.all <= 255:
        p.error("--all must be 0-255")
    if not 0 <= args.universe <= 0x7FFF:
        p.error("universe must be 0-32767")
    if not 2 <= args.length <= 512:
        p.error("length must be 2-512")
    if not 0.5 <= args.rate <= 100:
        p.error("rate must be 0.5-100")
    if args.mode == "poll":
        return cmd_poll(args)
    return run_sender(args)


if __name__ == "__main__":
    sys.exit(main())
