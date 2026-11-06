// poofer_dmx_rx.h - DMX512 frame parser and test-pattern checker (PooferCommon).
//
// Hardware-free: fed one RP2040/PL011 UARTDR word at a time (data in bits 7:0,
// FE bit 8, PE bit 9, BE bit 10, OE bit 11) plus a microsecond timestamp, so the
// same code runs in the UART IRQ on the board and in the host test harness.
//
// Frame = BREAK, start code, 1-512 slots. A frame ends at the next BREAK or after
// slot 512. The PL011 loads a single 0x00 with BE set for a break of any length
// (DMX breaks are >= 88 us, two character times at 250 kbaud 8N2).
//
// Test pattern (board-test/host/artnet_send.py): slot i = (seq + h(i)) & 0xFF,
// h(i) = dmx_pat_h() below. seq goes up by one per Art-Net packet.
#pragma once
#include <stdint.h>
#include <string.h>

#define DMX_DR_FE (1u << 8)
#define DMX_DR_PE (1u << 9)
#define DMX_DR_BE (1u << 10)
#define DMX_DR_OE (1u << 11)
#define DMX_LOSS_US 1000000u   // the poofer boards' DMX-loss timeout

// Must stay nonlinear in i: with a plain multiply, slot i+1 - slot i is nearly constant,
// and a frame shifted by one dropped byte would look like a valid frame with another seq.
static inline uint8_t dmx_pat_h(uint32_t i) {
  uint32_t x = i * 0x9E3779B1u;
  x ^= x >> 15; x *= 0x85EBCA6Bu; x ^= x >> 13;
  return (uint8_t)(x >> 24);
}

enum DmxPat : uint8_t { PAT_NONE, PAT_OK, PAT_PAD, PAT_TEAR, PAT_BAD };

struct DmxPatResult { DmxPat r; uint8_t seq; uint16_t firstBad; uint8_t got, exp; };

// Classify one frame against the test pattern. slots[1..n] (slots[0] = start code).
//
// Picks the seq that matches the most slots, trying the seq implied by slots 1-4,
// n/2, n-1 and n, so a bad byte or a torn end cannot hide the pattern. A frame is
// the pattern if that seq matches at least 16 slots (or half of a short frame):
// constant frames (blackout, ramp mode, QLC+ with a few channels up) match about 2
// of 512 by chance, so they read as "not the pattern" and are not graded.
//
// A frame that only lines up after shifting by 1 or 2 slots (byte dropped or added
// near the start) is PAT_BAD. Otherwise every mismatch is corruption, except:
//  - PAT_TEAR: the mismatches are one run touching either end of the frame and all
//    fit one nearby seq (within 16): the node sent the frame while updating its
//    buffer from the next (or previous) packet. A fixed fault on slot 1 or the last
//    slot can also look like this, so tears are reported, not passed silently.
//  - PAT_PAD: the pattern, then zeros to the end of the frame: the sender sent fewer
//    slots (-n) than the node puts out and the node pads with zeros.
static uint8_t dmx_H[516];   // dmx_pat_h(0..515), filled by dmx_pat_init()
static inline void dmx_pat_init() { for (uint32_t i = 0; i < 516; i++) dmx_H[i] = dmx_pat_h(i); }

static inline uint16_t dmx_pat_count(const uint8_t* slots, uint16_t n, uint8_t s, int k) {
  uint16_t m = 0;
  for (uint16_t i = (k < 0) ? (uint16_t)(1 - k) : 1; i <= n; i++) m += (slots[i] == (uint8_t)(s + dmx_H[i + k]));
  return m;
}

static inline DmxPatResult dmx_pat_check(const uint8_t* slots, uint16_t n) {
  DmxPatResult res = {PAT_NONE, 0, 0, 0, 0};
  if (n < 8) return res;
  const uint16_t cand[7] = {1, 2, 3, 4, (uint16_t)(n / 2), (uint16_t)(n - 1), n};
  uint16_t best = 0; uint8_t bestSeq = 0;
  for (uint16_t c : cand) {
    uint8_t s = (uint8_t)(slots[c] - dmx_H[c]);
    uint16_t m = dmx_pat_count(slots, n, s, 0);
    if (m > best) { best = m; bestSeq = s; }
    if (best == n) break;                                   // the usual clean frame: one pass
  }
  const uint16_t need = n < 32 ? (n + 1) / 2 : 16;
  if (best < need) {
    // A byte dropped (or added) near the start shifts most of the frame by one or two
    // slots. Look for that before calling the frame "not the pattern".
    for (int k = -2; k <= 2; k++) {
      if (!k) continue;
      uint16_t c = (k > 0) ? n : (uint16_t)(n - 1);        // a slot from the shifted region
      uint8_t s = (uint8_t)(slots[c] - dmx_H[c + k]);
      if (dmx_pat_count(slots, n, s, k) >= need) {
        uint16_t lo = 1;
        while (lo <= n && slots[lo] == (uint8_t)(s + dmx_H[lo])) lo++;
        if (lo > n) lo = n;
        res.r = PAT_BAD; res.seq = s; res.firstBad = lo; res.got = slots[lo]; res.exp = (uint8_t)(s + dmx_H[lo]);
        return res;
      }
    }
    return res;
  }
  res.seq = bestSeq;
  if (best == n) { res.r = PAT_OK; return res; }

  uint16_t lo = 0, hi = 0;   // first and last mismatching slot
  for (uint16_t i = 1; i <= n; i++) if (slots[i] != (uint8_t)(bestSeq + dmx_H[i])) { if (!lo) lo = i; hi = i; }
  res.firstBad = lo; res.got = slots[lo]; res.exp = (uint8_t)(bestSeq + dmx_H[lo]);
  res.r = PAT_BAD;
  if (lo != 1 && hi != n) return res;                       // run not touching an end
  if (lo > 16 && hi == n) {                                 // zero padding after the pattern?
    bool zeros = true;
    for (uint16_t i = lo; i <= n && zeros; i++) zeros = slots[i] == 0;
    if (zeros) { res.r = PAT_PAD; return res; }
  }
  uint8_t s2 = (uint8_t)(slots[lo] - dmx_H[lo]);
  int8_t d = (int8_t)(uint8_t)(s2 - bestSeq);
  if (d == 0 || d > 16 || d < -16) return res;              // not a neighbouring packet
  for (uint16_t i = lo; i <= hi; i++) if (slots[i] != (uint8_t)(s2 + dmx_H[i])) return res;
  res.r = PAT_TEAR;
  return res;
}

struct DmxStats {
  uint32_t bytes, breaks;
  uint32_t fe, oe, pe;            // framing errors not part of a break, FIFO overruns, parity
  uint32_t frames;                // complete frames, start code 0x00, no line error
  uint32_t errFrames;             // frames containing FE / OE / PE
  uint32_t altSc;                 // frames with a start code other than 0x00
  uint8_t  lastAltSc;
  uint32_t presync;               // words before the first break (listening started mid-frame:
                                  // the UART misreads data bits as start bits, so FE here is not a line fault)
  uint32_t stray;                 // data bytes after a break but outside a frame (past slot 512)
  uint16_t slotsMin, slotsMax;
  uint32_t perMinUs, perMaxUs;    // break-to-break
  uint64_t perSumUs; uint32_t perN;
  uint32_t losses;                // break-to-break gaps > 1 s
  uint32_t patOk, patPad, patTear, patBad, patNone;
  uint32_t seqChanges; uint8_t lastSeq; bool haveSeq;
  DmxPatResult firstBad;          // details of the first corrupt frame
  bool haveBad;
};

struct DmxRx {
  DmxStats st;
  // in-progress frame
  bool inFrame, frameErr, haveBreak;
  uint16_t idx;                   // 0 = waiting for start code
  uint8_t buf[513];
  uint32_t lastBreakUs;
  // last good frame, for the monitor
  uint8_t last[513];
  uint16_t lastN;
  uint32_t lastFrameUs;
  volatile uint32_t gen;          // bumps when `last` changes
};

static inline void dmx_reset(DmxRx* d) {
  dmx_pat_init();
  memset(&d->st, 0, sizeof d->st);
  d->st.slotsMin = 0xFFFF; d->st.perMinUs = 0xFFFFFFFFu;
  d->inFrame = d->frameErr = d->haveBreak = false;
  d->idx = 0; d->lastN = 0; d->lastBreakUs = 0; d->lastFrameUs = 0; d->gen = 0;
}

static inline void dmx_finish(DmxRx* d, uint32_t now) {
  if (!d->inFrame || d->idx == 0) { d->inFrame = false; return; }   // break with no start code
  uint16_t n = d->idx - 1;
  d->inFrame = false;
  if (d->frameErr) { d->st.errFrames++; return; }
  if (d->buf[0] != 0x00) { d->st.altSc++; d->st.lastAltSc = d->buf[0]; return; }
  if (n == 0) return;                                               // start code only
  d->st.frames++;
  if (n < d->st.slotsMin) d->st.slotsMin = n;
  if (n > d->st.slotsMax) d->st.slotsMax = n;
  DmxPatResult p = dmx_pat_check(d->buf, n);
  switch (p.r) {
    case PAT_OK:   d->st.patOk++; break;
    case PAT_PAD:  d->st.patPad++; break;
    case PAT_TEAR: d->st.patTear++; break;
    case PAT_BAD:  d->st.patBad++; if (!d->st.haveBad) { d->st.firstBad = p; d->st.haveBad = true; } break;
    default:       d->st.patNone++; break;
  }
  if (p.r == PAT_OK || p.r == PAT_PAD || p.r == PAT_TEAR) {
    if (d->st.haveSeq && p.seq != d->st.lastSeq) d->st.seqChanges++;
    d->st.lastSeq = p.seq; d->st.haveSeq = true;
  }
  memcpy(d->last, d->buf, n + 1); d->lastN = n; d->lastFrameUs = now; d->gen++;
}

// One UARTDR word.
static inline void dmx_feed(DmxRx* d, uint32_t dr, uint32_t now) {
  d->st.bytes++;
  if (dr & DMX_DR_BE) {
    dmx_finish(d, now);
    d->st.breaks++;
    if (d->haveBreak) {
      uint32_t p = now - d->lastBreakUs;
      if (p < d->st.perMinUs) d->st.perMinUs = p;
      if (p > d->st.perMaxUs) d->st.perMaxUs = p;
      d->st.perSumUs += p; d->st.perN++;
      if (p > DMX_LOSS_US) d->st.losses++;
    }
    d->haveBreak = true; d->lastBreakUs = now;
    d->inFrame = true; d->frameErr = false; d->idx = 0;
    return;
  }
  if (!d->haveBreak) { d->st.presync++; return; }
  if (dr & (DMX_DR_FE | DMX_DR_OE | DMX_DR_PE)) {
    if (dr & DMX_DR_FE) d->st.fe++;
    if (dr & DMX_DR_OE) d->st.oe++;
    if (dr & DMX_DR_PE) d->st.pe++;
    if (d->inFrame) d->frameErr = true;
    if (!(dr & DMX_DR_OE)) return;   // OE still carries a valid byte; FE/PE do not
  }
  if (!d->inFrame) { d->st.stray++; return; }
  d->buf[d->idx++] = (uint8_t)dr;
  if (d->idx == 513) dmx_finish(d, now);
}
