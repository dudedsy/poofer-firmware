// rig_core.cpp - see rig_core.h.
#include "rig_core.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

using namespace poofer;

namespace rig {

uint16_t crc16(const uint8_t* p, size_t n, uint16_t crc) {
  while (n--) {
    crc ^= (uint16_t)(*p++) << 8;
    for (int k = 0; k < 8; k++) crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
  }
  return crc;
}

static const char* faultCode(Fault f) {
  switch (f) {
    case Fault::NONE: return "none";
    case Fault::OVER_CURRENT: return "over_current";
    case Fault::SENSE_SATURATED: return "sense_saturated";
    case Fault::NO_CURRENT: return "no_current";
    case Fault::LOST_CURRENT: return "lost_current";
  }
  return "?";
}

static inline void put16(uint8_t* p, uint16_t v) { p[0] = v & 0xFF; p[1] = v >> 8; }
static inline void put32(uint8_t* p, uint32_t v) { for (int k = 0; k < 4; k++) p[k] = (v >> (8 * k)) & 0xFF; }
static inline int16_t mA(float a) { float m = a * 1000.0f; return (int16_t)(m > 32767 ? 32767 : (m < -32768 ? -32768 : m)); }

Core::Core(Platform& pf, DriveHw& valveHw, DriveHw& ledHw)
    : valve(valveHw), led(ledHw), pf_(pf), vhw_(valveHw), lhw_(ledHw) {}

void Core::begin() {
  pf_.beginDrive(0, valvePair, valveAdc, 20000);
  pf_.beginDrive(1, ledPair, ledAdc, 50000);
  valve.begin(SolenoidConfig());
  led.begin(LedConfig());
  led.setTarget(2.0f);
  pf_.syncLed(false);
  char b[64];
  snprintf(b, sizeof b, "#%s ready, '?' for settings", FW_VERSION);
  text(b);
}

bool Core::drivesOff() const {
  return valve.state() == SolenoidDriver::OFF && (led.state() == LedDriver::OFF || led.state() == LedDriver::FAULT) &&
         ss_ == IDLE;
}

// ---------------------------------------------------------------- output
bool Core::text(const char* s) {
  size_t n = strlen(s);
  if (pf_.outSpace() < n + 1) { textDrops_++; return false; }
  pf_.out((const uint8_t*)s, n);
  pf_.out((const uint8_t*)"\n", 1);
  return true;
}

void Core::event(const char* name, const char* extra) {
  char b[320];
  snprintf(b, sizeof b, "E,{\"t\":%lu,\"ev\":\"%s\"%s%s}", (unsigned long)t_, name, extra ? "," : "", extra ? extra : "");
  text(b);
}

// Packet: A5 5A 'D' len(u16) payload crc16(type, len, payload), little-endian.
// payload: u32 t0_ms, u8 n_ms, u8 n_ch, u8 frames_per_ms, u8 flags,
//          u16 samples[n_ms * fpm][n_ch]  (sum of SUB 12-bit samples, 0..16380),
//          telemetry[n_ms] x 10 bytes: u16 valve_duty*1e4, i16 valve_mA, u8 valve_state | fault<<4,
//                                      u16 led_duty*1e4,   i16 led_mA,   u8 led_state | fault<<4
// Frame f of block t was sampled over (t-1 + f/5, t-1 + (f+1)/5] ms. Telemetry of block t is
// the drive state after the control update at the end of block t.
void Core::flushPacket() {
  if (pktN_ == 0) return;
  static uint8_t buf[3 + 2 + 8 + sizeof(pktSamp_) + sizeof(pktTele_) + 2];
  size_t ns = (size_t)pktN_ * FPM * N_ADC * 2, nt = (size_t)pktN_ * 10;
  uint16_t len = (uint16_t)(8 + ns + nt);
  uint8_t* p = buf;
  *p++ = 0xA5; *p++ = 0x5A; *p++ = 'D';
  put16(p, len); p += 2;
  put32(p, pktT0_); p[4] = (uint8_t)pktN_; p[5] = N_ADC; p[6] = FPM; p[7] = 0; p += 8;
  for (size_t k = 0; k < ns / 2; k++) { put16(p, pktSamp_[k]); p += 2; }
  memcpy(p, pktTele_, nt); p += nt;
  uint16_t c = crc16(buf + 2, (size_t)(p - (buf + 2)));
  put16(p, c); p += 2;
  size_t total = (size_t)(p - buf);
  if (pf_.outSpace() >= total + 256) { pf_.out(buf, total); packets_++; }   // keep room for text
  else drops_++;
  pktN_ = 0;
}

void Core::recordMs(const uint16_t* frames) {
  if (pktN_ == 0) pktT0_ = t_;
  memcpy(&pktSamp_[pktN_ * FPM * N_ADC], frames, FPM * N_ADC * sizeof(uint16_t));
  uint8_t* q = &pktTele_[pktN_ * 10];
  put16(q, (uint16_t)(vhw_.duty() * 10000.0f + 0.5f));
  put16(q + 2, (uint16_t)mA(valve.current()));
  q[4] = (uint8_t)(valve.state() | ((uint8_t)valve.fault() << 4));
  put16(q + 5, (uint16_t)(lhw_.duty() * 10000.0f + 0.5f));
  put16(q + 7, (uint16_t)mA(led.current()));
  q[9] = (uint8_t)(led.state() | ((uint8_t)led.fault() << 4));
  if (++pktN_ == MS_PER_PKT) flushPacket();
}

void Core::startRecording() { rec_ = true; pktN_ = 0; }
void Core::stopRecording() { flushPacket(); rec_ = false; recManual_ = false; }

// ---------------------------------------------------------------- 1 ms block
void Core::block(const uint16_t* raw) {
  t_++;
  uint16_t frames[FPM * N_ADC];
  uint32_t chs[N_RR] = {0};
  for (int f = 0; f < FPM; f++) {
    for (int c = 0; c < N_RR; c++) {
      uint32_t s = 0;
      for (int k = 0; k < SUB; k++) {
        uint16_t v = raw[(f * SUB + k) * N_RR + c];
        if (v & 0x8000) adcErr_++;
        s += v & 0x0FFF;
      }
      if (c < N_ADC) frames[f * N_ADC + c] = (uint16_t)s;
      chs[c] += s;
    }
  }
  const float n = (float)(FPM * SUB);
  // Drive sense: the last frame (0.2 ms) of the block, not the 1 ms mean. The board already
  // filters ADC_SENSE (1 ms RC); a further 0.5 ms of averaging lag puts the sense behind the
  // duty filter in CurrentEstimator, which overestimates the current when the duty steps down.
  for (int c = 0; c < N_ADC; c++) sense[c] = frames[(FPM - 1) * N_ADC + c] / (float)SUB * 3.3f / 4095.0f;
  float vt = chs[4] / n * 3.3f / 4095.0f;
  chipC_ = 27.0f - (vt - 0.706f) / 0.001721f;   // RP2040 datasheet §4.9.5
  for (int c = 0; c < N_RR; c++) acc_[c] += chs[c] / n;
  accN_++;

  sequencer();
  valve.tick(t_);
  led.tick(t_);

  // drive state changes -> events, and faults end a running shot
  if ((uint8_t)valve.state() != lastV_) {
    char b[96];
    snprintf(b, sizeof b, "\"st\":\"%s\",\"i\":%.3f,\"f\":\"%s\"", SolenoidDriver::stateText(valve.state()), valve.current(),
             faultCode(valve.fault()));
    event("v_state", b);
    lastV_ = (uint8_t)valve.state();
    if (valve.state() == SolenoidDriver::FAULT && ss_ == RUNNING) endShot("valve_fault", true);
  }
  if ((uint8_t)led.state() != lastL_) {
    char b[96];
    snprintf(b, sizeof b, "\"st\":\"%s\",\"i\":%.3f,\"f\":\"%s\"", LedDriver::stateText(led.state()), led.current(),
             faultCode(led.fault()));
    event("l_state", b);
    lastL_ = (uint8_t)led.state();
    if (led.state() == LedDriver::FAULT && ss_ == RUNNING) endShot("led_fault", true);
  }

  // signal LED: LFSR timecode while recording a shot, else manual
  if (tcOn_) {
    if ((int32_t)(t_ - tcNext_) >= 0) {
      if (lfsr_ == TC_SEED && tcNext_ == tGo_ + 1) {
        char b[64];
        snprintf(b, sizeof b, "\"cell\":%u,\"seed\":%u", shot_.tcMs, TC_SEED);
        event("tc_start", b);
      }
      pf_.syncLed(lfsr_ & 1);
      uint16_t fb = (uint16_t)(((lfsr_ >> 0) ^ (lfsr_ >> 4)) & 1);
      lfsr_ = (uint16_t)((lfsr_ >> 1) | (fb << 8));
      tcNext_ += shot_.tcMs;
    }
  } else if (syncManual_ == 2) {
    pf_.syncLed((t_ / 500) & 1);
  }

  if (rec_) recordMs(frames);

  // thermocouple (ADS1115, ~10 Hz) and cold junction (1 Hz), igniter board on I2C
  if (t_ % 100 == 17) { int16_t r; tcOk_ = pf_.tcRead(r); if (tcOk_) tcRaw_ = r; }
  if (t_ % 1000 == 523) { float c; cjOk_ = pf_.cjRead(c); if (cjOk_) cjC_ = c; }

  if (monMs_ && t_ - tStatus_ >= monMs_) { tStatus_ = t_; status(); }
}

// ---------------------------------------------------------------- sequencer
void Core::sequencer() {
  if (ss_ != RUNNING) return;
  uint32_t t = t_ - tGo_;
  const Shot& s = shot_;
  if (s.dip && s.led) {
    if (!dip1Down_ && t >= s.dip1) { led.setTarget(ledShotTarget_ * s.dipFrac); dip1Down_ = true; event("dip_start"); }
    if (dip1Down_ && !dip1Up_ && t >= s.dip1 + s.dipMs) { led.setTarget(ledShotTarget_); dip1Up_ = true; event("dip_end"); }
  }
  if (!fired_ && t >= s.fire) {
    fired_ = true;
    if (s.valve) {
      if (s.led && s.ledReq && led.state() != LedDriver::RUN) { endShot("led_not_ready", true); return; }
      if (valve.state() != SolenoidDriver::OFF) { endShot("valve_not_off", true); return; }
      holdSaved_ = holdCur_ = valve.config().holdA;
      valve.fire(t_);
      char b[96];
      snprintf(b, sizeof b, "\"peak\":%.3f,\"peakms\":%lu,\"hold\":%.3f", valve.config().peakA,
               (unsigned long)valve.config().peakMs, valve.config().holdA);
      event("fire", b);
    } else {
      event("fire", "\"skipped\":1");
    }
  }
  if (fired_ && !released_) {
    if (s.valve && s.ramp != 0 && valve.state() == SolenoidDriver::HOLD && t >= s.fire + s.rampAt) {
      holdCur_ += s.ramp * 0.001f;
      if (holdCur_ < s.holdMin) holdCur_ = s.holdMin;
      valve.setHoldA(holdCur_);
    }
    if (t >= s.fire + s.open) {
      released_ = true;
      tRel_ = t;
      bool active = valve.state() == SolenoidDriver::PEAK || valve.state() == SolenoidDriver::HOLD;
      if (s.valve && active) valve.release(t_);
      char b[64];
      snprintf(b, sizeof b, "\"hold\":%.3f,\"active\":%d", holdCur_, active ? 1 : 0);
      event("release", b);
    }
  }
  if (released_) {
    uint32_t tr = t - tRel_;
    if (s.dip && s.led) {
      if (!dip2Down_ && tr >= s.dip2) { led.setTarget(ledShotTarget_ * s.dipFrac); dip2Down_ = true; event("dip_start"); }
      if (dip2Down_ && !dip2Up_ && tr >= s.dip2 + s.dipMs) { led.setTarget(ledShotTarget_); dip2Up_ = true; event("dip_end"); }
    }
    if (tr >= s.post) endShot("done", false);
  }
}

void Core::endShot(const char* why, bool aborted) {
  if (valve.state() == SolenoidDriver::PEAK || valve.state() == SolenoidDriver::HOLD) valve.release(t_);
  if (fired_ && shot_.valve) valve.setHoldA(holdSaved_);
  if (ledOwned_) led.off(t_);
  if (shot_.led && shot_.ledA > 0) led.setTarget(ledPrevTarget_);
  else if (shot_.led) led.setTarget(ledShotTarget_);
  ledOwned_ = false;
  tcOn_ = false;
  pf_.syncLed(false);
  ss_ = IDLE;
  char b[64];
  snprintf(b, sizeof b, "\"why\":\"%s\",\"shot\":%lu", why, (unsigned long)shotCount_);
  event(aborted ? "abort" : "end", b);
  stopRecording();
}

void Core::button() {
  if (ss_ == RUNNING) { endShot("button", true); return; }
  if (valve.state() == SolenoidDriver::PEAK || valve.state() == SolenoidDriver::HOLD) valve.release(t_);
  led.off(t_);
  event("button");
}

// ---------------------------------------------------------------- status / info
void Core::status() {
  char b[520];
  float a[N_RR];
  for (int c = 0; c < N_RR; c++) a[c] = accN_ ? (float)(acc_[c] / accN_) : 0;
  for (int c = 0; c < N_RR; c++) acc_[c] = 0;
  accN_ = 0;
  char tc[48], cj[24];
  if (tcOk_) snprintf(tc, sizeof tc, "%d", tcRaw_); else snprintf(tc, sizeof tc, "null");
  if (cjOk_) snprintf(cj, sizeof cj, "%.2f", cjC_); else snprintf(cj, sizeof cj, "null");
  snprintf(b, sizeof b,
           "S,{\"t\":%lu,\"adc\":[%.1f,%.1f,%.1f,%.1f],\"chip\":%.1f,\"tc\":%s,\"cj\":%s,"
           "\"v\":{\"st\":\"%s\",\"i\":%.3f,\"d\":%.4f,\"r\":%.2f,\"hold\":%.3f,\"f\":\"%s\"},"
           "\"l\":{\"st\":\"%s\",\"i\":%.3f,\"tgt\":%.3f,\"d\":%.4f,\"f\":\"%s\"},"
           "\"shot\":\"%s\",\"rec\":%d,\"drops\":%lu,\"tdrops\":%lu,\"adc_err\":%lu}",
           (unsigned long)t_, a[0], a[1], a[2], a[3], chipC_, tc, cj,
           SolenoidDriver::stateText(valve.state()), valve.current(), vhw_.duty(), valve.resistance(), valve.config().holdA,
           faultCode(valve.fault()), LedDriver::stateText(led.state()), led.current(), led.target(), lhw_.duty(),
           faultCode(led.fault()), ss_ == RUNNING ? "running" : "idle", rec_ ? 1 : 0, (unsigned long)drops_,
           (unsigned long)textDrops_, (unsigned long)adcErr_);
  text(b);
}

void Core::info() {
  char b[1200];
  const SolenoidConfig& v = valve.config();
  const LedConfig& l = led.config();
  const Shot& s = shot_;
  snprintf(b, sizeof b,
           "I,{\"fw\":\"%s\",\"t\":%lu,\"fs\":%d,\"n_ch\":%d,\"fpm\":%d,\"sub\":%d,\"adc_bits\":12,\"vref\":3.3,"
           "\"tc_seed\":%u,\"sim\":%d,\"tc_dev\":%d,\"cj_dev\":%d,\"mon\":%lu,"
           "\"valve\":{\"pair\":%d,\"adc\":%d,\"r\":%.3f,\"l\":%.4f,\"peak\":%.3f,\"peakms\":%lu,\"hold\":%.3f,"
           "\"holdmax\":%.3f,\"maxon\":%lu,\"vbus\":%.2f,\"trip\":%.2f,\"xover\":%.1f,\"kp\":%.4f,\"ki\":%.2f},"
           "\"led\":{\"pair\":%d,\"adc\":%d,\"imax\":%.2f,\"trip\":%.2f,\"vbus\":%.2f,\"maxon\":%.1f,\"rearm\":%.1f,\"i\":%.3f},"
           "\"shot\":{\"led\":%d,\"led_a\":%.3f,\"led_req\":%d,\"valve\":%d,\"fire\":%lu,\"open\":%lu,\"post\":%lu,"
           "\"tc\":%d,\"tc_ms\":%u,\"dip\":%d,\"dip1\":%lu,\"dip2\":%lu,\"dip_ms\":%u,\"dip_frac\":%.3f,"
           "\"ramp\":%.3f,\"ramp_at\":%lu,\"hold_min\":%.3f}}",
           FW_VERSION, (unsigned long)t_, FPM * 1000, N_ADC, FPM, SUB, TC_SEED, pf_.isSim() ? 1 : 0, tcOk_ ? 1 : 0,
           cjOk_ ? 1 : 0, (unsigned long)monMs_, valvePair, valveAdc, v.rCoil, v.lCoil, v.peakA, (unsigned long)v.peakMs,
           v.holdA, v.holdDutyMax, (unsigned long)v.maxOnMs, v.vbus, v.tripA, v.crossoverHz, valve.gains().kp,
           valve.gains().ki, ledPair, ledAdc, l.iMax, l.tripA, l.vbus, l.maxOnS, l.rearmS, led.target(), s.led, s.ledA,
           s.ledReq, s.valve, (unsigned long)s.fire, (unsigned long)s.open, (unsigned long)s.post, s.tc, s.tcMs, s.dip,
           (unsigned long)s.dip1, (unsigned long)s.dip2, s.dipMs, s.dipFrac, s.ramp, (unsigned long)s.rampAt, s.holdMin);
  text(b);
}

// ---------------------------------------------------------------- commands
bool Core::setKey(const char* k, const char* v) {
  float f = (float)atof(v);
  long n = strtol(v, nullptr, 10);
  if (!strcmp(k, "mon")) { monMs_ = f > 0 ? (uint32_t)(1000.0f / f) : 0; return true; }
  if (!strncmp(k, "v.", 2)) {
    if (valve.state() != SolenoidDriver::OFF || ss_ != IDLE) return false;
    SolenoidConfig& c = valve.config();
    const char* q = k + 2;
    if (!strcmp(q, "pair") || !strcmp(q, "adc")) {
      int p = !strcmp(q, "pair") ? (int)n : valvePair, a = !strcmp(q, "adc") ? (int)n : valveAdc;
      if (p < 0 || p > 3 || a < 0 || a > 3) return false;
      if (p && p == ledPair) return false;
      if (p && ledPair && a == ledAdc) return false;
      if (!pf_.beginDrive(0, p, a, c.pwmHz)) return false;
      valvePair = p; valveAdc = a;
    } else if (!strcmp(q, "r") && f > 0) c.rCoil = f;
    else if (!strcmp(q, "l") && f > 0) c.lCoil = f;
    else if (!strcmp(q, "peak") && f > 0) c.peakA = f;
    else if (!strcmp(q, "peakms") && n > 0) c.peakMs = (uint32_t)n;
    else if (!strcmp(q, "hold") && f >= 0) c.holdA = f;
    else if (!strcmp(q, "holdmax") && f > 0 && f <= 1) c.holdDutyMax = f;
    else if (!strcmp(q, "maxon") && n > 0) c.maxOnMs = (uint32_t)n;
    else if (!strcmp(q, "vbus") && f > 15 && f < 30) c.vbus = f;
    else if (!strcmp(q, "trip") && f > 0) c.tripA = f;
    else if (!strcmp(q, "xover") && f > 1 && f < 100) c.crossoverHz = f;
    else return false;
    valve.applyConfig();
    return true;
  }
  if (!strncmp(k, "l.", 2)) {
    const char* q = k + 2;
    if (!strcmp(q, "i")) { led.setTarget(f); return true; }
    bool lOff = led.state() == LedDriver::OFF || led.state() == LedDriver::FAULT;
    if (!lOff || ss_ != IDLE) return false;
    LedConfig& c = led.config();
    if (!strcmp(q, "pair") || !strcmp(q, "adc")) {
      int p = !strcmp(q, "pair") ? (int)n : ledPair, a = !strcmp(q, "adc") ? (int)n : ledAdc;
      if (p < 0 || p > 3 || a < 0 || a > 3) return false;
      if (p && p == valvePair) return false;
      if (p && valvePair && a == valveAdc) return false;
      if (!pf_.beginDrive(1, p, a, c.pwmHz)) return false;
      ledPair = p; ledAdc = a;
    } else if (!strcmp(q, "imax") && f > 0) c.iMax = f;
    else if (!strcmp(q, "trip") && f > 0) c.tripA = f;
    else if (!strcmp(q, "vbus") && f > 15 && f < 30) c.vbus = f;
    else if (!strcmp(q, "maxon") && f >= 0) c.maxOnS = f;
    else if (!strcmp(q, "rearm") && f >= 0) c.rearmS = f;
    else return false;
    led.applyConfig();
    return true;
  }
  return false;
}

bool Core::shotKey(const char* k, const char* v) {
  float f = (float)atof(v);
  long n = strtol(v, nullptr, 10);
  Shot& s = shot_;
  if (!strcmp(k, "led")) s.led = n ? 1 : 0;
  else if (!strcmp(k, "led_a") && f >= 0) s.ledA = f;
  else if (!strcmp(k, "led_req")) s.ledReq = n ? 1 : 0;
  else if (!strcmp(k, "valve")) s.valve = n ? 1 : 0;
  else if (!strcmp(k, "fire") && n >= 0 && n < 60000) s.fire = (uint32_t)n;
  else if (!strcmp(k, "open") && n > 0 && n < 60000) s.open = (uint32_t)n;
  else if (!strcmp(k, "post") && n >= 0 && n < 60000) s.post = (uint32_t)n;
  else if (!strcmp(k, "tc")) s.tc = n ? 1 : 0;
  else if (!strcmp(k, "tc_ms") && n >= 1 && n <= 1000) s.tcMs = (uint16_t)n;
  else if (!strcmp(k, "dip")) s.dip = n ? 1 : 0;
  else if (!strcmp(k, "dip1") && n >= 0) s.dip1 = (uint32_t)n;
  else if (!strcmp(k, "dip2") && n >= 0) s.dip2 = (uint32_t)n;
  else if (!strcmp(k, "dip_ms") && n > 0 && n < 1000) s.dipMs = (uint16_t)n;
  else if (!strcmp(k, "dip_frac") && f > 0 && f < 1) s.dipFrac = f;
  else if (!strcmp(k, "ramp")) s.ramp = f;
  else if (!strcmp(k, "ramp_at") && n >= 0) s.rampAt = (uint32_t)n;
  else if (!strcmp(k, "hold_min") && f >= 0) s.holdMin = f;
  else return false;
  return true;
}

void Core::line(const char* in) {
  char buf[240];
  size_t n = strlen(in);
  if (n >= sizeof buf) { text("ERR line too long"); return; }
  for (size_t k = 0; k <= n; k++) buf[k] = (char)tolower((unsigned char)in[k]);
  while (n && (buf[n - 1] == ' ' || buf[n - 1] == '\r')) buf[--n] = 0;
  char* s = buf;
  while (*s == ' ') s++;
  if (!*s) return;
  char* rest = strchr(s, ' ');
  if (rest) { *rest++ = 0; while (*rest == ' ') rest++; }
  char reply[320];

  if (!strcmp(s, "?")) { info(); return; }
  if (!strcmp(s, "ping")) { text("OK ping"); return; }
  if (!strcmp(s, "clear")) { valve.clearFault(); led.clearFault(); text("OK clear"); return; }
  if (!strcmp(s, "abort")) {
    if (ss_ == RUNNING) endShot("host", true);
    else {
      if (valve.state() == SolenoidDriver::PEAK || valve.state() == SolenoidDriver::HOLD) valve.release(t_);
      led.off(t_);
      if (rec_) stopRecording();
    }
    text("OK abort");
    return;
  }
  if (!strcmp(s, "go")) {
    const char* err = nullptr;
    if (ss_ == RUNNING) err = "busy";
    else if (valve.state() == SolenoidDriver::FAULT || led.state() == LedDriver::FAULT) err = "fault (clear first)";
    else if (shot_.valve && valvePair == 0) err = "no valve board (v.pair=0)";
    else if (shot_.led && ledPair == 0) err = "no LED board (l.pair=0)";
    else if (shot_.valve && valve.state() != SolenoidDriver::OFF) err = "valve not off";
    if (err) { snprintf(reply, sizeof reply, "ERR go %s", err); text(reply); return; }
    shotCount_++;
    ss_ = RUNNING;
    tGo_ = t_;
    fired_ = released_ = false;
    dip1Down_ = dip1Up_ = dip2Down_ = dip2Up_ = false;
    ledPrevTarget_ = led.target();
    ledOwned_ = false;
    if (shot_.led) {
      if (shot_.ledA > 0) led.setTarget(shot_.ledA);
      ledShotTarget_ = led.target();
      if (led.state() == LedDriver::OFF || led.state() == LedDriver::STOPPING) { led.on(t_); ledOwned_ = true; }
    }
    if (rec_) stopRecording();
    startRecording();
    if (shot_.tc) { tcOn_ = true; lfsr_ = TC_SEED; tcNext_ = t_ + 1; }
    char b[200];
    snprintf(b, sizeof b, "\"shot\":%lu,\"fire\":%lu,\"open\":%lu,\"post\":%lu,\"valve\":%d,\"led\":%d,\"led_a\":%.3f,\"tc\":%d,\"tc_ms\":%u,\"dip\":%d",
             (unsigned long)shotCount_, (unsigned long)shot_.fire, (unsigned long)shot_.open, (unsigned long)shot_.post,
             shot_.valve, shot_.led, ledShotTarget_, shot_.tc, shot_.tcMs, shot_.dip);
    event("go", b);
    text("OK go");
    return;
  }
  if (!strcmp(s, "led")) {
    if (!rest) { text("ERR led on|off|i <A>"); return; }
    char* arg = strchr(rest, ' ');
    if (arg) { *arg++ = 0; }
    if (ledPair == 0) { text("ERR led no LED board (l.pair=0)"); return; }
    if (!strcmp(rest, "on")) { if (arg) led.setTarget((float)atof(arg)); led.on(t_); text("OK led"); return; }
    if (!strcmp(rest, "off")) { if (ss_ == RUNNING) { text("ERR led shot running"); return; } led.off(t_); text("OK led"); return; }
    if (!strcmp(rest, "i") && arg) { led.setTarget((float)atof(arg)); text("OK led"); return; }
    text("ERR led on|off|i <A>");
    return;
  }
  if (!strcmp(s, "rec")) {
    if (ss_ == RUNNING) { text("ERR rec shot running"); return; }
    if (rest && !strcmp(rest, "on")) { startRecording(); recManual_ = true; event("rec_on"); }
    else { stopRecording(); event("rec_off"); }
    text("OK rec");
    return;
  }
  if (!strcmp(s, "sync")) {
    if (ss_ == RUNNING) { text("ERR sync shot running"); return; }
    syncManual_ = (rest && !strcmp(rest, "on")) ? 1 : (rest && !strcmp(rest, "blink")) ? 2 : 0;
    pf_.syncLed(syncManual_ == 1);
    text("OK sync");
    return;
  }
  if (!strcmp(s, "sim")) {
    if (!pf_.isSim()) { text("ERR sim not a simulator"); return; }
    pf_.simCommand(rest ? rest : "");
    text("OK sim");
    return;
  }
  if (!strcmp(s, "set") || !strcmp(s, "shot")) {
    bool isSet = !strcmp(s, "set");
    if (!isSet && ss_ == RUNNING) { text("ERR shot running"); return; }
    char* p = rest;
    while (p && *p) {
      char* sp = strchr(p, ' ');
      if (sp) *sp = 0;
      char* eq = strchr(p, '=');
      if (!eq) { snprintf(reply, sizeof reply, "ERR %s expected key=value: %s", s, p); text(reply); return; }
      *eq = 0;
      bool ok = isSet ? setKey(p, eq + 1) : shotKey(p, eq + 1);
      if (!ok) { snprintf(reply, sizeof reply, "ERR %s %s=%s (bad key/value, or drive not off)", s, p, eq + 1); text(reply); return; }
      p = sp ? sp + 1 : nullptr;
      while (p && *p == ' ') p++;
    }
    snprintf(reply, sizeof reply, "OK %s", s);
    text(reply);
    return;
  }
  snprintf(reply, sizeof reply, "ERR unknown %s", s);
  text(reply);
}

}  // namespace rig
