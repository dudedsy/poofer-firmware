/*
  solenoid_drive.ino - bench exercise for poofer::SolenoidDriver (PooferCommon).

  Closed-loop peak and hold on one solenoid driver board (Rev A): peak and hold are current
  targets regulated from the board's current sense, release follows solenoid spec Rev A §9
  (FIRE low, PWM 100 % for 20 ms, then PWM low). Reports coil current and the coil
  resistance estimate during hold.

  BENCH ONLY: this opens the valve. Gas closed at the upstream manual valve, nothing
  connected to a burner.

  Toolchain : Earle Philhower RP2040 core 6.3.0, board "Waveshare RP2040 Zero"
  Serial    : USB, line commands, 'help' lists them.
*/

#include <Arduino.h>
#include <poofer_board.h>
#include <poofer_drive.h>
#include <poofer_drive_pico.h>

using namespace poofer;

static PicoDriveHw hw;
static SolenoidDriver sol(hw);
static int g_pair = 1, g_adc = 0;
static uint32_t g_releaseAt = 0;          // auto release time (ms), 0 = none
static bool g_log = false;
static uint32_t g_logMs = 5, g_tLog = 0;

static void status() {
  const SolenoidConfig& c = sol.config();
  Serial.printf("state %s  coil %.3f A  sense %.3f V  duty %.3f  R est %.2f ohm\n",
                SolenoidDriver::stateText(sol.state()), sol.current(), sol.senseVolts(), hw.duty(), sol.resistance());
  Serial.printf("pair %d (FIRE GP%d, PWM GP%d)  adc %d (GP%d)  R %.2f ohm  L %.3f H  peak %.2f A %lu ms  hold %.2f A (duty max %.2f)  maxon %lu ms  trip %.2f A  vbus %.1f\n",
                g_pair, hw.firePin(), hw.pwmPin(), g_adc, hw.adcPin(), c.rCoil, c.lCoil, c.peakA,
                (unsigned long)c.peakMs, c.holdA, c.holdDutyMax, (unsigned long)c.maxOnMs, c.tripA, c.vbus);
  Serial.printf("gains kp %.3f ki %.1f (from R, L, %.0f Hz crossover)\n", sol.gains().kp, sol.gains().ki, c.crossoverHz);
  if (sol.state() == SolenoidDriver::FAULT) Serial.printf("fault: %s\n", faultText(sol.fault()));
}

static void help() {
  Serial.println(
    "commands (one per line). BENCH ONLY, gas closed:\n"
    "  fire <ms>      fire for <ms> (peak, hold, then spec release). fire 0 = until release/maxon\n"
    "  release        release now\n"
    "  s              status\n"
    "  log <ms>       CSV every <ms>: t_ms,state,coil_A,sense_V,duty,R_est. log 0 stops\n"
    "  clear          clear a latched fault\n"
    "  peak <A>       peak target (<= 2.8)       peakms <ms>  peak time (<= 250)\n"
    "  hold <A>       hold target                holdmax <d>  hold duty ceiling (0.5 for 12 V coils)\n"
    "  r <ohm>        coil resistance            l <H>        coil inductance estimate\n"
    "  maxon <ms>     firmware on-time limit (keep < T_max / 2)\n"
    "  vbus <V>       assumed bus voltage        pair <1-3> / adc <0-3>  (while off)\n"
    "  help");
}

static void command(char* s) {
  while (*s == ' ') s++;
  char* arg = strchr(s, ' ');
  float v = 0;
  bool has = false;
  if (arg) { *arg++ = 0; while (*arg == ' ') arg++; if (*arg) { v = atof(arg); has = true; } }
  for (char* p = s; *p; p++) *p = tolower(*p);
  uint32_t now = millis();
  SolenoidConfig& c = sol.config();
  bool off = sol.state() == SolenoidDriver::OFF || sol.state() == SolenoidDriver::FAULT;

  if (!strcmp(s, "fire")) {
    if (sol.state() != SolenoidDriver::OFF) { Serial.println("not OFF (clear a fault first)"); return; }
    sol.fire(now);
    g_releaseAt = (has && v > 0) ? now + (uint32_t)v : 0;
  } else if (!strcmp(s, "release")) {
    sol.release(now);
  } else if (!strcmp(s, "s") || !strcmp(s, "status")) {
    status();
  } else if (!strcmp(s, "log")) {
    g_logMs = has ? (uint32_t)v : 5;
    g_log = g_logMs > 0;
    if (g_log) Serial.println("t_ms,state,coil_A,sense_V,duty,R_est");
  } else if (!strcmp(s, "clear")) {
    sol.clearFault();
  } else if (!strcmp(s, "hold") && has && v >= 0) {
    sol.setHoldA(v);
  } else if (!off) {
    Serial.println("release first");
  } else if (!strcmp(s, "peak") && has) { c.peakA = v; sol.applyConfig(); }
  else if (!strcmp(s, "peakms") && has) { c.peakMs = (uint32_t)v; sol.applyConfig(); }
  else if (!strcmp(s, "holdmax") && has && v > 0) { c.holdDutyMax = v; sol.applyConfig(); }
  else if (!strcmp(s, "r") && has && v > 0) { c.rCoil = v; sol.applyConfig(); }
  else if (!strcmp(s, "l") && has && v > 0) { c.lCoil = v; sol.applyConfig(); }
  else if (!strcmp(s, "maxon") && has && v >= 0) { c.maxOnMs = (uint32_t)v; }
  else if (!strcmp(s, "vbus") && has && v > 15 && v < 30) { c.vbus = v; sol.applyConfig(); }
  else if ((!strcmp(s, "pair") || !strcmp(s, "adc")) && has) {
    int np = !strcmp(s, "pair") ? (int)v : g_pair, na = !strcmp(s, "adc") ? (int)v : g_adc;
    if (!hw.begin(np, na, c.pwmHz)) { Serial.println("pair 1-3, adc 0-3"); hw.begin(g_pair, g_adc, c.pwmHz); return; }
    g_pair = np; g_adc = na;
    sol.applyConfig();
    status();
  } else if (!strcmp(s, "help") || !strcmp(s, "?") || !strcmp(s, "h")) {
    help();
  } else {
    Serial.printf("? '%s' (help lists commands)\n", s);
  }
}

static void pollSerial() {
  static char line[64];
  static int n = 0;
  while (Serial.available()) {
    int ch = Serial.read();
    if (ch == '\r' || ch == '\n') {
      if (n) { line[n] = 0; command(line); n = 0; }
    } else if (n < (int)sizeof line - 1) {
      line[n++] = (char)ch;
    }
  }
}

void setup() {
  PicoDriveHw::allChannelsLow();
  SolenoidConfig cfg;
  hw.begin(g_pair, g_adc, cfg.pwmHz);
  sol.begin(cfg);
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 1500) {}
  Serial.println("\nsolenoid_drive: closed-loop peak and hold (PooferCommon SolenoidDriver). BENCH ONLY, gas closed.");
  help();
  status();
}

void loop() {
  static uint32_t next = micros();
  static SolenoidDriver::State last = SolenoidDriver::OFF;
  uint32_t now = micros();
  if ((int32_t)(now - next) >= 0) {
    next += 1000;
    if ((int32_t)(now - next) > 5000) next = now + 1000;
    uint32_t ms = millis();
    if (g_releaseAt && (int32_t)(ms - g_releaseAt) >= 0) { sol.release(ms); g_releaseAt = 0; }
    sol.tick(ms);
    if (sol.state() != last) {
      Serial.printf("%lu: %s -> %s\n", (unsigned long)ms, SolenoidDriver::stateText(last), SolenoidDriver::stateText(sol.state()));
      if (sol.state() == SolenoidDriver::FAULT) Serial.printf("fault: %s\n", faultText(sol.fault()));
      last = sol.state();
    }
    if (g_log && sol.state() != SolenoidDriver::OFF && ms - g_tLog >= g_logMs) {
      g_tLog = ms;
      Serial.printf("%lu,%s,%.3f,%.3f,%.3f,%.2f\n", (unsigned long)ms, SolenoidDriver::stateText(sol.state()),
                    sol.current(), sol.senseVolts(), hw.duty(), sol.resistance());
    }
  }
  pollSerial();
}
