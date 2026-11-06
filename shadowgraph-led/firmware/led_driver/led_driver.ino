/*
  led_driver.ino

  Current-regulated LED drive for the cold-test shadowgraph, using a stock solenoid
  driver board (Rev A) on the DMX base board (Rev A) as a buck converter:
    J1 pin 1 (VALVE_HI) -> inductor (Bourns 2200LL-221-H-RC) -> LED+
    470 uF 50 V electrolytic(s) across the LED, + at LED+
    LED- -> J1 pin 2 (VALVE_LOW)
  The control logic (feed-forward + PI on the board's current sense, soft start, faults,
  hardware-limiter re-arm) is poofer::LedDriver in PooferCommon (poofer_drive.h); this
  sketch is the serial and button front end. See ../../README.md.

  Toolchain : Arduino IDE / arduino-cli, Earle Philhower "Raspberry Pi Pico/RP2040" core
              6.3.0, Tools > Board: "Waveshare RP2040 Zero"
  Library   : PooferCommon (this repo, libraries/PooferCommon; see the top README)
  Serial    : USB, any baud. Line commands; 'help' lists them.
  Buttons   : UI board (if fitted) UP / DN = +/-0.1 A, SET = on/off.

  Bench / test rig only. Never on a solenoid board that drives a gas valve.
*/

#include <Arduino.h>
#include <poofer_board.h>
#include <poofer_drive.h>
#include <poofer_drive_pico.h>

using namespace poofer;

static PicoDriveHw hw;
static LedDriver led(hw);
static int g_pair = 1, g_adc = 0;
static bool g_log = false;
static uint32_t g_logMs = 100, g_tLog = 0;

static bool idle() { return led.state() == LedDriver::OFF || led.state() == LedDriver::FAULT; }

static void status() {
  const LedConfig& c = led.config();
  Serial.printf("state %s  target %.3f A  led %.3f A  sense %.3f V  duty %.4f  ff %.4f  max %.3f\n",
                LedDriver::stateText(led.state()), led.target(), led.current(), led.senseVolts(),
                hw.duty(), led.dutyFF(), led.dutyMax());
  Serial.printf("pair %d (FIRE GP%d, PWM GP%d)  adc %d (GP%d)  pwm %lu Hz (%lu steps)  vbus %.1f  imax %.2f  trip %.2f  maxon %.0f s  rearm %.1f s\n",
                g_pair, hw.firePin(), hw.pwmPin(), g_adc, hw.adcPin(), (unsigned long)c.pwmHz,
                (unsigned long)hw.pwmSteps(), c.vbus, c.iMax, c.tripA, c.maxOnS, c.rearmS);
  Serial.printf("kp %.4f  ki %.3f  ref rate %.1f A/s  ramp %.2f/s\n", c.gains.kp, c.gains.ki, c.gains.refRate, c.rampPerS);
  if (led.state() == LedDriver::FAULT) Serial.printf("fault: %s\n", faultText(led.fault()));
}

static void help() {
  Serial.println(
    "commands (one per line):\n"
    "  i <A>        set target and turn on (i 0 = off). Capped at imax\n"
    "  on / off     on at the current target / off\n"
    "  s            status\n"
    "  log <ms>     CSV every <ms>: t_ms,state,target_A,led_A,sense_V,duty. log 0 stops\n"
    "  clear        clear a latched fault\n"
    "  pair <1-3>   JP1 row (while off)\n"
    "  adc <0-3>    JP2 ADC line (while off)\n"
    "  pwm <Hz>     PWM frequency 20000-100000 (while off)\n"
    "  vbus <V>     assumed bus voltage for feed-forward\n"
    "  imax <A>     highest allowed target (<= 2.4)\n"
    "  trip <A>     software over-current trip\n"
    "  maxon <s>    firmware on-time limit, 0 = none\n"
    "  rearm <s>    re-arm the hardware limiter every <s> while on (LED blinks off), 0 = off\n"
    "  kp / ki / rate / ramp <x>   loop tuning\n"
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
  LedConfig& c = led.config();

  if (!strcmp(s, "i") && has) {
    if (v <= 0) { led.off(now); Serial.println("off"); return; }
    if (v > c.iMax) Serial.printf("capped at imax %.2f A\n", c.iMax);
    led.setTarget(v);
    if (led.state() == LedDriver::OFF) led.on(now);
    Serial.printf("target %.3f A\n", led.target());
  } else if (!strcmp(s, "on")) {
    if (led.state() == LedDriver::FAULT) Serial.println("fault latched: 'clear' first");
    else led.on(now);
  } else if (!strcmp(s, "off")) {
    led.off(now);
  } else if (!strcmp(s, "s") || !strcmp(s, "status")) {
    status();
  } else if (!strcmp(s, "log")) {
    g_logMs = has ? (uint32_t)v : 100;
    g_log = g_logMs > 0;
    if (g_log) Serial.println("t_ms,state,target_A,led_A,sense_V,duty");
  } else if (!strcmp(s, "clear")) {
    led.clearFault();
    Serial.println("cleared");
  } else if ((!strcmp(s, "pair") || !strcmp(s, "adc") || !strcmp(s, "pwm")) && has) {
    if (!idle()) { Serial.println("turn off first"); return; }
    int np = g_pair, na = g_adc;
    if (!strcmp(s, "pair")) np = (int)v;
    if (!strcmp(s, "adc")) na = (int)v;
    if (!strcmp(s, "pwm")) {
      if (v < 20000 || v > 100000) { Serial.println("pwm 20000-100000"); return; }
      c.pwmHz = (uint32_t)v;
    }
    if (!hw.begin(np, na, c.pwmHz)) { Serial.println("pair 1-3, adc 0-3"); hw.begin(g_pair, g_adc, c.pwmHz); return; }
    g_pair = np; g_adc = na;
    led.applyConfig();
    status();
  } else if (!strcmp(s, "vbus") && has && v > 15 && v < 30) {
    c.vbus = v; led.applyConfig();
  } else if (!strcmp(s, "imax") && has && v > 0 && v <= 2.4f) {
    c.iMax = v; led.applyConfig(); led.setTarget(led.target());
  } else if (!strcmp(s, "trip") && has && v > 0 && v <= 3.0f) {
    c.tripA = v;
  } else if (!strcmp(s, "maxon") && has && v >= 0) {
    c.maxOnS = v;
  } else if (!strcmp(s, "rearm") && has && v >= 0) {
    c.rearmS = v;
  } else if (!strcmp(s, "kp") && has && v >= 0) {
    c.gains.kp = v;
  } else if (!strcmp(s, "ki") && has && v >= 0) {
    c.gains.ki = v;
  } else if (!strcmp(s, "rate") && has && v > 0) {
    c.gains.refRate = v;
  } else if (!strcmp(s, "ramp") && has && v > 0) {
    c.rampPerS = v;
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

static void pollButtons() {
  static uint8_t stable[3] = {1, 1, 1}, raw[3] = {1, 1, 1};
  static uint32_t tChange[3] = {0, 0, 0};
  uint32_t now = millis();
  for (int i = 0; i < 3; i++) {
    uint8_t r = digitalRead(PIN_BTN[i]);
    if (r != raw[i]) { raw[i] = r; tChange[i] = now; }
    if (r == stable[i] || now - tChange[i] <= 30) continue;
    stable[i] = r;
    if (r != 0) continue;                                // act on press only
    if (i < 2) {
      float t = led.target() + (i == 0 ? 0.1f : -0.1f);
      led.setTarget(t < 0.1f ? 0.1f : t);
      Serial.printf("target %.3f A\n", led.target());
    } else if (led.state() == LedDriver::RAMP || led.state() == LedDriver::RUN) {
      led.off(now);
    } else {
      led.on(now);
    }
  }
}

void setup() {
  PicoDriveHw::allChannelsLow();                         // every gate line low first
  for (uint8_t p : PIN_BTN) pinMode(p, INPUT_PULLUP);
  LedConfig cfg;
  hw.begin(g_pair, g_adc, cfg.pwmHz);
  led.begin(cfg);
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 1500) {}
  Serial.println("\nled_driver: shadowgraph LED current control (PooferCommon LedDriver)");
  help();
  status();
}

void loop() {
  static uint32_t next = micros();
  uint32_t now = micros();
  if ((int32_t)(now - next) >= 0) {
    next += 1000;
    if ((int32_t)(now - next) > 5000) next = now + 1000;   // fell behind (USB stall): resync
    uint32_t ms = millis();
    led.tick(ms);
    if (g_log && ms - g_tLog >= g_logMs) {
      g_tLog = ms;
      Serial.printf("%lu,%s,%.3f,%.3f,%.3f,%.4f\n", (unsigned long)ms, LedDriver::stateText(led.state()),
                    led.target(), led.current(), led.senseVolts(), hw.duty());
    }
  }
  pollSerial();
  pollButtons();
}
