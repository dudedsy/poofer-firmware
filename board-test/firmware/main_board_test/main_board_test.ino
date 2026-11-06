/*
  main_board_test.ino

  Bring-up test for the DMX base board (Rev A) and the UI lid board (Rev A), plus
  the solenoid driver and igniter (spark) daughterboards when stacked.
  Works with: base board alone; base + UI board without OLED; base + UI + OLED;
  on USB power alone or with 24 V. On USB alone the +5V rail (U1 buck) is dead
  by design (D2 blocks the module's VBUS from back-feeding it), so UI LEDs,
  OLED VBAT and U3 are unpowered: those checks run electrically but the visual
  parts are skipped, and DMX receive is skipped.

  DMX receive uses the RP2040's UART0 RX on GP1 (U3 RO) at 250 kbaud 8N2, with
  break detection, not the PIO receiver planned for production firmware. It
  proves the hardware path: J1, D3, U3 and its isoPower supply (L1, L2), R40.
  Source: board-test/host/artnet_send.py on the Pi, via the CR011R node.

  Toolchain : Arduino IDE / arduino-cli, Earle Philhower "Raspberry Pi Pico/RP2040" core
              (Tools > Board: "Waveshare RP2040 Zero")
  Library   : Adafruit NeoPixel (for the on-module WS2812 on GP16)
  Output    : USB serial, 115200 (baud is ignored by USB CDC). Status also on the
              WS2812 and, if present, the UI RGB LEDs.

  WARNING: the full run DRIVES GP6-GP11 (J3 gate lines) and, with a solenoid
  board detected, fires its valve. Bench only: gas closed, nothing connected to
  a burner. An igniter wired to the selected channel will spark during the CH walk.

  Sequence (send 'a' to rerun, 'h' for the menu):
    1  boot info
    2  WS2812 colour cycle
    3  +5V rail inference (via +3V3_DMX on R38/R39) and DMX DE pulldown R37
    4  I2C line check (stuck / missing pull-ups) and bus recovery
    5  raw boot scan (before touching the UI board)
    6  FRAM: WP pull-up, WP effectiveness, scratch write/readback, restore
    7  UI board: TCA9555 power-on state, init, register identity, 12-line LED walk with readback
    8  OLED: reset release, VBAT on, init (internal IREF), test patterns
    9  full scan + expected/possible address report + stack device identity
   10  buttons: idle level, then press test (only if UI board present)
   11  ADC0-3 readings
   12  SPARE1 pull population
   13  CH1-CH6: pulldowns, pin-to-pin isolation, slow walk
   13a DMX receive, 2 s listen on J1 (24 V only). No source = INFO, not a fault.
       Pattern frames from artnet_send.py are checked slot by slot.
   14  igniter board, if detected at 0x48-0x4F: chip IDs, cold junction, thermocouple
       (output not toggled; the CH walk covers it)
   15  solenoid board, if an ADC line is held low by ADC_SENSE (needs the JP2 shunt):
       T1-T3 and T5 on JP1 pair 1, 24 V coil, no T_max; skipped without 24 V
   16  summary

  Menu commands run single tests; 'b', 'g' and 'v' run interactively from the
  menu even when their board was not detected. 'h' lists all commands.
*/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <stdarg.h>
#include <initializer_list>
#include "hardware/gpio.h"
#include "hardware/watchdog.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "hardware/sync.h"
#include "hardware/timer.h"
#include "pico/unique_id.h"
#include <poofer_board.h>    // PooferCommon library: base board pin map, I2C addresses
#include <poofer_dmx_rx.h>   // PooferCommon library: DMX parser

// ---------------------------------------------------------------- config
// WS2812 colour order. The "RED" step of the colour cycle should look red.
// If it looks green, change NEO_GRB to NEO_RGB.
#define WS2812_ORDER NEO_GRB
// SSD1315 IREF select (ADh). R11 is DNP on the built UI boards, so internal
// IREF is required or the panel stays dark. 0x30 = internal, 30 uA (max ISEG
// 240 uA). 0x10 = internal, 19 uA (dimmer, longer life). Per SSD1315 Rev 1.0.
#define OLED_IREF 0x30
#define I2C_SCAN_HZ 100000
#define FRAM_TEST_ADDR 0x1FF0   // last 16 bytes of the 8 KB part; restored after test
#define FRAM_TEST_LEN 16
#define OLED_IDLE_MS (5UL * 60UL * 1000UL)   // blank the OLED after 5 min untouched

// ---------------------------------------------------------------- pin map
// Pins, connector pins and I2C addresses come from PooferCommon's poofer_board.h.
// Refdes strings below are only for this test's fault messages.
static const char*   BTN_NAME[3] = {"UP", "DN", "SET"};
static const char*   BTN_R[3] = {"R34", "R35", "R36"};
static const char*   BTN_C[3] = {"C18", "C19", "C20"};

// TCA9555 bit positions (16-bit: P0.x = bit x, P1.x = bit 8+x)
#define TCA_VBAT_EN  (1u << 6)   // P0.6, TSSOP-24 pin 10
#define TCA_OLED_RES (1u << 7)   // P0.7, TSSOP-24 pin 11
#define TCA_LED_MASK 0x3F3Fu     // P0.0-P0.5, P1.0-P1.5
#define TCA_OUT_MASK 0x3FFFu     // every pin configured as output
static const uint8_t LED_BIT[4][3] = {{0, 1, 2}, {3, 4, 5}, {8, 9, 10}, {11, 12, 13}};

Adafruit_NeoPixel px(1, PIN_WS2812, WS2812_ORDER + NEO_KHZ800);

// ---------------------------------------------------------------- results
enum St : uint8_t { PASS, WARN, FAIL, INFO, SKIP };
enum Grp : uint8_t { G_BASE, G_UI, G_OLED, G_STACK, G_N };
static const char* ST_TXT[] = {"PASS", "WARN", "FAIL", "INFO", "SKIP"};
static const char* GRP_TXT[] = {"base", "ui", "oled", "stack"};
struct Result { Grp g; St st; char name[24]; char detail[200]; };
static Result R[96];
static int nR = 0;

static void rec(Grp g, St st, const char* name, const char* fmt, ...) {
  char buf[200];
  va_list ap; va_start(ap, fmt); vsnprintf(buf, sizeof buf, fmt, ap); va_end(ap);
  if (nR < (int)(sizeof R / sizeof R[0])) {
    R[nR].g = g; R[nR].st = st;
    strncpy(R[nR].name, name, sizeof R[nR].name - 1); R[nR].name[sizeof R[nR].name - 1] = 0;
    strncpy(R[nR].detail, buf, sizeof R[nR].detail - 1); R[nR].detail[sizeof R[nR].detail - 1] = 0;
    nR++;
  }
  Serial.printf("  [%s] %-18s %s\n", ST_TXT[st], name, buf);
}

// -1 = nothing graded, else PASS / WARN / FAIL
static int worst(int g) {
  int w = -1;
  for (int i = 0; i < nR; i++) {
    if (g >= 0 && R[i].g != g) continue;
    St s = R[i].st;
    if (s == FAIL) w = FAIL;
    else if (s == WARN && w != FAIL) w = WARN;
    else if (s == PASS && w == -1) w = PASS;
  }
  return w;
}

// ---------------------------------------------------------------- state
static bool g_wire = false;
static uint32_t g_i2cHz = 400000;
static bool g_5v = false;            // +5V rail inferred present (24 V in, U1 alive)
static bool g_uiPresent = false, g_uiInit = false;
static int  g_oledFound = -1;        // -1 untested, 0 not found, 1 found
static bool g_oledOn = false;
static uint32_t g_oledOnAt = 0;
static uint16_t g_tcaOut = 0xFF3F;
static bool ackBoot[128], ackNow[128];

// ---------------------------------------------------------------- serial helpers
// True only for a real keypress. CR, LF and spaces are swallowed, so the line ending
// the IDE sends after the command letter does not count as "any key".
static bool keyHit() {
  bool hit = false;
  while (Serial.available()) {
    int c = Serial.read();
    if (c != '\r' && c != '\n' && c != ' ') hit = true;
  }
  return hit;
}
static void hdr(const char* t) { Serial.printf("\n--- %s\n", t); }

// ---------------------------------------------------------------- I2C helpers
static void wireStop() { if (g_wire) { Wire.end(); g_wire = false; } }
static void wireStart(uint32_t hz) {
  wireStop();
  Wire.setSDA(PIN_SDA);
  Wire.setSCL(PIN_SCL);
  Wire.begin();
  Wire.setClock(hz);
  Wire.setTimeout(25);
  g_wire = true;
}
static bool probe(uint8_t a) { Wire.beginTransmission(a); return Wire.endTransmission() == 0; }
static bool regRead(uint8_t a, uint8_t reg, uint8_t* b, size_t n) {
  Wire.beginTransmission(a); Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(a, n) != n) return false;
  for (size_t i = 0; i < n; i++) b[i] = Wire.read();
  return true;
}
static bool regRead16be(uint8_t a, uint8_t reg, uint16_t* v) {
  uint8_t b[2]; if (!regRead(a, reg, b, 2)) return false;
  *v = ((uint16_t)b[0] << 8) | b[1]; return true;
}
static void scan(bool* ack) {
  for (int a = 0; a < 128; a++) ack[a] = false;
  for (int a = 0x08; a <= 0x77; a++) ack[a] = probe(a);
}
static void printAcks(const char* label, const bool* ack) {
  Serial.printf("  %s:", label);
  int n = 0;
  for (int a = 0; a < 128; a++) if (ack[a]) { Serial.printf(" 0x%02X", a); n++; }
  if (!n) Serial.print(" (none)");
  Serial.println();
}

// open-drain bit-bang helpers (Wire must be stopped)
static void odLow(uint8_t p) { gpio_put(p, 0); gpio_set_dir(p, GPIO_OUT); }
static void odRel(uint8_t p) { gpio_set_dir(p, GPIO_IN); }

// ---------------------------------------------------------------- 1 boot info
static void bootInfo() {
  hdr("Boot info");
  char id[2 * PICO_UNIQUE_BOARD_ID_SIZE_BYTES + 1];
  pico_get_unique_board_id_string(id, sizeof id);
  Serial.printf("  Firmware   : main_board_test, built %s %s\n", __DATE__, __TIME__);
  Serial.printf("  Module UID : %s  (flash unique ID; use it to label results per board)\n", id);
  Serial.printf("  Reset cause: %s\n", watchdog_caused_reboot() ? "watchdog" : "power-on / reset key / USB reflash");
  Serial.printf("  Core temp  : %.1f C (RP2040 internal sensor, +/- several C)\n", analogReadTemp());
  Serial.printf("  I2C clock  : scan %lu Hz, functional tests %lu Hz ('k' toggles)\n",
                (unsigned long)I2C_SCAN_HZ, (unsigned long)g_i2cHz);
}

// ---------------------------------------------------------------- 2 WS2812
static void ledRGB(uint8_t r, uint8_t g, uint8_t b) { px.setPixelColor(0, px.Color(r, g, b)); px.show(); }
static void testWs2812() {
  hdr("WS2812 (module LED, GP16)");
  Serial.println("  Watch the module LED: RED, GREEN, BLUE, WHITE, 0.6 s each.");
  ledRGB(60, 0, 0); delay(600);
  ledRGB(0, 60, 0); delay(600);
  ledRGB(0, 0, 60); delay(600);
  ledRGB(40, 40, 40); delay(600);
  ledRGB(30, 20, 0);  // amber = test running
  rec(G_BASE, INFO, "WS2812", "visual check only. If RED showed green, set WS2812_ORDER to NEO_RGB");
}

// ---------------------------------------------------------------- 3 DMX rail / DE
static void testDmxRail() {
  hdr("+5V rail and DMX logic side (U3 VCC = +3V3_DMX, from U5, from +5V)");
  // DE: R37 (10k to GND) must beat the ~50k internal pull-up.
  pinMode(PIN_DMX_DE, INPUT_PULLUP); delayMicroseconds(300);
  int de = digitalRead(PIN_DMX_DE);
  pinMode(PIN_DMX_DE, OUTPUT); digitalWrite(PIN_DMX_DE, LOW);   // receive only, permanently
  if (de == 0) rec(G_BASE, PASS, "DMX DE pulldown", "R37 holds DE low against internal pull-up");
  else rec(G_BASE, FAIL, "DMX DE pulldown", "DE floats high: R37 missing/open (check R37 pads and U3 pins 5/6)");

  // DI and RO: R38/R39 to +3V3_DMX (plus U3 RxD on RO) must beat the internal pulldown.
  // RO is sampled for 20 ms rather than once: with a live DMX source on J1 it toggles,
  // and a single read can land on a data bit.
  pinMode(PIN_DMX_DI, INPUT_PULLDOWN); pinMode(PIN_DMX_RO, INPUT_PULLDOWN); delay(2);
  int di = digitalRead(PIN_DMX_DI);
  uint32_t roHi = 0, roLo = 0, t0 = micros();
  while (micros() - t0 < 20000) { if (gpio_get(PIN_DMX_RO)) roHi++; else roLo++; }
  int ro = roHi > 0;
  pinMode(PIN_DMX_DI, INPUT); pinMode(PIN_DMX_RO, INPUT);

  g_5v = di;
  if (di && ro) rec(G_BASE, PASS, "+3V3_DMX / +5V", "present (R38/R39 pull GP0/GP1 high), so 24 V in, U1 buck and U5 are alive%s",
                    roLo ? ". RO toggling: DMX activity on J1" : "");
  else if (di && !ro) rec(G_BASE, WARN, "+3V3_DMX / +5V", "rail present but RO low at idle: line held at space (A/B reversed on a live driver?) or U3 bus side not running. Measure VISOOUT TP vs DMX_ISO_GND (~3.3 V)");
  else if (!di && !ro) rec(G_BASE, WARN, "+3V3_DMX / +5V", "absent: treating as USB-only power (UI LEDs, OLED VBAT dark). If 24 V IS connected this is a FAIL: check +5V TP / U1 pin 3, then +3V3_DMX TP / U5 pin 4");
  else rec(G_BASE, FAIL, "+3V3_DMX / +5V", "RO high but DI low: R38 open, or R48 open (GP0 side)");
}

// ---------------------------------------------------------------- 4 I2C lines
static bool i2cRecover() {
  pinMode(PIN_SDA, INPUT); pinMode(PIN_SCL, INPUT);
  for (int i = 0; i < 9 && !gpio_get(PIN_SDA); i++) {
    odLow(PIN_SCL); delayMicroseconds(5); odRel(PIN_SCL); delayMicroseconds(5);
  }
  odLow(PIN_SDA); delayMicroseconds(5);   // STOP: SDA low -> high while SCL high
  odRel(PIN_SCL); delayMicroseconds(5);
  odRel(PIN_SDA); delayMicroseconds(10);
  return gpio_get(PIN_SDA) && gpio_get(PIN_SCL);
}

static void lineState(int* sdaFree, int* sclFree, int* sdaPd, int* sclPd) {
  pinMode(PIN_SDA, INPUT); pinMode(PIN_SCL, INPUT); delayMicroseconds(100);
  *sdaFree = digitalRead(PIN_SDA); *sclFree = digitalRead(PIN_SCL);
  pinMode(PIN_SDA, INPUT_PULLDOWN); pinMode(PIN_SCL, INPUT_PULLDOWN); delayMicroseconds(100);
  *sdaPd = digitalRead(PIN_SDA); *sclPd = digitalRead(PIN_SCL);
  pinMode(PIN_SDA, INPUT); pinMode(PIN_SCL, INPUT);
}

static bool testI2CLines() {
  hdr("I2C lines (GP12 SDA, GP13 SCL)");
  wireStop();
  int sf, cf, sp, cp;
  lineState(&sf, &cf, &sp, &cp);
  bool recovered = false;
  if (!sp && cf && cp) {   // SDA low with SCL healthy: try recovery
    recovered = i2cRecover();
    lineState(&sf, &cf, &sp, &cp);
  }
  bool ok = sp && cp;
  if (ok) {
    rec(G_BASE, PASS, "I2C lines", "SDA and SCL hold high against ~50k internal pulldown (R3/R4 fitted, bus free)%s",
        recovered ? "; SDA was stuck low and was freed by 9-clock recovery (a device was mid-transfer)" : "");
  } else {
    const char* names[2] = {"SDA", "SCL"};
    int fr[2] = {sf, cf}, pd[2] = {sp, cp};
    const char* pu[2] = {"R3", "R4"};
    for (int i = 0; i < 2; i++) {
      if (pd[i]) continue;
      if (fr[i]) rec(G_BASE, FAIL, "I2C lines", "%s floats high but falls under 50k pulldown: %s pull-up missing/open", names[i], pu[i]);
      else rec(G_BASE, FAIL, "I2C lines", "%s held low%s. Unplug the UI ribbon and rerun; measure across %s (UI branch) and %s (stack branch), the lower side is the fault side",
               names[i], (i == 0 && cp) ? " even after 9-clock recovery" : "", i ? "R31" : "R30", i ? "R33" : "R32");
    }
  }
  wireStart(g_i2cHz);
  return ok;
}

// ---------------------------------------------------------------- 6 FRAM
static uint8_t g_framAddr = ADDR_FRAM;
static bool framWrite(uint16_t a, const uint8_t* d, size_t n) {
  Wire.beginTransmission(g_framAddr); Wire.write(a >> 8); Wire.write(a & 0xFF); Wire.write(d, n);
  return Wire.endTransmission() == 0;
}
static bool framRead(uint16_t a, uint8_t* d, size_t n) {
  Wire.beginTransmission(g_framAddr); Wire.write(a >> 8); Wire.write(a & 0xFF);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(g_framAddr, n) != n) return false;
  for (size_t i = 0; i < n; i++) d[i] = Wire.read();
  return true;
}

static void testFram() {
  hdr("FRAM U4 (MB85RC64TA)");
  // WP pull-up R47 must beat internal pulldown
  pinMode(PIN_FRAM_WP, INPUT_PULLDOWN); delayMicroseconds(200);
  int wp = digitalRead(PIN_FRAM_WP);
  pinMode(PIN_FRAM_WP, OUTPUT); digitalWrite(PIN_FRAM_WP, HIGH);   // protected
  if (wp) rec(G_BASE, PASS, "FRAM WP pull-up", "R47 holds WP high (protected) with GP15 released");
  else rec(G_BASE, FAIL, "FRAM WP pull-up", "WP falls under 50k pulldown: R47 missing/open (U4 unprotected at boot)");

  g_framAddr = 0;
  for (uint8_t a = 0x50; a <= 0x57; a++) if (probe(a)) { g_framAddr = a; break; }
  if (!g_framAddr) {
    rec(G_BASE, FAIL, "FRAM present", "no ACK on 0x50-0x57. Check U4 pin 8 (VDD, 3.3 V), pin 4 GND, pin 1 orientation, pins 5/6 to SDA/SCL");
    g_framAddr = ADDR_FRAM;
    return;
  }
  if (g_framAddr != ADDR_FRAM)
    rec(G_BASE, WARN, "FRAM address", "answers at 0x%02X, not 0x50: an R44-R46 strap is fitted or an R41-R43 pulldown is open", g_framAddr);
  else rec(G_BASE, PASS, "FRAM present", "ACK at 0x50 (straps as built)");

  // Device ID (reserved address 0x7C). Informational only.
  {
    uint8_t id[3] = {0};
    Wire.beginTransmission(0x7C); Wire.write(g_framAddr << 1);
    bool ok = Wire.endTransmission(false) == 0 && Wire.requestFrom((uint8_t)0x7C, (size_t)3) == 3;
    if (ok) { for (int i = 0; i < 3; i++) id[i] = Wire.read();
      rec(G_BASE, INFO, "FRAM device ID", "%02X %02X %02X (Fujitsu/RAMXEED mfr code is 0x00A in the top 12 bits)", id[0], id[1], id[2]); }
    else rec(G_BASE, INFO, "FRAM device ID", "no response on 0x7C (not every density supports Device ID; not a fault)");
  }

  uint8_t orig[FRAM_TEST_LEN], alias0[FRAM_TEST_LEN], pat[FRAM_TEST_LEN], rb[FRAM_TEST_LEN], alias1[FRAM_TEST_LEN];
  const uint16_t A = FRAM_TEST_ADDR, AL = FRAM_TEST_ADDR & 0x0FFF;   // AL = where A lands if A12 is ignored
  if (!framRead(A, orig, FRAM_TEST_LEN) || !framRead(AL, alias0, FRAM_TEST_LEN)) {
    rec(G_BASE, FAIL, "FRAM read", "ACKs but random read failed (bus error at %lu Hz; try 'k' then 'f')", (unsigned long)g_i2cHz);
    return;
  }
  for (int i = 0; i < FRAM_TEST_LEN; i++) pat[i] = ~orig[i];

  // 1. write with WP high must be ignored
  digitalWrite(PIN_FRAM_WP, HIGH); delayMicroseconds(10);
  framWrite(A, pat, FRAM_TEST_LEN);
  framRead(A, rb, FRAM_TEST_LEN);
  bool wpOk = memcmp(rb, orig, FRAM_TEST_LEN) == 0;

  // 2. write with WP low must stick
  digitalWrite(PIN_FRAM_WP, LOW); delayMicroseconds(10);
  bool wOk = framWrite(A, pat, FRAM_TEST_LEN);
  bool rOk = framRead(A, rb, FRAM_TEST_LEN);
  bool dataOk = wOk && rOk && memcmp(rb, pat, FRAM_TEST_LEN) == 0;
  framRead(AL, alias1, FRAM_TEST_LEN);
  bool aliasOk = memcmp(alias0, alias1, FRAM_TEST_LEN) == 0;

  // 3. restore, verify, protect
  framWrite(A, orig, FRAM_TEST_LEN);
  framRead(A, rb, FRAM_TEST_LEN);
  bool restOk = memcmp(rb, orig, FRAM_TEST_LEN) == 0;
  if (!wpOk) framWrite(A, orig, FRAM_TEST_LEN);
  digitalWrite(PIN_FRAM_WP, HIGH);

  if (wpOk) rec(G_BASE, PASS, "FRAM write protect", "write with WP high was ignored");
  else rec(G_BASE, FAIL, "FRAM write protect", "write landed with WP high: check GP15 to U4 pin 7 (measure across R47 while toggling 'f')");
  if (dataOk && aliasOk && restOk)
    rec(G_BASE, PASS, "FRAM write/read", "16 B at 0x%04X written, read back, no alias at 0x%04X, original restored", A, AL);
  else
    rec(G_BASE, FAIL, "FRAM write/read", "write %s, readback %s, alias %s, restore %s",
        wOk ? "ack" : "NAK", dataOk ? "ok" : "MISMATCH", aliasOk ? "none" : "SEEN (part < 64 kbit?)", restOk ? "ok" : "FAILED");
}

// ---------------------------------------------------------------- 7 UI board (TCA9555)
static bool tcaWrite16(uint8_t reg, uint16_t v) {
  Wire.beginTransmission(ADDR_TCA); Wire.write(reg); Wire.write(v & 0xFF); Wire.write(v >> 8);
  return Wire.endTransmission() == 0;
}
static bool tcaRead16(uint8_t reg, uint16_t* v) {
  uint8_t b[2]; if (!regRead(ADDR_TCA, reg, b, 2)) return false;
  *v = b[0] | ((uint16_t)b[1] << 8); return true;
}
static bool tcaSetOut(uint16_t v) { g_tcaOut = v; return tcaWrite16(0x02, v); }

static bool uiInit() {   // UI spec §9 steps 1-2
  bool ok = tcaSetOut(0xFF3F);          // LEDs off, VBAT_EN 0, OLED_RES# 0
  ok &= tcaWrite16(0x04, 0x0000);       // no input polarity inversion
  ok &= tcaWrite16(0x06, 0xC000);       // P0.0-P1.5 outputs, P1.6/P1.7 inputs
  g_uiInit = ok;
  return ok;
}
static uint16_t ledOff() { return (g_tcaOut & (TCA_VBAT_EN | TCA_OLED_RES)) | 0xFF3F; }
static void uiLed(int led, bool r, bool g, bool b) {
  if (!g_uiInit) return;
  uint16_t v = g_tcaOut;
  bool on[3] = {r, g, b};
  for (int c = 0; c < 3; c++) { if (on[c]) v &= ~(1u << LED_BIT[led][c]); else v |= (1u << LED_BIT[led][c]); }
  tcaSetOut(v);
}
static void bitsToStr(uint16_t diff, uint16_t expect, char* out, size_t n) {
  out[0] = 0;
  for (int b = 0; b < 16; b++) if (diff & (1u << b)) {
    char t[24]; snprintf(t, sizeof t, " P%d.%d(exp %d)", b / 8, b % 8, (expect >> b) & 1);
    strncat(out, t, n - strlen(out) - 1);
  }
}

static void testUiBoard() {
  hdr("UI board: TCA9555 U1 at 0x20");
  if (!probe(ADDR_TCA)) {
    g_uiPresent = false; g_uiInit = false;
    rec(G_UI, INFO, "UI board", "no ACK at 0x20: UI board unplugged, ribbon fault, or U1 dead. UI, OLED and button-press tests skipped");
    return;
  }
  g_uiPresent = true;

  uint16_t cfg = 0, out = 0, in = 0;
  if (tcaRead16(0x06, &cfg) && tcaRead16(0x02, &out) && tcaRead16(0x00, &in)) {
    if (cfg == 0xFFFF && out == 0xFFFF) {
      bool resLow = !(in & TCA_OLED_RES), vbLow = !(in & TCA_VBAT_EN);
      if (resLow && vbLow) rec(G_UI, PASS, "UI power-on state", "U1 at reset defaults; R10 holds RES# low, R13 holds VBAT_EN low (panel off, in reset)");
      else rec(G_UI, FAIL, "UI power-on state", "%s%s reads high with U1 pins as inputs: pulldown missing (R10 on RES#, R13 on VBAT_EN)",
               resLow ? "" : "RES# (P0.7) ", vbLow ? "" : "VBAT_EN (P0.6)");
      rec(G_UI, INFO, "UI spare pins", "P1.6=%d P1.7=%d as inputs (1 expected, test pads only)", (in >> 14) & 1, (in >> 15) & 1);
    } else {
      rec(G_UI, INFO, "UI power-on state", "U1 already configured (cfg 0x%04X) since its power-up, so default-state check skipped. Power-cycle to run it", cfg);
    }
  }

  if (!uiInit()) { rec(G_UI, FAIL, "UI init", "NAK writing U1 registers at %lu Hz", (unsigned long)g_i2cHz); return; }
  uint16_t pol = 0, cfgRb = 0;
  tcaWrite16(0x04, 0xA55A); tcaRead16(0x04, &pol); tcaWrite16(0x04, 0x0000);
  tcaRead16(0x06, &cfgRb);
  if (pol == 0xA55A && cfgRb == 0xC000) rec(G_UI, PASS, "UI U1 registers", "polarity register write/readback 0xA55A ok, config 0xC000 ok (real TCA9555-class device)");
  else rec(G_UI, FAIL, "UI U1 registers", "readback polarity 0x%04X (exp 0xA55A), config 0x%04X (exp 0xC000)", pol, cfgRb);

  const int stepMs = g_5v ? 300 : 5, whiteMs = g_5v ? 1000 : 5;
  if (g_5v) Serial.println("  LED walk: LED1..LED4, each red, green, blue, 0.3 s. Then all white 1 s.");
  else Serial.println("  +5V absent: LEDs stay dark (anodes on +5V). Running a fast electrical readback only.");
  uint16_t badDiff = 0, badExp = 0;
  for (int l = 0; l < 4; l++) for (int c = 0; c < 3; c++) {
    uint16_t v = ledOff() & ~(1u << LED_BIT[l][c]);
    tcaSetOut(v); delayMicroseconds(300);
    if (tcaRead16(0x00, &in)) { uint16_t d = (in ^ v) & TCA_OUT_MASK; if (d) { badDiff |= d; badExp = v; } }
    else badDiff |= 0x8000;
    delay(stepMs);
  }
  uint16_t white = ledOff() & ~TCA_LED_MASK;
  tcaSetOut(white); delayMicroseconds(300);
  if (tcaRead16(0x00, &in)) { uint16_t d = (in ^ white) & TCA_OUT_MASK; if (d) { badDiff |= d; badExp = white; } }
  delay(whiteMs);
  tcaSetOut(ledOff());
  if (!g_5v) {
    // With +5V dead, an output-high LED pin back-feeds the +5V rail through the red
    // shunt + series resistor (~2 mA each). Park the LED pins as inputs instead.
    tcaWrite16(0x06, 0xFF3F);
  }
  if (!badDiff) rec(G_UI, PASS, "UI LED lines", g_5v ? "all 12 sink lines read back correctly on and off. Visual: each LED R,G,B in order, then white; no dark or wrong colour"
                                                     : "all 12 sink lines read back correctly (electrical only; rerun with 24 V for the visual check)");
  else {
    char s[120]; bitsToStr(badDiff & 0x7FFF, badExp, s, sizeof s);
    rec(G_UI, FAIL, "UI LED lines", "pin readback mismatch:%s%s (open joint, or short to a neighbour / GND / +5V at U1 or R20-R35)", s, (badDiff & 0x8000) ? " +I2C read error" : "");
  }
}

// ---------------------------------------------------------------- 8 OLED
static uint8_t fb[1024];
static bool oledCmd(std::initializer_list<uint8_t> c) {
  Wire.beginTransmission(ADDR_OLED); Wire.write((uint8_t)0x00);
  for (uint8_t b : c) Wire.write(b);
  return Wire.endTransmission() == 0;
}
static bool oledFlush() {
  bool ok = oledCmd({0x21, 0, 127}) && oledCmd({0x22, 0, 7});
  for (int i = 0; ok && i < 1024; i += 32) {
    Wire.beginTransmission(ADDR_OLED); Wire.write((uint8_t)0x40); Wire.write(fb + i, 32);
    ok = Wire.endTransmission() == 0;
  }
  return ok;
}
static void px1(int x, int y) { if (x >= 0 && x < 128 && y >= 0 && y < 64) fb[(y >> 3) * 128 + x] |= (1 << (y & 7)); }

static bool oledInitSeq(bool chargePump) {   // UI spec §9 step 5, plus IREF and addressing mode
  bool ok = true;
  ok &= oledCmd({0xAE});
  ok &= oledCmd({0xD5, 0x90});
  ok &= oledCmd({0x81, 0xB0});
  ok &= oledCmd({0xD9, 0x22});
  ok &= oledCmd({0xA8, 0x3F});
  ok &= oledCmd({0xD3, 0x00});
  ok &= oledCmd({0x40});
  ok &= oledCmd({0xA1});
  ok &= oledCmd({0xC8});
  ok &= oledCmd({0xDA, 0x12});
  ok &= oledCmd({0xDB, 0x30});
  ok &= oledCmd({0xA6});
  ok &= oledCmd({0xA4});
  ok &= oledCmd({0xAD, OLED_IREF});   // internal IREF (R11 DNP)
  ok &= oledCmd({0x20, 0x00});        // horizontal addressing for full-frame writes
  if (chargePump) ok &= oledCmd({0x8D, 0x14});   // charge pump on (7.5 V)
  return ok;
}

static void oledSleep() {   // UI spec §9 sleep: AE, 8D 10, VBAT off, >= 50 ms
  if (!g_uiInit) return;
  if (g_oledOn) { oledCmd({0xAE}); oledCmd({0x8D, 0x10}); delay(100); }
  tcaSetOut(g_tcaOut & ~TCA_VBAT_EN);
  delay(60);
  g_oledOn = false;
}

static void testOled() {
  hdr("OLED DS1 (SSD1315 at 0x3C)");
  if (!g_uiPresent) { g_oledFound = -1; rec(G_OLED, SKIP, "OLED", "needs the UI board (its U1 drives RES# and VBAT)"); return; }
  if (!g_uiInit && !uiInit()) { rec(G_OLED, FAIL, "OLED", "cannot init U1"); return; }

  oledSleep();
  tcaSetOut(g_tcaOut & ~(TCA_VBAT_EN | TCA_OLED_RES)); delay(2);
  if (probe(ADDR_OLED))
    rec(G_OLED, WARN, "OLED reset", "0x3C answered while RES# should hold it in reset: check U1 P0.7 (pin 11) to ZIF pin 14, R10");

  tcaSetOut(g_tcaOut | TCA_OLED_RES); delay(5);           // step 3
  if (!probe(ADDR_OLED)) {
    g_oledFound = 0;
    tcaSetOut(g_tcaOut & ~TCA_OLED_RES);                   // back into reset
    rec(G_OLED, INFO, "OLED", "no ACK at 0x3C with RES# released: panel not plugged in, or tail not seated / contacts not making in ZIF J2");
    return;
  }
  g_oledFound = 1;
  if (!g_5v) {
    // VBAT comes from U2 (HT7540) off +5V, which is dead. Prove the command and data
    // path only: init without charge pump / display on, write one frame, leave VBAT off.
    bool ok = oledInitSeq(false);
    memset(fb, 0, sizeof fb); ok &= oledFlush();
    if (ok) rec(G_OLED, PASS, "OLED I2C", "ACK at 0x3C, init and a full frame accepted. No image: +5V absent so no VBAT. Rerun with 24 V for the visual check");
    else rec(G_OLED, FAIL, "OLED I2C", "NAK during init/frame writes at %lu Hz (ZIF contact, or SDA low level: scope SDA at base node during ACK, must be < 0.99 V)", (unsigned long)g_i2cHz);
    return;   // RES# stays released (VDD-only, VBAT floating is the datasheet's "off" state)
  }
  tcaSetOut(g_tcaOut | TCA_VBAT_EN); delay(100);          // step 4
  uint16_t in = 0;
  tcaRead16(0x00, &in);
  if ((in & (TCA_VBAT_EN | TCA_OLED_RES)) != (TCA_VBAT_EN | TCA_OLED_RES))
    rec(G_OLED, FAIL, "OLED control lines", "U1 readback VBAT_EN=%d RES#=%d with both driven high (U1 pin 10 / 11 loaded or shorted)",
        (in >> 6) & 1, (in >> 7) & 1);

  bool ok = oledInitSeq(true);
  memset(fb, 0, sizeof fb); ok &= oledFlush();
  ok &= oledCmd({0xAF}); delay(100);
  g_oledOn = true; g_oledOnAt = millis();

  Serial.println("  Full white 2 s. Measure VBAT now: UI VBAT test pad or Q1 pin 3, expect 4.0 V +/-2%.");
  memset(fb, 0xFF, sizeof fb); ok &= oledFlush(); delay(2000);
  Serial.println("  8x8 checkerboard 1.5 s.");
  for (int p = 0; p < 8; p++) for (int x = 0; x < 128; x++) fb[p * 128 + x] = (((x >> 3) + p) & 1) ? 0xFF : 0x00;
  ok &= oledFlush(); delay(1500);
  memset(fb, 0, sizeof fb);
  for (int x = 0; x < 128; x++) { px1(x, 0); px1(x, 63); px1(x, x / 2); }
  for (int y = 0; y < 64; y++) { px1(0, y); px1(127, y); }
  for (int x = 0; x < 16; x++) for (int y = 0; y < 16; y++) px1(x, y);
  ok &= oledFlush();
  Serial.println("  Now showing: border on all 4 edges, solid block TOP-LEFT, diagonal from top-left toward bottom-right.");

  if (ok) rec(G_OLED, PASS, "OLED", "ACK at 0x3C, init and 4 frames written with no NAK. Visual: white, checker, then orientation pattern (block elsewhere = flip A1/C8 in init)");
  else rec(G_OLED, FAIL, "OLED", "NAK during init/frame writes at %lu Hz: marginal ZIF contact or SDA low level (scope SDA at base node during ACK, must be < 0.99 V)", (unsigned long)g_i2cHz);
}

// ---------------------------------------------------------------- 9 address report
enum { K_BASE, K_UI, K_OLED, K_FAULT, K_STACK };
struct AddrEntry { uint8_t lo, hi, kind; const char* dev; const char* where; };
static const AddrEntry ADDRS[] = {
  {0x20, 0x20, K_UI,    "TCA9555 (UI U1)",            "UI board, A0-A2 to GND"},
  {0x21, 0x27, K_FAULT, "TCA9555 at alt address",     "UI U1 address pin not at GND"},
  {0x3C, 0x3C, K_OLED,  "SSD1315 OLED (UI DS1)",      "UI board, D/C# to GND"},
  {0x3D, 0x3D, K_FAULT, "SSD1315 at alt address",     "DS1 D/C# high (ZIF pin 15)"},
  {0x48, 0x48, K_STACK, "ADS1115 (igniter U3)",       "igniter J7 default"},
  {0x49, 0x49, K_STACK, "ADS1115 (igniter U3)",       "igniter J7 alternate"},
  {0x4A, 0x4B, K_STACK, "ADS1115 / TMP1075 restrap",  "igniter, not offered by J7/J9"},
  {0x4C, 0x4C, K_STACK, "TMP1075/LM75B (igniter U4)", "igniter J9 default"},
  {0x4D, 0x4D, K_STACK, "TMP1075/LM75B (igniter U4)", "igniter J9 alternate"},
  {0x4E, 0x4F, K_STACK, "TMP1075/LM75B restrap",      "igniter U4 A1/A2 changed"},
  {0x50, 0x50, K_BASE,  "MB85RC64TA FRAM (base U4)",  "base, as built"},
  {0x51, 0x57, K_FAULT, "FRAM at alt address",        "base R44-R46 fitted / strap fault"},
};

static void identifyStack(uint8_t a) {
  uint16_t cfg = 0, id = 0, t = 0, lo = 0, hi = 0;
  regRead16be(a, 0x01, &cfg);
  // ADS1115 threshold registers sit at 0x8000 / 0x7FFF from power-up (this firmware never
  // writes them), which tells it apart from a TMP1075/LM75 whatever its config register holds.
  if (regRead16be(a, 0x02, &lo) && regRead16be(a, 0x03, &hi) && lo == 0x8000 && hi == 0x7FFF) {
    rec(G_STACK, INFO, "stack ADS1115", "0x%02X: ADS1115 (threshold registers at reset values)", a);
    return;
  }
  if (regRead16be(a, 0x0F, &id) && id == 0x7500 && regRead16be(a, 0x00, &t)) {
    rec(G_STACK, INFO, "stack temp", "0x%02X: TMP1075 (ID 0x7500), %.2f C (cold junction)", a, (int16_t)t / 256.0f);
    return;
  }
  if (regRead16be(a, 0x00, &t)) {
    float c = (int16_t)t / 256.0f;
    if (c > -40 && c < 100) { rec(G_STACK, INFO, "stack temp", "0x%02X: LM75-compatible reading %.2f C (no TMP1075 ID; LM75B?). Could also be an ADS1115 not at reset config", a, c); return; }
  }
  rec(G_STACK, WARN, "stack unknown", "0x%02X ACKs but does not identify (cfg 0x%04X). ADS1115 configured since power-up, or a non-spec part", a, cfg);
}

static void addressReport() {
  hdr("I2C scan and address report");
  wireStart(I2C_SCAN_HZ);
  scan(ackNow);
  wireStart(g_i2cHz);
  printAcks("Boot scan (before UI init, OLED held in reset)", ackBoot);
  printAcks("Scan now                                     ", ackNow);

  Serial.println("  Addr       Device                       Expected on / set by               Result");
  bool claimed[128] = {false};
  for (const AddrEntry& e : ADDRS) {
    char found[48] = ""; int nf = 0;
    for (int a = e.lo; a <= e.hi; a++) {
      claimed[a] = true;
      if (ackNow[a]) { char t[8]; snprintf(t, sizeof t, " 0x%02X", a); strncat(found, t, sizeof found - strlen(found) - 1); nf++; }
    }
    const char* res;
    switch (e.kind) {
      case K_BASE:  res = nf ? "FOUND" : "MISSING (fault)"; break;
      case K_UI:    res = nf ? "FOUND" : "absent (UI board not connected?)"; break;
      case K_OLED:  res = nf ? "FOUND" : (!g_uiPresent ? "n/a (no UI board)" : "absent (unplugged / ZIF contact)"); break;
      case K_FAULT: res = nf ? "FOUND: unexpected, see device" : "none (good)"; break;
      default:      res = nf ? "FOUND" : "absent"; break;
    }
    char ar[16];
    if (e.lo == e.hi) snprintf(ar, sizeof ar, "0x%02X", e.lo); else snprintf(ar, sizeof ar, "0x%02X-%02X", e.lo, e.hi);
    Serial.printf("  %-10s %-28s %-34s %s%s\n", ar, e.dev, e.where, res, (nf && e.lo != e.hi) ? found : "");
  }

  // Graded results
  if (ackNow[ADDR_FRAM]) rec(G_BASE, PASS, "addr 0x50 FRAM", "present");
  else rec(G_BASE, FAIL, "addr 0x50 FRAM", "missing%s", g_framAddr != ADDR_FRAM ? " (answers at alt address)" : "");
  for (const AddrEntry& e : ADDRS) if (e.kind == K_FAULT)
    for (int a = e.lo; a <= e.hi; a++) if (ackNow[a])
      rec(e.lo == 0x51 ? G_BASE : G_UI, WARN, "alt address", "0x%02X answers: %s (%s)", a, e.dev, e.where);
  if (g_uiPresent && g_oledFound == 1 && !ackNow[ADDR_OLED])
    rec(G_OLED, WARN, "OLED dropped", "0x3C answered during the OLED test but not in this scan (intermittent ZIF contact?)");

  int nStack = 0;
  for (int a = 0x48; a <= 0x4F; a++) if (ackNow[a]) { identifyStack(a); nStack++; }
  if (!nStack) rec(G_STACK, INFO, "stack I2C", "no igniter-board devices (0x48-0x4F) on the bus");

  char unk[96] = ""; int nu = 0;
  for (int a = 0x08; a <= 0x77; a++) if (ackNow[a] && !claimed[a]) {
    char t[8]; snprintf(t, sizeof t, " 0x%02X", a); strncat(unk, t, sizeof unk - strlen(unk) - 1); nu++;
  }
  if (nu) rec(G_BASE, WARN, "unexpected I2C", "not in any spec:%s", unk);
  else rec(G_BASE, PASS, "unexpected I2C", "none");
}

// ---------------------------------------------------------------- 10 buttons
static void testButtons(bool interactive, bool fromMenu) {
  hdr("Buttons (GP3 UP, GP4 DN, GP5 SET)");
  for (int i = 0; i < 3; i++) pinMode(PIN_BTN[i], INPUT_PULLUP);
  delay(60);   // release RC is ~5-8 ms through the internal pull-up
  int stuck = 0;
  for (int i = 0; i < 3; i++) if (!digitalRead(PIN_BTN[i])) {
    stuck |= 1 << i;
    rec(G_BASE, FAIL, "button idle", "BTN_%s reads pressed at idle: short to GND. Unreworked tact switch (legs not clipped) on UI, or %s / %s / J2.%d. Measure across %s with UI unplugged",
        BTN_NAME[i], BTN_C[i], BTN_R[i], BTN_J2[i], BTN_R[i]);
  }
  if (!stuck) rec(G_BASE, PASS, "button idle", "all three read released (internal pull-ups, no shorts)");
  if (!interactive) { rec(G_UI, SKIP, "button press", "UI board not detected; use 'b' to force (e.g. tweezers J2.%d/%d/%d to GND)", BTN_J2[0], BTN_J2[1], BTN_J2[2]); return; }

  Serial.println("  Press each button when asked (10 s timeout; any key on serial skips).");
  for (int i = 0; i < 3; i++) {
    if (stuck & (1 << i)) continue;
    if (i < 3) uiLed(i, false, false, true);
    Serial.printf("  Press %s ...\n", BTN_NAME[i]);
    uint32_t t0 = millis(); bool got = false, skipped = false;
    while (millis() - t0 < 10000) {
      if (keyHit()) { skipped = true; break; }
      if (!digitalRead(PIN_BTN[i])) { got = true; break; }
    }
    if (skipped) { uiLed(i, false, false, false); rec(G_UI, SKIP, "button press", "skipped from serial"); break; }
    if (!got) {
      uiLed(i, true, false, false);
      rec(G_UI, FAIL, "button press", "%s not seen in 10 s: switch, UI J1.%d, ribbon, or base %s open", BTN_NAME[i], BTN_J2[i], BTN_R[i]);
      if (i == 0 && !fromMenu) { Serial.println("  No press at all; skipping the rest ('b' runs all three regardless)."); break; }
      continue;
    }
    int others = 0;
    for (int j = 0; j < 3; j++) if (j != i && !digitalRead(PIN_BTN[j])) others |= 1 << j;
    // count falling edges until released and stable for 150 ms (max 4 s)
    int edges = 1, last = 0; uint32_t stableAt = 0, tp = millis();
    while (millis() - tp < 4000) {
      int v = digitalRead(PIN_BTN[i]);
      for (int j = 0; j < 3; j++) if (j != i && !digitalRead(PIN_BTN[j])) others |= 1 << j;
      if (v != last) { if (!v) edges++; last = v; stableAt = millis(); }
      if (v && millis() - stableAt > 150) break;
    }
    uiLed(i, false, true, false);
    if (others) rec(G_BASE, FAIL, "button press", "%s also pulled %s%s%s low: nets shorted (J2 ribbon, R34-R36, or UI switch wiring)",
                    BTN_NAME[i], (others & 1) ? "UP " : "", (others & 2) ? "DN " : "", (others & 4) ? "SET" : "");
    else if (edges > 1) rec(G_UI, WARN, "button press", "%s ok but %d falling edges seen (bounce getting through: check %s fitted, Schmitt enabled)", BTN_NAME[i], edges, BTN_C[i]);
    else rec(G_UI, PASS, "button press", "%s: one clean press/release, no cross-talk", BTN_NAME[i]);
  }
  delay(300);
  for (int i = 0; i < 3; i++) uiLed(i, false, false, false);
}

// ---------------------------------------------------------------- 11 ADC
static void readAdc(float* v) {
  analogReadResolution(12);
  for (int i = 0; i < 4; i++) {
    uint32_t sum = 0;
    for (int k = 0; k < 32; k++) sum += analogRead(PIN_ADC[i]);
    v[i] = sum / 32.0f * 3.3f / 4095.0f;
  }
}
static void testAdc() {
  hdr("ADC0-3 (GP26-29 via R29-R26 to J3)");
  float v[4]; readAdc(v);
  rec(G_BASE, INFO, "ADC", "ADC0 %.3f  ADC1 %.3f  ADC2 %.3f  ADC3 %.3f V (J3.19/21/23/25). Floating and meaningless with nothing stacked", v[0], v[1], v[2], v[3]);
}
static void adcMonitor() {
  hdr("ADC monitor (any key stops)");
  while (!keyHit()) { float v[4]; readAdc(v); Serial.printf("  %.3f  %.3f  %.3f  %.3f V\n", v[0], v[1], v[2], v[3]); delay(300); }
}

// ---------------------------------------------------------------- 12 SPARE1
static void testSpare() {
  hdr("SPARE1 (GP14 via R20 to J3.29)");
  pinMode(PIN_SPARE1, INPUT_PULLUP); delayMicroseconds(200); int pu = digitalRead(PIN_SPARE1);
  pinMode(PIN_SPARE1, INPUT_PULLDOWN); delayMicroseconds(200); int pd = digitalRead(PIN_SPARE1);
  pinMode(PIN_SPARE1, INPUT);
  if (pu && !pd) rec(G_BASE, PASS, "SPARE1", "follows internal pulls both ways: no pull fitted (as shipped), no short");
  else if (!pu && !pd) rec(G_BASE, WARN, "SPARE1", "held low: R22 pulldown fitted, or short to GND");
  else if (pu && pd) rec(G_BASE, WARN, "SPARE1", "held high: R24 pull-up fitted, or short to 3V3");
  else rec(G_BASE, WARN, "SPARE1", "inconsistent reading (pu=%d pd=%d)", pu, pd);
}

// ---------------------------------------------------------------- 13 CH1-CH6
static void chAllLow() { for (int i = 0; i < 6; i++) { pinMode(PIN_CH[i], OUTPUT); digitalWrite(PIN_CH[i], LOW); } }

static void testChannels(bool walk) {
  hdr("CH1-CH6 (GP6-GP11, R5-R10 series, R11-R16 pulldowns, J3)");
  int bad = 0;
  for (int i = 0; i < 6; i++) {
    pinMode(PIN_CH[i], INPUT_PULLUP); delayMicroseconds(200); int pu = digitalRead(PIN_CH[i]);
    pinMode(PIN_CH[i], INPUT_PULLDOWN); delayMicroseconds(200); int pd = digitalRead(PIN_CH[i]);
    pinMode(PIN_CH[i], INPUT);
    if (pd) { bad++; rec(G_BASE, FAIL, "CH pulldown", "CH%d held high: short to 3V3 or a stacked board driving it (J3.%d)", i + 1, CH_J3[i]); }
    else if (pu) { bad++; rec(G_BASE, FAIL, "CH pulldown", "CH%d pulldown missing: R%d open or R%d open (pin follows internal pull-up)", i + 1, 11 + i, 5 + i); }
  }
  if (!bad) rec(G_BASE, PASS, "CH pulldowns", "all six held low by R11-R16 against internal pull-up");

  // isolation: drive one high (microseconds), others released; external pulldowns hold them low
  int isoBad = 0;
  for (int i = 0; i < 6; i++) pinMode(PIN_CH[i], INPUT);
  for (int i = 0; i < 6; i++) {
    pinMode(PIN_CH[i], OUTPUT); digitalWrite(PIN_CH[i], HIGH); delayMicroseconds(30);
    int self = gpio_get(PIN_CH[i]);
    int neigh = 0;
    for (int j = 0; j < 6; j++) if (j != i && gpio_get(PIN_CH[j])) neigh |= 1 << j;
    digitalWrite(PIN_CH[i], LOW); pinMode(PIN_CH[i], INPUT);
    if (!self) { isoBad++; rec(G_BASE, FAIL, "CH isolation", "CH%d cannot drive high: short to GND at R%d / J3.%d (J3 neighbours are GND)", i + 1, 5 + i, CH_J3[i]); }
    if (neigh) {
      isoBad++;
      char s[40] = ""; for (int j = 0; j < 6; j++) if (neigh & (1 << j)) { char t[6]; snprintf(t, sizeof t, " CH%d", j + 1); strncat(s, t, sizeof s - strlen(s) - 1); }
      rec(G_BASE, FAIL, "CH isolation", "driving CH%d also lifts%s: bridge between module castellations or J3 pins", i + 1, s);
    }
  }
  if (!isoBad) rec(G_BASE, PASS, "CH isolation", "each line drives high alone, no bridges, none shorted to GND");
  chAllLow();

  if (walk) {
    Serial.println("  Walk: each channel high 0.5 s, others low. Probe J3 pin or far side of R5-R10 for ~3.3 V.");
    for (int i = 0; i < 6; i++) {
      Serial.printf("    CH%d high (GP%d, J3.%d, R%d)\n", i + 1, PIN_CH[i], CH_J3[i], 5 + i);
      digitalWrite(PIN_CH[i], HIGH); delay(500); digitalWrite(PIN_CH[i], LOW);
    }
    rec(G_BASE, INFO, "CH walk", "done (visual/meter check only)");
  }
}

static void chPwmHold() {
  hdr("CH1-CH6 PWM 20 kHz, duty 10/20/30/40/50/60 % (any key stops)");
  Serial.println("  NOT with a solenoid board stacked: PWM on a FIRE line chops Q1 through its slow gate divider.");
  if (readChoice("  Continue? y/n", "yn", 'n') != 'y') { Serial.println("  cancelled"); return; }
  analogWriteFreq(20000);
  analogWriteRange(100);
  for (int i = 0; i < 6; i++) analogWrite(PIN_CH[i], 10 * (i + 1));
  while (!keyHit()) delay(10);
  for (int i = 0; i < 6; i++) analogWrite(PIN_CH[i], 0);
  chAllLow();
  Serial.println("  stopped, all low");
}

// ---------------------------------------------------------------- 13a / 'd' / 'e' DMX receive
// UART0 RX on GP1 (U3 RO via R40). GP0 (DI) stays a plain input and DE stays low, so
// this board never drives the bus. The IRQ drains the RX FIFO into dmx_feed() (poofer_dmx_rx.h).
static DmxRx g_dmx;
static bool g_dmxIrqSet = false;
static uint32_t g_dmxBaud = 0;

static void __not_in_flash_func(dmxIrq)() {
  uart_hw_t* u = uart_get_hw(uart0);
  uint32_t now = time_us_32();
  while (!(u->fr & UART_UARTFR_RXFE_BITS)) dmx_feed(&g_dmx, u->dr, now);
}

static void dmxStart() {
  irq_set_enabled(UART0_IRQ, false);
  dmx_reset(&g_dmx);
  g_dmxBaud = uart_init(uart0, 250000);
  uart_set_format(uart0, 8, 2, UART_PARITY_NONE);
  uart_set_hw_flow(uart0, false, false);
  uart_set_fifo_enabled(uart0, true);
  gpio_set_function(PIN_DMX_RO, GPIO_FUNC_UART);
  gpio_disable_pulls(PIN_DMX_RO);
  while (!(uart_get_hw(uart0)->fr & UART_UARTFR_RXFE_BITS)) (void)uart_get_hw(uart0)->dr;
  if (!g_dmxIrqSet) { irq_set_exclusive_handler(UART0_IRQ, dmxIrq); g_dmxIrqSet = true; }
  // RX interrupt at >= 4 entries (1/8 full) plus RX timeout (32 bit times idle with data waiting)
  hw_write_masked(&uart_get_hw(uart0)->ifls, 0u << UART_UARTIFLS_RXIFLSEL_LSB, UART_UARTIFLS_RXIFLSEL_BITS);
  uart_get_hw(uart0)->imsc = UART_UARTIMSC_RXIM_BITS | UART_UARTIMSC_RTIM_BITS;
  irq_set_enabled(UART0_IRQ, true);
}
static void dmxStop() {
  irq_set_enabled(UART0_IRQ, false);
  uart_get_hw(uart0)->imsc = 0;
  uart_deinit(uart0);
  pinMode(PIN_DMX_RO, INPUT);   // back to SIO for the rail check
}
static void dmxSnap(DmxStats* s, uint32_t* sinceBreakUs, bool* everBreak) {
  uint32_t m = save_and_disable_interrupts();
  *s = g_dmx.st;
  *everBreak = g_dmx.haveBreak;
  *sinceBreakUs = time_us_32() - g_dmx.lastBreakUs;
  restore_interrupts(m);
}
static uint16_t dmxSnapFrame(uint8_t* out, uint16_t maxSlots, uint16_t* fullN) {   // out[0] = start code
  uint32_t m = save_and_disable_interrupts();
  uint16_t all = g_dmx.lastN, n = all > maxSlots ? maxSlots : all;
  memcpy(out, g_dmx.last, n + 1);
  restore_interrupts(m);
  if (fullN) *fullN = all;
  return n;
}

static void dmxGrade(const DmxStats& s, uint32_t ms, bool fromMenu) {
  float sec = ms / 1000.0f;
  if (s.bytes == 0) {
    if (fromMenu) rec(G_BASE, FAIL, "DMX receive", "nothing on RO in %.1f s. Sender running and node in 'Artnet -> dmx'? XLR harness on J1? "
                      "Then VISOOUT TP to DMX_ISO_GND (~3.3 V; 0 V = L1 open or U3 converter dead), +3V3_DMX TP, scope DMX_RO TP", sec);
    else rec(G_BASE, INFO, "DMX receive", "no DMX on J1 in %.1f s (no source connected; fine for bring-up). 'd' tests it with artnet_send.py running", sec);
    return;
  }
  if (s.frames == 0) {
    rec(G_BASE, FAIL, "DMX receive", "%lu bytes, %lu breaks, %lu framing errors, no clean frame%s. A/B reversed in the harness or node (data+ = XLR 3 = J1.1/J1.6), "
        "wrong source baud, or a very noisy line",
        (unsigned long)s.bytes, (unsigned long)s.breaks, (unsigned long)s.fe,
        s.altSc ? " with start code 0x00 (only alternate start codes seen)" : "");
    return;
  }
  float hz = s.perN ? 1e6f * s.perN / (float)s.perSumUs : 0;
  rec(G_BASE, PASS, "DMX receive", "%lu frames in %.1f s (%.1f Hz), %u-%u slots, start code 0x00 (UART at %lu baud)",
      (unsigned long)s.frames, sec, hz, s.slotsMin, s.slotsMax, (unsigned long)g_dmxBaud);
  if (s.perN) rec(G_BASE, INFO, "DMX timing", "break to break min %.2f / avg %.2f / max %.2f ms. DMX512 allows up to 1 s; a full 512-slot frame takes ~22.7 ms",
                  s.perMinUs / 1000.0f, s.perSumUs / 1000.0f / s.perN, s.perMaxUs / 1000.0f);

  uint32_t lineErr = s.fe + s.oe + s.pe;
  // Words before the first break are not counted (listening can start mid-frame, misaligned).
  uint32_t synced = s.bytes - s.presync;
  if (!lineErr && !s.errFrames) rec(G_BASE, PASS, "DMX line errors", "none in %lu bytes after first break", (unsigned long)synced);
  else {
    St st = (s.errFrames * 100 <= s.frames) ? WARN : FAIL;
    rec(G_BASE, st, "DMX line errors", "%lu framing, %lu overrun, %lu parity in %lu bytes; %lu/%lu frames dropped. FE: noise, no JP1 termination, reflections. OE: firmware bug",
        (unsigned long)s.fe, (unsigned long)s.oe, (unsigned long)s.pe, (unsigned long)synced,
        (unsigned long)s.errFrames, (unsigned long)(s.errFrames + s.frames));
  }
  if (s.altSc) rec(G_BASE, INFO, "DMX alt start", "%lu frames with start code 0x%02X (0xCC RDM, 0x17 text, 0xCF SIP): ignored, not an error",
                   (unsigned long)s.altSc, s.lastAltSc);

  uint32_t np = s.patOk + s.patPad + s.patTear + s.patBad;
  if (!np) rec(G_BASE, INFO, "DMX pattern", "none of the %lu frames carry the test pattern (QLC+, set/ramp mode, or blackout): data not checked", (unsigned long)s.frames);
  else if (!s.patBad) rec(G_BASE, PASS, "DMX pattern", "%lu frames checked slot by slot, all correct%s", (unsigned long)np,
                          s.patTear ? ", some torn (see below)" : "");
  else rec(G_BASE, FAIL, "DMX pattern", "%lu of %lu pattern frames corrupt. First: slot %u got 0x%02X, expected 0x%02X (seq 0x%02X). Random bits = noise/termination; "
           "every slot after one point wrong = dropped or extra byte",
           (unsigned long)s.patBad, (unsigned long)np, s.firstBad.firstBad, s.firstBad.got, s.firstBad.exp, s.firstBad.seq);
  if (s.patPad) rec(G_BASE, INFO, "DMX padded frames", "%lu frames: pattern, then zeros to the end (sender -n shorter than the node's output; the node pads). Checked up to the padding",
                    (unsigned long)s.patPad);
  if (s.patTear) rec(G_BASE, INFO, "DMX torn frames", "%lu frames mix two consecutive packets (the node updates its buffer mid-frame). Harmless for the poofers; "
                     "if it is the same end slot every time, suspect a fault on that slot instead", (unsigned long)s.patTear);
  if (np && s.patNone) rec(G_BASE, INFO, "DMX other frames", "%lu frames not the pattern (blackout from the sender exiting, or a second Art-Net source such as QLC+)",
                           (unsigned long)s.patNone);
  if (np) {
    if (s.seqChanges) rec(G_BASE, INFO, "DMX updates", "pattern changed %lu times in %.1f s: ~%.0f packets/s reach the bus (sender default 30/s)",
                          (unsigned long)s.seqChanges, sec, s.seqChanges / sec);
    else rec(G_BASE, WARN, "DMX updates", "pattern never changed: the node is repeating one frame. Sender stopped? (If you stopped it on purpose, the node holds last data on Art-Net loss)");
  }
  if (s.losses) rec(G_BASE, WARN, "DMX gaps", "%lu gaps over 1 s between frames (max %.0f ms): a poofer board would have timed out",
                    (unsigned long)s.losses, s.perMaxUs / 1000.0f);
}

static void testDmx(bool fromMenu) {
  hdr("DMX receive (J1 -> D3 -> U3 -> R40 -> GP1)");
  if (!g_5v) { rec(G_BASE, SKIP, "DMX receive", "+5V absent (USB only?): U3 and U5 unpowered. Connect 24 V"); return; }
  const uint32_t ms = fromMenu ? 5000 : 2000;
  Serial.printf("  Listening %lu s%s...\n", (unsigned long)(ms / 1000), fromMenu ? " (any key aborts)" : "");
  dmxStart();
  uint32_t t0 = millis();
  bool aborted = false;
  while (millis() - t0 < ms) { if (fromMenu && keyHit()) { aborted = true; break; } delay(10); }
  uint32_t el = millis() - t0;
  DmxStats s; uint32_t since; bool everBreak;
  dmxSnap(&s, &since, &everBreak);
  uint8_t f[13]; uint16_t n = dmxSnapFrame(f, 12, nullptr);
  dmxStop();
  if (aborted) { rec(G_BASE, SKIP, "DMX receive", "aborted after %.1f s", el / 1000.0f); return; }
  if (n) {
    Serial.print("  Last frame, slots 1-12:");
    for (int i = 1; i <= n; i++) Serial.printf(" %u", f[i]);
    Serial.println();
  }
  dmxGrade(s, el, fromMenu);
}

static void dmxMonitor() {
  hdr("DMX monitor (any key stops). Counts are per 0.5 s line");
  if (!g_5v) Serial.println("  WARNING: +5V looked absent at the last check ('p' rechecks). U3 needs 24 V in.");
  Serial.println("  Hz    = frames per second       sl = slots in last frame");
  Serial.println("  ok/tr/BAD/oth = pattern frames: correct / torn / corrupt / not the pattern");
  Serial.println("  err   = frames dropped for line errors   seq = pattern sequence byte");
  dmxStart();
  DmxStats a; uint32_t since; bool eb;
  dmxSnap(&a, &since, &eb);
  uint32_t tPrev = millis();
  bool lostShown = false;
  while (!keyHit()) {
    delay(20);
    if (millis() - tPrev < 500) continue;
    DmxStats b; dmxSnap(&b, &since, &eb);
    float dt = (millis() - tPrev) / 1000.0f; tPrev = millis();
    uint32_t fr = b.frames - a.frames, ef = b.errFrames - a.errFrames;
    uint32_t ok = (b.patOk + b.patPad) - (a.patOk + a.patPad), tr = b.patTear - a.patTear, bad = b.patBad - a.patBad, oth = b.patNone - a.patNone;
    uint8_t f[13]; uint16_t all = 0; uint16_t n = dmxSnapFrame(f, 12, &all);
    if (!eb || since > DMX_LOSS_US) {
      if (!eb) Serial.println("  NO DMX seen yet");
      else Serial.printf("  NO DMX for %.1f s (a poofer board would time out)\n", since / 1e6f);
      ledRGB(60, 0, 0);
      lostShown = true;
    } else {
      if (lostShown) { Serial.println("  DMX back"); lostShown = false; }
      Serial.printf("  %5.1f Hz %3u sl  ok %3lu tr %2lu BAD %2lu oth %3lu  err %2lu  seq %02X |",
                    fr / dt, all, (unsigned long)ok, (unsigned long)tr, (unsigned long)bad, (unsigned long)oth,
                    (unsigned long)ef, b.lastSeq);
      for (int i = 1; i <= n; i++) Serial.printf(" %3u", f[i]);
      Serial.println();
      if (bad || ef) ledRGB(40, 25, 0); else if (fr) ledRGB(0, 50, 0); else ledRGB(0, 0, 40);
    }
    a = b;
  }
  dmxStop();
  DmxStats s; dmxSnap(&s, &since, &eb);
  Serial.printf("  Totals: %lu frames, %lu dropped for line errors (FE %lu OE %lu), pattern ok %lu torn %lu BAD %lu other %lu, gaps > 1 s: %lu\n",
                (unsigned long)s.frames, (unsigned long)s.errFrames, (unsigned long)s.fe, (unsigned long)s.oe,
                (unsigned long)(s.patOk + s.patPad), (unsigned long)s.patTear, (unsigned long)s.patBad, (unsigned long)s.patNone, (unsigned long)s.losses);
  if (s.haveBad) Serial.printf("  First corrupt frame: slot %u got 0x%02X, expected 0x%02X (seq 0x%02X)\n",
                               s.firstBad.firstBad, s.firstBad.got, s.firstBad.exp, s.firstBad.seq);
  showStatus();
}

// ---------------------------------------------------------------- 'i' raw FRAM-address diagnostic
// Tries every FRAM address two independent ways: the core's bit-banged address-only
// probe (what the scan uses) and a real hardware-I2C transaction (set address
// pointer 0x0000, then read 1 byte; nondestructive). At 100 kHz and 400 kHz.
static const char* wireErr(uint8_t e) {
  switch (e) { case 0: return "ACK"; case 2: return "addr NAK"; case 3: return "data NAK"; case 5: return "timeout"; default: return "NAK"; }
}
static void i2cDiag() {
  hdr("I2C diagnostic: FRAM addresses 0x50-0x57, two methods");
  const uint32_t hzs[2] = {100000, 400000};
  for (int h = 0; h < 2; h++) {
    wireStart(hzs[h]);
    Serial.printf("  @ %lu Hz   SDA=%d SCL=%d before\n", (unsigned long)hzs[h], gpio_get(PIN_SDA), gpio_get(PIN_SCL));
    for (uint8_t a = 0x50; a <= 0x57; a++) {
      bool pr = probe(a);
      Wire.beginTransmission(a); Wire.write((uint8_t)0x00); Wire.write((uint8_t)0x00);
      uint8_t e = Wire.endTransmission(false);
      size_t n = 0; int b = -1;
      if (e == 0) { n = Wire.requestFrom(a, (size_t)1); if (n) b = Wire.read(); }
      else { Wire.beginTransmission(a); Wire.endTransmission(); }   // make sure a STOP goes out
      Serial.printf("    0x%02X  probe %-4s  hw write %-9s  hw read %s", a, pr ? "ACK" : "NAK", wireErr(e), n ? "ok" : "-");
      if (n) Serial.printf(" (byte 0x%02X)", b);
      Serial.println();
    }
    Serial.printf("  SDA=%d SCL=%d after\n", gpio_get(PIN_SDA), gpio_get(PIN_SCL));
  }
  wireStart(g_i2cHz);
  Serial.println("  Any ACK on either method means U4 is alive. Probe NAK + hw ACK = firmware probe issue, not hardware.");
}

// ---------------------------------------------------------------- 't' slow line toggle
// Open-drain only (never drives high). Pattern repeats until a key:
//   SDA low 2 s, released 1 s, SCL low 2 s, released 1 s.
// Meter or scope U4 pin 5 (SDA) and pin 6 (SCL) against pin 4.
static void lineToggle() {
  hdr("Slow SDA/SCL toggle, open-drain (any key stops)");
  Serial.println("  Meter U4 pin 5 (SDA) and pin 6 (SCL) to pin 4. Low = ~0 V, released = ~3.3 V.");
  wireStop();
  // Internal pull-ups on, so "released" reads high even on a bare module out of its
  // socket (R3/R4 live on the base board). Harmless in-circuit: ~50k parallel 2.2k.
  pinMode(PIN_SDA, INPUT_PULLUP); pinMode(PIN_SCL, INPUT_PULLUP);
  // Release FRAM WP (GP15) for the duration: R47 (10k) still holds WP high, but a
  // solder bridge from WP to SCL can then be overpowered by GP13 instead of fighting
  // GP15's push-pull high. If SCL now goes low, suspect a U4 pin 6 / pin 7 bridge.
  pinMode(PIN_FRAM_WP, INPUT);
  Serial.println("  FRAM WP (GP15) released to R47 for this test.");
  struct Step { uint8_t pin; bool low; uint16_t ms; const char* txt; };
  const Step steps[4] = {{PIN_SDA, true, 2000, "SDA LOW  (U4 pin 5 ~0 V, pin 6 ~3.3 V)"},
                         {PIN_SDA, false, 1000, "both released (both ~3.3 V)"},
                         {PIN_SCL, true, 2000, "SCL LOW  (U4 pin 6 ~0 V, pin 5 ~3.3 V)"},
                         {PIN_SCL, false, 1000, "both released (both ~3.3 V)"}};
  Serial.println("  Module LED follows the readback: amber = neither low, purple = SDA low, cyan = SCL low, white = both low.");
  Serial.println("  (Check colours with 'l' first: if the WS2812 order is wrong, purple and cyan swap.)");
  auto showLines = []() {
    int sda = gpio_get(PIN_SDA), scl = gpio_get(PIN_SCL);
    if (!sda && !scl) ledRGB(40, 40, 40);        // white
    else if (!sda)    ledRGB(40, 0, 40);         // purple
    else if (!scl)    ledRGB(0, 40, 40);         // cyan
    else              ledRGB(40, 20, 0);         // amber
  };
  bool stop = false;
  while (!stop) {
    for (int i = 0; i < 4 && !stop; i++) {
      if (steps[i].low) odLow(steps[i].pin); else odRel(steps[i].pin);
      delayMicroseconds(50);
      Serial.printf("  %-40s readback SDA=%d SCL=%d\n", steps[i].txt, gpio_get(PIN_SDA), gpio_get(PIN_SCL));
      uint32_t t0 = millis();
      while (millis() - t0 < steps[i].ms) { showLines(); if (keyHit()) { stop = true; break; } delay(20); }
    }
  }
  odRel(PIN_SDA); odRel(PIN_SCL);
  pinMode(PIN_FRAM_WP, OUTPUT); digitalWrite(PIN_FRAM_WP, HIGH);   // protected again
  wireStart(g_i2cHz);
  showStatus();
  Serial.println("  stopped, I2C restarted");
}

// ---------------------------------------------------------------- serial prompt helper
// Waits for one character from `allowed`. A bare Enter returns `def`. 60 s timeout -> def.
static char readChoice(const char* prompt, const char* allowed, char def) {
  Serial.printf("%s [%c]: ", prompt, def);
  delay(50); while (Serial.available()) Serial.read();          // drop stale input
  uint32_t t0 = millis();
  while (millis() - t0 < 60000) {
    if (!Serial.available()) continue;
    int c = Serial.read();
    if (c == ' ') continue;
    if (c == '\r' || c == '\n') { Serial.println(def); return def; }
    if (c >= 'A' && c <= 'Z') c += 32;
    if (strchr(allowed, c)) { delay(20); while (Serial.available()) Serial.read(); Serial.println((char)c); return (char)c; }
  }
  Serial.println(def);
  return def;
}

// ---------------------------------------------------------------- 'v' solenoid board test
// Solenoid spec Rev A §9 (firmware contract) and §12 (bring-up / pre-show T1-T5).
// The valve opens only with FIRE (odd CH) AND PWM (even CH) both on.
static void solPwm(uint8_t pin, int pct) {   // 0 and 100 as plain GPIO, else 20 kHz PWM
  if (pct <= 0)        { pinMode(pin, OUTPUT); digitalWrite(pin, LOW); }
  else if (pct >= 100) { pinMode(pin, OUTPUT); digitalWrite(pin, HIGH); }
  else { analogWriteFreq(20000); analogWriteRange(1000); analogWrite(pin, pct * 10); }
}
static float solAmps(uint8_t adcPin) {        // ADC_SENSE: 0.5 V/A (R18 25 mOhm x INA180A1 gain 20)
  uint32_t sum = 0;
  for (int k = 0; k < 16; k++) sum += analogRead(adcPin);
  return (sum / 16.0f) * 3.3f / 4095.0f / 0.5f;
}
static void solRelease(uint8_t fire, uint8_t pwm) {   // §9: FIRE low, PWM 100 % >= 20 ms, then PWM low
  digitalWrite(fire, LOW);
  solPwm(pwm, 100); delay(25);
  solPwm(pwm, 0);
}

// A solenoid board's ADC_SENSE (INA180 output through R19 10k, ~0 V at idle) holds its JP2-selected
// ADC line low against the RP2040's ~50k internal pull-up. A bare line (nothing stacked, or only
// an igniter board, whose ADC pins are no-connect) floats up to ~3.3 V. Returns a bitmask of
// driven lines. A solenoid board with no JP2 shunt fitted is not detectable this way.
static uint8_t solDetect() {
  uint8_t mask = 0;
  analogReadResolution(12);
  for (int i = 0; i < 4; i++) {
    uint8_t p = PIN_ADC[i];
    (void)analogRead(p);                         // lets the core set the pad up for ADC first
    gpio_pull_up(p);                             // then add the pull (the core won't clear it again)
    delay(25);                                   // C9 0.1 uF charges through ~15k || 50k
    uint32_t sum = 0; for (int k = 0; k < 8; k++) sum += analogRead(p);
    float v = sum / 8.0f * 3.3f / 4095.0f;
    gpio_disable_pulls(p);
    if (v < 1.5f) mask |= 1 << i;
  }
  return mask;
}

static void testSolenoid(bool interactive) {
  hdr("Solenoid board (spec Rev A §12). Gas closed / valve on the bench only.");
  Serial.println("  Board needs: F1 fitted, JP1 two shunts in one row, JP2 one shunt, valve on J1, 24 V on.");
  uint8_t det = solDetect();
  char adcDef = '0';
  for (int i = 0; i < 4; i++) if (det & (1 << i)) { adcDef = '0' + i; break; }
  if (det) Serial.printf("  ADC_SENSE detected on ADC line(s):%s%s%s%s\n",
                         (det & 1) ? " 0" : "", (det & 2) ? " 1" : "", (det & 4) ? " 2" : "", (det & 8) ? " 3" : "");
  else Serial.println("  No ADC line looks driven (no solenoid board, or no JP2 shunt).");
  if (!interactive) Serial.printf("  Non-interactive: JP1 pair 1 (CH1/CH2), ADC%c, 24 V coil, no T_max test.\n", adcDef);
  char pc = interactive ? readChoice("  JP1 pair? 1=CH1/CH2  2=CH3/CH4  3=CH5/CH6", "123", '1') : '1';
  char ac = interactive ? readChoice("  JP2 ADC line? 0-3", "0123", adcDef) : adcDef;
  char cc = interactive ? readChoice("  Coil? 2=24 V coil (100 % hold)  1=12 V coil (peak 100 % 100 ms, hold 35 %)", "12", '2') : '2';
  int pair = pc - '0', adc = ac - '0';
  bool coil24 = (cc == '2');
  uint8_t FIRE = PIN_CH[2 * (pair - 1)], PWM = PIN_CH[2 * (pair - 1) + 1], ADCP = PIN_ADC[adc];
  int holdPct = coil24 ? 100 : 35;
  analogReadResolution(12);
  chAllLow();
  Serial.printf("  FIRE = CH%d (GP%d), PWM = CH%d (GP%d), sense = ADC%d (GP%d)\n",
                2 * pair - 1, FIRE, 2 * pair, PWM, adc, ADCP);
  if (!g_5v) Serial.println("  WARNING: +5V looked absent at the last check. U2 gate driver needs +5V; connect 24 V and run 'p'.");

  // T1 idle
  delay(200);
  float i0 = solAmps(ADCP);
  Serial.println("  T1 idle. Expect LED1 (FUSE) on, LED2 (HS) off, LED3 (LS) off.");
  if (i0 < 0.05f) rec(G_STACK, PASS, "sol T1 idle", "sense %.3f A (expect 0). Check LEDs: FUSE on, HS off, LS off", i0);
  else rec(G_STACK, WARN, "sol T1 idle", "sense %.3f A at idle: wrong JP2 line, or ADC line driven by another board", i0);

  // T2 FIRE only
  Serial.println("  T2 FIRE on, PWM off, 1 s. Expect LED2 (HS) on, LED3 off, no click.");
  digitalWrite(FIRE, HIGH); delay(1000);
  float i2 = solAmps(ADCP);
  digitalWrite(FIRE, LOW); delay(50);
  if (i2 < 0.05f) rec(G_STACK, PASS, "sol T2 FIRE only", "%.3f A: Q2 not shorted. Confirm HS LED lit during it", i2);
  else rec(G_STACK, FAIL, "sol T2 FIRE only", "%.3f A with PWM off: Q2 (or U2 output) shorted on", i2);

  // T3 PWM only
  Serial.println("  T3 FIRE off, PWM 100 %, 1 s. Expect LED3 (LS) on, LED2 off, no click.");
  solPwm(PWM, 100); delay(1000);
  float i3 = solAmps(ADCP);
  solPwm(PWM, 0); delay(50);
  if (i3 < 0.05f) rec(G_STACK, PASS, "sol T3 PWM only", "%.3f A: Q1/Q3 not shorted. Confirm LS LED lit during it", i3);
  else rec(G_STACK, FAIL, "sol T3 PWM only", "%.3f A with FIRE off: Q1 or Q3 shorted on (valve would open on PWM alone)", i3);

  // T5 normal fires, 3 clicks
  Serial.printf("  T5 three fires, 0.5 s each, 1 s apart (%s). Listen for clicks.\n", coil24 ? "100 %" : "peak 100 ms then 35 %");
  float pk = 0, hold = 0, rel = 0;
  for (int n = 0; n < 3; n++) {
    digitalWrite(FIRE, HIGH);
    solPwm(PWM, 100);
    delay(coil24 ? 80 : 90);
    float a = solAmps(ADCP); if (a > pk) pk = a;
    if (!coil24) { delay(10); solPwm(PWM, holdPct); }
    delay(coil24 ? 420 : 400);
    float h = solAmps(ADCP) * 100.0f / holdPct;  // filtered reading is duty x coil current
    hold = h;
    digitalWrite(FIRE, LOW);                      // release per §9
    solPwm(PWM, 100); delay(20);
    rel = solAmps(ADCP);
    delay(5); solPwm(PWM, 0);
    delay(1000);
  }
  if (pk < 0.05f && !interactive) rec(G_STACK, WARN, "sol T5 fire", "no current: no valve on J1 (normal at this stage), or JP1 not on pair 1. 'v' tests interactively with a valve fitted");
  else if (pk < 0.05f) rec(G_STACK, FAIL, "sol T5 fire", "no current with FIRE and PWM on: open coil, F1 blown, limiter tripped/never armed, wrong JP1 row, or wrong JP2 line. Check TP3/TP4 and LED2/LED3");
  else if (pk > 3.2f) rec(G_STACK, FAIL, "sol T5 fire", "%.2f A peak: above 3.2 A, shorted coil or wrong coil type", pk);
  else rec(G_STACK, PASS, "sol T5 fire", "peak %.2f A, hold %.2f A (coil current), %.3f A after 20 ms release. Expect 3 clicks; 24 V / R_coil should match peak", pk, hold, rel);
  if (pk >= 0.05f) {
    if (rel < 0.1f * pk + 0.02f) rec(G_STACK, PASS, "sol release", "current gone within 20 ms of FIRE low (fast decay through D5)");
    else rec(G_STACK, WARN, "sol release", "%.3f A still flowing 20 ms after FIRE low: slow decay. Check D5 (SMBJ30CA, bidirectional) and coil for internal suppression", rel);
  }

  // T4 limiter (optional)
  if (interactive && readChoice("  T4 measure T_max? Holds the valve open up to 30 s. y/n", "yn", 'n') == 'y' && pk >= 0.05f) {
    Serial.println("  T4: FIRE + PWM held until the limiter trips (LED2 goes out). Any key aborts.");
    digitalWrite(FIRE, HIGH); solPwm(PWM, 100);
    uint32_t t0 = millis(); bool tripped = false, aborted = false;
    if (!coil24) { delay(100); solPwm(PWM, holdPct); }
    delay(300);
    float ref = solAmps(ADCP);
    while (millis() - t0 < 30000) {
      if (keyHit()) { aborted = true; break; }
      if (solAmps(ADCP) < 0.3f * ref) { tripped = true; break; }
      delay(10);
    }
    float t = (millis() - t0) / 1000.0f;
    solRelease(FIRE, PWM);
    if (aborted) rec(G_STACK, SKIP, "sol T4 T_max", "aborted at %.1f s", t);
    else if (tripped) rec(G_STACK, PASS, "sol T4 T_max", "limiter tripped at %.1f s. Spec range ~1.2-17 s via RV1; write it in the TMAX box. Firmware on-time must stay < %.1f s", t, t / 2);
    else rec(G_STACK, FAIL, "sol T4 T_max", "no trip in 30 s: RV1 near max with R10/RV1 open (backstop ~36-48 s), or limiter dead (Q4 D-S short, U1 stuck). Re-test with RV1 turned down");
  }
  chAllLow();
  Serial.println("  Done. All CH lines low.");
}

// ---------------------------------------------------------------- 'g' / 'm' igniter (spark) board
// Igniter spec Rev A §7 (thermocouple front end), §8 (firmware contract), §10 (bring-up).
// U3 ADS1115 at 0x48/0x49 (J7), U4 TMP1075 or LM75B at 0x4C/0x4D (J9). Output on while the
// JP1-selected CH line is high; no PWM, no current sense, no hardware limiter.
static bool reg16Write(uint8_t a, uint8_t reg, uint16_t v) {
  Wire.beginTransmission(a); Wire.write(reg); Wire.write(v >> 8); Wire.write(v & 0xFF);
  return Wire.endTransmission() == 0;
}
static uint8_t igAds[8], igTmp[8]; static int nAds = 0, nTmp = 0;
static void igFind() {
  nAds = nTmp = 0;
  for (uint8_t a = 0x48; a <= 0x4F; a++) {
    if (!probe(a)) continue;
    uint16_t lo = 0, hi = 0, id = 0;
    // ADS1115 threshold registers power up at 0x8000 / 0x7FFF; this firmware never changes them.
    if (regRead16be(a, 0x02, &lo) && regRead16be(a, 0x03, &hi) && lo == 0x8000 && hi == 0x7FFF) { igAds[nAds++] = a; continue; }
    {
      if (regRead16be(a, 0x0F, &id) && id == 0x7500) { igTmp[nTmp++] = a; continue; }   // TMP1075 device ID
      uint16_t t; if (regRead16be(a, 0x00, &t)) { float c = (int16_t)t / 256.0f; if (c > -40 && c < 125) igTmp[nTmp++] = a; }  // LM75-compatible
    }
  }
}
// One single-shot conversion: AIN0-AIN1, PGA +/-0.256 V, 16 SPS, comparator off.
static bool igReadTc(uint8_t a, int16_t* raw) {
  if (!reg16Write(a, 0x01, 0x8B23)) return false;
  uint32_t t0 = millis(); uint16_t cfg = 0;
  do { delay(5); if (!regRead16be(a, 0x01, &cfg)) return false; } while (!(cfg & 0x8000) && millis() - t0 < 250);
  if (!(cfg & 0x8000)) return false;
  uint16_t v; if (!regRead16be(a, 0x00, &v)) return false;
  *raw = (int16_t)v; return true;
}
static bool igReadCj(uint8_t a, float* c) { uint16_t t; if (!regRead16be(a, 0x00, &t)) return false; *c = (int16_t)t / 256.0f; return true; }
// Linear approximations near room temperature only (K ~40.7 uV/C, N ~26.5 uV/C). Good enough
// to see a warmed tip rise; real conversion uses NIST ITS-90 polynomials later.
static void igDescribe(int16_t raw, float cj, bool haveCj, char* out, size_t n) {
  if (raw == 32767) { snprintf(out, n, "pegged +full scale: probe OPEN (expected with nothing on J4)"); return; }
  if (raw == -32768) { snprintf(out, n, "pegged -full scale: probe reversed and hot, or input fault"); return; }
  float uv = raw * 7.8125f;
  if (haveCj) snprintf(out, n, "EMF %+.0f uV -> tip ~%.1f C if K, ~%.1f C if N (CJ %.2f C, linear approx)", uv, cj + uv / 40.7f, cj + uv / 26.5f, cj);
  else snprintf(out, n, "EMF %+.0f uV (no cold-junction sensor found)", uv);
}

static void testIgniter(bool interactive) {
  hdr("Igniter board: thermocouple ADC, cold-junction sensor, output switching");
  igFind();
  if (!nAds && !nTmp) { rec(G_STACK, FAIL, "ign I2C", "no ADS1115 or TMP1075/LM75B at 0x48-0x4F: board not seated, or U3/U4 not powered (+3V3 on J2 pin 2)"); }
  for (int i = 0; i < nAds; i++) rec(G_STACK, PASS, "ign ADS1115", "U3 at 0x%02X (J7 %s), thresholds at reset values", igAds[i], igAds[i] == 0x48 ? "default" : igAds[i] == 0x49 ? "alternate" : "restrapped");
  for (int i = 0; i < nTmp; i++) {
    float c = 0; bool ok = igReadCj(igTmp[i], &c);
    uint16_t id = 0; bool isT = regRead16be(igTmp[i], 0x0F, &id) && id == 0x7500;
    if (ok && c > 0 && c < 60) rec(G_STACK, PASS, "ign CJ sensor", "U4 %s at 0x%02X reads %.2f C (plausible bench temp)", isT ? "TMP1075" : "LM75-type", igTmp[i], c);
    else rec(G_STACK, WARN, "ign CJ sensor", "U4 at 0x%02X reads %s%.2f C (implausible for a bench)", igTmp[i], ok ? "" : "ERROR ", c);
  }
  if (nAds && !nTmp) rec(G_STACK, WARN, "ign CJ sensor", "ADS1115 found but no cold-junction sensor at 0x4C-0x4F");
  if (nTmp && !nAds) rec(G_STACK, WARN, "ign ADS1115", "cold-junction sensor found but no ADS1115 at 0x48-0x4B");

  for (int i = 0; i < nAds; i++) {
    int16_t raw; char d[140];
    float cj = 0; bool haveCj = (i < nTmp) && igReadCj(igTmp[i], &cj);
    if (!igReadTc(igAds[i], &raw)) { rec(G_STACK, FAIL, "ign TC read", "0x%02X conversion failed (no ready flag / NAK)", igAds[i]); continue; }
    igDescribe(raw, cj, haveCj, d, sizeof d);
    St st = (raw == 32767) ? INFO : (raw == -32768) ? WARN : PASS;
    rec(G_STACK, st, "ign TC read", "0x%02X raw %d: %s", igAds[i], raw, d);
  }

  // Output switching
  if (!interactive) {
    // No current sense on this board, so the output can only be checked by eye or meter. The CH
    // walk already drives each channel high for 0.5 s; that is the check in the full run.
    rec(G_STACK, INFO, "ign output", "not toggled here: the CH walk drove each CH high 0.5 s; LED2 (OUT) should have blinked on the JP1 channel. 'g' for a metered test");
    return;
  }
  char cc = interactive ? readChoice("  Output test. JP1 channel (row with the shunt)? 1-6, or n to skip", "123456n", '6') : '6';
  if (cc == 'n') { rec(G_STACK, SKIP, "ign output", "skipped"); return; }
  int ch = cc - '0';
  char nc = interactive ? readChoice("  How many cycles of 3 s ON / 3 s OFF? 1-5", "12345", '3') : '3';
  int cycles = nc - '0';
  uint8_t pin = PIN_CH[ch - 1];
  if (!g_5v) Serial.println("  WARNING: +5V/24 V looked absent at the last check. The output switches 24 V; connect it.");
  Serial.println("  Meter J3 + to - (TP3 to TP6). Expect ~24 V ON with the 1-3 link fitted (or the buck voltage), ~0 V OFF.");
  Serial.println("  U1 empty? Measure TP2 (IGN_SW, Q2 drain) to TP6 instead. LED2 (OUT) should follow ON/OFF.");
  chAllLow();
  uint32_t errs = 0, reads = 0;
  for (int n = 0; n < cycles; n++) {
    Serial.printf("    cycle %d: CH%d ON\n", n + 1, ch);
    digitalWrite(pin, HIGH);
    uint32_t t0 = millis();
    while (millis() - t0 < 3000) {               // keep the bus busy: spark-noise check for later
      for (int i = 0; i < nAds; i++) { int16_t r; reads++; if (!igReadTc(igAds[i], &r)) errs++; }
      for (int i = 0; i < nTmp; i++) { float c; reads++; if (!igReadCj(igTmp[i], &c)) errs++; }
      if (!nAds && !nTmp) delay(50);
      if (keyHit()) { n = cycles; break; }
    }
    digitalWrite(pin, LOW);
    Serial.printf("    cycle %d: CH%d OFF\n", n + 1 > cycles ? cycles : n + 1, ch);
    delay(3000);
  }
  chAllLow();
  rec(G_STACK, INFO, "ign output", "CH%d toggled %d x (3 s on / 3 s off). Pass = meter and LED2 followed it. I2C during ON: %lu reads, %lu errors",
      ch, cycles, (unsigned long)reads, (unsigned long)errs);
  if (errs) rec(G_STACK, WARN, "ign I2C during ON", "%lu I2C errors while the output was on", (unsigned long)errs);
}

static void tcMonitor() {
  hdr("Thermocouple monitor (any key stops). Warm the tip: reading should rise. Unplug: should peg +FS.");
  igFind();
  if (!nAds) { Serial.println("  no ADS1115 found at 0x48-0x4F"); return; }
  while (!keyHit()) {
    for (int i = 0; i < nAds; i++) {
      int16_t raw; float cj = 0; bool haveCj = (i < nTmp) && igReadCj(igTmp[i], &cj); char d[140];
      if (igReadTc(igAds[i], &raw)) { igDescribe(raw, cj, haveCj, d, sizeof d); Serial.printf("  0x%02X raw %6d  %s\n", igAds[i], raw, d); }
      else Serial.printf("  0x%02X read error\n", igAds[i]);
    }
    delay(400);
  }
}

// ---------------------------------------------------------------- summary / status
static void showStatus() {
  int o = worst(-1);
  if (o == FAIL) ledRGB(60, 0, 0); else if (o == WARN) ledRGB(40, 25, 0); else if (o == PASS) ledRGB(0, 50, 0); else ledRGB(0, 0, 40);
  if (!g_uiInit) return;
  auto paint = [](int led, int w) {
    if (w == FAIL) uiLed(led, true, false, false);
    else if (w == WARN) uiLed(led, true, true, false);
    else if (w == PASS) uiLed(led, false, true, false);
    else uiLed(led, false, false, true);
  };
  int uiW = worst(G_UI), olW = worst(G_OLED);
  paint(0, worst(G_BASE));
  paint(1, uiW);
  paint(2, olW);
  paint(3, o);
}

static void printSummary() {
  hdr("Summary");
  int c[5] = {0};
  for (int i = 0; i < nR; i++) c[R[i].st]++;
  for (int g = 0; g < G_N; g++) {
    bool any = false;
    for (int i = 0; i < nR; i++) if (R[i].g == g) {
      if (!any) { Serial.printf("  [%s]\n", GRP_TXT[g]); any = true; }
      Serial.printf("    %-4s %-18s %s\n", ST_TXT[R[i].st], R[i].name, R[i].detail);
    }
  }
  int o = worst(-1);
  Serial.printf("  Totals: %d pass, %d warn, %d fail, %d info, %d skip  ->  OVERALL %s\n",
                c[PASS], c[WARN], c[FAIL], c[INFO], c[SKIP], o == FAIL ? "FAIL" : o == WARN ? "WARN" : o == PASS ? "PASS" : "n/a");
  Serial.printf("  Config: UI board %s, OLED %s, +5V rail %s\n", g_uiPresent ? "present" : "absent",
                g_oledFound == 1 ? "present" : g_oledFound == 0 ? "absent" : "n/a", g_5v ? "present (24 V)" : "absent (USB only?)");
  Serial.println("  LEDs: module = overall. UI LED1 base, LED2 UI, LED3 OLED, LED4 overall (24 V only). Green pass, yellow warn, red fail, blue nothing graded.");
}

static void printMenu() {
  Serial.println(
    "\nCommands:\n"
    "  a  run full sequence           s  I2C lines + scan + address report\n"
    "  f  FRAM test                   u  UI board (U1 + LED walk)\n"
    "  o  OLED test                   b  buttons (interactive, forced)\n"
    "  c  CH1-6 checks + walk         w  CH1-6 PWM hold\n"
    "  x  ADC monitor                 l  WS2812 colour cycle\n"
    "  k  toggle I2C 100k/400k        r  reprint summary\n"
    "  p  +5V rail / DE pulldown      i  I2C diagnostic (FRAM addrs)\n"
    "  t  slow SDA/SCL toggle         v  solenoid board test\n"
    "  g  igniter board test          m  thermocouple monitor\n"
    "  d  DMX receive test (5 s)      e  DMX live monitor\n"
    "  h  this menu");
}

static void clearResults() { nR = 0; }

static void runAll() {
  clearResults();
  ledRGB(30, 20, 0);
  Serial.println("\n================ main board + UI board test ================");
  Serial.println("  CH1-CH6 WILL BE DRIVEN. Bare base board or gas off / no valves only.");
  bootInfo();
  testWs2812();
  testDmxRail();
  testI2CLines();
  if (g_uiInit) oledSleep();     // rerun: shut the panel down properly first
  g_uiInit = false; g_uiPresent = false; g_oledFound = -1;
  wireStart(I2C_SCAN_HZ); scan(ackBoot); wireStart(g_i2cHz);
  testFram();
  testUiBoard();
  testOled();
  addressReport();
  testButtons(g_uiPresent, false);
  testAdc();
  testSpare();
  testChannels(true);
  testDmx(false);

  // Daughterboards, non-interactive, only if detected
  igFind();
  if (nAds || nTmp) testIgniter(false);
  else rec(G_STACK, INFO, "igniter board", "not detected (no ADS1115 / TMP1075 at 0x48-0x4F); 'g' runs it anyway");
  if (solDetect()) {
    if (g_5v) testSolenoid(false);
    else rec(G_STACK, SKIP, "solenoid board", "detected on an ADC line but +5V/24 V absent (gate driver needs it); rerun with 24 V or 'v'");
  } else rec(G_STACK, INFO, "solenoid board", "not detected (no ADC line driven by ADC_SENSE; needs the JP2 shunt); 'v' runs it anyway");

  printSummary();
  showStatus();
  printMenu();
}

// ---------------------------------------------------------------- Arduino entry points
void setup() {
  // Safe states before anything else.
  for (int i = 0; i < 6; i++) { pinMode(PIN_CH[i], OUTPUT); digitalWrite(PIN_CH[i], LOW); }
  pinMode(PIN_DMX_DE, OUTPUT); digitalWrite(PIN_DMX_DE, LOW);
  pinMode(PIN_FRAM_WP, OUTPUT); digitalWrite(PIN_FRAM_WP, HIGH);
  pinMode(PIN_DMX_DI, INPUT);
  for (int i = 0; i < 3; i++) pinMode(PIN_BTN[i], INPUT_PULLUP);

  px.begin(); px.setBrightness(255); ledRGB(30, 20, 0);
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 4000) delay(10);   // run anyway if no host
  delay(200);
  runAll();
}

void loop() {
  static bool wasConn = (bool)Serial;
  bool conn = (bool)Serial;
  if (conn && !wasConn) { delay(300); Serial.println("\n(host connected: results of the last run)"); printSummary(); printMenu(); }
  wasConn = conn;

  if (g_oledOn && millis() - g_oledOnAt > OLED_IDLE_MS) { oledSleep(); Serial.println("  (OLED blanked after idle timeout)"); }

  if (!Serial.available()) return;
  char ch = Serial.read();
  if (ch == '\r' || ch == '\n' || ch == ' ') return;
  switch (ch) {
    case 'a': runAll(); return;
    case 's': clearResults(); testI2CLines(); addressReport(); break;
    case 'f': clearResults(); testFram(); break;
    case 'u': clearResults(); oledSleep(); testUiBoard(); break;
    case 'o': clearResults(); if (!g_uiPresent && probe(ADDR_TCA)) { g_uiPresent = true; uiInit(); } testOled(); break;
    case 'b': clearResults(); if (!g_uiInit && probe(ADDR_TCA)) { g_uiPresent = true; uiInit(); } testButtons(true, true); break;
    case 'p': clearResults(); testDmxRail(); break;
    case 'i': i2cDiag(); return;
    case 't': lineToggle(); return;
    case 'v': clearResults(); testSolenoid(true); break;
    case 'g': clearResults(); testIgniter(true); break;
    case 'm': tcMonitor(); return;
    case 'c': clearResults(); testChannels(true); break;
    case 'd': clearResults(); testDmxRail(); testDmx(true); break;
    case 'e': dmxMonitor(); return;
    case 'w': chPwmHold(); return;
    case 'x': adcMonitor(); return;
    case 'l': testWs2812(); showStatus(); return;
    case 'k': g_i2cHz = (g_i2cHz == 400000) ? 100000 : 400000; wireStart(g_i2cHz);
              Serial.printf("  functional I2C clock now %lu Hz\n", (unsigned long)g_i2cHz); return;
    case 'r': printSummary(); return;
    case 'h': case '?': printMenu(); return;
    default: Serial.printf("  unknown '%c'\n", ch); printMenu(); return;
  }
  printSummary();
  showStatus();
}
