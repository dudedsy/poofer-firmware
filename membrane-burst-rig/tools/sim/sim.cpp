// sim.cpp - host simulator for burst_capture firmware.
//
// Builds the real sketch sources against mock Arduino / Wire / ADC / DMA / flash
// layers, with emulated UI-board devices on I2C (TCA9555 at 0x20, SSD1315 at 0x3C)
// and the base board FRAM (0x50). Virtual time; the ADC produces a modelled sensor
// signal through the same DMA ring / IRQ path the firmware uses on hardware.
//
// stdin  : bytes for the board's USB serial, except lines starting with '!', which
//          are simulator commands:
//            !wait <ms>                run the firmware for <ms> of virtual time
//            !btn <UP|DN|SET> <ms>     hold a button for <ms>, then release + 60 ms
//            !p <psi> [ramp_ms]        move the rig pressure to <psi> (linear ramp)
//            !burst                    membrane bursts now (0.5 ms blowdown)
//            !screen <file.pbm>        write the OLED's display RAM as an image
//            !leds                     print the four RGB LED colours to stderr
//            !mark <text>              echo <text> to stderr (log sync)
//            !tca_por                  power-on-reset the TCA9555 only (supply dip)
//            !glitch <ch> <counts> <ms>  add <counts> to one channel for <ms>
//            !noise <counts>           per-sample noise sigma (default 1.5)
//            !quit
// stdout : the board's USB serial output (binary safe)
// stderr : simulator diagnostics; lines starting with "SIMERR" are bugs caught by
//          the device models (I2C to a panel in reset, display on without VBAT, ...)
//
// Options: --bare (no I2C pull-ups: bare RP2040-Zero), --speed <x> (free-running
// virtual time per real time, default 20), --state <file> (load flash + FRAM from
// <file> if it exists and save them there on !quit, to test power cycles).
#include <Arduino.h>
#include <Wire.h>
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/flash.h"

#include <atomic>
#include <chrono>
#include <deque>
#include <iostream>
#include <mutex>
#include <random>
#include <string>
#include <thread>
#include <unistd.h>

void setup();
void loop();

// ---------------------------------------------------------------- time
static uint64_t vt_us = 0;
static void sim_advance(uint64_t us);
uint32_t millis() { return (uint32_t)(vt_us / 1000); }
void delay(uint32_t ms) { sim_advance((uint64_t)ms * 1000); }
void delayMicroseconds(uint32_t us) { sim_advance(us); }

// ---------------------------------------------------------------- gpio
static bool bare = false;
static bool btn_down[3];
static int pin_out[32];
void pinMode(int, int) {}
int digitalRead(int pin) {
  if (pin == 12 || pin == 13) return bare ? LOW : HIGH;
  if (pin >= 3 && pin <= 5) return btn_down[pin - 3] ? LOW : HIGH;
  return pin_out[pin];
}
void digitalWrite(int pin, int v) { if (pin >= 0 && pin < 32) pin_out[pin] = v; }

// ---------------------------------------------------------------- serial
SimSerial Serial;
static std::mutex rx_mu;
static std::deque<uint8_t> rx;
SimSerial::operator bool() const { return true; }
int SimSerial::available() { std::lock_guard<std::mutex> l(rx_mu); return (int)rx.size(); }
int SimSerial::read() {
  std::lock_guard<std::mutex> l(rx_mu);
  if (rx.empty()) return -1;
  int c = rx.front(); rx.pop_front(); return c;
}
int SimSerial::availableForWrite() { return 4096; }
size_t SimSerial::write(uint8_t b) { fputc(b, stdout); return 1; }
size_t SimSerial::write(const uint8_t *p, size_t n) {
  fwrite(p, 1, n, stdout);
  sim_advance(n / 1000 + 1);            // ~1 MB/s USB full speed
  return n;
}
size_t SimSerial::print(const char *s) { return write((const uint8_t *)s, strlen(s)); }
size_t SimSerial::print(double v, int digits) { char b[48]; snprintf(b, sizeof b, "%.*f", digits, v); return print(b); }

// ---------------------------------------------------------------- flash
uint8_t sim_flash[2 * 1024 * 1024];
uint32_t sim_binary_end_offset() { return 92 * 1024; }
void flash_range_erase(uint32_t off, size_t len) {
  if (off % FLASH_SECTOR_SIZE || len % FLASH_SECTOR_SIZE) fprintf(stderr, "SIMERR erase misaligned %u %zu\n", off, len);
  if (off < sim_binary_end_offset()) fprintf(stderr, "SIMERR erase inside the sketch at %u\n", off);
  memset(sim_flash + off, 0xFF, len);
  sim_advance(45000 * (len / FLASH_SECTOR_SIZE));
}
void flash_range_program(uint32_t off, const uint8_t *data, size_t len) {
  if (off % FLASH_PAGE_SIZE || len % FLASH_PAGE_SIZE) fprintf(stderr, "SIMERR program misaligned %u %zu\n", off, len);
  for (size_t i = 0; i < len; i++) sim_flash[off + i] &= data[i];   // NOR: 1 -> 0 only
  sim_advance(400 * (len / FLASH_PAGE_SIZE));
}

// ---------------------------------------------------------------- ADC + DMA
static sim_adc_hw_t adc_regs;
sim_adc_hw_t *adc_hw = &adc_regs;
static sim_dma_hw_t dma_regs;
sim_dma_hw_t *dma_hw = &dma_regs;
static irq_handler_t dma_irq = nullptr;
static bool adc_running = false;
static uint32_t rr_mask = 1;
struct SimDma { uint16_t *dst = nullptr; uint32_t count = 0; int chain = -1; bool claimed = false; };
static SimDma dch[12];
static int dma_active = -1;
static uint64_t next_block_us = 0;
static uint64_t acq_sample = 0;          // samples since adc_run(true)
static uint64_t acq_t0_us = 0;

static bool temp_en = false;
static int adc_input = 0;
void adc_init() {}
void adc_gpio_init(int) {}
void adc_run(bool run) {
  if (run && !adc_running) { acq_sample = 0; acq_t0_us = vt_us; next_block_us = vt_us + 10000; }
  adc_running = run;
}
void adc_fifo_drain() {}
void adc_select_input(int ch) { adc_input = ch; }
void adc_set_round_robin(uint32_t m) { rr_mask = m ? m : 1; }
void adc_fifo_setup(bool, bool, int, bool, bool) {}
void adc_set_clkdiv(float) {}
void adc_set_temp_sensor_enabled(bool en) { temp_en = en; }
uint16_t adc_read() {                     // one-shot: only the die temperature is modelled
  if (adc_running) fprintf(stderr, "SIMERR adc_read while free-running\n");
  if (adc_input != 4 || !temp_en) { fprintf(stderr, "SIMERR adc_read of input %d (temp sensor %s)\n", adc_input, temp_en ? "on" : "off"); return 0; }
  sim_advance(2);
  return 881;                             // 0.7098 V: 25 C by the datasheet formula
}

dma_channel_config dma_channel_get_default_config(int) { return dma_channel_config{-1}; }
void channel_config_set_transfer_data_size(dma_channel_config *, int) {}
void channel_config_set_read_increment(dma_channel_config *, bool) {}
void channel_config_set_write_increment(dma_channel_config *, bool) {}
void channel_config_set_dreq(dma_channel_config *, int) {}
void channel_config_set_chain_to(dma_channel_config *c, int ch) { c->chain_to = ch; }
void dma_channel_configure(int ch, const dma_channel_config *c, volatile void *dst, const volatile void *,
                           uint32_t count, bool start) {
  dch[ch].dst = (uint16_t *)dst; dch[ch].count = count; dch[ch].chain = c->chain_to;
  dma_hw->ch[ch].al1_ctrl = 1;
  if (start) dma_active = ch;
}
void dma_channel_start(int ch) { dma_active = ch; }
void dma_channel_abort(int ch) { if (dma_active == ch) dma_active = -1; }
void dma_channel_set_write_addr(int ch, volatile void *dst, bool) { dch[ch].dst = (uint16_t *)dst; }
int dma_claim_unused_channel(bool) { for (int i = 0; i < 12; i++) if (!dch[i].claimed) { dch[i].claimed = true; return i; } return -1; }
void dma_channel_set_irq1_enabled(int, bool) {}
void irq_add_shared_handler(int, irq_handler_t h, int) { dma_irq = h; }
void irq_set_enabled(int, bool) {}

// ---------------------------------------------------------------- rig model
static const double CPP = 0.8 * 4095.0 / 100.0;
static double zero_counts[3] = {410, 406, 413};
static double tau_s[3] = {0.5e-3, 1.5e-3, 4.0e-3};      // sensor lags
static double gain[3] = {1.0, 0.97, 1.03};               // sensor gain errors (calibration tests)
static double p_from = 0, p_to = 0;                       // psig
static uint64_t ramp_t0 = 0, ramp_us = 1;
static bool bursting = false;
static uint64_t burst_t0 = 0;
static double p_burst = 0;
static double sens[3] = {0, 0, 0};
static std::mt19937 rng(1);
static std::normal_distribution<double> noise(0.0, 1.0);
static double noise_counts = 1.5;                         // per-sample noise sigma (!noise)
static int glitch_ch = -1;                                // !glitch: offset one channel briefly
static double glitch_counts = 0;
static uint64_t glitch_t0 = 0, glitch_us = 0;

static double rig_psi(uint64_t t) {
  if (bursting) {
    if (t < burst_t0) return p_burst;
    return p_burst * exp(-(double)(t - burst_t0) / 500.0);   // tau 0.5 ms
  }
  if (t >= ramp_t0 + ramp_us) return p_to;
  if (t <= ramp_t0) return p_from;
  return p_from + (p_to - p_from) * (double)(t - ramp_t0) / (double)ramp_us;
}

static void produce_block() {
  SimDma &d = dch[dma_active];
  int nch = __builtin_popcount(rr_mask);
  for (uint32_t i = 0; i < d.count; i++) {
    uint64_t k = acq_sample++;
    int c = (int)(k % nch);
    double t_us = acq_t0_us + k * (1e6 / 300000.0);
    double p = rig_psi((uint64_t)t_us);
    double dt = nch / 300000.0;
    sens[c] += (p - sens[c]) * (1.0 - exp(-dt / tau_s[c]));
    double v = zero_counts[c] + sens[c] * CPP * gain[c] + noise(rng) * noise_counts;
    if (c == glitch_ch && t_us >= glitch_t0 && t_us < glitch_t0 + glitch_us) v += glitch_counts;
    int counts = (int)lround(v);
    if (counts < 0) counts = 0;
    if (counts > 4095) counts = 4095;
    d.dst[i] = (uint16_t)counts;
  }
  dma_hw->ints1.v |= 1u << dma_active;
  int finished = dma_active;
  dma_active = d.chain;                  // chained channel starts
  if (dma_irq) dma_irq();
  (void)finished;
}

static void sim_advance(uint64_t us) {
  uint64_t end = vt_us + us;
  while (adc_running && dma_active >= 0 && next_block_us <= end) {
    vt_us = next_block_us;
    produce_block();
    next_block_us += 10000;
  }
  vt_us = end;
}

// ---------------------------------------------------------------- I2C devices
struct Tca { uint8_t reg = 0, out[2] = {0xFF, 0xFF}, cfg[2] = {0xFF, 0xFF}; } tca;
static bool tca_bit(int port, int bit) {   // effective pin level (outputs or pulled)
  if (tca.cfg[port] & (1 << bit)) return false;   // input: pulled down by board resistor
  return tca.out[port] & (1 << bit);
}
static bool oled_res_high() { return tca_bit(0, 7); }
static bool vbat_on() { return tca_bit(0, 6); }

struct Oled {
  uint8_t ram[8][128];
  int col = 0, page = 0, c0 = 0, c1 = 127, p0 = 0, p1 = 7, mode = 2;
  bool on = false, pump = false, iref_internal = false;
  bool was_reset = true;
} oled;

static void oled_reset() { oled.on = false; oled.pump = false; oled.mode = 2; oled.was_reset = true; }

static int cmd_args(uint8_t c) {
  switch (c) {
    case 0x21: case 0x22: return 2;
    case 0xD5: case 0x81: case 0xD9: case 0xA8: case 0xD3: case 0xDA: case 0xDB: case 0xAD: case 0x20: case 0x8D: return 1;
    default: return 0;
  }
}

static void oled_rx(const uint8_t *b, size_t n) {
  if (n == 0) return;
  if (b[0] == 0x40) {                     // data
    for (size_t i = 1; i < n; i++) {
      oled.ram[oled.page][oled.col] = b[i];
      if (oled.mode == 0) {
        if (++oled.col > oled.c1) { oled.col = oled.c0; if (++oled.page > oled.p1) oled.page = oled.p0; }
      } else if (oled.col < 127) oled.col++;
    }
    return;
  }
  if (b[0] != 0x00) { fprintf(stderr, "SIMERR oled control byte 0x%02X\n", b[0]); return; }
  for (size_t i = 1; i < n;) {
    uint8_t c = b[i];
    int na = cmd_args(c);
    if (i + na >= n) { fprintf(stderr, "SIMERR oled command 0x%02X missing args\n", c); return; }
    const uint8_t *a = &b[i + 1];
    switch (c) {
      case 0xAE: oled.on = false; break;
      case 0xAF:
        if (!vbat_on()) fprintf(stderr, "SIMERR display on with VBAT off\n");
        if (!oled.pump) fprintf(stderr, "SIMERR display on with charge pump off\n");
        oled.on = true; break;
      case 0x8D: oled.pump = (a[0] & 0x04) != 0; break;
      case 0x20: oled.mode = a[0] & 3; break;
      case 0x21: oled.c0 = a[0] & 127; oled.c1 = a[1] & 127; oled.col = oled.c0; break;
      case 0x22: oled.p0 = a[0] & 7; oled.p1 = a[1] & 7; oled.page = oled.p0; break;
      case 0xAD: oled.iref_internal = (a[0] & 0x30) != 0; break;
      default:
        if (c >= 0xB0 && c <= 0xB7) oled.page = c & 7;
        break;
    }
    i += 1 + na;
  }
}

static uint8_t fram[8192];
static uint16_t fram_ptr = 0;

SimWire Wire;
void SimWire::beginTransmission(int a) { addr_ = a; ntx_ = 0; }
size_t SimWire::write(uint8_t b) { if (ntx_ < sizeof tx_) tx_[ntx_++] = b; else fprintf(stderr, "SIMERR wire tx overflow\n"); return 1; }
size_t SimWire::write(const uint8_t *p, size_t n) { for (size_t i = 0; i < n; i++) write(p[i]); return n; }
uint8_t SimWire::endTransmission(bool) {
  sim_advance((ntx_ + 1) * 9 * 1000000ULL / 400000ULL);
  if (ntx_ > 256) fprintf(stderr, "SIMERR wire transaction %zu bytes > 256 buffer\n", ntx_);
  switch (addr_) {
    case 0x20:
      if (bare) return 2;
      if (ntx_ >= 1) {
        tca.reg = tx_[0];
        for (size_t i = 1; i < ntx_; i++) {
          uint8_t r = tca.reg;
          bool res_before = oled_res_high(), vbat_before = vbat_on();
          if (r == 2 || r == 3) tca.out[r - 2] = tx_[i];
          if (r == 6 || r == 7) tca.cfg[r - 6] = tx_[i];
          tca.reg = (r & ~1) | ((r + 1) & 1);
          if (res_before && !oled_res_high()) oled_reset();
          if (vbat_before && !vbat_on() && oled.on) fprintf(stderr, "SIMERR VBAT removed with display on\n");
        }
      }
      return 0;
    case 0x3C:
      if (bare || !oled_res_high()) return 2;    // in reset: no ACK
      oled_rx(tx_, ntx_);
      return 0;
    case 0x50:
      if (bare) return 2;
      if (ntx_ >= 2) {
        fram_ptr = ((tx_[0] << 8) | tx_[1]) & 0x1FFF;
        if (ntx_ > 2) {
          if (pin_out[15] != LOW) { fprintf(stderr, "SIM fram write ignored (WP high)\n"); return 0; }
          for (size_t i = 2; i < ntx_; i++) { fram[fram_ptr] = tx_[i]; fram_ptr = (fram_ptr + 1) & 0x1FFF; }
        }
      }
      return 0;
    default:
      return 2;
  }
}
size_t SimWire::requestFrom(int a, size_t n, bool) {
  nrx_ = 0; rpos_ = 0;
  if (bare) return 0;
  if (a == 0x50) { for (size_t i = 0; i < n; i++) { rx_[nrx_++] = fram[fram_ptr]; fram_ptr = (fram_ptr + 1) & 0x1FFF; } }
  else if (a == 0x20) {
    for (size_t i = 0; i < n; i++) {
      uint8_t r = tca.reg, v = 0;
      if (r <= 1) v = 0;
      else if (r <= 3) v = tca.out[r - 2];
      else if (r >= 6 && r <= 7) v = tca.cfg[r - 6];
      rx_[nrx_++] = v;
      tca.reg = (r & ~1) | ((r + 1) & 1);
    }
  }
  else return 0;
  sim_advance((n + 1) * 9 * 1000000ULL / 400000ULL);
  return nrx_;
}
int SimWire::read() { return rpos_ < nrx_ ? rx_[rpos_++] : -1; }
int SimWire::available() { return (int)(nrx_ - rpos_); }

// ---------------------------------------------------------------- commands
static std::mutex cmd_mu;
static std::deque<std::string> cmds;
static std::atomic<bool> eof{false};

static void reader() {
  std::string line;
  while (std::getline(std::cin, line)) {
    // Serial lines go through the same queue as simulator commands so a script's
    // order is kept; they reach the board's RX buffer when their turn comes.
    std::lock_guard<std::mutex> l(cmd_mu);
    cmds.push_back(line);
  }
  eof = true;
}

static void run_for(uint64_t us) {
  uint64_t end = vt_us + us;
  while (vt_us < end) { loop(); sim_advance(100); }
  fflush(stdout);
}

static void write_screen(const std::string &path) {
  FILE *f = fopen(path.c_str(), "wb");
  if (!f) return;
  bool lit = oled.on && oled.pump && vbat_on() && oled_res_high();
  fprintf(f, "P1\n128 64\n");
  for (int y = 0; y < 64; y++) {
    for (int x = 0; x < 128; x++) fputs(lit && ((oled.ram[y >> 3][x] >> (y & 7)) & 1) ? "1 " : "0 ", f);
    fputc('\n', f);
  }
  fclose(f);
  fprintf(stderr, "SIM screen %s lit=%d iref_internal=%d t=%.3fs\n", path.c_str(), lit, oled.iref_internal, vt_us / 1e6);
}

static const char *led_name(int bits) {
  static const char *n[] = {"off", "red", "green", "amber", "blue", "magenta", "cyan", "white"};
  return n[bits & 7];
}

static bool do_cmd(const std::string &c) {
  char a[256] = "";
  if (c.empty() || c[0] != '!') {                 // board serial input
    { std::lock_guard<std::mutex> l(rx_mu); for (char ch : c) rx.push_back((uint8_t)ch); rx.push_back('\n'); }
    run_for(20000);                               // let the board handle it
    return true;
  }
  double x = 0, y = 0;
  if (sscanf(c.c_str(), "!wait %lf", &x) == 1) run_for((uint64_t)(x * 1000));
  else if (sscanf(c.c_str(), "!btn %31s %lf", a, &x) == 2) {
    int i = !strcmp(a, "UP") ? 0 : !strcmp(a, "DN") ? 1 : 2;
    btn_down[i] = true; run_for((uint64_t)(x * 1000));
    btn_down[i] = false; run_for(60000);
  } else if (sscanf(c.c_str(), "!p %lf %lf", &x, &y) >= 1) {
    p_from = rig_psi(vt_us); p_to = x; ramp_t0 = vt_us; ramp_us = (uint64_t)(std::max(y, 0.001) * 1000);
    bursting = false;
  } else if (c == "!tca_por") {             // supply dip resets the expander only
    bool on = oled.on;
    tca = Tca();
    oled_reset();                              // RES# falls as the pins become inputs
    fprintf(stderr, "SIM tca power-on reset (display was %s)\n", on ? "on" : "off");
  } else if (sscanf(c.c_str(), "!glitch %d %lf %lf", &glitch_ch, &x, &y) == 3) {
    glitch_counts = x; glitch_t0 = vt_us; glitch_us = (uint64_t)(y * 1000);   // ch, counts, ms
  } else if (sscanf(c.c_str(), "!noise %lf", &x) == 1) {
    noise_counts = x;
  } else if (c == "!burst") {
    p_burst = rig_psi(vt_us); bursting = true; burst_t0 = vt_us;
  } else if (sscanf(c.c_str(), "!screen %255s", a) == 1) write_screen(a);
  else if (c == "!leds") {
    int o0 = ~tca.out[0], o1 = ~tca.out[1];
    fprintf(stderr, "SIM leds %s %s %s %s\n", led_name(o0), led_name(o0 >> 3), led_name(o1), led_name(o1 >> 3));
  } else if (sscanf(c.c_str(), "!mark %255[^\n]", a) == 1) fprintf(stderr, "SIM mark %s\n", a);
  else if (c == "!quit") return false;
  else fprintf(stderr, "SIM unknown command %s\n", c.c_str());
  fflush(stdout);
  return true;
}

static void state_io(const char *path, bool save) {
  if (!path) return;
  FILE *f = fopen(path, save ? "wb" : "rb");
  if (!f) return;
  if (save) { fwrite(sim_flash, 1, sizeof sim_flash, f); fwrite(fram, 1, sizeof fram, f); }
  else if (fread(sim_flash, 1, sizeof sim_flash, f) != sizeof sim_flash || fread(fram, 1, sizeof fram, f) != sizeof fram)
    fprintf(stderr, "SIM state file short\n");
  fclose(f);
}

int main(int argc, char **argv) {
  double speed = 20;
  const char *state = nullptr;
  memset(sim_flash, 0xFF, sizeof sim_flash);
  memset(fram, 0x00, sizeof fram);
  for (int i = 1; i < argc; i++) {
    if (!strcmp(argv[i], "--bare")) bare = true;
    else if (!strcmp(argv[i], "--speed") && i + 1 < argc) speed = atof(argv[++i]);
    else if (!strcmp(argv[i], "--state") && i + 1 < argc) state = argv[++i];
  }
  state_io(state, false);
  for (int i = 0; i < 3; i++) pin_out[i] = 0;
  std::thread(reader).detach();
  setup();
  fflush(stdout);
  auto real0 = std::chrono::steady_clock::now();
  uint64_t vt0 = vt_us;
  for (;;) {
    std::string c;
    bool have = false;
    {
      std::lock_guard<std::mutex> l(cmd_mu);
      if (!cmds.empty()) { c = cmds.front(); cmds.pop_front(); have = true; }
    }
    if (have) {
      if (!do_cmd(c)) break;
      real0 = std::chrono::steady_clock::now(); vt0 = vt_us;
      continue;
    }
    if (eof) { std::lock_guard<std::mutex> l(cmd_mu); if (cmds.empty()) break; }
    // free-run: keep virtual time at `speed` x real time
    loop(); sim_advance(100);
    fflush(stdout);
    double real_us = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - real0).count();
    if ((double)(vt_us - vt0) > real_us * speed) usleep(200);
  }
  fflush(stdout);
  state_io(state, true);
  return 0;
}
