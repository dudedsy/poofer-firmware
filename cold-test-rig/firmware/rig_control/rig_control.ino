/*
  rig_control.ino - cold-test rig controller on the DMX base board Rev A.

  Records 2-3 pressure sensors and the drive current senses at 5 kHz, runs one valve
  (closed-loop peak and hold, PooferCommon SolenoidDriver) and the shadowgraph LED buck
  (LedDriver), sequences each shot on the board's own 1 ms clock, blinks a timecode on a
  signal LED for video sync, and reads the igniter board's thermocouple. All logic is in
  rig_core.cpp (shared with the simulator in ../../tools/sim); this file is the RP2040
  platform: DMA ADC, PWM pins, I2C, buttons, USB serial.

  Wiring (defaults, change with 'set'):
    ADC0 (J3.19)  pressure 1          ADC1 (J3.21)  pressure 2
    ADC2 (J3.23)  LED board JP2, or pressure 3 with no LED board
    ADC3 (J3.25)  valve board JP2
    valve board JP1 row 1 (CH1 FIRE / CH2 PWM), LED board JP1 row 2 (CH3 / CH4)
    signal LED: SPARE1 (J3.29, GP14) -> ~150 ohm -> LED -> GND
    igniter board (optional): ADS1115 0x48 thermocouple, TMP1075/LM75B 0x4C cold junction
    any UI button: abort (valve released, LED off)

  The laptop tool (cold-test-rig/host, 'rigctl') drives this over USB serial.

  Toolchain: Earle Philhower RP2040 core 6.3.0, board "Waveshare RP2040 Zero".
*/

#include <Arduino.h>
#include <Wire.h>
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/gpio.h"
#include <poofer_board.h>
#include <poofer_drive.h>
#include <poofer_drive_pico.h>
#include "rig_core.h"

using namespace poofer;

// ---------------------------------------------------------------- drives
class RigHw : public PicoDriveHw {
 public:
  void setSense(const float* s) { sense_ = s; }
  bool enable(int pair, int adcLine, uint32_t hz) {
    if (pair == 0) { PicoDriveHw::setDuty(0); PicoDriveHw::setFire(false); on_ = false; return true; }
    on_ = PicoDriveHw::begin(pair, adcLine, hz);
    return on_;
  }
  void setFire(bool on) override { if (on_) PicoDriveHw::setFire(on); }
  void setDuty(float d) override { if (on_) PicoDriveHw::setDuty(d); }
  float senseVolts() override { return (on_ && sense_) ? sense_[adcPin() - PIN_ADC[0]] : 0.0f; }
 private:
  const float* sense_ = nullptr;
  bool on_ = false;
};


// ---------------------------------------------------------------- output queue
static const size_t OUTQ = 16384;
static uint8_t outq[OUTQ];
static size_t oh = 0, ot = 0;     // head (write), tail (read)
static size_t qUsed() { return (oh + OUTQ - ot) % OUTQ; }

class Pico : public rig::Platform {
 public:
  RigHw* hw[2] = {nullptr, nullptr};
  bool tcConfigured = false;
  size_t outSpace() override { return OUTQ - 1 - qUsed(); }
  void out(const uint8_t* p, size_t n) override {
    for (size_t k = 0; k < n; k++) { outq[oh] = p[k]; oh = (oh + 1) % OUTQ; }
  }
  bool beginDrive(int which, int pair, int adcLine, uint32_t hz) override { return hw[which]->enable(pair, adcLine, hz); }
  bool tcRead(int16_t& raw) override {
    if (!tcConfigured) {
      // ADS1115: AIN0-AIN1 differential, PGA +-0.256 V, continuous, 128 SPS, comparator off
      Wire.beginTransmission(0x48);
      Wire.write(0x01); Wire.write(0x0A); Wire.write(0x83);
      if (Wire.endTransmission() != 0) return false;
      tcConfigured = true;
    }
    Wire.beginTransmission(0x48);
    Wire.write(0x00);
    if (Wire.endTransmission(false) != 0 || Wire.requestFrom(0x48, 2) != 2) { tcConfigured = false; return false; }
    uint8_t hi = Wire.read(), lo = Wire.read();
    raw = (int16_t)((hi << 8) | lo);
    return true;
  }
  bool cjRead(float& c) override {
    Wire.beginTransmission(0x4C);
    Wire.write(0x00);
    if (Wire.endTransmission(false) != 0 || Wire.requestFrom(0x4C, 2) != 2) return false;
    uint8_t hi = Wire.read(), lo = Wire.read();
    c = (int16_t)((hi << 8) | lo) / 16 * 0.0625f;   // TMP1075 12 bit; LM75B reads the same to 0.125 C
    return true;
  }
  void syncLed(bool on) override { digitalWrite(PIN_SPARE1, on ? HIGH : LOW); }
};

static Pico pico;
static RigHw valveHw, ledHw;
static rig::Core coreObj(pico, valveHw, ledHw);
static rig::Core* const core = &coreObj;

static void flushOut() {
  if (!Serial) { ot = oh; return; }          // no host: drop, so a late connect gets fresh data
  while (qUsed()) {
    int room = Serial.availableForWrite();
    if (room <= 0) return;
    size_t run = (oh >= ot) ? oh - ot : OUTQ - ot;
    if (run > (size_t)room) run = room;
    Serial.write(&outq[ot], run);
    ot = (ot + run) % OUTQ;
  }
}

// ---------------------------------------------------------------- ADC DMA (as burst_capture)
static const int NBLK = 32;
static uint16_t ring[NBLK * rig::SAMP_PER_MS] __attribute__((aligned(4)));
static int dma_a, dma_b;
static volatile uint32_t blocks_done = 0;
static uint32_t processed = 0, overruns = 0;

static inline uint16_t* block_ptr(uint32_t b) { return &ring[(b % NBLK) * rig::SAMP_PER_MS]; }

static void dma_irq_handler() {
  uint32_t ints = dma_hw->ints1;
  if (!(ints & ((1u << dma_a) | (1u << dma_b)))) return;
  for (int k = 0; k < 2; k++) {
    int ch = k ? dma_b : dma_a;
    if (ints & (1u << ch)) {
      dma_hw->ints1 = 1u << ch;
      uint32_t done = blocks_done + 1;
      blocks_done = done;
      dma_channel_set_write_addr(ch, block_ptr(done + 1), false);
    }
  }
}

static void config_channel(int ch, int chain_to, uint16_t* dst) {
  dma_channel_config c = dma_channel_get_default_config(ch);
  channel_config_set_transfer_data_size(&c, DMA_SIZE_16);
  channel_config_set_read_increment(&c, false);
  channel_config_set_write_increment(&c, true);
  channel_config_set_dreq(&c, DREQ_ADC);
  channel_config_set_chain_to(&c, chain_to);
  dma_channel_configure(ch, &c, dst, &adc_hw->fifo, rig::SAMP_PER_MS, false);
}

static void start_acq() {
  adc_init();
  for (int k = 0; k < 4; k++) adc_gpio_init(PIN_ADC[k]);
  adc_set_temp_sensor_enabled(true);
  adc_run(false);
  adc_fifo_drain();
  adc_select_input(0);
  adc_set_round_robin(0x1F);                          // ADC0..3 + temperature
  adc_fifo_setup(true, true, 1, true, false);         // FIFO, DREQ, thresh 1, error bit 15
  adc_set_clkdiv(48000000.0f / (rig::SAMP_PER_MS * 1000.0f) - 1.0f);   // 100 ksps
  dma_a = dma_claim_unused_channel(true);
  dma_b = dma_claim_unused_channel(true);
  irq_add_shared_handler(DMA_IRQ_1, dma_irq_handler, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
  dma_channel_set_irq1_enabled(dma_a, true);
  dma_channel_set_irq1_enabled(dma_b, true);
  irq_set_enabled(DMA_IRQ_1, true);
  config_channel(dma_a, dma_b, block_ptr(0));
  config_channel(dma_b, dma_a, block_ptr(1));
  dma_hw->ints1 = (1u << dma_a) | (1u << dma_b);
  dma_channel_start(dma_a);
  adc_run(true);
}

// ---------------------------------------------------------------- buttons, serial input
static uint8_t btnLowMs = 0;
static bool btnLatched = false;
static void pollButtons() {
  bool low = false;
  for (uint8_t p : PIN_BTN) low |= digitalRead(p) == LOW;
  if (low) { if (btnLowMs < 255) btnLowMs++; } else { btnLowMs = 0; btnLatched = false; }
  if (btnLowMs >= 20 && !btnLatched) { btnLatched = true; core->button(); }
}

static void pollSerial() {
  static char line[256];
  static size_t n = 0;
  while (Serial.available()) {
    int c = Serial.read();
    if (c == '\n' || c == '\r') { if (n) { line[n] = 0; core->line(line); n = 0; } }
    else if (n < sizeof line - 1) line[n++] = (char)c;
  }
}

void setup() {
  PicoDriveHw::allChannelsLow();
  pinMode(PIN_DMX_DE, OUTPUT);
  digitalWrite(PIN_DMX_DE, LOW);
  pinMode(PIN_SPARE1, OUTPUT);
  digitalWrite(PIN_SPARE1, LOW);
  gpio_set_drive_strength(PIN_SPARE1, GPIO_DRIVE_STRENGTH_12MA);
  for (uint8_t p : PIN_BTN) pinMode(p, INPUT_PULLUP);
  Wire.setSDA(PIN_SDA);
  Wire.setSCL(PIN_SCL);
  Wire.begin();
  Wire.setClock(400000);
  Wire.setTimeout(5);
  Serial.begin(115200);

  valveHw.setSense(core->sense);
  ledHw.setSense(core->sense);
  pico.hw[0] = &valveHw;
  pico.hw[1] = &ledHw;
  core->begin();
  start_acq();
}

void loop() {
  uint32_t done = blocks_done;
  if (done - processed >= (uint32_t)(NBLK - 2)) {   // fell behind the DMA: skip, keep the newest
    overruns++;
    processed = done - 1;
    char b[48];
    snprintf(b, sizeof b, "#adc overrun %lu", (unsigned long)overruns);
    if (pico.outSpace() > 64) { pico.out((const uint8_t*)b, strlen(b)); pico.out((const uint8_t*)"\n", 1); }
  }
  while (processed < done) {
    core->block(block_ptr(processed));
    processed++;
    pollButtons();
  }
  pollSerial();
  flushOut();
}
