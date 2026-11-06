// poofer_drive_pico.cpp - see poofer_drive_pico.h.
#if defined(ARDUINO) && defined(ARDUINO_ARCH_RP2040)
#include "poofer_drive_pico.h"
#include "poofer_board.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"

namespace poofer {

void PicoDriveHw::allChannelsLow() {
  for (uint8_t p : PIN_CH) {
    gpio_set_function(p, GPIO_FUNC_SIO);
    pinMode(p, OUTPUT);
    digitalWrite(p, LOW);
  }
}

bool PicoDriveHw::begin(int pair, int adcLine, uint32_t pwmHz) {
  if (pair < 1 || pair > 3 || adcLine < 0 || adcLine > 3) return false;
  if (ready_) { setDuty(0); setFire(false); gpio_set_function(pwm_, GPIO_FUNC_SIO); }
  fire_ = PIN_CH[2 * (pair - 1)];
  pwm_ = PIN_CH[2 * (pair - 1) + 1];
  adc_ = PIN_ADC[adcLine];
  pinMode(fire_, OUTPUT);
  digitalWrite(fire_, LOW);
  analogReadResolution(12);
  gpio_set_function(pwm_, GPIO_FUNC_PWM);
  slice_ = pwm_gpio_to_slice_num(pwm_);
  chan_ = pwm_gpio_to_channel(pwm_);
  ready_ = true;
  setPwmHz(pwmHz);
  return true;
}

void PicoDriveHw::setPwmHz(uint32_t hz) {
  if (!ready_) return;
  if (hz < 1000) hz = 1000;
  uint32_t sys = clock_get_hz(clk_sys);
  uint32_t top = sys / hz - 1;
  float div = 1.0f;
  while (top > 65534) {           // level top+1 (fully on) must fit in 16 bits
    div *= 2;
    top = (uint32_t)(sys / div / hz) - 1;
  }
  top_ = top;
  pwm_set_clkdiv(slice_, div);
  pwm_set_wrap(slice_, top_);
  pwm_set_enabled(slice_, true);
  setDuty(duty_);
}

void PicoDriveHw::setDuty(float d) {
  if (d < 0) d = 0;
  if (d > 1) d = 1;
  duty_ = d;
  if (!ready_) return;
  uint32_t level = (uint32_t)(d * (top_ + 1) + 0.5f);   // top_ + 1 = fully on
  pwm_set_chan_level(slice_, chan_, (uint16_t)(level > top_ + 1 ? top_ + 1 : level));
}

void PicoDriveHw::setFire(bool on) {
  if (ready_) digitalWrite(fire_, on ? HIGH : LOW);
}

float PicoDriveHw::senseVolts() {
  if (!ready_) return 0;
  uint32_t sum = 0;
  for (int k = 0; k < 8; k++) sum += analogRead(adc_);
  return sum / 8.0f * 3.3f / 4095.0f;
}

}  // namespace poofer
#endif
