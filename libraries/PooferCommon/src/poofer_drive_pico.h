// poofer_drive_pico.h - RP2040 implementation of poofer::DriveHw for one solenoid driver
// board on the DMX base board Rev A (PooferCommon).
//
// pair 1..3 is the JP1 row: FIRE = CH(2n-1), PWM = CH(2n), i.e. GP6/GP7, GP8/GP9, GP10/GP11
// (solenoid spec §9). The PWM lines are the B outputs of slices 3, 4, 5; the FIRE pins are
// the A outputs of the same slices but stay plain GPIO. adcLine 0..3 is the JP2 line
// (GP26..GP29).
#pragma once
#ifdef ARDUINO
#include <Arduino.h>
#include "poofer_drive.h"

namespace poofer {

class PicoDriveHw : public DriveHw {
 public:
  // Configures the pins and drives FIRE and PWM low. Returns false on a bad pair/line.
  bool begin(int pair, int adcLine, uint32_t pwmHz);
  void setFire(bool on) override;
  void setDuty(float d) override;
  float duty() const override { return duty_; }
  float senseVolts() override;
  void setPwmHz(uint32_t hz) override;
  uint8_t firePin() const { return fire_; }
  uint8_t pwmPin() const { return pwm_; }
  uint8_t adcPin() const { return adc_; }
  uint32_t pwmSteps() const { return top_ + 1; }
  // Every CH line low as plain GPIO: call once at boot before any channel begins.
  static void allChannelsLow();
 private:
  uint8_t fire_ = 0, pwm_ = 0, adc_ = 0;
  unsigned slice_ = 0, chan_ = 0;
  uint32_t top_ = 0;
  float duty_ = 0;
  bool ready_ = false;
};

}  // namespace poofer
#endif
