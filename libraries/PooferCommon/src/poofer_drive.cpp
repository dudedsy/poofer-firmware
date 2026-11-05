// poofer_drive.cpp - see poofer_drive.h. Plain C++: no Arduino or pico SDK calls here.
#include "poofer_drive.h"

namespace poofer {

static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

const char* faultText(Fault f) {
  switch (f) {
    case Fault::NONE:            return "none";
    case Fault::OVER_CURRENT:    return "over-current (shorted load?)";
    case Fault::SENSE_SATURATED: return "current sense saturated (short on the J1 side?)";
    case Fault::NO_CURRENT:      return "no current (open load, F1, limiter tripped, wrong JP1 row / JP2 line, no 24 V)";
    case Fault::LOST_CURRENT:    return "current lost while on (hardware limiter tripped, F1, open load)";
  }
  return "?";
}

// ---------------------------------------------------------------- CurrentLoop
float CurrentLoop::step(float target, float iMeas, const LoadModel& m, float vbus, float dMin,
                        float dMax, float dt, const LoopGains& g) {
  float r = g.refRate * dt;
  if (target > ref_ + r) ref_ += r;
  else if (target < ref_ - r) ref_ -= r;
  else ref_ = target;

  float ff = m.dutyFor(ref_, vbus);
  float err = ref_ - iMeas;
  float integNew = clampf(integ_ + g.ki * err * dt, -1.0f, 1.0f);
  float want = ff + integNew + g.kp * err;
  float d = clampf(want, dMin, dMax);
  // conditional integration: freeze the integrator while clamped in the direction of the error
  bool pushingHigh = want > dMax && err > 0;
  bool pushingLow = want < dMin && err < 0;
  if (!pushingHigh && !pushingLow) integ_ = integNew;
  return d;
}

// ---------------------------------------------------------------- SolenoidDriver
const char* SolenoidDriver::stateText(State s) {
  static const char* t[] = {"OFF", "PEAK", "HOLD", "RELEASE", "FAULT"};
  return s <= FAULT ? t[s] : "?";
}

void SolenoidDriver::applyConfig() {
  if (cfg_.peakA > 2.8f) cfg_.peakA = 2.8f;           // spec §9
  if (cfg_.peakMs > 250) cfg_.peakMs = 250;            // spec §9
  if (cfg_.holdDutyMax > 1.0f) cfg_.holdDutyMax = 1.0f;
  model_ = {0.0f, cfg_.rCoil + cfg_.rPath, cfg_.vFree};
  float w = 6.2831853f * cfg_.crossoverHz, vEff = cfg_.vbus + cfg_.vFree;
  gains_ = {w * cfg_.lCoil / vEff, w * model_.r / vEff, cfg_.refRate};
  hw_.setPwmHz(cfg_.pwmHz);
}

void SolenoidDriver::begin(const SolenoidConfig& c) {
  cfg_ = c;
  hw_.setDuty(0);
  hw_.setFire(false);
  applyConfig();
  st_ = OFF;
  fault_ = Fault::NONE;
}

void SolenoidDriver::setHoldA(float a) { cfg_.holdA = a < 0 ? 0 : a; }

void SolenoidDriver::fire(uint32_t now) {
  if (st_ != OFF) return;
  float d0 = model_.dutyFor(cfg_.peakA, cfg_.vbus);
  hw_.setFire(true);                                   // spec §9: FIRE first preferred
  loop_.reset(d0, cfg_.peakA, model_, cfg_.vbus);
  hw_.setDuty(d0);
  overCnt_ = 0;
  lowSince_ = 0;
  rEst_ = 0;
  est_.reset();
  peakReached_ = false;
  tFire_ = now;
  enter(PEAK, now);
}

void SolenoidDriver::release(uint32_t now) {
  if (st_ != PEAK && st_ != HOLD) return;
  hw_.setFire(false);                                  // spec §9 release: FIRE low first,
  hw_.setDuty(1.0f);                                   // PWM 100 % so release energy goes into D5
  enter(RELEASE, now);
}

void SolenoidDriver::trip(Fault f, uint32_t now) {
  hw_.setFire(false);                                  // spec §9 diagnostics: drop FIRE and PWM, latch
  hw_.setDuty(0);
  fault_ = f;
  enter(FAULT, now);
}

void SolenoidDriver::clearFault() {
  if (st_ != FAULT) return;
  hw_.setFire(false);
  hw_.setDuty(0);
  fault_ = Fault::NONE;
  st_ = OFF;
}

void SolenoidDriver::tick(uint32_t now) {
  const float dt = 0.001f;
  vSense_ = hw_.senseVolts();
  iEst_ = est_.update(vSense_, hw_.duty());

  switch (st_) {
    case OFF:
    case FAULT:
      return;
    case RELEASE:
      if (now - tState_ >= 20) { hw_.setDuty(0); enter(OFF, now); }
      return;
    case PEAK: {
      // While the current is still rising on the coil's L/R time constant the error is large
      // and an integrator would wind up and overshoot the cap; hold it off until near target.
      if (iEst_ >= cfg_.peakA - 0.25f) peakReached_ = true;
      LoopGains g = gains_;
      if (!peakReached_) g.ki = 0;
      hw_.setDuty(loop_.step(cfg_.peakA, iEst_, model_, cfg_.vbus, DUTY_FLOOR, 1.0f, dt, g));
      if (now - tState_ >= cfg_.peakMs) {
        loop_.reset(hw_.duty(), iEst_, model_, cfg_.vbus);   // ramp down from where we are
        enter(HOLD, now);
      }
      break;
    }
    case HOLD: {
      float d = loop_.step(cfg_.holdA, iEst_, model_, cfg_.vbus, DUTY_FLOOR, cfg_.holdDutyMax, dt, gains_);
      hw_.setDuty(d);
      if (iEst_ > 0.2f) {
        float v = d * cfg_.vbus - (1.0f - d) * cfg_.vFree;
        rEst_ = v / iEst_ - cfg_.rPath;
      }
      break;
    }
  }

  // protection while energised (PEAK, HOLD)
  if (vSense_ > SENSE_SAT_V) { trip(Fault::SENSE_SATURATED, now); return; }
  if (iEst_ > cfg_.tripA) { if (++overCnt_ >= 3) { trip(Fault::OVER_CURRENT, now); return; } }
  else overCnt_ = 0;
  uint32_t onMs = now - tFire_;
  if (st_ == PEAK && onMs >= 20) {                     // spec §9: no current after 20 ms
    if (iEst_ < 0.1f) {
      if (!lowSince_) lowSince_ = now;
      if (now - lowSince_ >= 5) { trip(Fault::NO_CURRENT, now); return; }
    } else lowSince_ = 0;
  }
  if (st_ == HOLD && cfg_.holdA > 0.1f) {
    if (iEst_ < 0.3f * cfg_.holdA && hw_.duty() >= 0.95f * cfg_.holdDutyMax) {
      if (!lowSince_) lowSince_ = now;
      if (now - lowSince_ >= 20) { trip(Fault::LOST_CURRENT, now); return; }
    } else lowSince_ = 0;
  }
  if (cfg_.maxOnMs && onMs >= cfg_.maxOnMs) release(now);
}

// ---------------------------------------------------------------- LedDriver
const char* LedDriver::stateText(State s) {
  static const char* t[] = {"OFF", "RAMP", "RUN", "STOPPING", "REARM", "FAULT"};
  return s <= FAULT ? t[s] : "?";
}

void LedDriver::applyConfig() {
  if (cfg_.iMax > 2.4f) cfg_.iMax = 2.4f;
  model_ = {cfg_.vf0, cfg_.rd + cfg_.rPath, cfg_.vFree};
  if (target_ > cfg_.iMax) target_ = cfg_.iMax;
  hw_.setPwmHz(cfg_.pwmHz);
}

void LedDriver::begin(const LedConfig& c) {
  cfg_ = c;
  hw_.setDuty(0);
  hw_.setFire(false);
  applyConfig();
  st_ = OFF;
  fault_ = Fault::NONE;
}

float LedDriver::dutyMax() const {
  // what tripA would need at a 10 % low bus, plus margin
  float d = (cfg_.vf0 + (cfg_.rd + cfg_.rPath) * cfg_.tripA + cfg_.vFree) / (0.9f * cfg_.vbus + cfg_.vFree) + 0.03f;
  return d > 0.8f ? 0.8f : d;
}

void LedDriver::setTarget(float a) { target_ = clampf(a, 0.0f, cfg_.iMax); }

void LedDriver::on(uint32_t now) {
  if (st_ == FAULT || st_ == RAMP || st_ == RUN) return;
  hw_.setDuty(0);
  hw_.setFire(true);
  est_.reset();
  overCnt_ = 0;
  lowSince_ = 0;
  tOn_ = tRearm_ = now;
  enter(RAMP, now);
}

void LedDriver::off(uint32_t now) {
  if (st_ == OFF || st_ == FAULT || st_ == STOPPING) return;
  hw_.setDuty(0);                                      // L and C discharge into the LED
  enter(STOPPING, now);                                // FIRE low 20 ms later
}

void LedDriver::trip(Fault f, uint32_t now) {
  hw_.setDuty(0);
  fault_ = f;
  enter(FAULT, now);                                   // FIRE dropped 20 ms later in tick()
}

void LedDriver::clearFault() {
  if (st_ != FAULT) return;
  hw_.setFire(false);
  hw_.setDuty(0);
  fault_ = Fault::NONE;
  st_ = OFF;
}

void LedDriver::tick(uint32_t now) {
  const float dt = 0.001f;
  vSense_ = hw_.senseVolts();
  iEst_ = est_.update(vSense_, hw_.duty());
  uint32_t e = now - tState_;

  switch (st_) {
    case OFF:
      return;
    case STOPPING:
      if (e >= 20) { hw_.setFire(false); enter(OFF, now); }
      return;
    case FAULT:
      if (e >= 20) hw_.setFire(false);
      return;
    case REARM:                                        // duty already 0; FIRE low 40 ms re-arms C4
      if (e >= 5 && e < 45) hw_.setFire(false);
      if (e >= 45) { hw_.setFire(true); lowSince_ = 0; tRearm_ = now; enter(RAMP, now); }
      return;
    case RAMP: {
      // fast duty ramp while the LED is dark (capacitor charging), stopping just short of the
      // LED threshold; the loop brings the current up from there along the reference ramp
      float d = hw_.duty() + cfg_.rampPerS * dt;
      float dKnee = model_.dutyFor(0, cfg_.vbus) - 0.02f;
      if (d >= dKnee) {
        d = dKnee;
        loop_.reset(d, 0.0f, model_, cfg_.vbus);
        enter(RUN, now);
      }
      hw_.setDuty(d);
      break;
    }
    case RUN:
      hw_.setDuty(loop_.step(target_, iEst_, model_, cfg_.vbus, DUTY_FLOOR, dutyMax(), dt, cfg_.gains));
      break;
  }

  // protection while energised (RAMP, RUN)
  if (vSense_ > SENSE_SAT_V) { trip(Fault::SENSE_SATURATED, now); return; }
  if (iEst_ > cfg_.tripA) { if (++overCnt_ >= 3) { trip(Fault::OVER_CURRENT, now); return; } }
  else overCnt_ = 0;
  if (st_ == RUN && target_ > 0.05f) {
    // duty well past what should light the LED, but little current
    if (hw_.duty() > model_.dutyFor(0.3f * target_, cfg_.vbus) + 0.05f && iEst_ < 0.2f * target_) {
      if (!lowSince_) lowSince_ = now;
      if (now - lowSince_ >= 200) { trip(Fault::NO_CURRENT, now); return; }
    } else lowSince_ = 0;
  }
  if (cfg_.maxOnS > 0 && (now - tOn_) >= (uint32_t)(cfg_.maxOnS * 1000.0f)) { off(now); return; }
  if (st_ == RUN && cfg_.rearmS > 0 && (now - tRearm_) >= (uint32_t)(cfg_.rearmS * 1000.0f)) {
    hw_.setDuty(0);
    enter(REARM, now);
  }
}

}  // namespace poofer
