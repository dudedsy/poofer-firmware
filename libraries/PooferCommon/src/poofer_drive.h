// poofer_drive.h - closed-loop current drive for solenoid driver boards (PooferCommon).
//
// One solenoid driver board (Rev A) is one channel: FIRE (odd CH line) switches the high
// side Q1 through the hardware max-on-time limiter, PWM (even CH line) chops the low side
// Q2, and the board's current sense (R18 25 mOhm x U3 gain 20 = 0.5 V/A, 1 ms RC) reports
// Q2 current on a JP2-selected ADC line. With an inductive load and D2 freewheeling back
// to +24V_F1 through Q1, the averaged sense is duty x load current, so
//     load current = sense / 0.5 V/A / duty.
// A PI loop on that estimate trims a feed-forward duty from a simple load model, tracking
// a reference that ramps toward the target at a set rate.
//
// Two load types share that core:
//   SolenoidDriver - a valve coil: peak, hold, release (solenoid spec Rev A §9).
//   LedDriver      - an LED behind an inductor + capacitor on J1 (shadowgraph light).
//
// This file and poofer_drive.cpp are plain C++ with no Arduino or pico SDK calls, so the
// logic builds and is tested on the host (libraries/PooferCommon/tests). Hardware access is
// through DriveHw; poofer_drive_pico.h provides the RP2040 implementation.
//
// Firmware is not a safety layer (solenoid spec §1, §9). The hardware limiter, FIRE gating
// and fuses stand on their own; these drivers keep the hardware inside its ratings and
// report faults.
#pragma once
#include <stdint.h>
#include <math.h>

namespace poofer {

static const float SENSE_V_PER_A = 0.5f;   // R18 x INA180A1 gain 20
static const float SENSE_SAT_V   = 3.0f;   // ADC_SENSE near the 3.3 V rail: ~6 A in Q2
static const float DUTY_FLOOR    = 0.05f;  // minimum duty while regulating (keeps current observable)

// ---------------------------------------------------------------- hardware interface
class DriveHw {
 public:
  virtual ~DriveHw() {}
  virtual void setFire(bool on) = 0;           // FIRE line (high side via limiter)
  virtual void setDuty(float d) = 0;           // PWM line, 0..1 (0 and 1 are static levels)
  virtual float duty() const = 0;              // last applied duty
  virtual float senseVolts() = 0;              // ADC_SENSE, volts (already 1 ms filtered on the board)
  virtual void setPwmHz(uint32_t hz) = 0;
};

// ---------------------------------------------------------------- load model and loop
// Load voltage = v0 + r * I. Averaged applied voltage = D * Vbus - (1 - D) * vFree, where
// vFree is the freewheel drop (D2 + Q1). Feed-forward solves for D.
struct LoadModel {
  float v0;      // V at zero current (LED knee; 0 for a coil)
  float r;       // ohm, load + inductor DCR + Q1 + Q2 + R18 + F1
  float vFree;   // V, freewheel path drop
  float dutyFor(float i, float vbus) const {
    float d = (v0 + r * i + vFree) / (vbus + vFree);
    return d < 0 ? 0 : (d > 1 ? 1 : d);
  }
};

struct LoopGains {
  float kp;          // duty per A
  float ki;          // duty per A.s
  float refRate;     // A/s, how fast the reference follows the target
};

// PI around a feed-forward duty. The reference ramps toward the target at refRate, so the
// loop never sees a large step it would wind up on; the integrator also stops (conditional
// integration) whenever the output is clamped and the error would push it further.
class CurrentLoop {
 public:
  // Bumpless start: the next output equals dutyNow, with the reference at refNow.
  void reset(float dutyNow, float refNow, const LoadModel& m, float vbus) {
    ref_ = refNow;
    integ_ = dutyNow - m.dutyFor(refNow, vbus);
  }
  // dMin keeps a little duty while energised so the current stays observable through the
  // sense (it only sees Q2 current); dMax caps the output (e.g. the hold duty ceiling).
  float step(float target, float iMeas, const LoadModel& m, float vbus, float dMin, float dMax,
             float dt, const LoopGains& g);
  float ref() const { return ref_; }
 private:
  float ref_ = 0, integ_ = 0;
};

// Load current from the averaged sense. The board's sense is an RC (1 ms) average of
// duty x load current, so it lags duty changes; dividing by the instantaneous duty turns
// every duty step into a false current step and the loop limit-cycles. The estimator runs
// the applied duty through the same 1 ms filter and divides by that instead. Below a few
// percent filtered duty the estimate is unobservable, so the last value is held.
struct CurrentEstimator {
  float tauMs = 1.0f;          // board RC: R19 10k x C9 0.1 uF
  float dFilt = 0, last = 0;
  void reset() { dFilt = 0; last = 0; }
  // dutyApplied: the duty held over the tick that just ended. Call once per 1 ms tick.
  float update(float senseV, float dutyApplied) {
    float a = 1.0f - expf(-1.0f / tauMs);
    dFilt += a * (dutyApplied - dFilt);
    if (dFilt > 0.03f) last = senseV / SENSE_V_PER_A / dFilt;
    return last;
  }
};

enum class Fault : uint8_t { NONE, OVER_CURRENT, SENSE_SATURATED, NO_CURRENT, LOST_CURRENT };
const char* faultText(Fault f);

// ---------------------------------------------------------------- solenoid
struct SolenoidConfig {
  float rCoil       = 8.5f;     // ohm, coil resistance (3/4" and 1" USS2, about 8.5)
  float rPath       = 0.15f;    // ohm, Q1 + Q2 + R18 + F1 + wiring
  float vFree       = 0.8f;     // V, D2 + Q1
  float vbus        = 24.0f;    // assumed bus voltage for feed-forward
  float peakA       = 2.7f;     // A, peak target. Spec §9: never above 2.8 A; 2.7 leaves room for ~0.1 A overshoot at a high bus
  uint32_t peakMs   = 100;      // ms, peak duration. Spec §9: default 100, never over 250
  float holdA       = 1.0f;     // A, hold target. Spec §9: >= 2 x measured dropout, <= rated
  float holdDutyMax = 0.5f;     // hold duty ceiling: 0.5 for 12 V coils on 24 V, 1.0 for 24 V coils
  float tripA       = 3.2f;     // A, shorted-coil trip (spec §9 diagnostics)
  uint32_t maxOnMs  = 3000;     // firmware on-time limit; keep < T_max / 2 (spec §9)
  uint32_t pwmHz    = 20000;    // spec §9: 20 kHz
  // Loop gains are derived from the coil model in applyConfig(): the PI zero cancels the
  // coil's L/R pole and the loop crosses over at crossoverHz, so a different coil only needs
  // rCoil and lCoil. kp = w L / (Vbus + vFree), ki = w R / (Vbus + vFree).
  float lCoil       = 0.1f;     // H, coil inductance estimate (unmeasured; spec §14 item 9)
  float crossoverHz = 20.0f;    // well below the 1 ms sense filter and 1 kHz loop
  float refRate     = 1000.0f;  // A/s, peak -> hold reference ramp
};

class SolenoidDriver {
 public:
  enum State : uint8_t { OFF, PEAK, HOLD, RELEASE, FAULT };
  explicit SolenoidDriver(DriveHw& hw) : hw_(hw) {}
  void begin(const SolenoidConfig& c);
  void fire(uint32_t nowMs);           // FIRE high, peak then hold. Ignored unless OFF
  void release(uint32_t nowMs);        // spec §9: FIRE low, PWM 100 % for 20 ms, then PWM low
  void tick(uint32_t nowMs);           // call every 1 ms
  void clearFault();
  void setHoldA(float a);              // takes effect immediately in HOLD
  SolenoidConfig& config() { return cfg_; }
  void applyConfig();                  // after editing config() while OFF
  State state() const { return st_; }
  Fault fault() const { return fault_; }
  float current() const { return iEst_; }       // coil current estimate, A
  float senseVolts() const { return vSense_; }
  float resistance() const { return rEst_; }    // coil resistance estimate in HOLD, ohm (0 if unknown)
  const LoopGains& gains() const { return gains_; }
  static const char* stateText(State s);
 private:
  void enter(State s, uint32_t now) { st_ = s; tState_ = now; }
  void trip(Fault f, uint32_t now);
  DriveHw& hw_;
  SolenoidConfig cfg_;
  LoadModel model_ = {0, 8.65f, 0.8f};
  LoopGains gains_ = {0.5f, 44.0f, 1000.0f};
  CurrentLoop loop_;
  CurrentEstimator est_;
  State st_ = OFF;
  Fault fault_ = Fault::NONE;
  uint32_t tState_ = 0, tFire_ = 0, lowSince_ = 0;
  int overCnt_ = 0;
  bool peakReached_ = false;   // integrator held off in PEAK until the current nears the target
  float iEst_ = 0, vSense_ = 0, rEst_ = 0;
};

// ---------------------------------------------------------------- LED (inductor + capacitor on J1)
struct LedConfig {
  float vf0       = 10.81f;    // V, LED fit Vf = vf0 + rd x I (XHP70.2 12 V, measured 2026-11-04)
  float rd        = 0.57f;     // ohm
  float rPath     = 0.15f;     // ohm, inductor DCR + Q1 + Q2 + R18 + F1
  float vFree     = 0.8f;      // V, D2 + Q1
  float vbus      = 24.0f;
  float iMax      = 2.3f;      // A, highest allowed target (LED max 2.4 A)
  float tripA     = 2.6f;      // A, software over-current trip
  float maxOnS    = 20.0f;     // s, firmware on-time limit (heat), 0 = none
  float rearmS    = 0.0f;      // s, re-arm the hardware limiter while on, 0 = off. LED ONLY
  float rampPerS  = 5.0f;      // start-up duty ramp while the LED is dark
  uint32_t pwmHz  = 50000;
  LoopGains gains = {0.003f, 2.0f, 30.0f};  // plant ~32 A/duty, ~10 Hz crossover
};

class LedDriver {
 public:
  enum State : uint8_t { OFF, RAMP, RUN, STOPPING, REARM, FAULT };
  explicit LedDriver(DriveHw& hw) : hw_(hw) {}
  void begin(const LedConfig& c);
  void on(uint32_t nowMs);
  void off(uint32_t nowMs);            // duty 0 first, FIRE low 20 ms later
  void setTarget(float a);             // capped at iMax
  void tick(uint32_t nowMs);           // call every 1 ms
  void clearFault();
  LedConfig& config() { return cfg_; }
  void applyConfig();                  // after editing config() while OFF
  State state() const { return st_; }
  Fault fault() const { return fault_; }
  float target() const { return target_; }
  float current() const { return iEst_; }
  float senseVolts() const { return vSense_; }
  float dutyFF() const { return model_.dutyFor(target_, cfg_.vbus); }
  float dutyMax() const;
  static const char* stateText(State s);
 private:
  void enter(State s, uint32_t now) { st_ = s; tState_ = now; }
  void trip(Fault f, uint32_t now);
  DriveHw& hw_;
  LedConfig cfg_;
  LoadModel model_ = {10.81f, 0.72f, 0.8f};
  CurrentLoop loop_;
  CurrentEstimator est_;
  State st_ = OFF;
  Fault fault_ = Fault::NONE;
  float target_ = 1.0f;
  uint32_t tState_ = 0, tOn_ = 0, tRearm_ = 0, lowSince_ = 0;
  int overCnt_ = 0;
  float iEst_ = 0, vSense_ = 0;
};

}  // namespace poofer
