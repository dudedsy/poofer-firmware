// drive_sim.cpp - host tests for poofer_drive (LedDriver, SolenoidDriver) against
// averaged plant models of a solenoid driver board. Build and run: make -C <this dir> test
//
// Plants (averaged over a PWM period, integrated at 2 us, controller ticked every 1 ms):
//   LED  : J1 -> L -> (C || LED), LED = 0 below vf0, (v - vf0) / rd above.
//   coil : J1 -> L + R.
// Applied voltage with Q1 on: D * Vbus - (1 - D) * vFree (D2 + Q1 freewheel).
// With Q1 off (FIRE low or hardware limiter tripped) any load current is driven into the
// D5 clamp, about -33 V, and cannot reverse.
// Sense: RC 1 ms on duty x load current (Q2 current averaged), plus optional noise.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <vector>
#include "../src/poofer_drive.h"

using namespace poofer;

static int g_fail = 0, g_pass = 0;
#define CHECK(cond, ...) do { if (cond) { g_pass++; } else { g_fail++; printf("  FAIL: "); printf(__VA_ARGS__); printf("\n"); } } while (0)

struct Plant {
  // parameters
  bool led = true;
  float vbus = 24.0f, L = 170e-6f, C = 940e-6f, rPath = 0.15f, vFree = 0.8f;
  float vf0 = 10.81f, rd = 0.57f;          // LED
  float rCoil = 8.5f;                      // coil
  bool open = false;                       // open load
  int limiterTripMs = -1;                  // hardware limiter opens Q1 this long after FIRE rises (-1 = never)
  float noiseV = 0.0f;
  // state
  float il = 0, vc = 0, sense = 0;
  bool fire = false;
  float duty = 0;
  int fireMs = 0;
  float iLoad() const {
    if (!led) return il;
    return vc > vf0 ? (vc - vf0) / rd : 0.0f;
  }
  void advance1ms() {
    const float dt = 2e-6f;
    bool q1 = fire && !(limiterTripMs >= 0 && fireMs >= limiterTripMs);
    for (int k = 0; k < 500; k++) {
      float vApp;
      if (q1) vApp = duty * vbus - (1.0f - duty) * vFree;
      else vApp = il > 0 ? -33.0f : 0.0f;
      if (open) { il = 0; }
      else if (led) {
        il += (vApp - vc - il * rPath) / L * dt;
        if (il < 0) il = 0;
        vc += (il - iLoad()) / C * dt;
        if (vc < 0) vc = 0;
      } else {
        il += (vApp - il * (rCoil + rPath)) / L * dt;
        if (il < 0) il = 0;
      }
      sense += (duty * il * SENSE_V_PER_A - sense) * dt / 1e-3f;
    }
    if (fire) fireMs++; else fireMs = 0;
  }
};

class SimHw : public DriveHw {
 public:
  explicit SimHw(Plant& p) : p_(p) {}
  void setFire(bool on) override { p_.fire = on; fireLog.push_back({t, on}); }
  void setDuty(float d) override { p_.duty = d < 0 ? 0 : (d > 1 ? 1 : d); }
  float duty() const override { return p_.duty; }
  float senseVolts() override {
    float n = p_.noiseV > 0 ? p_.noiseV * ((rand() / (float)RAND_MAX) * 2 - 1) : 0;
    float v = p_.sense + n;
    return v < 0 ? 0 : (v > 3.3f ? 3.3f : v);
  }
  void setPwmHz(uint32_t) override {}
  uint32_t t = 0;
  struct Ev { uint32_t t; bool on; };
  std::vector<Ev> fireLog;
 private:
  Plant& p_;
};

struct Sample { uint32_t t; float iLoad, iEst, duty; bool fire; int state; };

// ---------------------------------------------------------------- LED scenarios
struct LedRun {
  std::vector<Sample> s;
  LedDriver::State end;
  Fault fault;
};

static LedRun runLed(Plant p, float target, uint32_t ms, LedConfig cfg = LedConfig(),
                     std::function<void(uint32_t, LedDriver&, Plant&)> script = nullptr) {
  p.led = true;
  SimHw hw(p);
  LedDriver d(hw);
  d.begin(cfg);
  d.setTarget(target);
  d.on(0);
  LedRun r;
  for (uint32_t t = 1; t <= ms; t++) {
    hw.t = t;
    p.advance1ms();
    if (script) script(t, d, p);
    d.tick(t);
    r.s.push_back({t, p.iLoad(), d.current(), p.duty, p.fire, (int)d.state()});
  }
  r.end = d.state();
  r.fault = d.fault();
  return r;
}

static float peakAfter(const std::vector<Sample>& s, uint32_t t0) {
  float m = 0;
  for (auto& x : s) if (x.t >= t0 && x.iLoad > m) m = x.iLoad;
  return m;
}
// first time after t0 from which the load current stays within tol of target
static int settleMs(const std::vector<Sample>& s, uint32_t t0, float target, float tol) {
  int last = -1;
  for (auto& x : s) {
    if (x.t < t0) continue;
    if (std::fabs(x.iLoad - target) > tol) last = (int)x.t;
  }
  return last < 0 ? 0 : last + 1 - (int)t0;
}

static void ledTests() {
  printf("LED driver\n");
  for (float vb : {22.0f, 24.0f, 26.0f}) {
    for (float L : {120e-6f, 170e-6f, 220e-6f}) {
      Plant p; p.vbus = vb; p.L = L; p.noiseV = 0.003f;
      auto r = runLed(p, 2.0f, 800);
      float pk = peakAfter(r.s, 0), fin = r.s.back().iLoad;
      int st = settleMs(r.s, 0, 2.0f, 0.04f);
      printf("  vbus %.0f L %.0f uH: final %.3f A, peak %.3f A, settle %d ms\n", vb, L * 1e6, fin, pk, st);
      CHECK(r.end == LedDriver::RUN, "LED state %s at vbus %.0f", LedDriver::stateText(r.end), vb);
      CHECK(std::fabs(fin - 2.0f) < 0.04f, "LED final %.3f A at vbus %.0f L %.0f", fin, vb, L * 1e6);
      CHECK(pk < 2.10f, "LED overshoot %.3f A at vbus %.0f L %.0f", pk, vb, L * 1e6);
      CHECK(st < 450, "LED settle %d ms at vbus %.0f L %.0f", st, vb, L * 1e6);
    }
  }
  {  // estimate vs truth in steady state
    Plant p; auto r = runLed(p, 1.5f, 600);
    float e = r.s.back().iEst, t = r.s.back().iLoad;
    CHECK(std::fabs(e - t) < 0.03f, "LED estimate %.3f vs true %.3f", e, t);
  }
  {  // setpoint steps
    Plant p;
    auto r = runLed(p, 1.0f, 1400, LedConfig(), [](uint32_t t, LedDriver& d, Plant&) {
      if (t == 600) d.setTarget(2.0f);
      if (t == 1000) d.setTarget(0.5f);
    });
    float pk = peakAfter(r.s, 600);
    int s1 = settleMs(std::vector<Sample>(r.s.begin(), r.s.begin() + 999), 600, 2.0f, 0.04f);
    int s2 = settleMs(r.s, 1000, 0.5f, 0.02f);
    printf("  steps 1->2 A settle %d ms (peak %.3f), 2->0.5 A settle %d ms\n", s1, pk, s2);
    CHECK(pk < 2.10f, "LED step overshoot %.3f", pk);
    CHECK(s1 < 200 && s2 < 200, "LED step settle %d / %d ms", s1, s2);
  }
  {  // off: duty 0 first, FIRE low >= 20 ms later
    Plant p; p.led = true;
    SimHw hw(p); LedDriver d(hw); d.begin(LedConfig()); d.setTarget(1.5f); d.on(0);
    uint32_t tDuty0 = 0, tFireLow = 0;
    for (uint32_t t = 1; t <= 900; t++) {
      hw.t = t; p.advance1ms();
      if (t == 600) d.off(t);
      d.tick(t);
      if (t >= 600 && !tDuty0 && p.duty == 0) tDuty0 = t;
      if (t >= 600 && !tFireLow && !p.fire) tFireLow = t;
    }
    CHECK(d.state() == LedDriver::OFF, "LED not OFF after off()");
    CHECK(tDuty0 == 600 && tFireLow >= 620, "LED off order: duty 0 at %u, FIRE low at %u", tDuty0, tFireLow);
    CHECK(p.iLoad() < 0.01f, "LED current %.3f A after off", p.iLoad());
  }
  {  // open LED -> NO_CURRENT
    Plant p; p.open = true;
    auto r = runLed(p, 1.5f, 1000);
    CHECK(r.end == LedDriver::FAULT && r.fault == Fault::NO_CURRENT, "open LED: state %s fault %s",
          LedDriver::stateText(r.end), faultText(r.fault));
  }
  {  // hardware limiter trips while on -> NO_CURRENT, FIRE dropped
    Plant p; p.limiterTripMs = 2000;
    auto r = runLed(p, 1.5f, 3000, [] { LedConfig c; c.maxOnS = 0; return c; }());
    bool fireLow = !r.s.back().fire;
    CHECK(r.end == LedDriver::FAULT && r.fault == Fault::NO_CURRENT && fireLow,
          "LED limiter trip: state %s fault %s fire %d", LedDriver::stateText(r.end), faultText(r.fault), !fireLow);
  }
  {  // rearm keeps it alive past a 2 s hardware limiter, FIRE low >= 15 ms each time
    Plant p; p.limiterTripMs = 2000;
    LedConfig c; c.maxOnS = 0; c.rearmS = 1.5f;
    auto r = runLed(p, 1.5f, 6000, c);
    int minLow = 100000, run = 0, rearms = 0;
    for (auto& x : r.s) {
      if (!x.fire) run++;
      else { if (run > 0) { rearms++; if (run < minLow) minLow = run; } run = 0; }
    }
    CHECK(r.end == LedDriver::RUN && std::fabs(r.s.back().iLoad - 1.5f) < 0.05f,
          "LED rearm: state %s, %.3f A", LedDriver::stateText(r.end), r.s.back().iLoad);
    CHECK(rearms >= 3 && minLow >= 15, "LED rearm: %d re-arms, shortest FIRE-low %d ms", rearms, minLow);
  }
  {  // firmware on-time limit
    Plant p; LedConfig c; c.maxOnS = 1.0f;
    auto r = runLed(p, 1.0f, 1200, c);
    CHECK(r.end == LedDriver::OFF, "LED maxOn: state %s", LedDriver::stateText(r.end));
  }
  {  // shorted LED (knee at 0 V): over-current or sense saturation trip, never sustained > trip
    Plant p; p.vf0 = 0.5f; p.rd = 0.05f;
    auto r = runLed(p, 1.5f, 1000);
    CHECK(r.end == LedDriver::FAULT, "shorted LED: state %s", LedDriver::stateText(r.end));
  }
}

// ---------------------------------------------------------------- solenoid scenarios
struct SolRun {
  std::vector<Sample> s;
  SolenoidDriver::State end;
  Fault fault;
  float rEst;
};

static SolRun runSol(Plant p, uint32_t releaseAt, uint32_t ms, SolenoidConfig cfg = SolenoidConfig(),
                     SimHw** hwOut = nullptr) {
  p.led = false;
  if (p.L < 1e-3f) p.L = 0.1f;                 // coil default ~0.1 H (est.)
  static SimHw* keep = nullptr;
  SimHw* hw = new SimHw(p);
  SolenoidDriver d(*hw);
  d.begin(cfg);
  d.fire(0);
  SolRun r;
  for (uint32_t t = 1; t <= ms; t++) {
    hw->t = t;
    p.advance1ms();
    if (t == releaseAt) d.release(t);
    d.tick(t);
    r.s.push_back({t, p.il, d.current(), p.duty, p.fire, (int)d.state()});
  }
  r.end = d.state();
  r.fault = d.fault();
  r.rEst = d.resistance();
  if (hwOut) *hwOut = hw; else delete hw;
  (void)keep;
  return r;
}

static void solenoidTests() {
  printf("Solenoid driver\n");
  for (float vb : {22.0f, 24.0f, 26.0f})
    for (float L : {0.05f, 0.1f, 0.2f})
      for (float R : {8.5f, 10.5f}) {          // 10.5 ohm ~ hot coil (+23 %)
        Plant p; p.vbus = vb; p.L = L; p.rCoil = R; p.noiseV = 0.003f;
        SolenoidConfig c;
        auto r = runSol(p, 600, 700, c);
        float pkI = 0; for (auto& x : r.s) if (x.t <= 100 && x.iLoad > pkI) pkI = x.iLoad;
        float maxAll = 0; for (auto& x : r.s) if (x.iLoad > maxAll) maxAll = x.iLoad;
        float holdMaxDuty = 0;
        for (size_t k = 1; k < r.s.size(); k++)   // skip the PEAK->HOLD transition tick (peak duty still applied)
          if (r.s[k].state == SolenoidDriver::HOLD && r.s[k - 1].state == SolenoidDriver::HOLD && r.s[k].duty > holdMaxDuty) holdMaxDuty = r.s[k].duty;
        int st = settleMs(std::vector<Sample>(r.s.begin(), r.s.begin() + 599), 100, 1.0f, 0.05f);
        float hold = r.s[598].iLoad;
        float relI = r.s[619].iLoad;
        float reachable = std::fmin(c.peakA, vb / (R + 0.15f));
        bool fullyStable = true;
        printf("  vbus %.0f L %.2f R %.1f: peak %.2f A (reach %.2f), hold %.3f A settle %d ms, hold duty max %.2f, 20 ms after release %.3f A, R est %.2f\n",
               vb, L, R, pkI, reachable, hold, st, holdMaxDuty, relI, r.rEst);
        CHECK(maxAll <= 2.8f, "sol above the 2.8 A spec cap:  %.2f A (vbus %.0f L %.2f R %.1f)", maxAll, vb, L, R);
        if (L <= 0.1f) CHECK(pkI >= 0.9f * reachable, "sol peak %.2f < 90%% of %.2f (vbus %.0f L %.2f R %.1f)", pkI, reachable, vb, L, R);
        CHECK(std::fabs(hold - 1.0f) < 0.05f, "sol hold %.3f A (vbus %.0f L %.2f R %.1f)", hold, vb, L, R);
        CHECK(st < 120, "sol hold settle %d ms (vbus %.0f L %.2f R %.1f)", st, vb, L, R);
        CHECK(holdMaxDuty <= c.holdDutyMax + 1e-4f, "sol hold duty %.3f > ceiling", holdMaxDuty);
        CHECK(relI < 0.05f, "sol release: %.3f A 20 ms after release", relI);
        CHECK(r.end == SolenoidDriver::OFF, "sol end state %s", SolenoidDriver::stateText(r.end));
        if (vb == 24.0f) CHECK(std::fabs(r.rEst - R) < 0.1f * R, "sol R estimate %.2f vs %.2f", r.rEst, R);
        (void)fullyStable;
      }
  {  // release order: FIRE low first, duty 100 % >= 20 ms, then 0
    Plant p; SimHw* hw = nullptr;
    auto r = runSol(p, 400, 450, SolenoidConfig(), &hw);
    bool ok = true; uint32_t tLow = 0;
    for (auto& x : r.s) {
      if (x.t >= 400 && !x.fire && !tLow) tLow = x.t;
      if (x.t >= 400 && x.t < 419 && x.duty < 0.999f) ok = false;
      if (x.t >= 421 && x.duty != 0) ok = false;
    }
    CHECK(tLow == 400 && ok, "sol release order: FIRE low at %u, duty pattern ok %d", tLow, ok);
    delete hw;
  }
  {  // 2" valve style: lower R, peak capped by current, not duty
    Plant p; p.rCoil = 5.0f;
    SolenoidConfig c; c.rCoil = 5.0f; c.peakA = 1.9f; c.holdA = 0.8f;
    auto r = runSol(p, 600, 650, c);
    float mx = 0; for (auto& x : r.s) if (x.iLoad > mx) mx = x.iLoad;
    CHECK(mx <= 1.98f, "low-R coil peak %.2f A, cap 1.9", mx);
    CHECK(std::fabs(r.s[598].iLoad - 0.8f) < 0.05f, "low-R coil hold %.3f A", r.s[598].iLoad);
  }
  {  // config R wrong by 2x (feed-forward off): loop still lands on hold
    Plant p; p.rCoil = 8.5f;
    SolenoidConfig c; c.rCoil = 17.0f;
    auto r = runSol(p, 600, 650, c);
    CHECK(std::fabs(r.s[598].iLoad - 1.0f) < 0.05f, "mis-set R: hold %.3f A", r.s[598].iLoad);
  }
  {  // shorted coil -> OVER_CURRENT or SENSE_SATURATED, FIRE and PWM dropped
    Plant p; p.rCoil = 1.0f; p.L = 0.02f;
    auto r = runSol(p, 0, 200);
    CHECK(r.end == SolenoidDriver::FAULT && (r.fault == Fault::OVER_CURRENT || r.fault == Fault::SENSE_SATURATED) &&
          !r.s.back().fire && r.s.back().duty == 0, "shorted coil: state %s fault %s", SolenoidDriver::stateText(r.end), faultText(r.fault));
  }
  {  // open coil -> NO_CURRENT within ~25 ms
    Plant p; p.open = true;
    auto r = runSol(p, 0, 100);
    int tf = -1; for (auto& x : r.s) if (x.state == SolenoidDriver::FAULT) { tf = (int)x.t; break; }
    CHECK(r.fault == Fault::NO_CURRENT && tf > 0 && tf <= 30, "open coil: fault %s at %d ms", faultText(r.fault), tf);
  }
  {  // hardware limiter trips during hold -> LOST_CURRENT
    Plant p; p.limiterTripMs = 1500;
    SolenoidConfig c; c.maxOnMs = 0;
    auto r = runSol(p, 0, 2000, c);
    CHECK(r.end == SolenoidDriver::FAULT && r.fault == Fault::LOST_CURRENT, "limiter trip in hold: state %s fault %s",
          SolenoidDriver::stateText(r.end), faultText(r.fault));
  }
  {  // firmware on-time limit releases
    Plant p; SolenoidConfig c; c.maxOnMs = 500;
    auto r = runSol(p, 0, 600, c);
    CHECK(r.end == SolenoidDriver::OFF, "sol maxOn: state %s", SolenoidDriver::stateText(r.end));
  }
  {  // spec limits enforced on config
    Plant p; SolenoidConfig c; c.peakA = 5.0f; c.peakMs = 1000;
    p.led = false; SimHw hw(p); SolenoidDriver d(hw); d.begin(c);
    CHECK(d.config().peakA <= 2.8f && d.config().peakMs <= 250, "spec caps: peak %.2f A %u ms", d.config().peakA, d.config().peakMs);
  }
}

int main() {
  srand(1);
  ledTests();
  solenoidTests();
  printf("\n%d checks passed, %d failed\n", g_pass, g_fail);
  return g_fail ? 1 : 0;
}
