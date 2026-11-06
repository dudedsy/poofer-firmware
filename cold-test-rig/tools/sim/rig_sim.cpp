// rig_sim.cpp - the rig_control firmware core (rig_core.cpp + PooferCommon drives) running
// against a plant model, in real time, served over TCP so rigctl connects to it exactly as
// to the board:  rigctl --port socket://127.0.0.1:5555 ...
//
// Plant (averaged over a PWM period, 2 us steps):
//   valve coil  flux psi = L(x) i, dpsi/dt = v - R i, L(x) = L0 + dL x. Armature x moves to 1
//               over ~6 ms once i > I_pull for 2 ms, back to 0 over ~4 ms when i < I_drop.
//               The current dips while x moves (back EMF), as on a real valve.
//   main valve  opening o follows x: tau 12 ms opening, 20 ms closing (semi-direct, est.)
//   gas         accumulator P_a decays with o; nozzle P_n -> beta o P_a (tau 4 ms); refill
//               toward a target with "sim fill=<psi>". Sensors: first order 1.5 ms, noise.
//   LED buck    J1 -> L -> (C || LED), as libraries/PooferCommon/tests/drive_sim.cpp.
//   sense       Q2 current (duty x load current) x 0.5 V/A through a 1 ms RC.
//   thermo      gas temperature cools while flowing; type K ~40.6 uV/C into the ADS1115.
// Every ms the true state is appended to --truth (CSV) for the fake camera and the tests.
#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <random>
#include <string>
#include "rig_core.h"

using namespace poofer;

static std::mt19937 rng(12345);
static float gauss(float s) { std::normal_distribution<float> d(0.0f, s); return d(rng); }

struct Plant {
  // valve electrical / mechanical
  float vbus = 24.3f, rCoil = 8.7f, L0 = 0.075f, dL = 0.045f, vFree = 0.8f, rPath = 0.15f;
  float iPull = 0.55f, iDrop = 0.32f;
  float tMaxMs = 12000;            // hardware limiter T_max
  bool vFire = false; float vDuty = 0;
  float psi = 0, x = 0, o = 0, vSense = 0, pullMs = 0, fireMs = 0;
  float iCoil() const { return psi / (L0 + dL * x); }
  // LED buck
  bool lFire = false; float lDuty = 0;
  float lL = 170e-6f, lC = 940e-6f, vf0 = 10.81f, rd = 0.57f, il = 0, vc = 0, lSense = 0, lFireMs = 0;
  float iLed() const { return vc > vf0 ? (vc - vf0) / rd : 0.0f; }
  // gas
  float pAtm = 12.7f, pa = 60.0f, pn = 0, fill = 0, fillRate = 15.0f, kA = 0.35f, beta = 0.55f;
  float s1 = 60.0f, s2 = 0, s3 = 0;     // sensor outputs, psi
  int zero1 = 6, zero2 = -4, zero3 = 2; // sensor zero offsets, counts
  // temperatures
  float tGas = 21.0f, tCj = 22.5f, tChip = 30.0f;
  bool tcPresent = true;
  bool ledBoard = true;
  // sync LED
  bool sync = false;

  // One ms of plant, writing the 100 interleaved ADC samples (20 per input) at the times the
  // RP2040's round robin would take them.
  void step1ms(uint16_t* raw) {
    const int N = 500;
    const float dt = 1e-3f / N;
    const int perSample = N / (rig::SAMP_PER_MS / rig::N_RR);   // 25 steps per round
    bool q1 = vFire && fireMs < tMaxMs;
    bool lq1 = lFire && lFireMs < tMaxMs;
    // armature and main valve at 1 ms resolution (6 ms motion, 12-20 ms valve)
    float i0 = iCoil();
    if (i0 > iPull) pullMs += 1; else pullMs = 0;
    float xTarget = x;
    if (pullMs >= 2 && x < 1) xTarget = fminf(1.0f, x + 1.0f / 6.0f);
    if (i0 < iDrop && x > 0) xTarget = fmaxf(0.0f, x - 1.0f / 4.0f);
    float dx = (xTarget - x) / N;
    int r = 0;
    for (int k = 0; k < N; k++) {
      x += dx;
      // coil (flux linkage, so armature motion produces back EMF)
      float i = iCoil(), v;
      if (q1) v = (i > 0 || vDuty > 0) ? vDuty * vbus - (1.0f - vDuty) * vFree : 0.0f;
      else v = i > 0 ? -33.0f : 0.0f;
      psi += (v - i * (rCoil + rPath)) * dt;
      if (psi < 0) psi = 0;
      vSense += (vDuty * iCoil() * SENSE_V_PER_A - vSense) * dt / 1e-3f;
      // LED
      float vl = lq1 ? lDuty * vbus - (1.0f - lDuty) * vFree : (il > 0 ? -33.0f : 0.0f);
      il += (vl - vc - il * rPath) / lL * dt;
      if (il < 0) il = 0;
      vc += (il - iLed()) / lC * dt;
      if (vc < 0) vc = 0;
      lSense += (lDuty * il * SENSE_V_PER_A - lSense) * dt / 1e-3f;
      // gas and sensors
      float oT = x >= 0.999f ? 1.0f : 0.0f;
      o += (oT - o) * dt / (oT > 0 ? 12e-3f : 20e-3f);
      pa -= kA * o * (pa + pAtm) * dt;
      if (fill > 0 && o < 1e-4f && pa < fill) pa = fminf(fill, pa + fillRate * dt);
      pn += (beta * o * pa - pn) * dt / 4e-3f;
      s1 += (pa - s1) * dt / 1.5e-3f;
      s2 += (pn - s2) * dt / 1.5e-3f;
      s3 += (0.3f * pn - s3) * dt / 1.5e-3f;
      if (k % perSample == perSample / 2 && r < rig::SAMP_PER_MS / rig::N_RR) {
        uint16_t* s = &raw[r * rig::N_RR];
        s[0] = psiCounts(s1, zero1);
        s[1] = psiCounts(s2, zero2);
        s[2] = ledBoard ? counts(lSense) : psiCounts(s3, zero3);
        s[3] = counts(vSense);
        s[4] = counts(0.706f - (tChip - 27.0f) * 0.001721f);
        r++;
      }
    }
    x = xTarget;
    if (o < 1e-4f) o = 0;
    tGas += (21.0f - tGas) * 1e-3f / 15.0f - o * (tGas + 20.0f) * 1e-3f / 0.8f;
    fireMs = vFire ? fireMs + 1 : 0;
    lFireMs = lFire ? lFireMs + 1 : 0;
  }
  static uint16_t counts(float v) {
    int c = (int)lroundf(v / 3.3f * 4095.0f + gauss(0.7f));
    return (uint16_t)(c < 0 ? 0 : (c > 4095 ? 4095 : c));
  }
  static uint16_t psiCounts(float p, int zero) {
    int c = (int)lroundf(409.5f + zero + p * 32.76f + gauss(2.5f));
    return (uint16_t)(c < 0 ? 0 : (c > 4095 ? 4095 : c));
  }
};

static Plant plant;

class SimHw : public DriveHw {
 public:
  SimHw(bool valve, const float* sense) : valve_(valve), sense_(sense) {}
  void setFire(bool on) override { if (!en) return; (valve_ ? plant.vFire : plant.lFire) = on; }
  void setDuty(float d) override {
    if (!en) return;
    d = d < 0 ? 0 : (d > 1 ? 1 : d);
    duty_ = d;
    (valve_ ? plant.vDuty : plant.lDuty) = d;
  }
  float duty() const override { return duty_; }
  float senseVolts() override { return en ? sense_[adc] : 0.0f; }
  void setPwmHz(uint32_t) override {}
  bool en = false;
  int adc = 0;
 private:
  bool valve_;
  const float* sense_;
  float duty_ = 0;
};

static std::deque<uint8_t> outq;
static const size_t OUTQ_MAX = 1 << 20;

class SimPlatform : public rig::Platform {
 public:
  SimHw* hw[2];
  size_t outSpace() override { return OUTQ_MAX - outq.size(); }
  void out(const uint8_t* p, size_t n) override { outq.insert(outq.end(), p, p + n); }
  bool beginDrive(int which, int pair, int adcLine, uint32_t) override {
    hw[which]->setDuty(0);
    hw[which]->setFire(false);
    hw[which]->en = pair != 0;
    hw[which]->adc = adcLine;
    if (which == 1) plant.ledBoard = pair != 0;
    return true;
  }
  bool tcRead(int16_t& raw) override {
    if (!plant.tcPresent) return false;
    float uv = 40.6f * (plant.tGas - plant.tCj);
    raw = (int16_t)lroundf(uv / 7.8125f + gauss(1.0f));
    return true;
  }
  bool cjRead(float& c) override {
    if (!plant.tcPresent) return false;
    c = roundf(plant.tCj * 16) / 16;
    return true;
  }
  void syncLed(bool on) override { plant.sync = on; }
  bool isSim() const override { return true; }
  void simCommand(const char* s) override {
    char buf[200];
    snprintf(buf, sizeof buf, "%s", s);
    for (char* tok = strtok(buf, " "); tok; tok = strtok(nullptr, " ")) {
      char* eq = strchr(tok, '=');
      if (!eq) continue;
      *eq = 0;
      float v = (float)atof(eq + 1);
      if (!strcmp(tok, "fill")) plant.fill = v;
      else if (!strcmp(tok, "p")) plant.pa = plant.s1 = v;
      else if (!strcmp(tok, "tmax")) plant.tMaxMs = v;
      else if (!strcmp(tok, "tc")) plant.tcPresent = v != 0;
      else if (!strcmp(tok, "drop")) plant.iDrop = v;
      else if (!strcmp(tok, "pull")) plant.iPull = v;
      else if (!strcmp(tok, "vbus")) plant.vbus = v;
      else if (!strcmp(tok, "r")) plant.rCoil = v;
    }
    char b[300];
    snprintf(b, sizeof b,
             "#sim {\"vbus\":%.2f,\"r_coil\":%.3f,\"l0\":%.4f,\"dl\":%.4f,\"i_pull\":%.3f,\"i_drop\":%.3f,\"tmax_ms\":%.0f,"
             "\"fill\":%.1f,\"pa\":%.2f,\"zero\":[%d,%d,%d]}\n",
             plant.vbus, plant.rCoil, plant.L0, plant.dL, plant.iPull, plant.iDrop, plant.tMaxMs, plant.fill, plant.pa,
             plant.zero1, plant.zero2, plant.zero3);
    out((const uint8_t*)b, strlen(b));
  }
};

static double nowS() {
  timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return ts.tv_sec + ts.tv_nsec * 1e-9;
}

int main(int argc, char** argv) {
  int port = 5555;
  const char* truth = nullptr;
  double runFor = 0;
  for (int k = 1; k < argc; k++) {
    if (!strcmp(argv[k], "--port") && k + 1 < argc) port = atoi(argv[++k]);
    else if (!strcmp(argv[k], "--truth") && k + 1 < argc) truth = argv[++k];
    else if (!strcmp(argv[k], "--seconds") && k + 1 < argc) runFor = atof(argv[++k]);
    else { fprintf(stderr, "usage: rig_sim [--port 5555] [--truth truth.csv] [--seconds N]\n"); return 2; }
  }
  FILE* tf = truth ? fopen(truth, "w") : nullptr;
  if (tf) fprintf(tf, "t_ms,sync,led_a,coil_a,armature,open,p_acc,p_noz,t_gas\n");

  int ls = socket(AF_INET, SOCK_STREAM, 0);
  int one = 1;
  setsockopt(ls, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
  sockaddr_in a{};
  a.sin_family = AF_INET;
  a.sin_port = htons(port);
  a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  if (bind(ls, (sockaddr*)&a, sizeof a) || listen(ls, 1)) { perror("bind/listen"); return 1; }
  fcntl(ls, F_SETFL, O_NONBLOCK);
  printf("rig_sim listening on 127.0.0.1:%d\n", port);
  fflush(stdout);

  SimPlatform pf;
  static float dummy[4];
  SimHw vhw(true, dummy), lhw(false, dummy);
  rig::Core core(pf, vhw, lhw);
  vhw = SimHw(true, core.sense);   // the drives read the core's 1 ms sense means
  lhw = SimHw(false, core.sense);
  pf.hw[0] = &vhw;
  pf.hw[1] = &lhw;
  core.begin();

  int cs = -1;
  std::string inLine;
  uint16_t raw[rig::SAMP_PER_MS];
  double t0 = nowS();
  uint64_t ms = 0;
  for (;;) {
    double target = t0 + (ms + 1) * 1e-3;
    double now = nowS();
    if (now < target) {
      timespec ts{0, (long)((target - now) * 1e9)};
      nanosleep(&ts, nullptr);
    }
    // one ADC block per ms; catch up if late
    plant.step1ms(raw);
    core.block(raw);
    ms++;
    if (tf) fprintf(tf, "%lu,%d,%.4f,%.4f,%.3f,%.4f,%.3f,%.3f,%.2f\n", (unsigned long)core.now(), plant.sync ? 1 : 0,
                    plant.iLed(), plant.iCoil(), plant.x, plant.o, plant.pa, plant.pn, plant.tGas);
    if (tf && ms % 50 == 0) fflush(tf);
    if (runFor > 0 && ms >= runFor * 1000) break;

    if (cs < 0) {
      cs = accept(ls, nullptr, nullptr);
      if (cs >= 0) {
        fcntl(cs, F_SETFL, O_NONBLOCK);
        setsockopt(cs, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);
        outq.clear();
      }
      continue;
    }
    char rb[512];
    for (;;) {
      ssize_t n = recv(cs, rb, sizeof rb, 0);
      if (n > 0) {
        for (ssize_t k = 0; k < n; k++) {
          char c = rb[k];
          if (c == '\n' || c == '\r') { if (!inLine.empty()) { core.line(inLine.c_str()); inLine.clear(); } }
          else if (inLine.size() < 250) inLine += c;
        }
        continue;
      }
      if (n == 0 || (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK)) { close(cs); cs = -1; outq.clear(); }
      break;
    }
    while (cs >= 0 && !outq.empty()) {
      static uint8_t wb[8192];
      size_t m = outq.size() < sizeof wb ? outq.size() : sizeof wb;
      std::copy(outq.begin(), outq.begin() + m, wb);
      ssize_t n = send(cs, wb, m, MSG_NOSIGNAL);
      if (n > 0) outq.erase(outq.begin(), outq.begin() + n);
      else break;
    }
    if (cs < 0) outq.clear();
  }
  if (tf) fclose(tf);
  return 0;
}
