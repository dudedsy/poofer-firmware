// rig_core.h - cold-test rig controller core: acquisition framing, shot sequencer, serial
// protocol. Plain C++ (no Arduino or pico SDK), so the same code runs in rig_control.ino on
// the RP2040 and in the host simulator (tools/sim).
//
// Hardware it expects (DMX base board Rev A stack):
//   ADC0..ADC3 recorded at 5 kHz per line (round robin 20 kHz per line, 4 samples summed).
//     Default map: ADC0 pressure 1, ADC1 pressure 2, ADC2 LED board current sense (or
//     pressure 3 when no LED board), ADC3 valve board current sense.
//   Chip temperature (ADC input 4) in the same round robin, status only.
//   Valve: one solenoid driver board, poofer::SolenoidDriver (closed-loop peak and hold).
//   LED:   one solenoid driver board as a buck, poofer::LedDriver.
//   Signal LED on SPARE1 (GP14) blinking a 9-bit LFSR timecode while recording, for video sync.
//   Igniter board (optional): ADS1115 thermocouple + TMP1075/LM75B cold junction on I2C.
//
// Protocol, device -> host. Text lines end in '\n'; binary packets start with 0xA5 0x5A,
// a byte that never appears in text.
//   I,{json}         info and settings (reply to '?')
//   S,{json}         status, default 10 Hz
//   E,{json}         event: go, fire, release, end, abort, tc_start, v_state, l_state, ...
//   OK <cmd> / ERR <cmd> <reason>
//   #text            log
//   A5 5A 'D' len16 payload crc16   recorded samples (see rig_core.cpp emitPacket)
// Host -> device: one command per line, see Core::line().
#pragma once
#include <stdint.h>
#include <stddef.h>
#include "poofer_drive.h"

namespace rig {

static const char* const FW_VERSION = "rig_control 1.0";

static const int N_ADC = 4;                       // recorded lines ADC0..ADC3
static const int N_RR = 5;                        // round robin: ADC0..3 + chip temperature
static const int SUB = 4;                         // samples summed per output frame
static const int FPM = 5;                         // output frames per ms per line (5 kHz)
static const int SAMP_PER_MS = N_RR * SUB * FPM;  // 100 samples per ms = 100 ksps total
static const int MS_PER_PKT = 10;
static const uint16_t TC_SEED = 0x1FF;            // LFSR x^9 + x^5 + 1, period 511

// Platform services the core needs. Implemented by rig_control.ino and the simulator.
class Platform {
 public:
  virtual ~Platform() {}
  virtual size_t outSpace() = 0;                       // bytes that can be queued without blocking
  virtual void out(const uint8_t* p, size_t n) = 0;    // queue bytes to the host
  virtual bool beginDrive(int which, int pair, int adcLine, uint32_t pwmHz) = 0;  // which: 0 valve, 1 LED
  virtual bool tcRead(int16_t& raw) = 0;               // ADS1115 conversion, false if absent
  virtual bool cjRead(float& degC) = 0;                // cold-junction sensor, false if absent
  virtual void syncLed(bool on) = 0;
  virtual void simCommand(const char*) {}              // simulator-only commands ("sim ...")
  virtual bool isSim() const { return false; }
};

struct Shot {
  uint8_t led = 1;          // turn the shadowgraph LED on at go (and off at end if we turned it on)
  float ledA = 0;           // LED target for the shot, 0 = leave as set
  uint8_t ledReq = 1;       // abort before fire unless the LED is regulating (RUN)
  uint8_t valve = 1;        // 0 = no valve (camera / LED / sync test)
  uint32_t fire = 800;      // ms from go to fire
  uint32_t open = 300;      // ms from fire to release
  uint32_t post = 1500;     // ms from release to end of recording
  uint8_t tc = 1;           // signal LED timecode while recording
  uint16_t tcMs = 10;       // timecode bit cell, ms
  uint8_t dip = 0;          // main LED dip markers (fallback sync when no signal LED)
  uint32_t dip1 = 300;      // ms from go
  uint32_t dip2 = 400;      // ms after release
  uint16_t dipMs = 50;
  float dipFrac = 0.5f;
  float ramp = 0;           // A/s hold current ramp from rampAt after fire (negative: dropout test)
  uint32_t rampAt = 300;
  float holdMin = 0.1f;
};

class Core {
 public:
  Core(Platform& pf, poofer::DriveHw& valveHw, poofer::DriveHw& ledHw);
  void begin();
  // One 1 ms ADC block: SAMP_PER_MS samples interleaved ch0,ch1,ch2,ch3,temp,ch0,...
  // Bit 15 set = ADC error flag. Runs the drives and the sequencer once.
  void block(const uint16_t* raw);
  void line(const char* s);          // one host command, without the newline
  void button();                     // any UI button: abort
  uint32_t now() const { return t_; }
  float sense[N_ADC] = {0, 0, 0, 0}; // 1 ms means of each line, volts (DriveHw reads these)
  poofer::SolenoidDriver valve;
  poofer::LedDriver led;
  int valvePair = 1, valveAdc = 3, ledPair = 2, ledAdc = 2;

 private:
  enum ShotState : uint8_t { IDLE, RUNNING };
  void sequencer();
  void endShot(const char* why, bool aborted);
  void startRecording();
  void stopRecording();
  void recordMs(const uint16_t* frames);
  void flushPacket();
  void status();
  void info();
  bool setKey(const char* k, const char* v);
  bool shotKey(const char* k, const char* v);
  bool text(const char* s);                 // queue a line; false (and counted) if no room
  void event(const char* name, const char* extraJson = nullptr);
  bool drivesOff() const;

  Platform& pf_;
  poofer::DriveHw& vhw_;
  poofer::DriveHw& lhw_;
  uint32_t t_ = 0;                  // ms since boot (ADC block count)
  // status accumulation
  double acc_[N_RR] = {0};
  uint32_t accN_ = 0;
  uint32_t monMs_ = 100, tStatus_ = 0;
  float chipC_ = 0;
  uint32_t adcErr_ = 0;
  // thermocouple
  bool tcOk_ = false, cjOk_ = false;
  int16_t tcRaw_ = 0;
  float cjC_ = 0;
  // recording
  bool rec_ = false, recManual_ = false;
  uint32_t pktT0_ = 0;
  int pktN_ = 0;
  uint16_t pktSamp_[MS_PER_PKT * FPM * N_ADC];
  uint8_t pktTele_[MS_PER_PKT * 10];
  uint32_t drops_ = 0, textDrops_ = 0, packets_ = 0;
  // timecode
  bool tcOn_ = false;
  uint16_t lfsr_ = TC_SEED;
  uint32_t tcNext_ = 0;
  uint8_t syncManual_ = 0;          // 0 off, 1 on, 2 slow blink (aiming)
  // shot
  Shot shot_;
  ShotState ss_ = IDLE;
  uint32_t tGo_ = 0, tRel_ = 0;
  bool ledOwned_ = false, fired_ = false, released_ = false;
  bool dip1Down_ = false, dip1Up_ = false, dip2Down_ = false, dip2Up_ = false;
  float ledShotTarget_ = 0, ledPrevTarget_ = 0, holdCur_ = 0, holdSaved_ = 0;
  uint8_t lastV_ = 0, lastL_ = 0;
  uint32_t shotCount_ = 0;
};

uint16_t crc16(const uint8_t* p, size_t n, uint16_t crc = 0xFFFF);  // CRC-16/CCITT-FALSE

}  // namespace rig
