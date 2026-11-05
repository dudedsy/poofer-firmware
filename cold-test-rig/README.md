# Cold-test rig

Runs nozzle cold-test experiments from the laptop. The DMX base board does everything that
needs timing:
- fires the valve, closed-loop peak and hold (PooferCommon `SolenoidDriver`);
- runs the shadowgraph LED (`LedDriver`);
- records 2 or 3 pressure sensors and both current senses at 5 kHz;
- reads the igniter board's thermocouple;
- blinks a timecode on a small signal LED so video frames can be timed afterwards.

The laptop tool, `rigctl`:
- runs experiments from YAML files (hardware conditions × parameter sweeps × repeats), with prompts for nozzle swaps and accumulator fills;
- records the camera with ffmpeg;
- saves the raw data and writes HTML reports.

**Status:** firmware compiles (arduino-pico 6.3.0, Waveshare RP2040 Zero, no warnings). The whole chain (firmware core, serial protocol, rigctl, video sync, analysis, reports) passes end-to-end tests against a simulator built from the same firmware code. **Not yet run on hardware.**

**Gas safety:** `rigctl` asks before every shot unless you pass `--auto`. The board's buttons abort a shot. The hardware T_max limiter and FIRE gating stand on their own; firmware is not a safety layer (solenoid spec §1, §9).

## Hardware

One stack: base board + UI board + one valve board + one LED board. The igniter board is optional (thermocouple).

| What | Where | Notes |
|---|---|---|
| Pressure 1 (accumulator) | J3.19, ADC0 | Fit C17 = 1 nF C0G (see membrane-burst-rig README) |
| Pressure 2 (nozzle) | J3.21, ADC1 | Fit C16 = 1 nF C0G |
| LED board current sense | JP2 → ADC2 (J3.23) | Or pressure 3 here (fit C15) when there is no LED board |
| Valve board current sense | JP2 → ADC3 (J3.25) | |
| Valve board | JP1 row 1 (CH1 FIRE / CH2 PWM) | |
| LED board | JP1 row 2 (CH3 / CH4) | Buck wiring in `shadowgraph-led/README.md` |
| Signal LED | SPARE1 = J3.29 (GP14) → ~150 Ω → LED → GND | In the camera's view on a dark card. For a 3 W LED, drive it with a logic-level N-FET from +5 V (about 10 Ω, 1 W) |
| Thermocouple | Igniter board J4, ADS1115 at 0x48, TMP1075/LM75B at 0x4C | Defaults: J7 and J9 shunts to GND |
| Abort | Any UI board button | Releases the valve per spec §9, LED off |

- **ADC budget:** the RP2040 has 4 ADC lines. With an LED board that leaves 2 pressure sensors. A third pressure sensor needs the LED board's JP2 shunt pulled (`led: null` in rig.yaml), e.g. for valve characterization.
- **24 V must be on** for the drives (and for the UI board OLED, which this firmware doesn't use).
- **Bus voltage:** measure +24V with a meter and put it in `rig.yaml` `bus_voltage`. It feeds the feed-forward and the coil resistance estimate.
- **T_max:** measure the hardware limiter (solenoid spec §12) and set `valve.t_max_s`. Shots that would need the firmware on-time limit above T_max / 2 are refused.

## Firmware: `firmware/rig_control`

Arduino IDE or arduino-cli, Earle Philhower RP2040 core 6.3.0, board Waveshare RP2040 Zero, with PooferCommon linked (top README). Flash `rig_control.ino`.

- **Acquisition:**
  - ADC round robin over ADC0–3 plus the chip temperature, 20 kHz per input, by DMA in 1 ms blocks.
  - Each output sample is the sum of 4 readings, so 5 kHz per line.
  - The 1 ms block is the controller's clock: drives, sequencer and timecode all step on it, so event times are exact sample indices.
- **Shot sequence** (on the board, from `go`):
  1. LED on (if it was off) and recording starts.
  2. Timecode starts on the signal LED.
  3. Fire at `fire_ms`; optional hold-current ramp.
  4. Release (spec §9) at `fire_ms + open_ms`.
  5. Recording ends at `post_ms` after release; LED off again if the shot turned it on.
  - Faults or a button press abort the shot.
- **Timecode:** a 9-bit LFSR (x⁹ + x⁵ + 1, period 511). The cell length is chosen by rigctl from the frame rate: 7 ms at 200 fps.
  - It must not be a whole multiple of the frame period, or the frames always sample the edges at the same phase and the timing error doesn't average out.
  - Fallback when no signal LED is wired: two dips of the main LED (`shot.dip: true`), about 70 ms wide, before fire and after release only.
- **Serial protocol:** text lines (`I,` info, `S,` status at 10 Hz, `E,` events, `OK`/`ERR`) plus binary sample packets every 10 ms (0xA5 0x5A, CRC-16). The packets carry the raw ADC sums and 1 kHz drive telemetry (duty, current estimate, state). Details are at the top of `rig_core.h`; `?` prints every setting.

`rig_core.cpp` is plain C++ shared with the simulator, so the logic under test is the logic that runs on the board.

## Laptop: `host/` (rigctl)

```
pip install -e cold-test-rig/host        (Python 3.10+; ffmpeg and ffprobe on PATH)
copy cold-test-rig/examples/rig.yaml .   and edit: port, wiring, bus voltage, camera
```

Getting started, in order:

| Step | Command |
|---|---|
| Find the port | `rigctl ports` |
| Check the board | `rigctl info`, then `rigctl monitor` (live pressures, drive states, temperatures) |
| Pressure zeros, sensors vented | `rigctl zero` (writes rig_cal.json next to rig.yaml) |
| LED check | `rigctl led on 2.0`, `rigctl led off` |
| Camera exposure etc. | `rigctl camera dialog` (driver dialog; ffmpeg can't set exposure). Whether settings survive a replug is still unchecked |
| Aim, pick boxes | `rigctl sync blink`, `rigctl camera snapshot`, then set `video.sync_roi`, `nozzle_px`, `mm_per_px` (ruler on the screen) |
| Sync test, no gas | `rigctl shot --no-valve`: LED, camera and timecode only; the report should say "synced" |
| Valve characterization | `rigctl characterize uss2-34 --pressure 60`, see below |
| Experiment | `rigctl run screening.yaml --name 2026-11-07-first` (`--resume` continues an interrupted run) |
| Re-analyse | `rigctl report data/2026-11-07-first` |

### Valve characterization

`rigctl characterize NAME --pressure P` runs three tests with gas on, LED and camera off, and writes `data/valves/NAME.json`:

| Test | Shots | Gives |
|---|---|---|
| coil | 1 × 300 ms | Coil R (hold, from duty and the measured bus voltage); L with the armature open and closed, from the rise slope after undoing the 1 ms sense filter; when the armature starts and stops moving (its back EMF shows as a jump in the apparent inductance) |
| response | 3 × 300 ms | Opening (10/50/90 %) and closing (90/50/10 %) delays at the nozzle sensor, from fire and from release |
| dropout | 2 × ramp | Hold current ramped down at 0.5 A/s until the nozzle/accumulator pressure ratio falls. Dropout current is the coil current one closing delay earlier |

Recommended hold = 2 × dropout, rounded up to 0.05 A (spec §9). Check it against the coil rating. Experiments that name the valve (`valve: NAME`) use its R, L (for the loop gains) and hold current. The dropout test needs T_max above about 5 s.

### Experiments

See `examples/screening.yaml`.
- **Conditions** are hardware set-ups (nozzle, diffuser, anything). You are prompted to swap between them, and the labels go into every run.
- **Sweep** parameters are crossed inside each condition, times `repeats`.
- Before each shot rigctl shows the live accumulator pressure until it sits within `pressure_tol` of `pressure_psi` for 1 s, then asks to fire.
- After each shot it prints the key numbers and offers redo, next or quit.

Parameters: `open_ms fire_ms post_ms pressure_psi pressure_tol peak_a peak_ms hold_a led_a camera led valve tc_ms dip ramp ramp_at hold_min max_on_ms`.

### Data

```
data/<experiment name>/
  experiment.yaml  plan.json  summary.html  summary.csv  summary_*.png
  001_N1-straight_none_open_ms150_pressure_psi40_r1/
    run.json        params, labels, rig.yaml snapshot, device settings, camera log, status
    samples.npz     raw ADC sums (5 kHz x 4 lines), 1 kHz drive telemetry, valid mask
    events.jsonl    board events (go, fire, release, state changes, faults, end)
    status.jsonl    10 Hz status incl. thermocouple and temperatures
    video.avi       camera recording (raw YUYV as captured)
    analysis.json   all metrics      report.html + *.png
data/valves/<name>.json  and  data/valves/<name>/<time>/<test runs>
```

Raw data is kept as recorded (ADC counts, not psi), with the calibration in `run.json`, so everything can be re-analysed.

### What the reports measure, and how far to trust it

- **Pressure:** accumulator before and just after the shot (drop = gas used); nozzle plateau and peak; valve opening and closing delays at the nozzle sensor.
  - These include the sensor's own lag, about 1 ms or more, which the membrane burst rig is meant to measure.
- **Coil:** firmware current estimate and duty at 1 kHz, raw Q2 current at 5 kHz, R, L, armature motion, dropout.
  - R depends on the bus voltage you entered. L and armature timing are estimates from a 1 ms-filtered sense.
- **Temperatures:** thermocouple via NIST ITS-90 type K or N with cold-junction compensation, the igniter board's cold-junction sensor, and the RP2040 die, all at 10 Hz.
- **Video:** frame times from the timecode. In the simulator the fit is within 0.25 ms over a whole recording, and it flags suspected dropped frames.
  - Jet penetration vs time, front speed and a contact sheet, from background-subtracted frames. This is a first pass: check it against the contact sheet before trusting it.

## Simulator: `tools/sim`

```
make -C cold-test-rig/tools/sim
rigctl sim --truth sim_truth.csv                                   (terminal 1, Linux or WSL)
rigctl --rig cold-test-rig/examples/rig_sim.yaml characterize sim-valve --pressure 60 --auto
rigctl --rig cold-test-rig/examples/rig_sim.yaml run cold-test-rig/examples/screening.yaml --auto
```

The firmware core runs against a plant model in real time on TCP, so rigctl connects as if to the board. The model includes:
- a coil with armature (its back EMF), and a semi-direct valve;
- the accumulator and nozzle pressures, sensors with lag and noise;
- the LED buck;
- a cooling thermocouple.

The fake camera renders a shadowgraph-like video from the simulator's truth log, at a deliberately wrong frame offset and period, so the sync is tested for real. `tests/test_rig.py` runs all of it in CI and checks the results against the simulator's parameters.

The model's numbers (valve timing, dropout, jet speed) are made up to look plausible. Only the measurement chain is being tested, not the physics.

## Known limits

- Not run on hardware: USB throughput, real ADC noise and crosstalk, I2C timing, and the signal LED's visibility against the retroreflector are all unverified.
- The signal LED switches in the 1 ms loop. If the loop ever falls behind (it shouldn't; it reports `adc overrun`), those edges are late and the sync fit's residual shows it.
- The firmware current estimate overshoots about 5 % for a few ms when the duty steps down (peak to hold). It's the estimator's filtered-duty approximation, well inside the 3.2 A trip.
- Camera exposure, gain and white balance aren't set by rigctl. Set them in the driver dialog and keep them fixed across a series.
