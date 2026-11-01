# Membrane burst rig

A small pressurized chamber sealed by a thin membrane. Popping the membrane drops the pressure faster than any of the sensors can follow, so the recorded traces show each sensor's own response time. The goal is to know which pressure sensors can be trusted for the nozzle cold tests, and at which supply voltage.

**Status:** firmware 3.0 compiles (arduino-pico core 6.3.1) and passes end-to-end tests in a host simulator (`tools/sim`), but has not been run on hardware. The Python tools are tested on synthetic and simulated data only.

Firmware 3.0 runs on the poofer DMX base board with the UI lid board: standalone data collection with captures stored on the board, a dump mode for copying them to the laptop, and the original live-to-laptop mode. It still runs on a bare RP2040-Zero, where it behaves like the old firmware (live mode only).

## What it measures

- **Each sensor's response time.** This is modeled as a first-order time constant, tau, plus a small delay relative to a reference sensor.
- **Whether sensors behave the same at 3.3 V and at 5 V.** Run one sensor at each voltage side by side.
- **Clipping, ringing, overshoot, and resets.** These show up in the plots and residuals.

The sensors under test are the generic 0-100 psi, 0.5-4.5 V ratiometric transducers (ceramic element, 1/8" NPT, about $16 each). The listings claim at most 3 ms response, and they likely have a snubber in the port. A sensor useful for plenum transients needs roughly 0.2 ms or better; about 1 ms is fine for fill and steady-state pressure.

## Rig hardware

All 1/2" black iron (or brass) pipe fittings.

| Part | Role |
|---|---|
| 1/2" cross | The chamber |
| Front port | Union holding the membrane between its faces. Use a flat-face gasketed union, or check that a ground-joint union doesn't crease the membrane. The outlet half is cut back past the nut so the gas vents straight out and a pin can reach the membrane. |
| Both side ports | 1/2" x 1/8" NPT bushings, one sensor each. Opposite ports give both sensors the same path. |
| Back port | Tee: fill needle valve on one side, and on the other side either a sensor (3rd channel) or a capped nipple that adds a known volume. |

**Measure the real numbers, don't trust these.** Find the volume by filling everything that sees pressure with water from a syringe, with the sensors installed. Use the union's bore as the opening diameter: 1/2" Sch 40 ID is about 0.622" (15.8 mm).

**Expected event timing** (moderate confidence):

| Stage | Time |
|---|---|
| Membrane tearing open | about 0.1 to 1 ms |
| Expansion wave reaching the sensors | about 0.1 ms |
| Chamber blowdown | time constant roughly 0.4 to 0.7 ms |

That's comfortably faster than a 3 ms sensor, so a 3 ms lag will be obvious. The rig can't resolve sensors much faster than about 0.2 ms.

**Membranes.**
- Thin Mylar (0.5 to 1 mil) or a patch of balloon latex clamped across the bore.
- At 30 psi on a 1/2" bore the load is about 9 lbf, so both materials are feasible.
- Avoid aluminum foil: it tears progressively, which gives a slow, messy opening.

**Two ways to burst:**
- **Pin:** pressurize, then pierce. Use a long pin or a pin on a stick.
- **Over-pressure (cleaner):** fill slowly through the needle valve until the membrane bursts on its own. Set the burst pressure with membrane thickness or the number of layers.

**Chamber ringing.** The chamber has its own acoustic modes in the kHz range. They appear identically on all sensors, so they belong to the chamber, not the sensors.

## Electronics

RP2040-Zero, running sketch `firmware/burst_capture/burst_capture.ino`, either on the DMX base board (with the UI board on J2) or bare.

### On the DMX base board

The sensors land on the J3 stack socket. Each ADC line already has a 4.7 kOhm series resistor on the base board (R29 for ADC0, R28 for ADC1), so wire the sensor output straight to the J3 pin; don't add the 1 kOhm.

| Signal | J3 pin | Module pin |
|---|---|---|
| Channel 0 sensor out | 19 (ADC0, via R29) | GP26 |
| Channel 1 sensor out | 21 (ADC1, via R28) | GP27 |
| Channel 2 sensor out | 23 (ADC2, via R27) | GP28 |
| Sensor + (3.3 V mode) | 2 (+3V3) | |
| Sensor - | 20 or 22 (GND) | |

- **Filter cap:** fit 1 nF C0G 0805 in the DNP shunt footprints on the module side of the series resistor: C17 for ADC0, C16 for ADC1, C15 for ADC2. With the 4.7 kOhm that is a ~34 kHz corner (about 5 us of lag) and it gives the ADC's sample-and-hold a charge reservoir when it switches channels. Don't use the 10 nF the base board spec suggests for those footprints: that is ~50 us of lag on top of the sensor. (Moderate confidence on the RP2040 sampling-charge argument; the bring-up crosstalk check below tests it.)
- **Power:** the UI board's OLED and LEDs run from the base board's +5V, which only exists with 24 V on J4. On USB power alone the firmware runs and dump mode works, but the display stays dark. Standalone collection needs 24 V.
- **Reference:** the RP2040-Zero feeds its ADC reference from 3V3 through 200 Ohm (R6) with 2.2 uF (C11). The ADC's ~150 uA leaves the reference roughly 30 mV below the 3V3 that powers the sensor (per the Pico datasheet, varies chip to chip), so readings run ~1% high. The fit normalizes each capture, so tau is unaffected; the Calibrate menu step absorbs it for the psi display and burst pressure.
- **Crosstalk check at bring-up:** with 2 channels, leave the channel 1 sensor vented and pressurize channel 0 to ~50 psi. Channel 1's reading should not move by more than a count or two. If it follows A, fit the C16/C17 caps (or larger) and recheck.

### On a bare RP2040-Zero

| Signal | RP2040-Zero pin |
|---|---|
| Channel 0 sensor | GP26 (ADC0) |
| Channel 1 sensor | GP27 (ADC1) |
| Channel 2 sensor | GP28 (ADC2) |
| 5 V rail divider (5 V mode only) | GP29 (ADC3) |

### 3.3 V supply

Use this only if the static gauge check shows the sensor is linear and stable at 3.3 V.

- Sensor + to 3V3, - to GND.
- Signal through 1 kΩ to the ADC pin, with 1 nF from the pin to GND.
- No rail channel is needed: the sensor is ratiometric to the same 3V3 that feeds ADC_VREF (through R6 on the Zero).
- Running below the 5 V spec can cause problems that only appear in dynamic tests:
  - clipping on overshoot,
  - supply noise passing through,
  - brownout resets on supply dips,
  - slower response.
- Compare one sensor at 3.3 V against one at 5 V in the same burst before trusting 3.3 V.

### 5 V supply

- Sensor + to 5 V.
- Signal through a 10k / 15k divider, with 1 nF at the pin (about 26 kHz corner).
- Put the same divider from 5 V to GP29.
- Use matched or 0.1% resistors.
- In the sketch, set `RAIL_CH` to the divider's channel index (2 for two sensors plus the rail), and select 3 channels. The menu supports up to 3 channels, so rail mode allows two sensors. The OLED psi readout assumes the 3.3 V ratiometric scaling and is wrong in 5 V mode; the captures are fine.
- In the analysis config, set `"supply_mode": "rail"` and `"rail_ch": 3`.

**Keep the filter caps small.** A 100 nF cap would put about 1 ms of lag in front of the sensor and contaminate the very thing being measured.

**Power the sensors from a clean supply,** separate from anything switching heavy loads.

## Firmware

Setup:
1. In Arduino IDE, add the board URL `https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json`.
2. Install "Raspberry Pi Pico/RP2040/RP2350" (built and tested against 6.3.1).
3. Select **Waveshare RP2040 Zero**, and **Tools > Flash Size > 2MB (no FS)**. Any filesystem option overlaps the capture store.

Acquisition (all modes):
- 300 ksps total, round-robin over 1 to 3 channels (300 / 150 / 100 kHz per channel).
- DMA into a 156 KB ring buffer, 260 ms deep, processed as 1 ms means for display and triggering.
- **Trigger:** any sensor's 1 ms mean falls more than the drop-trigger threshold (default 1.0 psi, about 33 counts) below the mean of its previous 20 ms and stays below that level for 10 consecutive ms. The trigger point is the first of those ms. Earlier versions compared against the 20 ms maximum with no hold, so a single 1 ms glitch of a few psi in either direction fired it as soon as Collect armed. The keep/drop screen and each capture's json (`trig_ch`, `trig_drop_psi`) say which channel tripped and by how much.
- **Capture window:** 20 ms before the trigger and 80 ms after (was 50 / 200). On synthetic data the fit recovers tau to within 0.3% with this window; 10 / 50 was also fine and 5 / 30 started biasing the slowest (4 ms) sensor by ~1%. 20 / 80 leaves room for real sensors slower than the synthetic ones. `PRE_MS` / `POST_MS` are compile-time constants.

### Modes and buttons

Channels are numbered by ADC input: 0, 1, 2 = ADC0, ADC1, ADC2 (GP26, GP27, GP28). With N channels selected the firmware uses 0 to N-1, so wire sensors from channel 0 up without gaps; an unconnected channel floats and can false-arm or false-trigger.

At boot the OLED shows a menu. UP / DN move, SET selects. **Hold SET** (0.8 s) to get back to the menu from any mode. Without the UI board (no I2C pull-ups) it boots straight into Live mode.

| Mode | What it does |
|---|---|
| **Collect** | Standalone. Live pressure on the OLED. **Arms** automatically once pressure stays above the auto-arm level for 100 ms, or immediately with **SET**. While armed, the drop trigger is live. After a capture the ADC stops and the screen shows each channel's pre-burst and settled pressure, ADC errors, and how many more captures fit; **SET keeps** it in flash, **DN drops** it. **UP** forces a capture (it records -20 to +80 ms around the press). SET while armed disarms. If the rig vents slowly below half the arm level without a trigger, it disarms itself. After a capture, auto-arm waits until pressure has fallen below half the arm level first. |
| **Live (USB)** | The original laptop workflow: 1 kHz slow stream plus binary captures streamed to the logger. The trigger is always armed. UP forces a capture. |
| **Dump (USB)** | ADC stopped; waits for `burst_logger.py --dump`. Dump commands also work from the menu and Live mode, but not from Collect. |

Menu settings (stored in the base board FRAM, so they survive power cycles):

| Item | |
|---|---|
| Channels | 1 to 3 (GP26 up) |
| Tag | 0 to 15, stored with each capture. Use one tag per rig configuration (membrane, volume) so the dump sorts captures into separate run folders |
| Auto arm | off, or 1 to 90 psi (default 5) |
| Drop trig | 0.5 to 10 psi (default 1.0) |
| Zero sensors | vent the rig, SET: averages 0.5 s and stores each channel's zero counts for the psi display. Refused if a reading is more than ~3.7 psi from the nominal 410 counts. Keeps the calibrated gain |
| Calibrate (gauge) | static calibration against a gauge. Set the rig to a pressure, dial the gauge reading in with UP/DN, SET records a point (2 to 10 points, at least 10 psi apart overall). Hold SET to fit counts = zero + gain x psi per channel; the result screen shows gain (counts/psi), zero and worst residual, SET saves, DN discards. Refused if a gain is more than 25% off nominal (32.76) or a zero is far from 410 |
| Captures | list of stored captures (newest first: id, tag, pre-burst psi, F = forced). SET on one asks to delete it |
| Delete all | asks for confirmation; needs a **long** SET press |

Status LEDs on the UI board: LED1 green = menu / waiting, red = armed or triggered, blue = keep/drop prompt or dump mode, magenta = saving, cyan = live. LED2 amber = rig above 2 psi. The OLED blanks after 5 minutes with no button presses and the rig at 0 psi (burn-in); any button wakes it without acting, and so does pressurizing the rig. **If the screen stays dark** (for example after powering up from a slow-rising supply), hold SET: every long SET press also restarts the panel (VBAT off, reset, full init). The firmware also waits 300 ms at boot before starting the panel, and restarts it automatically if the UI board's expander was reset by a supply dip.

### Storage

Captures go in the RP2040-Zero's own 2 MB flash, in fixed 48 KB slots from 256 KB up (37 slots). Samples are packed to 12 bits (45 KB per capture), with a header and CRC32 per slot. The header is written last, so a power loss mid-save leaves a free slot, not a corrupt capture. Saving takes about 0.6 s, with the ADC stopped.

Reflashing the firmware does not erase stored captures, provided the sketch stays below 256 KB (it is ~92 KB; the firmware checks at boot and disables storage with a message if not) and the Flash Size option stays at "no FS". Settings, the capture counter and the session counter live in FRAM at 0x1F00, clear of anything the DMX firmware would use at the bottom.

### Serial commands

| Command | Action |
|---|---|
| `t` | Force a capture now (Live, Collect). |
| `?` | Print the JSON header. |
| `d<N>` | Set the drop trigger to N counts, e.g. `d15`. |
| `N<k>` | Set the channel count, 1 to 3. |
| `ML` / `MD` / `MM` | Switch to Live / Dump / Menu. |
| `L` | List stored captures (`J,{...}` lines). |
| `R<id>` | Send stored capture `<id>`: a `J` meta line (including `crc_ok`), then the same `C,...` binary block as a live capture. |
| `X<id>` | Delete stored capture `<id>`. |

`L`, `R`, `X` are refused in Collect mode.

## Logging

Install:
```
pip install -r requirements.txt
```

### Live

```
python host/burst_logger.py COM5 --label mylar_base --notes "base volume, 3.3V" --channels 2
```

The logger switches the board to Live mode itself (older firmware ignores the command).

- Each run gets its own folder under `runs/`, holding:
  - `run.json`
  - `slow.csv`
  - `capture_NNN.npz` (raw 12-bit counts, shape frames x channels)
  - `capture_NNN.json` (sample rate, trigger index, flags)
- The console prints each capture's pre-burst and settled counts, and flags ADC errors or framing errors.
- While it runs, type `t`, `d<N>`, `?`, or `q` then Enter.

### Dump

Put the board in the menu or Dump mode, then:

```
python host/burst_logger.py COM5 --dump --tags 1=mylar_base,2=mylar_capped
python host/burst_logger.py COM5 --dump --tags 1=mylar_base --erase
```

- One folder per tag, `runs/<time>_<tag label>/capture_<id>.npz/.json`, so the analysis globs work unchanged. Tags without a label become `tag<N>`.
- Each capture's json carries the board's id, session, tag, zeros, gains (`cpp`, counts per psi), pre-burst psi, ADC error count and `crc_ok`, and `chip_temp_c`: the RP2040 die temperature read right after the capture. It reads the chip, not the gas, so treat it as rough ambient for the notes and for `T0_K`. Live captures carry it too.
- Captures already copied (same id and CRC under `--outdir`) are skipped, so dumping twice is harmless.
- `--erase` deletes each capture from the board only after its copy is written and its CRC, framing and length check out. Without it, nothing is deleted; use the menu to clear the board.

## Analysis

```
python analysis/blowdown_fit.py rig_config.json --plots plots --per-capture
```

Steps:
1. Measure the chamber volume for each configuration (water and syringe) and the opening bore.
2. Copy `analysis/rig_config.example.json` to `rig_config.json` and fill it in:
   - gas,
   - ambient pressure (about 87 kPa in Reno, about 101 kPa at sea level),
   - one `groups` entry per volume and membrane type, matched to run labels by a glob,
   - `fit.pre_ms` / `fit.end_ms`: the windows for each sensor's pre-burst and settled levels, in ms from the trigger. The example's `[-18, -5]` / `[50, 78]` suit 3.0 captures (-20 to +80 ms) and old live captures alike. The script stops with a message if a window falls outside the capture.
3. Record the vented zero counts for each sensor. This is optional and only used as a sanity check.

**Model.** Lumped adiabatic blowdown through the opening:
- Choked flow, then subsonic flow.
- The opening area ramps linearly over `t_open` from the burst time.
- The chamber pressure is solved exactly from a table, so fits are fast.
- Each sensor sees that pressure, shifted by a relative delay, through a first-order lag.

**Calibration.** Each sensor's own pre-burst and settled readings serve as its 1 and 0. So offset and gain errors don't bias tau.

**Fitted parameters:**
- Cd per membrane type,
- tau and relative delay per sensor,
- burst time and opening time per capture.

`--per-capture` refits each capture alone and plots tau against burst pressure.

### Getting trustworthy results

- **Use two volumes** (add the capped nipple) and a spread of burst pressures, about 10 to 60 psig. With one volume, blowdown time and sensor lag trade off, and the script warns about it.
- **About 6 or more bursts per volume** per sensor configuration.
- **Trust the per-capture scatter, not the joint fit's +/- values.** Those assume the model is exact and will be far too small on real data.
- **Read the residual plots.** Systematic structure means one of three things:
  - acoustic ringing (shared by all sensors),
  - a sensor that isn't first-order (overshoot, or steps from its conditioning chip's update rate),
  - a non-linear membrane opening.
- **Test the sensor, not the plumbing.** Mount each sensor as close to the chamber as possible, with minimal dead volume.

### Synthetic check

Truth: tau = 0.5 / 1.5 / 4.0 ms, delays 0 / 0.05 / -0.03 ms, Cd 0.72. The synthetic pressure is integrated independently of the fitter, with per-sensor gain and offset errors, noise, and 12-bit quantization.

| Case | Joint fit | Per-capture medians |
|---|---|---|
| Exact model | within 0.1% | within 0.1% |
| ±8% Cd scatter per capture + 5% acoustic ring, 50 / 200 ms window | within 2% (0.491 / 1.498 / 4.000 ms) | within 0.1% |
| Same, 20 / 80 ms window (firmware 3.0) | within 0.3% (0.499 / 1.502 / 4.001 ms) | within 0.1% |

The 20 / 80 result needed a fix in `blowdown_fit.py`: the burst-time estimate used to pick up the smoothing filter's edge artifact when the capture starts only 20 ms before the trigger.

This demonstrates the code and that the parameters are identifiable. It says nothing about real-world accuracy.

```
python analysis/make_synthetic.py syn --mismatch
python analysis/blowdown_fit.py syn/rig_config.json --plots syn/plots --per-capture
python tests/test_parser.py
```

## Simulator and tests

`tools/sim` builds the real firmware sources on a PC against mock Arduino, Wire, ADC, DMA and flash layers. The I2C bus carries emulated UI-board parts (TCA9555, SSD1315 with its display RAM) and the FRAM, and the ADC sees a modelled rig: pressure ramps, a 0.5 ms blowdown, and sensors with 0.5 / 1.5 / 4 ms lags. The device models flag sequencing errors (talking to the panel while it is in reset, display on without VBAT, FRAM writes with WP high).

```
make -C tools/sim
python tests/test_sim.py
```

The tests cover auto-arm, keep, dump with `--erase` and loading into the analysis; manual arm, forced capture and drop; slow vent disarm; deleting one and all from the menu; zeroing and settings surviving a reboot; filling all 37 slots; Live mode on a bare RP2040-Zero and from the menu; and display blanking and waking. `tools/render_screens.cpp` draws every screen to images for layout checks.

What the simulator can't tell you: real ADC behaviour (sampling charge, noise, crosstalk), real flash and I2C timing, the panel's actual brightness with the internal IREF setting (`0xAD 0x10`; try `0x30` if too dim), and button feel.

## Safety

- Face shield and hearing protection.
- Point the open end away from people.
- Start at 10 to 20 psi and work up.
- Shop air is fine for all of these tests.
