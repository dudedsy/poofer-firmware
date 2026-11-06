# Shadowgraph LED driver

Current-regulated drive for the cold-test shadowgraph light (Cree XHP70.2, 12 V configuration), using a stock **solenoid driver board (Rev A)** on the **DMX base board (Rev A)**. The solenoid board becomes a buck converter with an inductor and a capacitor on its valve connector; the RP2040 sets the LED current in firmware.

The control code is `poofer::LedDriver` in [PooferCommon](../libraries/PooferCommon/) (`poofer_drive.h`), sharing the current loop and estimator with `poofer::SolenoidDriver`. This sketch is a serial and button front end.

**Status:** compiles (arduino-pico 6.3.0, Waveshare RP2040 Zero, no warnings). The library's host simulation (`make -C libraries/PooferCommon/tests test`) covers a 22 to 26 V bus and 120 to 220 µH effective inductance: 2 A target settles in about 0.16 to 0.18 s, worst overshoot about 2.08 A, setpoint steps settle in about 30 ms. That is a model, not a measurement. **Not yet run on hardware.**

**Bench / test rig only.** Never on a solenoid board that drives a gas valve.

## Hardware

| Part | Choice |
|---|---|
| Inductor | **Bourns 2200LL-221-H-RC**: 220 µH, 4.4 A (30 °C rise), 63 mΩ, sendust; 103 µH at 4.4 A, roughly 160 to 180 µH at 2 A (est.) |
| Capacitor(s) | **Panasonic ECA-1HHG471** (base board C2 spare): 470 µF 50 V 105 °C, ~795 mA high-frequency ripple rating. Two in parallel if spares allow |
| LED | Cree XHP70.2 12 V on 20 mm MCPCB. Measured Vf ≈ 10.81 V + 0.57 Ω × I. Max 2.4 A, **reverse voltage 1 V max** |

Wiring on the solenoid board's J1:

```
J1 pin 1 (VALVE_HI) ── inductor ──┬── LED+
                                  │
                                 [C] 470 µF (+ at LED+), 1 or 2 in parallel
                                  │
J1 pin 2 (VALVE_LOW) ─────────────┴── LED−
```

- Polarity matters: LED+ and capacitor + on the inductor side.
- Strain-relieve the inductor and wires; J1 is held by SMD pads only.
- JP1: two shunts in one row (pair 1 = CH1/CH2 is the firmware default). JP2: one shunt (ADC0 default).
- **Scope ground only at board GND (TP7).** VALVE_LOW sits near +24 V whenever Q2 is off; a ground clip on the LED or inductor side shorts it to ground, bypasses Q2 and lights the LED.

### How it works

FIRE high turns Q1 on; PWM chops Q2 (via U2). D2, which returns to +24V_F1 through Q1, is the freewheel diode, exactly the valve's hold path. Average voltage across L + LED is about duty × V_bus; L and C filter it into DC. If the hardware limiter trips (Q1 off), the inductor's energy goes into D5 and the LED current keeps flowing forward, so the LED never sees reverse voltage.

The current sense (R18 25 mΩ × U3 gain 20 = 0.5 V/A, 1 ms RC) only sees Q2 current, which averages to duty × inductor current. The library estimates **LED current = ADC_SENSE / 0.5 / filtered duty**, where the duty goes through the same 1 ms filter as the sense (dividing by the instantaneous duty makes the loop limit-cycle). Below a filtered duty of 0.03 the last estimate is held, and the loop keeps a 0.05 duty floor so the current stays observable.

Rough numbers at 2 A, 50 kHz (est.): inductor ripple about 0.7 A p-p, peak about 2.35 A through Q2/R18; capacitor ripple current about 0.2 A RMS; LED ripple about 5 % with one capacitor, about 2.5 % with two. A 122 µs exposure averages about 6 ripple cycles, so frame-to-frame brightness variation should stay under about 1 %. Bus draw about 1.2 A at 24 V.

## Building

- Arduino IDE or arduino-cli, Earle Philhower **Raspberry Pi Pico/RP2040** core 6.3.0, board **Waveshare RP2040 Zero**.
- Needs **PooferCommon** (see the top README for the one-time IDE link; arduino-cli: `--library libraries/PooferCommon`).
- Open `firmware/led_driver/led_driver.ino`.

## Use

USB serial, any baud, one command per line:

| Command | |
|---|---|
| `i 1.5` | set target to 1.5 A and turn on (`i 0` = off). Capped at `imax` |
| `on` / `off` | on at the current target / off |
| `s` | status: state, target, LED current estimate, raw sense, duty, feed-forward, limits |
| `log 100` | CSV every 100 ms: `t_ms,state,target_A,led_A,sense_V,duty`. `log 0` stops |
| `clear` | clear a latched fault |
| `pair 1-3`, `adc 0-3` | JP1 row and JP2 line (only while off) |
| `pwm 50000` | PWM frequency, 20 to 100 kHz (only while off) |
| `vbus 24` | assumed bus voltage for feed-forward |
| `imax 2.3`, `trip 2.6` | highest allowed target; software over-current trip |
| `maxon 20` | firmware on-time limit in s (heat), `0` = none |
| `rearm 5` | re-arm the hardware limiter every 5 s while on (LED blinks off ~50 ms), `0` = off |
| `kp`, `ki`, `rate`, `ramp` | loop tuning: PI gains (0.003, 2.0), reference ramp in A/s (30), start-up duty ramp per s (5) |

UI board buttons, if fitted: **UP / DN** = ±0.1 A, **SET** = on / off.

### Sequence

1. **Off:** all CH lines low, PWM 0.
2. **Ramp:** FIRE high, duty ramps at 5/s while the LED is still dark (capacitor charging) and stops 0.02 short of the LED threshold. A step would ring the undamped LC and overshoot.
3. **Run:** the reference ramps toward the target at `rate` A/s; duty = feed-forward `(10.81 + 0.72 × I + 0.8) / (vbus + 0.8)` plus PI on the current estimate, with conditional integration (the integrator freezes while the duty is clamped).
4. **Off:** duty to 0 first (L and C discharge gently into the LED), FIRE low 20 ms later.

### Protection

- **Over-current:** LED estimate above `trip` (default 2.6 A) for 3 ms → fault.
- **Sense saturated** (> 3.0 V, about 6 A in Q2) → fault.
- **No current:** duty well past what should light the LED but the estimate below 20 % of target for 200 ms → fault. Causes: hardware limiter tripped, open LED or inductor, blown F1, wrong JP1 row or JP2 line, no 24 V.
- **Duty ceiling:** what `trip` would need at 10 % low bus, plus 0.03, capped at 0.8.
- **Firmware on-time limit** (`maxon`, default 20 s): the LED dissipates about 24 W at 2 A, a lot for a 40 mm heatsink.
- **Hardware limiter (T_max, RV1, about 1.2 to 17 s)** still applies. For long alignment sessions, turn RV1 up and/or use `rearm`, and raise `maxon`.
- Faults latch (duty 0, FIRE low after 20 ms) until `clear`.

## First power-up

1. Base + solenoid board, no LED/inductor fitted, 24 V on. `s`: sense near 0.
2. Fit inductor, capacitor(s) and LED (check polarity). Bench supply on the 24 V input with current limit about 2 A for the first run.
3. `log 50`, then `i 0.3`. Expect the estimate to settle at 0.30 A within about 0.2 s and duty near 0.48.
4. Confirm against a meter: a 0.5 Ω shunt in the LED lead (0.15 V at 0.3 A), or the bench supply's input current (about duty × LED current, plus a little).
5. Step up: `i 1.0`, `i 1.5`, `i 2.0`. Watch TP9 (unfiltered sense, 0.5 V/A, scope ground at TP7): a sawtooth during Q2's on-time with peaks around 1.2 V at 2 A.
6. If the estimate and the meter disagree by more than a few percent, the duty-division assumption or the 0.5 V/A scale is off; report the numbers.
