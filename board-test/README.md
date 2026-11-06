# Board bring-up test

Firmware for bench-testing the production boards: the DMX base board (Rev A), the UI lid board (Rev A), and, when stacked, the solenoid driver and igniter (spark) daughterboards. It runs a full test sequence at boot, prints a graded report over USB serial, and then takes single-letter commands to rerun individual tests.

`host/artnet_send.py` is the DMX source for the receive test: it runs on the controller Pi and sends Art-Net to the CR011R node, which puts DMX on the XLR into the base board's J1.

**Status:** run on hardware. A base board with UI board and OLED passes the full sequence on 24 V (all status LEDs green). The solenoid test (`v`) and igniter test (`g`) have been run on one board each. DMX receive (`d`) passes on one base board with the Pi + CR011R sending `artnet_send.py`'s pattern: 30 Hz, 512 slots, every slot correct. The CR011R puts out one DMX frame per Art-Net packet (pattern changed on all but one frame), not a fixed refresh. The `e` monitor and the Art-Net loss test are not yet run.

**Bench only.** The full run drives the CH1-CH6 gate lines and, if a solenoid board is detected, opens its valve. Keep the gas closed and nothing connected to a burner. An igniter wired to the selected channel will spark during the CH walk.

## Building

- Arduino IDE with the Earle Philhower **Raspberry Pi Pico/RP2040** core. Built against core 6.3.0.
- Tools > Board: **Waveshare RP2040 Zero**.
- Library: **Adafruit NeoPixel** (Library Manager), for the module's WS2812 on GP16.
- Library: **PooferCommon** from this repo (pin map, DMX parser). One-time junction setup in the [top README](../README.md#one-time-setup-make-the-ide-see-poofercommon); without it the build fails with `poofer_board.h: No such file or directory`.
- Open `firmware/main_board_test/main_board_test.ino`. First upload goes to the UF2 port (hold BOOT while plugging in); after that the board shows up as a normal serial port.
- Serial Monitor at any baud (USB). Commands are single letters; line endings are ignored.

## Power

Most of the base board tests run on USB alone. +5V comes only from the U1 buck off 24 V (D2 stops the module's USB 5 V from back-feeding it), so on USB alone the UI LEDs, the OLED's VBAT, the DMX transceiver and the solenoid gate driver are unpowered. The firmware detects this and runs those parts electrically only, or skips them. Connect 24 V for the full visual pass.

## Full sequence (`a`, also runs at boot)

| Step | Checks |
|---|---|
| Boot info | Module unique ID, reset cause, core temperature |
| WS2812 | Red, green, blue, white (visual) |
| +5V / DMX logic side | +3V3_DMX present via R38/R39 (infers 24 V and U1, U5 alive); R37 DE pulldown |
| I2C lines | Both lines pulled up by R3/R4, not stuck; 9-clock recovery if SDA is held |
| FRAM U4 | R47 WP pull-up, ACK at 0x50, device ID, write protect works, write/readback/restore at 0x1FF0, no address aliasing |
| UI board U1 (TCA9555, 0x20) | Power-on state (R10, R13 pulldowns), register identity, all 12 LED sink lines read back; visual LED walk on 24 V |
| OLED (SSD1315, 0x3C) | Held in reset until U1 releases it; init with internal IREF; white, checkerboard, orientation pattern on 24 V |
| Address report | Every expected and possible address on the bus (base, UI, igniter), found / missing / unexpected |
| Buttons | Idle level; press test for UP, DN, SET with bounce and cross-talk check (UI board present) |
| ADC0-3 | Readings (floating with nothing stacked) |
| SPARE1 | Which pull, if any, is fitted |
| CH1-CH6 | R11-R16 pulldowns, line-to-line isolation, 0.5 s walk per channel |
| DMX receive | 2 s listen on J1 (24 V only). No DMX at all is INFO, not a fault. With a source: frame rate, slot count, line errors, and slot-by-slot check of `artnet_send.py` pattern frames. See [DMX receive](#dmx-receive) |
| Igniter board | If anything answers at 0x48-0x4F: ADS1115 and TMP1075/LM75B identity, cold-junction temperature, thermocouple reading (pegged full scale = open probe, expected with nothing on J4). Output is not toggled; LED2 (OUT) blinks during the CH walk |
| Solenoid board | If an ADC line is held low by ADC_SENSE (needs the JP2 shunt): T1-T3 and T5 from the solenoid spec on JP1 pair 1, 24 V coil, no T_max. No current is a WARN here (no valve fitted yet is normal). Skipped without 24 V |
| Summary | Graded PASS / WARN / FAIL / INFO / SKIP, grouped by board |

Status LEDs at the end: module WS2812 shows the overall result; on 24 V the UI LEDs show base, UI, OLED and overall. Green pass, yellow warn, red fail, blue nothing graded.

## Commands

| Key | Does |
|---|---|
| `a` | Full sequence |
| `s` | I2C line check, scan and address report |
| `f` | FRAM test |
| `u` | UI board (U1 and LED walk) |
| `o` | OLED test |
| `b` | Button press test, runs even if no UI board was detected |
| `c` | CH1-CH6 checks and walk |
| `d` | DMX receive test, 5 s, graded. No DMX is a FAIL here |
| `e` | DMX live monitor: rate, pattern check and slots 1-12 twice a second; flags gaps over 1 s |
| `w` | 20 kHz PWM on CH1-CH6 at 10-60 % for scoping. Asks first; not with a solenoid board stacked |
| `x` | ADC monitor |
| `l` | WS2812 colour cycle |
| `p` | +5V rail / DE pulldown check |
| `i` | I2C diagnostic: tries 0x50-0x57 with two methods at 100 and 400 kHz |
| `t` | Slow open-drain SDA/SCL toggle for metering; module LED shows amber (neither low), purple (SDA low), cyan (SCL low) |
| `v` | Solenoid board, interactive: asks JP1 pair, JP2 line, coil type; optional T_max measurement |
| `g` | Igniter board, interactive: chip checks, thermocouple, then toggles the JP1 channel (default 6) 3 s on / 3 s off with I2C error count |
| `m` | Live thermocouple monitor |
| `k` | Toggle functional I2C clock 100 / 400 kHz |
| `r` | Reprint summary |
| `h` | Menu |

## DMX receive

Path under test: XLR harness, J1, D3, U3 (ADM2587E) and its isoPower supply (L1, L2), R40, GP1. The firmware reads GP1 with the RP2040's UART0 at 250 kbaud 8N2 with break detection. That is not the PIO receiver planned for production firmware, so a pass here proves the hardware, not production receive code. DE stays low and GP0 is not driven, so the board never talks on the bus.

### Setup

1. Pi and CR011R as in the controller notes: direct Ethernet, Pi `eth0` 10.0.0.1/8, node 10.0.0.10, node mode "Artnet -> dmx", universe 0:0:0.
2. XLR from the node's DMX out to the base board's J1 harness (XLR 3 data+ to J1.1/J1.6, XLR 2 data- to J1.2/J1.5, XLR 1 to J1.3/J1.4). Fit the JP1 termination shunt (this board is the end of the line).
3. Base board on 24 V. U3 is powered from +5V, which only exists on 24 V.
4. **Only one Art-Net source.** Quit QLC+ from its own menu (normal exit, so the launcher leaves it closed and the labwc menu can start it again), or unpatch its universe 1 output. Two sources on one universe is undefined at the node.
5. Get the script onto the Pi (stdlib only, no installs):
   ```
   wget https://raw.githubusercontent.com/dudedsy/poofer-firmware/main/board-test/host/artnet_send.py
   ```

**Bench only.** The sender drives every slot of the universe, including values that would arm and fire a poofer running production firmware. The board-test firmware itself never acts on DMX.

### Sender (`host/artnet_send.py`, on the Pi)

| Command | Sends |
|---|---|
| `python3 artnet_send.py poll` | ArtPoll; lists nodes that answer, with firmware and universe settings. Not all nodes answer |
| `python3 artnet_send.py` | Test pattern, 512 slots, 30 packets/s, until Ctrl-C. Sends a blackout on exit |
| `python3 artnet_send.py pattern -d 20` | Pattern for 20 s, then blackout |
| `python3 artnet_send.py set 1=255 2=128` | Fixed values, all other slots 0 (`--all N` for the rest) |
| `python3 artnet_send.py ramp` | Every slot the same value, rising by 1 per packet |
| `python3 artnet_send.py pattern -d 10 --no-blackout` | Stops dead after 10 s, leaving the last frame on the node (Art-Net loss test) |

Other options: `--dest` (default 10.0.0.10), `--broadcast` (10.255.255.255), `--bind` (default 10.0.0.1; `--bind ''` on another machine), `-u` universe (15-bit Port-Address, default 0 = QLC+ universe 1), `-r` packets/s, `-n` slots. If it says it cannot bind 10.0.0.1, `eth0` has no link: node unpowered or cable out (`ip addr show eth0`).

Pattern: slot i = (seq + h(i)) mod 256, with h a fixed 8-bit hash of the slot number and seq going up by one per packet. The board recovers seq from each frame, so it checks every slot without being told what was sent.

### Firmware side

Run the sender, then `d` (graded) or `e` (live). What the results mean:

| Result | Meaning |
|---|---|
| DMX receive FAIL, nothing on RO | No signal reaching GP1. Sender running? Node in Art-Net to DMX mode? Then VISOOUT TP to DMX_ISO_GND (~3.3 V; 0 V = L1 open or U3's converter dead), +3V3_DMX TP, scope DMX_RO TP |
| DMX receive FAIL, bytes but no clean frame | Mostly framing errors: A/B reversed in the harness or node, or a very noisy line |
| DMX line errors | Framing errors = noise, missing termination, reflections. Overruns = the firmware fell behind (a firmware bug, not hardware). Anything before the first break is ignored: listening usually starts mid-frame, and the UART misreads data bits until it syncs |
| DMX pattern FAIL | Wrong data in a clean frame. Random bits = noise; everything wrong from one slot on = a dropped or extra byte. Prints the first bad slot, got vs expected |
| DMX torn frames (INFO) | A frame that is half one packet and half the next: the node updated its buffer mid-frame. Harmless for the poofers |
| DMX padded frames (INFO) | Pattern then zeros: the sender sent fewer slots (`-n`) than the node outputs |
| DMX updates WARN | The pattern never changed: the node is repeating one frame (sender stopped) |
| DMX gaps WARN | More than 1 s between frames: a poofer board's DMX-loss timeout would have tripped |

`e` prints, every 0.5 s: frames per second, slots in the last frame, pattern frames ok / torn / BAD / other, frames dropped for line errors, the pattern seq, and slots 1-12. With no break for over 1 s it prints `NO DMX for x s`. The module LED is green while clean frames arrive, yellow on errors, red with no DMX.

### Art-Net loss test (open item in the controller notes)

Does the CR011R keep sending its last frame when Art-Net stops? The `e` monitor answers it without a scope:

1. Start `e` on the board, then `python3 artnet_send.py pattern -d 10 --no-blackout` on the Pi.
2. After 10 s the sender stops dead. Watch `e`:
   - Frames keep coming with `seq` frozen: **the node holds last data**. A poofer board would keep seeing valid DMX, its 1 s DMX-loss timeout would never trip, and only the hardware max-on-time limiter would end a burn. Needs a controller-side fix.
   - `NO DMX`: the node stops output on Art-Net loss, and the poofer timeout does its job.
   - Frames with "other" climbing and slots at 0: the node blacks out (zeros) on loss.
3. Repeat with the Ethernet cable pulled mid-run, and with `pkill -STOP -f artnet_send` (sender hung, link up), then `pkill -CONT -f artnet_send`. Note how long the node takes to change state, if it does.

## Settings in the sketch

- `WS2812_ORDER`: `NEO_GRB`. If the red step shows green, change to `NEO_RGB`.
- `OLED_IREF`: `0x30` (internal, 30 uA). Required because R11 is DNP on the built UI boards. `0x10` is the dimmer 19 uA setting.
- OLED orientation is set by `0xA1` / `0xC8` in `oledInitSeq()`. On the first board the orientation block showed bottom-right instead of top-left; swap to `0xA0` / `0xC0` once the lid mounting is settled.

## Host tests

```
python3 board-test/tests/test_dmx.py
```

Needs `g++`. Compiles `tests/dmx_harness.cpp` around the firmware's own `libraries/PooferCommon/src/poofer_dmx_rx.h` and feeds it synthetic UART streams built from the sender's pattern function: clean frames, short frames, seq wrap, a corrupt byte, dropped and extra bytes, torn frames, zero padding, framing errors, overruns, alternate start codes, gaps over 1 s and A/B-reversed garbage. Also checks the ArtDmx and ArtPollReply layouts.

## Notes from bring-up

- **Screen the RP2040-Zero modules with this test.** The first module had a damaged GP13: it toggled bare (against the ~50k internal pull-up) but could not pull SCL low against R4 (2.2k), so every I2C device looked absent. The `t` command and a module swap found it. A module that passes the FRAM and I2C checks here has good GP12/GP13 under real load.
- Thermocouple temperatures printed by `g` and `m` use a linear approximation near room temperature (K ~40.7 uV/C, N ~26.5 uV/C). Good for a warm-finger check, not for flame temperatures; production firmware should use the NIST ITS-90 polynomials.
- The igniter board has no current sense, so its output can only be checked by LED2 or a meter (TP2 to TP6 with U1 empty, J3 with the link or buck fitted).
