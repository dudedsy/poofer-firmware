# poofer-firmware

Firmware, test rigs and host tools for the Brain Freeze propane poofer project (DMX-controlled flame effects for Burning Man 2027). Formerly `poofer-test`; GitHub redirects the old URLs. Board hardware (KiCad, gerbers, board specs) is in the separate `poofer-hardware` repo for now.

| Folder | What |
|---|---|
| [`firmware/poofer_main/`](firmware/poofer_main/) | Production firmware for the DMX base board. **Skeleton only**: safe pin states and a serial banner. The behaviour to build is in the Poofer Firmware Spec. |
| [`libraries/PooferCommon/`](libraries/PooferCommon/) | Arduino library shared by every sketch here: base board pin map and I2C addresses (`poofer_board.h`), DMX frame parser and test-pattern checker (`poofer_dmx_rx.h`). |
| [`board-test/`](board-test/) | Bring-up test firmware for the DMX base board, UI lid board, solenoid driver and igniter boards, plus `host/artnet_send.py` (Art-Net source for the DMX receive test). |
| [`membrane-burst-rig/`](membrane-burst-rig/) | Membrane burst rig for measuring pressure-sensor response time: RP2040 capture firmware, laptop logger, and the blowdown/sensor-lag fit. |

## Building the sketches

All sketches use the Arduino IDE (or arduino-cli) with the Earle Philhower **Raspberry Pi Pico/RP2040** core 6.3.0, board **Waveshare RP2040 Zero**. The board test also needs **Adafruit NeoPixel** from the Library Manager.

### One-time setup: make the IDE see PooferCommon

The Arduino IDE only compiles what is in the sketch folder plus installed libraries, so the shared library is linked into your sketchbook's `libraries` folder once. Check the sketchbook path under File > Preferences > Sketchbook location (default `Documents\Arduino`).

Windows, in PowerShell (no admin needed for a junction). Replace the target with where your clone is:

```
New-Item -ItemType Junction `
  -Path   "$env:USERPROFILE\Documents\Arduino\libraries\PooferCommon" `
  -Target "$env:USERPROFILE\Documents\poofers\firmware\poofer-firmware\libraries\PooferCommon"
```

Restart the IDE. PooferCommon then shows under Sketch > Include Library, and edits in the repo take effect immediately. If you move or rename the clone, delete the junction (`Remove-Item` on the junction path; this does not touch the repo) and make it again.

arduino-cli needs no setup: pass `--library libraries/PooferCommon`, as CI does:

```
arduino-cli compile --fqbn rp2040:rp2040:waveshare_rp2040_zero --library libraries/PooferCommon firmware/poofer_main
```

## Tests and CI

GitHub Actions (`.github/workflows/ci.yml`) runs on every push to main and every PR:

- Host tests: `board-test/tests/test_dmx.py` (DMX parser on synthetic UART streams, Art-Net packet layout), `membrane-burst-rig/tests/test_parser.py`, and the burst rig simulator tests (`make -C membrane-burst-rig/tools/sim`, then `test_sim.py`).
- Firmware builds: all three sketches compiled with arduino-cli against arduino-pico 6.3.0.

Run the host tests locally (needs Python 3, `g++`, `make` and `membrane-burst-rig/requirements.txt`; WSL works):

```
python3 board-test/tests/test_dmx.py
python3 membrane-burst-rig/tests/test_parser.py
make -C membrane-burst-rig/tools/sim && python3 membrane-burst-rig/tests/test_sim.py
```
