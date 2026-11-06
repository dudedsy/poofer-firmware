/*
  poofer_main.ino - production firmware for the poofer DMX base board (skeleton).

  Status: SKELETON. Puts every pin in its safe state and reports over USB serial.
  It does not receive DMX and never drives an output. Behaviour to implement is in
  the Poofer Firmware Spec (project doc claude/poofer-firmware-spec.md).

  Toolchain : Arduino IDE / arduino-cli, Earle Philhower "Raspberry Pi Pico/RP2040"
              core 6.3.0, board "Waveshare RP2040 Zero"
  Library   : PooferCommon (this repo, libraries/PooferCommon; see the top README)
*/

#include <Arduino.h>
#include <poofer_board.h>

#define FW_VERSION "0.0.1-skeleton"

static void safePins() {
  // CH1-CH6 low: R11-R16 already hold them low, but drive them too once we can.
  for (uint8_t p : PIN_CH) { pinMode(p, OUTPUT); digitalWrite(p, LOW); }
  pinMode(PIN_DMX_DE, OUTPUT); digitalWrite(PIN_DMX_DE, LOW);      // receive only
  pinMode(PIN_DMX_DI, INPUT);                                      // never drive the bus
  pinMode(PIN_FRAM_WP, OUTPUT); digitalWrite(PIN_FRAM_WP, HIGH);   // FRAM protected
  for (uint8_t p : PIN_BTN) pinMode(p, INPUT_PULLUP);
}

void setup() {
  safePins();
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 2000) delay(10);
  Serial.printf("poofer_main %s (built %s %s)\n", FW_VERSION, __DATE__, __TIME__);
  Serial.println("Skeleton: outputs held off, DMX not implemented.");
}

void loop() {
  delay(1000);
}
