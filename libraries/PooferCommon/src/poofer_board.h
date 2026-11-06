// poofer_board.h - DMX base board Rev A pin map and stack I2C addresses (PooferCommon).
//
// Source of truth: dmx-base-board-schematic-spec-revA.md §5 (module pins), §8 (J2/J3
// pinouts), §7 (FRAM). Shared by the board test and production firmware so a pin
// change is made once. Board-test-only diagnostics (refdes strings for fault
// messages) stay in the board-test sketch.
#pragma once
#include <stdint.h>

// ---- RP2040-Zero GPIO (base spec §5)
static const uint8_t PIN_DMX_DI = 0;    // to U3 TxD via R48, R38 pull-up to +3V3_DMX. Reserved (RDM); never driven
static const uint8_t PIN_DMX_RO = 1;    // from U3 RxD via R40, R39 pull-up to +3V3_DMX
static const uint8_t PIN_DMX_DE = 2;    // to U3 DE,/RE via R49, R37 pulldown. Keep LOW (receive only)
static const uint8_t PIN_BTN[3] = {3, 4, 5};          // BTN_UP, BTN_DN, BTN_SET; to GND when pressed, internal pull-ups
static const uint8_t PIN_CH[6] = {6, 7, 8, 9, 10, 11}; // CH1-CH6 to J3, R5-R10 series, R11-R16 pulldowns
static const uint8_t PIN_SDA = 12, PIN_SCL = 13;      // I2C0, R3/R4 2.2k pull-ups (the only ones on the bus)
static const uint8_t PIN_SPARE1 = 14;                 // J3.29 via R20; no pull fitted as shipped
static const uint8_t PIN_FRAM_WP = 15;                // U4 WP, R47 pull-up: HIGH = protected
static const uint8_t PIN_WS2812 = 16;                 // on-module RGB LED
static const uint8_t PIN_ADC[4] = {26, 27, 28, 29};   // ADC0-ADC3 to J3 via R29-R26 (4.7k)

// ---- connector pins, for messages (base spec §8)
static const uint8_t BTN_J2[3] = {7, 8, 9};
static const uint8_t CH_J3[6] = {5, 7, 9, 11, 13, 15};
static const uint8_t ADC_J3[4] = {19, 21, 23, 25};

// ---- I2C addresses (7-bit)
static const uint8_t ADDR_TCA = 0x20;    // UI board U1 TCA9555
static const uint8_t ADDR_OLED = 0x3C;   // UI board DS1 SSD1315
static const uint8_t ADDR_FRAM = 0x50;   // base U4 MB85RC64TA, straps as built

// ---- DMX (poofer-firmware-spec §3, §4)
static const uint16_t DMX_HEARTBEAT_SLOT = 512;
static const uint8_t DMX_HEARTBEAT_A = 85, DMX_HEARTBEAT_B = 170;
static const uint32_t DMX_HEARTBEAT_TIMEOUT_MS = 1000;
