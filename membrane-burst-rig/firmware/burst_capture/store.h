// store.h - capture storage in the RP2040-Zero's QSPI flash, settings in the base
// board FRAM (U4, MB85RC64TA at 0x50, write-protect on GP15).
//
// Captures go in fixed-size slots above the sketch (STORE_OFFSET up to the last
// sector, which arduino-pico reserves for EEPROM emulation). Uploading new firmware
// does not touch the slots, as long as the sketch stays below STORE_OFFSET; the
// firmware checks that at boot and refuses to use storage otherwise.
//
// Slot layout: page 0 = CapHdr (programmed last, so a slot only counts once the
// whole capture is down), data from byte 256: 12-bit samples packed two per three
// bytes, channel-interleaved exactly as the ADC produced them.
#pragma once
#include <stdint.h>
#include <stddef.h>

#define STORE_OFFSET   0x40000u              // 256 KB; sketch must end below this
#define SLOT_BYTES     (48u * 1024u)         // 12 sectors
#define SLOT_DATA_OFF  256u
#define CAP_MAGIC      0x31535242u           // "BRS1"

struct CapHdr {
  uint32_t magic;
  uint16_t version;          // 4 (added cpp, chip temperature, trigger channel/drop)
  uint16_t hdr_bytes;
  uint32_t id;               // capture number, unique and increasing
  uint32_t session;          // collect-mode session number
  uint8_t  tag;              // user tag set in the menu (rig configuration)
  uint8_t  n_ch;
  uint8_t  forced;
  uint8_t  packing;          // 1 = 12-bit pairs
  uint32_t frames;
  uint32_t trig_frame;
  uint32_t fs_ch;
  uint32_t sps_total;
  uint16_t pre_ms, post_ms;
  uint16_t trig_drop;        // counts
  uint16_t err_samples;
  uint32_t uptime_ms;
  uint16_t zero[4];          // per-channel vented counts (calibration offset)
  float    cpp[4];           // per-channel counts per psi (calibration gain)
  float    pre_mean[4];      // counts
  float    end_mean[4];      // counts
  int16_t  chip_temp_x10;    // RP2040 die temperature at capture, 0.1 C (rough ambient)
  uint8_t  trig_ch;          // channel that tripped the drop trigger, 0xFF = forced (v4+)
  uint8_t  trig_drop_x10;    // its drop below the 20 ms baseline when it tripped, 0.1 psi, max 25.5 (v4+)
  uint32_t data_bytes;
  uint32_t data_crc;
  uint32_t hdr_crc;          // CRC32 of every byte above
};

struct SlotInfo {
  bool used;
  CapHdr h;
};

uint32_t crc32_update(uint32_t crc, const uint8_t *p, size_t n);  // start with 0

// ---- flash slots ----
bool     store_begin();                 // false if the sketch overlaps the store
bool     store_ok();
int      store_slots();                 // total
int      store_used();
int      store_free();
// Fills `out` with used slot indices, newest first; returns count.
int      store_list(int *out, int max);
const CapHdr *store_hdr(int slot);      // nullptr if empty
uint32_t store_max_id();
// Saves one capture. `sample(k)` returns raw ADC word k (0..n-1), bit 15 = error flag.
// Header fields data_bytes/data_crc/err_samples/hdr_crc/magic are filled in here.
int      store_save(CapHdr &h, uint32_t n_samples, uint16_t (*sample)(uint32_t k));
bool     store_delete(int slot);
int      store_find(uint32_t id);       // slot or -1
// Reads unpacked samples [k0, k0+n) of a stored capture into dst (12-bit values).
void     store_read(int slot, uint32_t k0, uint32_t n, uint16_t *dst);
bool     store_check_crc(int slot);

// ---- settings in FRAM ----
struct Settings {
  uint32_t magic;            // 'BCF2'
  uint8_t  n_ch;             // 1..3
  uint8_t  tag;              // 0..15
  int16_t  arm_psi_x10;      // < 0 = auto-arm off
  uint16_t trig_psi_x10;     // drop trigger
  uint16_t zero[4];          // vented counts per channel
  float    cpp[4];           // counts per psi per channel (nominal 32.76)
  uint32_t next_id;
  uint32_t session;
  uint32_t crc;
};

bool settings_load(Settings &s);        // false if FRAM absent or blank (defaults set)
bool settings_save(Settings &s);
void settings_defaults(Settings &s);
bool fram_present();
