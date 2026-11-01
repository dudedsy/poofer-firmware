// store.cpp - see store.h
#include <Arduino.h>
#include <Wire.h>
#include <string.h>
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "store.h"

#ifndef PICO_FLASH_SIZE_BYTES
#define PICO_FLASH_SIZE_BYTES (2u * 1024u * 1024u)
#endif
// arduino-pico keeps the last sector for EEPROM emulation.
#define STORE_END   (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)
#define NSLOT_MAX   ((STORE_END - STORE_OFFSET) / SLOT_BYTES)

static_assert(STORE_OFFSET % FLASH_SECTOR_SIZE == 0, "store must be sector aligned");
static_assert(SLOT_BYTES % FLASH_SECTOR_SIZE == 0, "slots must be whole sectors");
static_assert(sizeof(CapHdr) <= SLOT_DATA_OFF, "header must fit in page 0");

#ifdef BURST_SIM
uint32_t sim_binary_end_offset();
static uint32_t binary_end_offset() { return sim_binary_end_offset(); }
#else
extern "C" char __flash_binary_end;
static uint32_t binary_end_offset() { return (uint32_t)((uintptr_t)&__flash_binary_end - XIP_BASE); }
#endif

static bool ok = false;
static SlotInfo slots[NSLOT_MAX];
static uint8_t pagebuf[FLASH_SECTOR_SIZE] __attribute__((aligned(4)));

uint32_t crc32_update(uint32_t crc, const uint8_t *p, size_t n) {
  crc = ~crc;
  while (n--) {
    crc ^= *p++;
    for (int k = 0; k < 8; k++) crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
  }
  return ~crc;
}

static inline const uint8_t *slot_ptr(int s) {
  return (const uint8_t *)((uintptr_t)XIP_BASE + STORE_OFFSET + (uint32_t)s * SLOT_BYTES);
}
static inline uint32_t slot_off(int s) { return STORE_OFFSET + (uint32_t)s * SLOT_BYTES; }

static uint32_t hdr_crc(const CapHdr &h) {
  return crc32_update(0, (const uint8_t *)&h, offsetof(CapHdr, hdr_crc));
}

// Flash writes run with interrupts off (code and ISRs execute from flash). Only
// core 0 runs in this sketch, so there is no other core to park. Erase one sector
// per critical section so USB is never starved for more than one erase (~45 ms
// typical for the W25Q16, longer worst case).
static void flash_erase(uint32_t off, uint32_t len) {
  for (uint32_t o = 0; o < len; o += FLASH_SECTOR_SIZE) {
    uint32_t st = save_and_disable_interrupts();
    flash_range_erase(off + o, FLASH_SECTOR_SIZE);
    restore_interrupts(st);
  }
}
static void flash_prog(uint32_t off, const uint8_t *src, uint32_t len) {
  uint32_t st = save_and_disable_interrupts();
  flash_range_program(off, src, len);
  restore_interrupts(st);
}

static void scan_slot(int s) {
  CapHdr h;
  memcpy(&h, slot_ptr(s), sizeof h);
  slots[s].used = h.magic == CAP_MAGIC && h.hdr_bytes == sizeof(CapHdr) && h.hdr_crc == hdr_crc(h);
  slots[s].h = h;
}

bool store_begin() {
  ok = binary_end_offset() <= STORE_OFFSET;
  if (!ok) return false;
  for (int s = 0; s < (int)NSLOT_MAX; s++) scan_slot(s);
  return true;
}

bool store_ok() { return ok; }
int store_slots() { return ok ? (int)NSLOT_MAX : 0; }

int store_used() {
  int n = 0;
  if (ok) for (int s = 0; s < (int)NSLOT_MAX; s++) n += slots[s].used;
  return n;
}
int store_free() { return store_slots() - store_used(); }

int store_list(int *out, int max) {
  int n = 0;
  if (!ok) return 0;
  for (int s = 0; s < (int)NSLOT_MAX; s++) if (slots[s].used && n < max) out[n++] = s;
  // newest first (insertion sort by id, small n)
  for (int i = 1; i < n; i++) {
    int v = out[i], j = i;
    while (j > 0 && slots[out[j - 1]].h.id < slots[v].h.id) { out[j] = out[j - 1]; j--; }
    out[j] = v;
  }
  return n;
}

const CapHdr *store_hdr(int s) {
  if (!ok || s < 0 || s >= (int)NSLOT_MAX || !slots[s].used) return nullptr;
  return &slots[s].h;
}

uint32_t store_max_id() {
  uint32_t m = 0;
  if (ok) for (int s = 0; s < (int)NSLOT_MAX; s++) if (slots[s].used && slots[s].h.id > m) m = slots[s].h.id;
  return m;
}

int store_find(uint32_t id) {
  if (ok) for (int s = 0; s < (int)NSLOT_MAX; s++) if (slots[s].used && slots[s].h.id == id) return s;
  return -1;
}

int store_save(CapHdr &h, uint32_t n, uint16_t (*sample)(uint32_t)) {
  if (!ok) return -1;
  uint32_t data_bytes = (n + 1) / 2 * 3;
  if (SLOT_DATA_OFF + data_bytes > SLOT_BYTES) return -1;
  // Next free slot after the newest capture, so wear spreads round the store.
  int newest = -1;
  for (int s = 0; s < (int)NSLOT_MAX; s++)
    if (slots[s].used && (newest < 0 || slots[s].h.id > slots[newest].h.id)) newest = s;
  int slot = -1;
  for (int i = 1; i <= (int)NSLOT_MAX; i++) {
    int s = (newest + i + (int)NSLOT_MAX) % (int)NSLOT_MAX;
    if (!slots[s].used) { slot = s; break; }
  }
  if (slot < 0) return -1;

  uint32_t base = slot_off(slot);
  flash_erase(base, SLOT_BYTES);

  // Pack and program the data, one sector-sized buffer at a time.
  uint32_t crc = 0, err = 0, written = 0, fill = 0;
  for (uint32_t k = 0; k < n; k += 2) {
    uint16_t a = sample(k);
    uint16_t b = (k + 1 < n) ? sample(k + 1) : 0;
    err += (a >> 15) + ((k + 1 < n) ? (b >> 15) : 0);
    a &= 0x0FFF; b &= 0x0FFF;
    uint8_t t[3] = {(uint8_t)a, (uint8_t)((a >> 8) | (b << 4)), (uint8_t)(b >> 4)};
    for (int j = 0; j < 3; j++) {
      pagebuf[fill++] = t[j];
      if (fill == sizeof pagebuf) {
        crc = crc32_update(crc, pagebuf, fill);
        flash_prog(base + SLOT_DATA_OFF + written, pagebuf, fill);
        written += fill; fill = 0;
      }
    }
  }
  if (fill) {
    crc = crc32_update(crc, pagebuf, fill);
    uint32_t padded = (fill + FLASH_PAGE_SIZE - 1) / FLASH_PAGE_SIZE * FLASH_PAGE_SIZE;
    memset(pagebuf + fill, 0xFF, padded - fill);
    flash_prog(base + SLOT_DATA_OFF + written, pagebuf, padded);
    written += fill;
  }

  h.magic = CAP_MAGIC;
  h.version = 4;
  h.hdr_bytes = sizeof(CapHdr);
  h.packing = 1;
  h.data_bytes = data_bytes;
  h.data_crc = crc;
  h.err_samples = err > 0xFFFF ? 0xFFFF : (uint16_t)err;
  h.hdr_crc = hdr_crc(h);
  memset(pagebuf, 0xFF, FLASH_PAGE_SIZE);
  memcpy(pagebuf, &h, sizeof h);
  flash_prog(base, pagebuf, FLASH_PAGE_SIZE);   // commit

  scan_slot(slot);
  return slots[slot].used ? slot : -1;
}

bool store_delete(int s) {
  if (!ok || s < 0 || s >= (int)NSLOT_MAX || !slots[s].used) return false;
  flash_erase(slot_off(s), FLASH_SECTOR_SIZE);  // wipes the header, frees the slot
  scan_slot(s);
  return !slots[s].used;
}

void store_read(int s, uint32_t k0, uint32_t n, uint16_t *dst) {
  const uint8_t *d = slot_ptr(s) + SLOT_DATA_OFF;
  for (uint32_t i = 0; i < n; i++) {
    uint32_t k = k0 + i;
    const uint8_t *p = d + (k >> 1) * 3;
    dst[i] = (k & 1) ? (uint16_t)((p[1] >> 4) | (p[2] << 4)) : (uint16_t)(p[0] | ((p[1] & 0x0F) << 8));
  }
}

bool store_check_crc(int s) {
  const CapHdr *h = store_hdr(s);
  if (!h) return false;
  return crc32_update(0, slot_ptr(s) + SLOT_DATA_OFF, h->data_bytes) == h->data_crc;
}

// ---------------- FRAM settings ----------------
#define FRAM_ADDR     0x50
#define FRAM_WP_PIN   15
#define SETTINGS_AT   0x1F00          // near the top of the 8 KB part, clear of DMX config
#define SETTINGS_MAGIC 0x32464342u    // "BCF2"

static bool fram_ok = false;

bool fram_present() { return fram_ok; }

static bool fram_read(uint16_t addr, uint8_t *dst, size_t n) {
  Wire.beginTransmission(FRAM_ADDR);
  Wire.write(addr >> 8); Wire.write(addr & 0xFF);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(FRAM_ADDR, n) != n) return false;
  for (size_t i = 0; i < n; i++) dst[i] = Wire.read();
  return true;
}

static bool fram_write(uint16_t addr, const uint8_t *src, size_t n) {
  digitalWrite(FRAM_WP_PIN, LOW);             // R47 holds WP high otherwise
  Wire.beginTransmission(FRAM_ADDR);
  Wire.write(addr >> 8); Wire.write(addr & 0xFF);
  Wire.write(src, n);
  bool r = Wire.endTransmission() == 0;
  digitalWrite(FRAM_WP_PIN, HIGH);
  return r;
}

void settings_defaults(Settings &s) {
  memset(&s, 0, sizeof s);
  s.magic = SETTINGS_MAGIC;
  s.n_ch = 2;
  s.tag = 1;
  s.arm_psi_x10 = 50;
  s.trig_psi_x10 = 10;
  for (int c = 0; c < 4; c++) {
    s.zero[c] = 410;          // 0.1 x 4095: 0.5 V of a 5 V-scale sensor
    s.cpp[c] = 32.76f;        // 0.8 x 4095 counts over 100 psi
  }
  s.next_id = 1;
  s.session = 0;
}

bool settings_load(Settings &s) {
  pinMode(FRAM_WP_PIN, OUTPUT);
  digitalWrite(FRAM_WP_PIN, HIGH);
  settings_defaults(s);
  Settings t;
  fram_ok = fram_read(SETTINGS_AT, (uint8_t *)&t, sizeof t);
  if (!fram_ok) return false;
  if (t.magic != SETTINGS_MAGIC || t.crc != crc32_update(0, (uint8_t *)&t, offsetof(Settings, crc))) return false;
  if (t.n_ch < 1 || t.n_ch > 3) t.n_ch = 2;
  for (int c = 0; c < 4; c++) if (!(t.cpp[c] > 20.0f && t.cpp[c] < 50.0f)) t.cpp[c] = 32.76f;
  s = t;
  return true;
}

bool settings_save(Settings &s) {
  s.magic = SETTINGS_MAGIC;
  s.crc = crc32_update(0, (uint8_t *)&s, offsetof(Settings, crc));
  return fram_ok && fram_write(SETTINGS_AT, (const uint8_t *)&s, sizeof s);
}
