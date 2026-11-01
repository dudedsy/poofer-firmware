// burst_capture.ino
// Membrane burst rig: fast pressure-transient capture on the DMX base board
// (socketed RP2040-Zero) with the UI lid board, or on a bare RP2040-Zero.
//
// Board: "Waveshare RP2040 Zero" in the arduino-pico core (Earle Philhower),
// Tools > Flash Size: "2MB (no FS)". Captures are stored in raw flash slots above
// the sketch (see store.h); a filesystem option would overlap them.
//
// Modes (chosen from the OLED menu with UP/DN/SET; hold SET to return to the menu)
//   Collect  standalone. Live pressure on the OLED. Arms automatically when pressure
//            rises past the auto-arm threshold, or manually with SET. When the drop
//            trigger fires it captures PRE_MS + POST_MS, then asks whether to keep
//            the capture in flash (SET) or drop it (DN). UP forces a capture.
//   Live     the original laptop workflow: 1 kHz slow stream and binary captures
//            straight to the USB host. The trigger is always armed.
//   Dump     ADC stopped, waiting for the host to list/read/delete stored captures.
// Without the UI board (bare RP2040-Zero, no I2C pull-ups) it boots straight into
// Live mode, which matches the previous firmware's behaviour.
//
// ADC: round robin over n_ch inputs from GP26 (1..3, set in the menu) at SPS_TOTAL,
// DMA into a ring buffer, processed in 1 ms means for display and triggering.
// Trigger: any sensor's 1 ms mean drops more than the trigger threshold below its
// maximum over the previous HIST_MS.
//
// Serial protocol (USB CDC, text lines unless noted)
//   host -> device:  t        force a capture now (Live, Collect)
//                    ?        print the JSON header line
//                    d<N>     set the drop trigger to N counts
//                    N<k>     set channel count (1..3)
//                    ML / MD / MM   switch to Live / Dump / Menu
//                    L        list stored captures          (not in Collect)
//                    R<id>    send stored capture <id>      (not in Collect)
//                    X<id>    delete stored capture <id>    (not in Collect)
//   device -> host:  {...}    JSON header (one line, starts with '{')
//                    S,ms,m0,m1,...   slow-stream 1 ms means (counts), Live mode
//                    J,{...}  JSON record: "t" is entry | list_end | meta | deleted | error
//                    #text    info messages
//                    C,num,frames,n_ch,trig_frame,fs_ch,forced\n
//                        then frames*n_ch*2 bytes of little-endian uint16, channel-
//                        interleaved, then "\nE\n". Bits 0..11 = ADC counts, bit 15 =
//                        ADC error flag (live captures only; stored captures carry the
//                        error count in their J meta record instead).

#include <Arduino.h>
#include <Wire.h>
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "gfx.h"
#include "screens.h"
#include "ui.h"
#include "store.h"

#define FW_VERSION  "burst_capture 3.0"

// ---------------- configuration ----------------
#define RAIL_CH     (-1)        // channel index carrying a 5 V rail divider, or -1
#define SPS_TOTAL   300000UL    // total ADC rate across all channels (max ~500k)
#define PRE_MS      20          // capture length before the trigger point
#define POST_MS     80          // capture length after the trigger point
#define HIST_MS     20          // look-back window for the drop trigger
#define HOLDOFF_MS  300         // re-arm delay after a capture / restart
#define TRIG_HOLD_MS 10         // drop must persist this many ms (rejects glitches); < PRE_MS

// Sensor scaling for the display and thresholds (0-100 psi, 10%..90% of supply,
// ratiometric to the 3V3 that also feeds ADC_VREF).
#define RANGE_PSI      100.0f
#define COUNTS_PER_PSI (0.8f * 4095.0f / RANGE_PSI)   // 32.76, nominal; the menu's
                                                       // Calibrate step stores per-channel values

#define PRESSURIZED_PSI 2.0f    // LED2 and display-wake threshold
#define BLANK_AFTER_MS  (5UL * 60UL * 1000UL)

// ---------------- derived sizes ----------------
#define SPMS        (SPS_TOTAL / 1000UL)       // samples per ms, all channels (300)
#define BLOCK_MS    10
#define BLOCK_SAMP  (SPMS * BLOCK_MS)          // 3000 samples = 6000 bytes
#define NBLK        26
#define RING_SAMP   (BLOCK_SAMP * NBLK)        // 78000 samples = 156000 bytes
#define RING_MS     (BLOCK_MS * NBLK)          // 260 ms
#define CAP_SAMP    ((PRE_MS + POST_MS) * SPMS)

#if (SPS_TOTAL % 1000UL) != 0 || (SPMS % 6) != 0
#error "SPS_TOTAL/1000 must be a multiple of 1, 2 and 3 so frames stay aligned"
#endif
#if (PRE_MS + POST_MS + 3 * BLOCK_MS + HIST_MS + 100) >= RING_MS
#error "capture window plus 100 ms of loop slack does not fit in the ring buffer"
#endif
static_assert(SLOT_DATA_OFF + (CAP_SAMP + 1) / 2 * 3 <= SLOT_BYTES, "capture does not fit a flash slot");

static uint16_t ring[RING_SAMP] __attribute__((aligned(4)));

static int dma_a, dma_b;
static volatile uint32_t blocks_done = 0;   // blocks completed since (re)start
static uint32_t blocks_processed = 0;
static bool     acq_running = false;
static uint32_t epoch_ms = 0;               // ms offset carried across restarts (slow stream)

// ---------------- runtime state ----------------
enum Mode { MODE_MENU, MODE_COLLECT, MODE_LIVE, MODE_DUMP };
static Mode     mode = MODE_MENU;
static Settings cfg;
static int      n_ch = 2;                   // active while acquiring
static bool     have_ui = false;

// trigger state
static float    hist[MAX_CH][HIST_MS];
static uint32_t hist_n = 0;
static uint32_t arm_at_ms = HOLDOFF_MS;
static bool     pending = false;
static bool     pending_forced = false;
static int      trig_ch = -1;               // channel that tripped, -1 = forced
static float    trig_drop = 0;              // its drop below baseline at the first ms, counts
static uint8_t  drop_run[MAX_CH];           // consecutive ms each channel has been below baseline
static float    drop_first[MAX_CH];         // drop at the first ms of that run, counts
static float    drop_base[MAX_CH];          // baseline frozen at the first ms of that run
static uint32_t trig_ms = 0;
static uint32_t live_sent = 0;

// pressure display
static float    disp_counts[MAX_CH];
static bool     disp_valid = false;
static float    acc[MAX_CH];
static uint32_t acc_n = 0;
static float    last_ms_counts[MAX_CH];
static uint32_t last_busy_ms = 0;           // last time the rig was pressurized / armed

// collect state
static CollectState cstate = CS_WAIT;
static bool     manual_armed = false;
static bool     autoarm_blocked = false;    // until pressure falls below half the arm level
static uint32_t above_ms = 0, below_ms = 0;
static float    peak_psi = 0;

// capture being held for the keep/drop prompt
static uint32_t cap_start = 0;              // absolute sample index since restart
static PromptView prompt;
static CapHdr   cap_hdr;

// ---------------- helpers ----------------
static inline float to_psi(int c, float counts) { return (counts - cfg.zero[c]) / cfg.cpp[c]; }
static inline uint16_t trig_counts(int c = 0) { return (uint16_t)(cfg.trig_psi_x10 / 10.0f * cfg.cpp[c] + 0.5f); }
static inline float arm_psi() { return cfg.arm_psi_x10 < 0 ? -1.0f : cfg.arm_psi_x10 / 10.0f; }
static inline uint32_t fpms() { return SPMS / n_ch; }
static inline bool host() { return (bool)Serial; }

static void info(const char *s) { if (host()) { Serial.print('#'); Serial.println(s); } }

static float max_psi_now() {
  float m = -1e9f;
  for (int c = 0; c < n_ch; c++) if (c != RAIL_CH) { float p = to_psi(c, last_ms_counts[c]); if (p > m) m = p; }
  return m;
}

// ---------------- DMA ----------------
static inline uint16_t *block_ptr(uint32_t b) { return &ring[(b % NBLK) * BLOCK_SAMP]; }

static void dma_irq_handler() {
  uint32_t ints = dma_hw->ints1;
  uint32_t mask = (1u << dma_a) | (1u << dma_b);
  if (!(ints & mask)) return;                // shared IRQ, not ours
  // Exactly one of our channels finishes per block. The next block is already being
  // written by the other channel (chained), so point the finished channel at the
  // block after that. TRANS_COUNT reloads automatically on retrigger.
  for (int k = 0; k < 2; k++) {
    int ch = k ? dma_b : dma_a;
    if (ints & (1u << ch)) {
      dma_hw->ints1 = 1u << ch;              // acknowledge
      uint32_t done = blocks_done + 1;
      blocks_done = done;
      dma_channel_set_write_addr(ch, block_ptr(done + 1), false);
    }
  }
}

static void config_channel(int ch, int chain_to, uint16_t *dst) {
  dma_channel_config c = dma_channel_get_default_config(ch);
  channel_config_set_transfer_data_size(&c, DMA_SIZE_16);
  channel_config_set_read_increment(&c, false);
  channel_config_set_write_increment(&c, true);
  channel_config_set_dreq(&c, DREQ_ADC);
  channel_config_set_chain_to(&c, chain_to);
  dma_channel_configure(ch, &c, dst, &adc_hw->fifo, BLOCK_SAMP, false);
}

static void start_acq() {
  n_ch = cfg.n_ch;
  for (int g = 26; g < 26 + n_ch; g++) adc_gpio_init(g);
  adc_run(false);
  adc_fifo_drain();
  adc_select_input(0);                       // round robin starts at channel 0
  adc_set_round_robin((1u << n_ch) - 1);
  adc_fifo_setup(true, true, 1, true, false);  // FIFO on, DREQ on, thresh 1, err bit, no shift
  adc_set_clkdiv(48000000.0f / SPS_TOTAL - 1.0f);

  blocks_done = 0;
  blocks_processed = 0;
  hist_n = 0;
  acc_n = 0;
  arm_at_ms = HOLDOFF_MS;
  config_channel(dma_a, dma_b, block_ptr(0));
  config_channel(dma_b, dma_a, block_ptr(1));
  dma_hw->ints1 = (1u << dma_a) | (1u << dma_b);
  dma_channel_start(dma_a);
  adc_run(true);
  acq_running = true;
}

static void stop_acq() {
  if (!acq_running) return;
  adc_run(false);
  // Disable both channels before aborting so an abort cannot fire the chain (RP2040-E13).
  hw_clear_bits(&dma_hw->ch[dma_a].al1_ctrl, DMA_CH0_CTRL_TRIG_EN_BITS);
  hw_clear_bits(&dma_hw->ch[dma_b].al1_ctrl, DMA_CH0_CTRL_TRIG_EN_BITS);
  dma_channel_abort(dma_a);
  dma_channel_abort(dma_b);
  dma_hw->ints1 = (1u << dma_a) | (1u << dma_b);
  adc_fifo_drain();
  epoch_ms += blocks_processed * BLOCK_MS;
  acq_running = false;
  disp_valid = false;
}

// ---------------- output ----------------
static void print_header() {
  static const char *MODE_NAMES[] = {"menu", "collect", "live", "dump"};
  Serial.print("{\"fw\":\"" FW_VERSION "\",\"sps_total\":");
  Serial.print(SPS_TOTAL);
  Serial.print(",\"n_ch\":");      Serial.print(cfg.n_ch);
  Serial.print(",\"fs_ch\":");     Serial.print(SPS_TOTAL / cfg.n_ch);
  Serial.print(",\"rail_ch\":");   Serial.print(RAIL_CH);
  Serial.print(",\"pre_ms\":");    Serial.print(PRE_MS);
  Serial.print(",\"post_ms\":");   Serial.print(POST_MS);
  Serial.print(",\"hist_ms\":");   Serial.print(HIST_MS);
  Serial.print(",\"ring_ms\":");   Serial.print(RING_MS);
  Serial.print(",\"trig_drop\":"); Serial.print(trig_counts());
  Serial.print(",\"adc_bits\":12,\"gpio\":[");
  for (int c = 0; c < cfg.n_ch; c++) { if (c) Serial.print(','); Serial.print(26 + c); }
  Serial.print("],\"mode\":\"");   Serial.print(MODE_NAMES[mode]);
  Serial.print("\",\"ui\":");      Serial.print(have_ui ? "true" : "false");
  Serial.print(",\"fram\":");      Serial.print(fram_present() ? "true" : "false");
  Serial.print(",\"store_slots\":"); Serial.print(store_slots());
  Serial.print(",\"store_used\":");  Serial.print(store_used());
  Serial.println("}");
}

static void send_c_line(uint32_t num, uint32_t frames, int nch, uint32_t trig_frame, uint32_t fs, bool forced) {
  Serial.print("C,"); Serial.print(num);
  Serial.print(','); Serial.print(frames);
  Serial.print(','); Serial.print(nch);
  Serial.print(','); Serial.print(trig_frame);
  Serial.print(','); Serial.print(fs);
  Serial.print(','); Serial.println(forced ? 1 : 0);
}

static void send_ring_capture(uint32_t num, bool forced) {
  uint32_t frames = CAP_SAMP / n_ch;
  send_c_line(num, frames, n_ch, (uint32_t)PRE_MS * fpms(), SPS_TOTAL / n_ch, forced);
  uint32_t idx = cap_start % RING_SAMP;
  uint32_t left = CAP_SAMP;
  while (left) {
    uint32_t run = RING_SAMP - idx;
    if (run > left) run = left;
    const uint8_t *p = (const uint8_t *)&ring[idx];
    uint32_t bytes = run * 2;
    while (bytes) {
      uint32_t chunk = bytes > 4096 ? 4096 : bytes;
      Serial.write(p, chunk);
      p += chunk; bytes -= chunk;
    }
    left -= run;
    idx = 0;
  }
  Serial.print("\nE\n");
  Serial.flush();
}

// Prints [x0,x1,...]. With zero/cpp given, converts counts to psi.
static void print_array(const float *v, int n, const uint16_t *zero = nullptr, const float *cpp = nullptr,
                        int digits = 1) {
  Serial.print('[');
  for (int c = 0; c < n; c++) {
    if (c) Serial.print(',');
    Serial.print(zero ? (v[c] - zero[c]) / cpp[c] : v[c], zero ? 2 : digits);
  }
  Serial.print(']');
}

static inline float hdr_psi(const CapHdr &h, int c) { return (h.pre_mean[c] - h.zero[c]) / h.cpp[c]; }

static void send_entry(const CapHdr &h, const char *type) {
  Serial.print("J,{\"t\":\"");       Serial.print(type);
  Serial.print("\",\"id\":");        Serial.print(h.id);
  Serial.print(",\"session\":");     Serial.print(h.session);
  Serial.print(",\"tag\":");         Serial.print(h.tag);
  Serial.print(",\"n_ch\":");        Serial.print(h.n_ch);
  Serial.print(",\"frames\":");      Serial.print(h.frames);
  Serial.print(",\"trig_frame\":");  Serial.print(h.trig_frame);
  Serial.print(",\"fs_ch\":");       Serial.print(h.fs_ch);
  Serial.print(",\"sps_total\":");   Serial.print(h.sps_total);
  Serial.print(",\"pre_ms\":");      Serial.print(h.pre_ms);
  Serial.print(",\"post_ms\":");     Serial.print(h.post_ms);
  Serial.print(",\"trig_drop\":");   Serial.print(h.trig_drop);
  Serial.print(",\"forced\":");      Serial.print(h.forced);
  Serial.print(",\"err_samples\":"); Serial.print(h.err_samples);
  Serial.print(",\"uptime_ms\":");   Serial.print(h.uptime_ms);
  Serial.print(",\"chip_temp_c\":"); Serial.print(h.chip_temp_x10 / 10.0f, 1);
  if (h.version >= 4) {
    Serial.print(",\"trig_ch\":"); if (h.trig_ch == 0xFF) Serial.print("null"); else Serial.print(h.trig_ch);
    Serial.print(",\"trig_drop_psi\":"); Serial.print(h.trig_drop_x10 / 10.0f, 1);
  }
  Serial.print(",\"zero\":[");
  for (int c = 0; c < h.n_ch; c++) { if (c) Serial.print(','); Serial.print(h.zero[c]); }
  Serial.print("],\"cpp\":");        print_array(h.cpp, h.n_ch, nullptr, nullptr, 3);
  Serial.print(",\"pre_mean\":");    print_array(h.pre_mean, h.n_ch);
  Serial.print(",\"end_mean\":");    print_array(h.end_mean, h.n_ch);
  Serial.print(",\"p0_psi\":");      print_array(h.pre_mean, h.n_ch, h.zero, h.cpp);
  Serial.print(",\"data_bytes\":");  Serial.print(h.data_bytes);
  Serial.print(",\"data_crc\":");    Serial.print(h.data_crc);
}

static void cmd_list() {
  static int idx[64];
  int n = store_list(idx, 64);
  for (int i = n - 1; i >= 0; i--) {         // oldest first
    send_entry(*store_hdr(idx[i]), "entry");
    Serial.println('}');
  }
  Serial.print("J,{\"t\":\"list_end\",\"count\":"); Serial.print(n);
  Serial.print(",\"free\":"); Serial.print(store_free());
  Serial.print(",\"slots\":"); Serial.print(store_slots());
  Serial.println('}');
}

static void cmd_read(uint32_t id) {
  int s = store_find(id);
  if (s < 0) { Serial.print("J,{\"t\":\"error\",\"id\":"); Serial.print(id); Serial.println(",\"msg\":\"no such capture\"}"); return; }
  const CapHdr &h = *store_hdr(s);
  bool crc_ok = store_check_crc(s);
  send_entry(h, "meta");
  Serial.print(",\"crc_ok\":"); Serial.print(crc_ok ? "true" : "false");
  Serial.println('}');
  send_c_line(h.id, h.frames, h.n_ch, h.trig_frame, h.fs_ch, h.forced);
  static uint16_t buf[1024];
  uint32_t total = h.frames * h.n_ch;
  for (uint32_t k = 0; k < total; k += 1024) {
    uint32_t n = total - k < 1024 ? total - k : 1024;
    store_read(s, k, n, buf);
    Serial.write((const uint8_t *)buf, n * 2);
  }
  Serial.print("\nE\n");
  Serial.flush();
}

static void cmd_delete(uint32_t id) {
  int s = store_find(id);
  bool ok = s >= 0 && store_delete(s);
  Serial.print("J,{\"t\":\"deleted\",\"id\":"); Serial.print(id);
  Serial.print(",\"ok\":"); Serial.print(ok ? "true" : "false");
  Serial.println('}');
}

// ---------------- mode changes ----------------
static void enter_mode(Mode m);

static void force_capture() {
  if (!acq_running || pending) return;
  uint32_t now_ms = blocks_processed * BLOCK_MS;
  if (now_ms >= PRE_MS) { pending = true; pending_forced = true; trig_ms = now_ms; trig_ch = -1; trig_drop = 0; if (mode == MODE_COLLECT) cstate = CS_PENDING; }
  else info("too soon after restart, try again");
}

// ---------------- per-ms processing ----------------
static void process_ms(const uint16_t *s, uint32_t ms_index) {
  float mean[MAX_CH];
  uint32_t sum[MAX_CH] = {0};
  uint32_t f = fpms();
  for (uint32_t i = 0; i < SPMS; i += n_ch)
    for (int c = 0; c < n_ch; c++) sum[c] += s[i + c] & 0x0FFF;
  for (int c = 0; c < n_ch; c++) { mean[c] = (float)sum[c] / f; last_ms_counts[c] = mean[c]; }

  // display average, 100 ms
  for (int c = 0; c < n_ch; c++) acc[c] = (acc_n ? acc[c] : 0) + mean[c];
  if (++acc_n >= 100) {
    for (int c = 0; c < n_ch; c++) disp_counts[c] = acc[c] / acc_n;
    acc_n = 0;
    disp_valid = true;
  }

  float pmax = max_psi_now();
  if (pmax > PRESSURIZED_PSI) last_busy_ms = millis();

  // slow stream (Live only; skipped if the host is not keeping up)
  if (mode == MODE_LIVE && !pending && host() && Serial.availableForWrite() > 64) {
    Serial.print("S,"); Serial.print(epoch_ms + ms_index);
    for (int c = 0; c < n_ch; c++) { Serial.print(','); Serial.print(mean[c], 1); }
    Serial.println();
  }

  if (mode == MODE_COLLECT) {
    float ap = arm_psi();
    if (cstate == CS_WAIT) {
      if (autoarm_blocked && (ap < 0 || pmax < ap * 0.5f)) autoarm_blocked = false;
      if (ap >= 0 && !autoarm_blocked && pmax >= ap) {
        if (++above_ms >= 100) { cstate = CS_ARMED; manual_armed = false; peak_psi = 0; below_ms = 0; }
      } else above_ms = 0;
    } else if (cstate == CS_ARMED) {
      if (pmax > peak_psi) peak_psi = pmax;
      // auto-armed and vented slowly without a burst: go back to waiting
      if (!manual_armed && ap >= 0 && pmax < ap * 0.5f) {
        if (++below_ms >= 500) { cstate = CS_WAIT; above_ms = 0; info("vented without a trigger, disarmed"); }
      } else below_ms = 0;
    }
  }

  // drop trigger: always in Live, only when armed in Collect
  bool trig_on = (mode == MODE_LIVE) || (mode == MODE_COLLECT && cstate == CS_ARMED);
  // A channel trips when its 1 ms mean drops more than the threshold below the mean
  // of its previous 20 ms and stays below that (frozen) baseline minus the threshold
  // for TRIG_HOLD_MS ms in a row. A mean baseline barely moves for a brief upward
  // blip, and the hold rejects anything that recovers; a burst drops and stays down.
  // The trigger point is the first ms of the run, so the pre-burst window is not
  // shifted by the hold.
  if (trig_on && !pending && hist_n >= HIST_MS && ms_index >= arm_at_ms) {
    for (int c = 0; c < n_ch; c++) {
      if (c == RAIL_CH) continue;
      float base;
      if (drop_run[c]) base = drop_base[c];
      else {
        base = 0;
        for (int k = 0; k < HIST_MS; k++) base += hist[c][k];
        base /= HIST_MS;
      }
      float drop = base - mean[c];
      if (drop > trig_counts(c)) {
        if (drop_run[c] == 0) { drop_first[c] = drop; drop_base[c] = base; }
        if (++drop_run[c] >= TRIG_HOLD_MS) {
          pending = true; pending_forced = false;
          trig_ms = ms_index - (TRIG_HOLD_MS - 1);
          trig_ch = c; trig_drop = drop_first[c];
          if (mode == MODE_COLLECT) cstate = CS_PENDING;
          break;
        }
      } else drop_run[c] = 0;
    }
  } else {
    for (int c = 0; c < MAX_CH; c++) drop_run[c] = 0;
  }
  for (int c = 0; c < n_ch; c++) hist[c][ms_index % HIST_MS] = mean[c];
  hist_n++;
}

static void acq_service() {
  if (!acq_running) return;
  uint32_t done = blocks_done;
  if (done - blocks_processed >= NBLK - 2) {
    // Fell too far behind. Skip ahead; history is now stale.
    blocks_processed = done - 1;
    hist_n = 0;
    if (pending) {
      pending = false;
      if (mode == MODE_COLLECT) cstate = CS_WAIT;
      info("overrun, capture lost");
    } else info("overrun, skipped blocks");
  }
  while (blocks_processed < done) {
    const uint16_t *b = block_ptr(blocks_processed);
    for (uint32_t m = 0; m < BLOCK_MS; m++)
      process_ms(b + m * SPMS, blocks_processed * BLOCK_MS + m);
    blocks_processed++;
  }
}

static uint16_t ring_sample(uint32_t k) { return ring[(cap_start + k) % RING_SAMP]; }

static void ring_means(uint32_t f0, uint32_t f1, float *out) {
  for (int c = 0; c < n_ch; c++) {
    uint32_t sum = 0;
    for (uint32_t f = f0; f < f1; f++) sum += ring_sample(f * n_ch + c) & 0x0FFF;
    out[c] = (float)sum / (f1 - f0);
  }
}

// Called once POST_MS of data exist after the trigger. Stops the ADC.
static bool finish_capture() {
  uint32_t done = blocks_done;
  stop_acq();
  pending = false;
  cap_start = (trig_ms - PRE_MS) * SPMS;
  // Blocks d and d+1 are being (re)written, so only blocks >= d+2-NBLK are intact.
  if ((int32_t)(cap_start / BLOCK_SAMP) < (int32_t)done + 2 - NBLK) {
    info("capture overwritten before it could be read (loop stalled)");
    return false;
  }
  return true;
}

// RP2040 die temperature (datasheet formula: 27 C at 0.706 V, -1.721 mV/C). Reads the
// chip, not the air: a rough ambient figure for the notes. Call only with the ADC
// stopped. The sensor is switched off again afterwards because its current flows
// through the Zero's R6 and would shift the ADC reference.
static float read_chip_temp() {
  adc_set_round_robin(0);
  adc_fifo_setup(false, false, 0, false, false);
  adc_set_temp_sensor_enabled(true);
  adc_select_input(4);
  delayMicroseconds(100);
  uint32_t sum = 0;
  for (int i = 0; i < 64; i++) sum += adc_read() & 0x0FFF;
  adc_set_temp_sensor_enabled(false);
  adc_fifo_drain();
  float v = sum / 64.0f * 3.3f / 4096.0f;
  return 27.0f - (v - 0.706f) / 0.001721f;
}

static void handle_live_capture() {
  if (finish_capture() && host()) {
    Serial.print("J,{\"t\":\"live_meta\",\"num\":"); Serial.print(live_sent + 1);
    Serial.print(",\"chip_temp_c\":"); Serial.print(read_chip_temp(), 1);
    Serial.print(",\"trig_ch\":"); if (trig_ch < 0) Serial.print("null"); else Serial.print(trig_ch);
    Serial.print(",\"trig_drop_psi\":"); Serial.print(trig_ch < 0 ? 0.0f : trig_drop / cfg.cpp[trig_ch], 2);
    Serial.println('}');
    send_ring_capture(++live_sent, pending_forced);
  }
  start_acq();
}

static void handle_collect_capture() {
  if (!finish_capture()) { cstate = CS_WAIT; start_acq(); return; }
  uint32_t f = fpms();
  uint32_t frames = CAP_SAMP / n_ch;
  memset(&cap_hdr, 0, sizeof cap_hdr);
  cap_hdr.id = cfg.next_id;
  cap_hdr.session = cfg.session;
  cap_hdr.tag = cfg.tag;
  cap_hdr.n_ch = n_ch;
  cap_hdr.forced = pending_forced;
  cap_hdr.trig_ch = trig_ch < 0 ? 0xFF : (uint8_t)trig_ch;
  {
    float dp = trig_ch < 0 ? 0 : trig_drop / cfg.cpp[trig_ch] * 10.0f + 0.5f;
    cap_hdr.trig_drop_x10 = dp > 255 ? 255 : (uint8_t)dp;
  }
  cap_hdr.frames = frames;
  cap_hdr.trig_frame = PRE_MS * f;
  cap_hdr.fs_ch = SPS_TOTAL / n_ch;
  cap_hdr.sps_total = SPS_TOTAL;
  cap_hdr.pre_ms = PRE_MS;
  cap_hdr.post_ms = POST_MS;
  cap_hdr.trig_drop = trig_counts();
  cap_hdr.uptime_ms = millis();
  float tc = read_chip_temp();
  cap_hdr.chip_temp_x10 = (int16_t)(tc * 10.0f + (tc >= 0 ? 0.5f : -0.5f));
  for (int c = 0; c < 4; c++) { cap_hdr.zero[c] = cfg.zero[c]; cap_hdr.cpp[c] = cfg.cpp[c]; }
  // pre-burst level: up to 5 ms before the trigger; settled level: last 20 ms
  ring_means(0, (PRE_MS - 5) * f, cap_hdr.pre_mean);
  ring_means(frames - 20 * f, frames, cap_hdr.end_mean);
  uint32_t err = 0;
  for (uint32_t k = 0; k < CAP_SAMP; k++) err += ring_sample(k) >> 15;

  memset(&prompt, 0, sizeof prompt);
  prompt.id = cap_hdr.id;
  prompt.tag = cfg.tag;
  prompt.n_ch = n_ch;
  prompt.forced = pending_forced;
  for (int c = 0; c < n_ch; c++) {
    prompt.p0_psi[c] = to_psi(c, cap_hdr.pre_mean[c]);
    prompt.end_psi[c] = to_psi(c, cap_hdr.end_mean[c]);
  }
  prompt.err_samples = err;
  prompt.chip_temp_c = (int)(tc + (tc >= 0 ? 0.5f : -0.5f));
  prompt.trig_ch = trig_ch;
  prompt.trig_drop_psi = trig_ch < 0 ? 0 : trig_drop / cfg.cpp[trig_ch];
  prompt.free_slots = store_free();
  prompt.storage_ok = store_ok();
  cstate = CS_PROMPT;
  if (host()) {
    Serial.print("#captured id="); Serial.print(cap_hdr.id);
    Serial.print(" p0_psi="); Serial.print(prompt.p0_psi[0], 1);
    Serial.println(" (keep/drop on the board)");
  }
}

// ---------------- serial ----------------
static void handle_line(char *buf) {
  uint32_t v = (uint32_t)atol(buf + 1);
  switch (buf[0]) {
    case 'd': {
      int n = atoi(buf + 1);
      if (n > 0 && n < 4096) {
        cfg.trig_psi_x10 = (uint16_t)(n / cfg.cpp[0] * 10.0f + 0.5f);
        if (cfg.trig_psi_x10 < 1) cfg.trig_psi_x10 = 1;
        settings_save(cfg);
        Serial.print("#trig_drop="); Serial.println(trig_counts());
      }
      return;
    }
    case 'N':
      if (v >= 1 && v <= MAX_CH && mode != MODE_COLLECT) {
        cfg.n_ch = v; settings_save(cfg);
        if (acq_running) { stop_acq(); start_acq(); }
        print_header();
      }
      return;
    case 'M':
      if (buf[1] == 'L') enter_mode(MODE_LIVE);
      else if (buf[1] == 'D') enter_mode(MODE_DUMP);
      else if (buf[1] == 'M') enter_mode(MODE_MENU);
      print_header();
      return;
    case 'L': case 'R': case 'X': {
      if (mode == MODE_COLLECT) { Serial.println("J,{\"t\":\"error\",\"msg\":\"board is in Collect mode; hold SET for the menu\"}"); return; }
      bool was = acq_running;
      stop_acq();
      if (buf[0] == 'L') cmd_list();
      else if (buf[0] == 'R') cmd_read(v);
      else cmd_delete(v);
      if (was) start_acq();
      return;
    }
  }
}

static void handle_serial() {
  static char buf[24];
  static uint8_t n = 0;
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\r') continue;
    if (ch != '\n' && n < sizeof(buf) - 1) buf[n++] = ch;
    // single-character commands act immediately (no Enter needed)
    if (n == 1 && (buf[0] == 't' || buf[0] == '?')) {
      if (buf[0] == '?') print_header();
      else force_capture();
      n = 0;
      continue;
    }
    if (ch == '\n') {
      buf[n] = 0;
      if (n) handle_line(buf);
      n = 0;
    }
  }
}

// ---------------- menu ----------------
enum MenuPage { MP_MAIN, MP_CAPS, MP_DEL_ONE, MP_DEL_ALL, MP_ZERO, MP_CAL, MP_CAL_RESULT, MP_MSG };
static MenuPage mpage = MP_MAIN;
static int  msel = 0, csel = 0;
static bool editing = false;
static int  del_slot = -1;
static char msg_lines[8][22];
static int  msg_n = 0;
static uint32_t msg_until = 0;
static bool zeroing = false;
static uint32_t zero_t0 = 0;

enum { MI_COLLECT, MI_LIVE, MI_DUMP, MI_CH, MI_TAG, MI_ARM, MI_TRIG, MI_ZERO, MI_CAL, MI_CAPS, MI_DELALL, MI_N };

// Calibrate: the user sets the rig to a pressure, reads the gauge, dials that value
// in with UP/DN and records a point with SET. Hold SET to fit counts = zero + cpp*psi
// per channel by least squares.
#define CAL_MAX 10
static int   cal_n = 0, cal_gauge = 0;
static float cal_psi[CAL_MAX], cal_cnt[CAL_MAX][MAX_CH];
static bool  cal_measuring = false;
static uint32_t cal_t0 = 0;
static float cal_zero[MAX_CH], cal_cpp[MAX_CH], cal_err[MAX_CH];
static bool  cal_good = false;
static char  cal_why[22];

static void cal_fit() {
  cal_good = true;
  cal_why[0] = 0;
  float lo = cal_psi[0], hi = cal_psi[0];
  for (int i = 1; i < cal_n; i++) { if (cal_psi[i] < lo) lo = cal_psi[i]; if (cal_psi[i] > hi) hi = cal_psi[i]; }
  if (hi - lo < 10) { cal_good = false; snprintf(cal_why, sizeof cal_why, "need >=10 psi span"); }
  for (int c = 0; c < n_ch; c++) {
    double sx = 0, sy = 0, sxx = 0, sxy = 0;
    for (int i = 0; i < cal_n; i++) {
      sx += cal_psi[i]; sy += cal_cnt[i][c]; sxx += (double)cal_psi[i] * cal_psi[i]; sxy += (double)cal_psi[i] * cal_cnt[i][c];
    }
    double d = cal_n * sxx - sx * sx;
    double m = d > 0 ? (cal_n * sxy - sx * sy) / d : 0;
    double b = (sy - m * sx) / cal_n;
    cal_cpp[c] = (float)m;
    cal_zero[c] = (float)b;
    float e = 0;
    for (int i = 0; i < cal_n; i++) {
      float r = m > 0 ? fabsf((float)((cal_cnt[i][c] - b) / m) - cal_psi[i]) : 999.0f;
      if (r > e) e = r;
    }
    cal_err[c] = e;
    if (cal_good && (m < 0.75 * COUNTS_PER_PSI || m > 1.25 * COUNTS_PER_PSI)) {
      cal_good = false; snprintf(cal_why, sizeof cal_why, "%c gain off >25%%", '0' + c);
    }
    if (cal_good && fabs(b - 409.5) > 150) {
      cal_good = false; snprintf(cal_why, sizeof cal_why, "%c zero off", '0' + c);
    }
  }
}

static void show_msg(int ms, const char *a, const char *b = "", const char *c = "", const char *d = "") {
  const char *src[4] = {a, b, c, d};
  msg_n = 4;
  for (int i = 0; i < 4; i++) { strncpy(msg_lines[i], src[i], 21); msg_lines[i][21] = 0; }
  msg_until = millis() + ms;
  mpage = MP_MSG;
}

static void edit_value(int item, int dir) {
  switch (item) {
    case MI_CH:  cfg.n_ch = (uint8_t)((cfg.n_ch - 1 + dir + MAX_CH) % MAX_CH + 1); break;   // wraps 3 -> 1
    case MI_TAG: cfg.tag = (uint8_t)((cfg.tag + 16 + dir) % 16); break;
    case MI_ARM: {
      int v = cfg.arm_psi_x10 < 0 ? 0 : cfg.arm_psi_x10 / 10;   // 0 = off
      v = constrain(v + dir, 0, 90);
      cfg.arm_psi_x10 = v == 0 ? -1 : v * 10;
      break;
    }
    case MI_TRIG: cfg.trig_psi_x10 = constrain((int)cfg.trig_psi_x10 + dir * 5, 5, 100); break;
  }
}

static void menu_event(uint8_t ev) {
  switch (mpage) {
    case MP_MAIN:
      if (editing) {
        if (ev & EV_UP) edit_value(msel, +1);
        if (ev & EV_DN) edit_value(msel, -1);
        if (ev & (EV_SET | EV_SET_LONG)) {
          editing = false;
          settings_save(cfg);
          if (msel == MI_CH && cfg.n_ch != n_ch) { stop_acq(); start_acq(); }
        }
        return;
      }
      if (ev & EV_UP) msel = (msel + MI_N - 1) % MI_N;
      if (ev & EV_DN) msel = (msel + 1) % MI_N;
      if (ev & EV_SET) {
        switch (msel) {
          case MI_COLLECT: enter_mode(MODE_COLLECT); break;
          case MI_LIVE:    enter_mode(MODE_LIVE); break;
          case MI_DUMP:    enter_mode(MODE_DUMP); break;
          case MI_ZERO:    mpage = MP_ZERO; zeroing = false; break;
          case MI_CAL:     mpage = MP_CAL; cal_n = 0; cal_gauge = 0; cal_measuring = false; break;
          case MI_CAPS:    mpage = MP_CAPS; csel = 0; break;
          case MI_DELALL:  if (store_used()) mpage = MP_DEL_ALL; else show_msg(1500, "NOTHING STORED"); break;
          default:         editing = true; break;
        }
      }
      return;
    case MP_CAPS: {
      int n = store_used() + 1;                 // + "back"
      if (ev & EV_UP) csel = (csel + n - 1) % n;
      if (ev & EV_DN) csel = (csel + 1) % n;
      if (ev & EV_SET_LONG) { mpage = MP_MAIN; return; }
      if (ev & EV_SET) {
        if (csel == 0) { mpage = MP_MAIN; return; }
        static int idx[64];
        int m = store_list(idx, 64);
        if (csel - 1 < m) { del_slot = idx[csel - 1]; mpage = MP_DEL_ONE; }
      }
      return;
    }
    case MP_DEL_ONE:
      if (ev & EV_SET) {
        const CapHdr *h = store_hdr(del_slot);
        char a[22]; snprintf(a, sizeof a, "#%04lu deleted", h ? (unsigned long)h->id : 0UL);
        bool ok = store_delete(del_slot);
        show_msg(1200, ok ? a : "DELETE FAILED");
        csel = 0;
      } else if (ev & (EV_DN | EV_SET_LONG)) mpage = MP_CAPS;
      return;
    case MP_DEL_ALL:
      if (ev & EV_SET_LONG) {
        static int idx[64];
        int m = store_list(idx, 64), bad = 0;
        for (int i = 0; i < m; i++) bad += !store_delete(idx[i]);
        char a[22]; snprintf(a, sizeof a, "deleted %d", m - bad);
        show_msg(1500, bad ? "SOME DELETES FAILED" : "ALL DELETED", a);
      } else if (ev & EV_DN) mpage = MP_MAIN;
      return;
    case MP_ZERO:
      if (zeroing) return;
      if (ev & EV_SET) { zeroing = true; zero_t0 = millis(); }
      else if (ev & (EV_DN | EV_SET_LONG)) mpage = MP_MAIN;
      return;
    case MP_CAL:
      if (cal_measuring) return;
      if (ev & EV_UP) cal_gauge = constrain(cal_gauge + 1, 0, 100);
      if (ev & EV_DN) cal_gauge = constrain(cal_gauge - 1, 0, 100);
      if ((ev & EV_SET) && cal_n < CAL_MAX) { cal_measuring = true; cal_t0 = millis(); }
      if (ev & EV_SET_LONG) {
        if (cal_n >= 2) { cal_fit(); mpage = MP_CAL_RESULT; }
        else show_msg(1500, "CAL CANCELLED", "needs 2+ points");
      }
      return;
    case MP_CAL_RESULT:
      if ((ev & EV_SET) && cal_good) {
        for (int c = 0; c < n_ch; c++) { cfg.zero[c] = (uint16_t)(cal_zero[c] + 0.5f); cfg.cpp[c] = cal_cpp[c]; }
        show_msg(2000, settings_save(cfg) ? "CAL SAVED" : "CAL SET (no FRAM)");
      } else if (ev & (EV_DN | EV_SET_LONG)) show_msg(1500, "CAL DISCARDED");
      return;
    case MP_MSG:
      if (ev) msg_until = 0;
      return;
  }
}

static void menu_tick() {
  if (mpage == MP_MSG && (int32_t)(millis() - msg_until) >= 0)
    mpage = (msel == MI_CAPS) ? MP_CAPS : MP_MAIN;
  if (mpage == MP_CAL && cal_measuring && millis() - cal_t0 >= 600) {
    cal_measuring = false;
    if (disp_valid) {                            // 100 ms average, all after SET
      cal_psi[cal_n] = (float)cal_gauge;
      for (int c = 0; c < n_ch; c++) cal_cnt[cal_n][c] = disp_counts[c];
      cal_n++;
    }
  }
  if (mpage == MP_ZERO && zeroing && millis() - zero_t0 >= 600) {
    zeroing = false;
    // disp_counts is a 100 ms average; take it after 600 ms so it is all post-SET
    bool good = disp_valid;
    char a[22] = "", b[22] = "";
    for (int c = 0; c < n_ch && good; c++) if (fabsf(disp_counts[c] - 409.5f) > 120.0f) good = false;
    int pos = 0;
    for (int c = 0; c < n_ch; c++) pos += snprintf(b + pos, sizeof b - pos, "%c:%d ", '0' + c, (int)(disp_counts[c] + 0.5f));
    if (good) {
      for (int c = 0; c < n_ch; c++) cfg.zero[c] = (uint16_t)(disp_counts[c] + 0.5f);
      bool saved = settings_save(cfg);
      snprintf(a, sizeof a, saved ? "ZEROED" : "ZEROED (no FRAM)");
      show_msg(2000, a, b);
    } else {
      show_msg(3000, "NOT ZEROED", b, "expected ~410 counts", "rig vented?");
    }
  }
}

static void draw_menu_page(Gfx &g) {
  static char items[MI_N][22];
  static char right[22];
  char p[8];
  int pos = 0;
  right[0] = 0;
  if (disp_valid)
    for (int c = 0; c < n_ch; c++) {
      fmt_psi(p, to_psi(c, disp_counts[c]));
      pos += snprintf(right + pos, sizeof right - pos, "%s%c:%s", c ? " " : "", '0' + c, p);
    }

  switch (mpage) {
    case MP_MAIN: {
      MenuView v{};
      v.title = "MENU";
      v.title_right = right;
      snprintf(items[MI_COLLECT], 22, "Collect");
      snprintf(items[MI_LIVE], 22, "Live (USB)");
      snprintf(items[MI_DUMP], 22, "Dump (USB)");
      snprintf(items[MI_CH], 22, "Channels        %d", cfg.n_ch);
      snprintf(items[MI_TAG], 22, "Tag            %2d", cfg.tag);
      if (cfg.arm_psi_x10 < 0) snprintf(items[MI_ARM], 22, "Auto arm      off");
      else snprintf(items[MI_ARM], 22, "Auto arm   %2d psi", cfg.arm_psi_x10 / 10);
      fmt_psi(p, cfg.trig_psi_x10 / 10.0f);
      snprintf(items[MI_TRIG], 22, "Drop trig %4s psi", p);
      snprintf(items[MI_ZERO], 22, "Zero sensors");
      snprintf(items[MI_CAL], 22, "Calibrate (gauge)");
      if (store_ok()) snprintf(items[MI_CAPS], 22, "Captures   %2d/%d", store_used(), store_slots());
      else snprintf(items[MI_CAPS], 22, "Captures   (none)");
      snprintf(items[MI_DELALL], 22, "Delete all");
      for (int i = 0; i < MI_N; i++) v.items[i] = items[i];
      v.n_items = MI_N; v.sel = msel; v.editing = editing;
      draw_menu(g, v);
      break;
    }
    case MP_CAPS: {
      static char citems[24][32];
      static int idx[64];
      int m = store_list(idx, 64);
      MenuView v{};
      v.title = "CAPTURES (SET=del)";
      v.items[0] = "< back";
      int shown = m < 23 ? m : 23;              // MenuView holds 24 rows
      for (int i = 0; i < shown; i++) {
        const CapHdr *h = store_hdr(idx[i]);
        fmt_psi(p, hdr_psi(*h, 0));
        snprintf(citems[i], sizeof citems[i], "#%04lu T%-2d %5spsi%s", (unsigned long)h->id, h->tag, p, h->forced ? "F" : "");
        v.items[i + 1] = citems[i];
      }
      v.n_items = shown + 1;
      if (csel >= v.n_items) csel = v.n_items - 1;
      v.sel = csel;
      draw_menu(g, v);
      break;
    }
    case MP_DEL_ONE: {
      const CapHdr *h = store_hdr(del_slot);
      char a[22] = "", b[22] = "";
      if (h) {
        snprintf(a, sizeof a, "DELETE #%04lu ?", (unsigned long)h->id);
        fmt_psi(p, hdr_psi(*h, 0));
        snprintf(b, sizeof b, "T%d  %s psi  s%lu", h->tag, p, (unsigned long)h->session);
      }
      const char *L[] = {a, "", b, "", "", "", "SET delete", "DN cancel"};
      draw_message(g, L, 8);
      break;
    }
    case MP_DEL_ALL: {
      char b[22];
      snprintf(b, sizeof b, "%d stored captures", store_used());
      const char *L[] = {"DELETE ALL?", "", b, "dump them first!", "", "", "HOLD SET to delete", "DN cancel"};
      draw_message(g, L, 8);
      break;
    }
    case MP_ZERO: {
      char b[22] = "";
      int pos2 = 0;
      if (disp_valid)
        for (int c = 0; c < n_ch; c++) pos2 += snprintf(b + pos2, sizeof b - pos2, "%c:%d ", '0' + c, (int)(disp_counts[c] + 0.5f));
      const char *L[] = {"ZERO SENSORS", "vent the rig fully", "", "counts now:", b, "",
                         zeroing ? "measuring..." : "SET zero", zeroing ? "" : "DN cancel"};
      draw_message(g, L, 8);
      break;
    }
    case MP_CAL: {
      char a[22], b[22], c[22] = "", d[22] = "";
      snprintf(a, sizeof a, "CALIBRATE    pts %d", cal_n);
      snprintf(b, sizeof b, "gauge reads %3d psi", cal_gauge);
      int pos2 = snprintf(c, sizeof c, "now");
      if (disp_valid)
        for (int ch = 0; ch < n_ch; ch++) pos2 += snprintf(c + pos2, sizeof c - pos2, " %c:%d", '0' + ch, (int)(disp_counts[ch] + 0.5f));
      if (cal_n) {
        int pos3 = snprintf(d, sizeof d, "last %d:", (int)cal_psi[cal_n - 1]);
        for (int ch = 0; ch < n_ch; ch++) pos3 += snprintf(d + pos3, sizeof d - pos3, " %d", (int)(cal_cnt[cal_n - 1][ch] + 0.5f));
      }
      const char *L[] = {a, "UP/DN: set to gauge", b, c, d, "",
                         cal_measuring ? "measuring..." : (cal_n < CAL_MAX ? "SET record point" : "max points"),
                         "hold SET: fit/cancel"};
      draw_message(g, L, 8);
      g.invert(72, 16, 24, 8);                   // highlight the gauge value
      break;
    }
    case MP_CAL_RESULT: {
      static char r[MAX_CH][32];
      char a[22];
      snprintf(a, sizeof a, "CAL FIT  %d points", cal_n);
      const char *L[8] = {a, "", "", "", "", "", "", ""};
      for (int ch = 0; ch < n_ch; ch++) {
        char e[8]; fmt_psi(e, cal_err[ch]);
        int g100 = (int)(cal_cpp[ch] * 100.0f + 0.5f);   // no float printf on the target
        snprintf(r[ch], sizeof r[ch], "%c %2d.%02d/psi z%d e%s", '0' + ch, g100 / 100, g100 % 100,
                 (int)(cal_zero[ch] + 0.5f), e);
        L[1 + ch] = r[ch];
      }
      L[4] = "(counts/psi, zero,";
      L[5] = " worst residual psi)";
      L[6] = cal_good ? "SET save" : cal_why;
      L[7] = "DN discard";
      draw_message(g, L, 8);
      break;
    }
    case MP_MSG: {
      const char *L[8];
      for (int i = 0; i < msg_n; i++) L[i] = msg_lines[i];
      draw_message(g, L, msg_n);
      break;
    }
  }
}

// ---------------- collect ----------------
static void collect_event(uint8_t ev) {
  if (ev & EV_SET_LONG) {
    if (cstate == CS_PROMPT) return;           // decide keep/drop first
    enter_mode(MODE_MENU);
    return;
  }
  switch (cstate) {
    case CS_WAIT:
      if (ev & EV_SET) { cstate = CS_ARMED; manual_armed = true; peak_psi = 0; }
      if (ev & EV_UP) force_capture();
      break;
    case CS_ARMED:
      if (ev & EV_SET) { cstate = CS_WAIT; autoarm_blocked = true; above_ms = 0; }
      if (ev & EV_UP) force_capture();
      break;
    case CS_PROMPT:
      if ((ev & EV_SET) && store_ok() && store_free() > 0) {
        cstate = CS_SAVING;
        Gfx g; CollectView v{}; v.p.n_ch = n_ch; v.state = CS_SAVING; v.tag = cfg.tag;
        v.free_slots = store_free(); v.storage_ok = true;
        draw_collect(g, v); ui_show(g);
        ui_set_led(0, LED_MAGENTA);
        int slot = store_save(cap_hdr, CAP_SAMP, ring_sample);
        if (slot >= 0) {
          cfg.next_id = cap_hdr.id + 1;
          settings_save(cfg);
          if (host()) { Serial.print("#saved id="); Serial.println(cap_hdr.id); }
        } else info("save failed");
        cstate = CS_WAIT; autoarm_blocked = true; above_ms = 0;
        start_acq();
      } else if (ev & EV_DN) {
        info("capture dropped");
        cstate = CS_WAIT; autoarm_blocked = true; above_ms = 0;
        start_acq();
      }
      break;
    default:
      break;
  }
}

// ---------------- modes ----------------
static void enter_mode(Mode m) {
  stop_acq();
  pending = false;
  editing = false;
  mode = m;
  switch (m) {
    case MODE_MENU:
      mpage = MP_MAIN;
      start_acq();
      break;
    case MODE_COLLECT:
      cfg.session++;
      settings_save(cfg);
      cstate = CS_WAIT; manual_armed = false; autoarm_blocked = true; above_ms = 0;
      start_acq();
      break;
    case MODE_LIVE:
      live_sent = 0;
      start_acq();
      break;
    case MODE_DUMP:
      break;
  }
}

static uint32_t last_draw = 0;
static bool redraw_now = true;

static void draw() {
  Gfx g;
  PressureView pv{};
  pv.n_ch = n_ch;
  pv.valid = disp_valid && acq_running;
  for (int c = 0; c < n_ch; c++) pv.psi[c] = to_psi(c, disp_counts[c]);

  switch (mode) {
    case MODE_MENU:
      draw_menu_page(g);
      break;
    case MODE_COLLECT:
      if (cstate == CS_PROMPT) {
        draw_prompt(g, prompt);
      } else {
        CollectView v{};
        v.p = pv; v.state = cstate; v.arm_psi = arm_psi();
        v.peak_psi = peak_psi;
        v.free_slots = store_free(); v.tag = cfg.tag;
        v.manual_armed = manual_armed; v.storage_ok = store_ok();
        draw_collect(g, v);
      }
      break;
    case MODE_LIVE: {
      LiveView v{};
      v.p = pv; v.sent = live_sent; v.trig_psi = cfg.trig_psi_x10 / 10.0f;
      v.host = host(); v.pending = pending;
      draw_live(g, v);
      break;
    }
    case MODE_DUMP: {
      char a[22], b[22];
      snprintf(a, sizeof a, "stored %d  free %d", store_used(), store_free());
      snprintf(b, sizeof b, "host: %s", host() ? "connected" : "none");
      const char *L[] = {"DUMP (USB)", a, "", b, "", "run burst_logger", "  --dump", "hold SET: menu"};
      draw_message(g, L, 8);
      break;
    }
  }
  ui_show(g);
}

static void update_leds() {
  LedColor s = LED_OFF;
  switch (mode) {
    case MODE_MENU: s = LED_GREEN; break;
    case MODE_LIVE: s = LED_CYAN; break;
    case MODE_DUMP: s = LED_BLUE; break;
    case MODE_COLLECT:
      s = (cstate == CS_WAIT) ? LED_GREEN : (cstate == CS_PROMPT) ? LED_BLUE :
          (cstate == CS_SAVING) ? LED_MAGENTA : LED_RED;
      break;
  }
  ui_set_led(0, s);
  bool pressurized = acq_running && disp_valid && max_psi_now() > PRESSURIZED_PSI;
  ui_set_led(1, pressurized ? LED_AMBER : LED_OFF);
}

// ---------------- setup / loop ----------------
void setup() {
  Serial.begin(115200);

  // The base board has 2.2 kOhm I2C pull-ups; a bare RP2040-Zero has none and the
  // pads' pull-downs win. Only start I2C if the bus idles high.
  pinMode(12, INPUT_PULLDOWN);
  pinMode(13, INPUT_PULLDOWN);
  delay(2);
  bool i2c_bus = digitalRead(12) && digitalRead(13);
  if (i2c_bus) {
    Wire.setSDA(12);
    Wire.setSCL(13);
    Wire.begin();
    Wire.setClock(400000);
    Wire.setTimeout(25, true);
  }
  if (i2c_bus) settings_load(cfg);           // falls back to defaults if FRAM is blank
  else settings_defaults(cfg);
  // Let +5V settle before starting the panel: the RP2040 runs as soon as its 3.3 V
  // is up, which on a slow-rising 24 V supply can be while +5V (and so the panel's
  // 4.0 V VBAT) is still low. Holding SET later also restarts the panel.
  if (i2c_bus) delay(300);
  have_ui = i2c_bus && ui_begin();
  store_begin();
  uint32_t mx = store_max_id();
  if (cfg.next_id <= mx) cfg.next_id = mx + 1;

  adc_init();
  dma_a = dma_claim_unused_channel(true);
  dma_b = dma_claim_unused_channel(true);
  irq_add_shared_handler(DMA_IRQ_1, dma_irq_handler, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
  dma_channel_set_irq1_enabled(dma_a, true);
  dma_channel_set_irq1_enabled(dma_b, true);
  irq_set_enabled(DMA_IRQ_1, true);

  enter_mode(have_ui ? MODE_MENU : MODE_LIVE);
  last_busy_ms = millis();
  if (have_ui && !store_ok()) show_msg(4000, "STORAGE DISABLED", "sketch overlaps", "flash store");
}

void loop() {
  handle_serial();
  acq_service();

  // finished captures
  if (pending && acq_running && blocks_processed * BLOCK_MS >= trig_ms + POST_MS) {
    if (mode == MODE_LIVE) handle_live_capture();
    else if (mode == MODE_COLLECT) handle_collect_capture();
    else { stop_acq(); pending = false; start_acq(); }
    redraw_now = true;
  }

  uint8_t ev = ui_buttons();
  if (ev && ui_blanked()) { ev = 0; redraw_now = true; }   // a press only wakes the panel
  if (ev & EV_SET_LONG) { ui_reset_display(); redraw_now = true; }   // any long SET also restarts the panel
  if (ev) {
    redraw_now = true;
    if (mode == MODE_MENU) menu_event(ev);
    else if (mode == MODE_COLLECT) collect_event(ev);
    else if (ev & EV_SET_LONG) enter_mode(MODE_MENU);
    else if (mode == MODE_LIVE && (ev & EV_UP)) force_capture();
  }
  if (mode == MODE_MENU) menu_tick();

  if (!have_ui) return;

  // Blank the panel when nothing is happening (OLED burn-in), wake on pressure.
  bool busy = (mode == MODE_COLLECT && cstate != CS_WAIT) || pending;
  if (busy) last_busy_ms = millis();
  uint32_t idle_from = max(ui_last_activity_ms(), last_busy_ms);
  if (!ui_blanked() && millis() - idle_from > BLANK_AFTER_MS) ui_blank();
  bool wake = ui_blanked() && millis() - last_busy_ms < 1000 && !pending;

  if (!ui_blanked() || redraw_now || wake) {
    if (redraw_now || wake || millis() - last_draw >= 150) {
      draw();
      last_draw = millis();
      redraw_now = false;
    }
  }
  ui_service();
  update_leds();
}
