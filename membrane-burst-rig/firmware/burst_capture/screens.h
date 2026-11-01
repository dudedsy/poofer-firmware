// screens.h - draws each UI screen into a Gfx framebuffer from a plain view struct.
// Pure functions, no hardware access, so tools/render_screens can build them on a PC.
#pragma once
#include "gfx.h"

#define MAX_CH 3

// Formats psi with one decimal into buf (at least 8 bytes). Avoids printf float support.
void fmt_psi(char *buf, float psi);

struct MenuView {
  const char *title;          // line 0, left
  const char *title_right;    // line 0, right-justified (live pressure); may be null
  const char *items[24];
  int n_items;
  int sel;                    // selected item
  bool editing;               // selected item's value is being edited
};
void draw_menu(Gfx &g, const MenuView &v);

enum CollectState { CS_WAIT, CS_ARMED, CS_PENDING, CS_PROMPT, CS_SAVING };

struct PressureView {
  int n_ch;
  float psi[MAX_CH];
  bool valid;                 // false while the ADC is stopped
};

struct CollectView {
  PressureView p;
  CollectState state;
  float arm_psi;              // < 0 means auto-arm off
  float peak_psi;             // peak since armed
  int free_slots;
  int tag;
  bool manual_armed;
  bool storage_ok;
};
void draw_collect(Gfx &g, const CollectView &v);

struct PromptView {
  unsigned id;
  int tag;
  int n_ch;
  bool forced;
  float p0_psi[MAX_CH];       // pre-burst pressure
  float end_psi[MAX_CH];      // settled pressure at end of capture
  unsigned err_samples;
  int chip_temp_c;
  int trig_ch;                // -1 = forced
  float trig_drop_psi;
  int free_slots;             // slots free before this capture is stored
  bool storage_ok;
};
void draw_prompt(Gfx &g, const PromptView &v);

struct LiveView {
  PressureView p;
  unsigned sent;              // captures sent to the host since entering live mode
  float trig_psi;
  bool host;                  // USB host has the port open
  bool pending;
};
void draw_live(Gfx &g, const LiveView &v);

// Up to 8 lines of plain text; line 0 is drawn inverted as a title bar.
void draw_message(Gfx &g, const char *const lines[], int n);
