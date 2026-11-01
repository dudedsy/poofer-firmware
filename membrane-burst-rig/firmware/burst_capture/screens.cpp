// screens.cpp - see screens.h
#include "screens.h"
#include <stdio.h>
#include <string.h>

void fmt_psi(char *buf, float psi) {
  int t = (int)(psi * 10.0f + (psi >= 0 ? 0.5f : -0.5f));
  bool neg = t < 0;
  if (neg) t = -t;
  if (t > 9999) t = 9999;
  snprintf(buf, 8, "%s%d.%d", neg ? "-" : "", t / 10, t % 10);
}

static const char CH_NAME[MAX_CH] = {'0', '1', '2'};   // ADC input numbers

// Pressure readout in the band y = 9..41. One channel big, two medium, three small.
static void draw_pressures(Gfx &g, const PressureView &p) {
  char v[8], s[24];
  if (!p.valid) {
    g.text(10, 13, "--.-", 3);
    return;
  }
  if (p.n_ch <= 1) {
    fmt_psi(v, p.psi[0]);
    g.text_right(104, 13, v, 3);
    g.text(110, 29, "psi");
  } else if (p.n_ch == 2) {
    for (int c = 0; c < 2; c++) {
      int y = 12 + c * 17;
      char nm[2] = {CH_NAME[c], 0};
      g.text(0, y, nm, 2);
      fmt_psi(v, p.psi[c]);
      g.text_right(104, y, v, 2);
      g.text(110, y + 8, "psi");
    }
  } else {
    for (int c = 0; c < p.n_ch && c < MAX_CH; c++) {
      fmt_psi(v, p.psi[c]);
      snprintf(s, sizeof s, "%c %7s psi", CH_NAME[c], v);
      g.text(0, 12 + c * 10, s);
    }
  }
}

void draw_menu(Gfx &g, const MenuView &v) {
  g.clear();
  int rw = v.title_right ? Gfx::width(v.title_right) : 0;
  if (v.title && Gfx::width(v.title) + 6 + rw <= GFX_W) g.line(0, v.title);   // drop the title if no room
  if (v.title_right) g.text_right(127, 0, v.title_right);
  g.hline(0, 127, 8);
  const int rows = 6;                               // lines 2..7 hold items
  int top = v.sel - rows / 2;
  if (top > v.n_items - rows) top = v.n_items - rows;
  if (top < 0) top = 0;
  for (int r = 0; r < rows && top + r < v.n_items; r++) {
    int i = top + r;
    int y = 10 + r * 9;
    g.text(6, y + 1, v.items[i]);
    if (i == v.sel) {
      g.invert(0, y, 128, 9);
      if (v.editing) {                              // edit marker at the right edge
        g.fill(120, y + 1, 8, 7, false);
        g.text(121, y + 1, "*");
        g.invert(120, y, 8, 9);
      }
    }
  }
  if (top > 0) g.text(122, 10, "^");
  if (top + rows < v.n_items) g.text(122, 56, "v");
}

void draw_collect(Gfx &g, const CollectView &v) {
  char s[32], a[8];
  g.clear();
  snprintf(s, sizeof s, "COLLECT T%d", v.tag);
  g.line(0, s);
  if (v.storage_ok) snprintf(s, sizeof s, "%d free", v.free_slots);
  else snprintf(s, sizeof s, "NO STORE");
  g.text_right(127, 0, s);
  g.hline(0, 127, 9);
  draw_pressures(g, v.p);

  switch (v.state) {
    case CS_WAIT:
      if (v.arm_psi >= 0) { fmt_psi(a, v.arm_psi); snprintf(s, sizeof s, "WAIT  arms >%s psi", a); }
      else snprintf(s, sizeof s, "WAIT  auto-arm off");
      g.line(6, s);
      g.line(7, "SET arm   UP force");
      break;
    case CS_ARMED:
      fmt_psi(a, v.peak_psi);
      snprintf(s, sizeof s, "ARMED%s  peak %s", v.manual_armed ? "(m)" : "", a);
      g.line(6, s);
      g.invert(0, 47, 128, 9);
      g.line(7, "SET disarm UP force");
      break;
    case CS_PENDING:
      g.line(6, "TRIGGERED...");
      g.invert(0, 47, 128, 9);
      break;
    case CS_SAVING:
      g.line(6, "SAVING...");
      break;
    default:
      break;
  }
}

void draw_prompt(Gfx &g, const PromptView &v) {
  char s[32], a[8], b[8];
  g.clear();
  snprintf(s, sizeof s, "CAPTURE #%04u T%d%s", v.id, v.tag, v.forced ? " F" : "");
  g.line(0, s);
  g.invert(0, 0, 128, 8);
  for (int c = 0; c < v.n_ch && c < MAX_CH; c++) {
    fmt_psi(a, v.p0_psi[c]);
    fmt_psi(b, v.end_psi[c]);
    snprintf(s, sizeof s, "%c %5s -> %4s psi", CH_NAME[c], a, b);
    g.line(1 + c, s);
  }
  snprintf(s, sizeof s, "adc err %u%s  chip %dC", v.err_samples, v.err_samples ? "!" : "", v.chip_temp_c);
  g.line(4, s);
  if (v.trig_ch >= 0) {
    fmt_psi(a, v.trig_drop_psi);
    snprintf(s, sizeof s, "trig ch%d drop %s psi", v.trig_ch, a);
    g.line(6, s);
  }
  if (!v.storage_ok) g.line(5, "STORAGE UNAVAILABLE");
  else if (v.free_slots <= 0) g.line(5, "STORAGE FULL");
  else {
    snprintf(s, sizeof s, "room for %d more", v.free_slots);
    g.line(5, s);
  }
  if (v.storage_ok && v.free_slots > 0) g.line(7, "SET keep   DN drop");
  else g.line(7, "DN drop");
  g.invert(0, 55, 128, 9);
}

void draw_live(Gfx &g, const LiveView &v) {
  char s[32], a[8];
  g.clear();
  g.line(0, "LIVE (USB)");
  g.text_right(127, 0, v.host ? "host ok" : "no host");
  g.hline(0, 127, 9);
  draw_pressures(g, v.p);
  fmt_psi(a, v.trig_psi);
  snprintf(s, sizeof s, "sent %u  trig %s", v.sent, a);
  g.line(6, s);
  g.line(7, v.pending ? "TRIGGERED..." : "UP force  hold SET");
}

void draw_message(Gfx &g, const char *const lines[], int n) {
  g.clear();
  for (int i = 0; i < n && i < 8; i++) g.line(i, lines[i]);
  if (n > 0) g.invert(0, 0, 128, 8);
}
