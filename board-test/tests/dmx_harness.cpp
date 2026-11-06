// Host harness for libraries/PooferCommon/src/poofer_dmx_rx.h.
// stdin: one "<time_us> <UARTDR word, hex>" per line. stdout: final stats as key=value.
#include <cstdio>
#include "../../libraries/PooferCommon/src/poofer_dmx_rx.h"

static DmxRx d;

int main() {
  dmx_reset(&d);
  unsigned long t, dr;
  while (scanf("%lu %lx", &t, &dr) == 2) dmx_feed(&d, (uint32_t)dr, (uint32_t)t);
  const DmxStats& s = d.st;
  printf("bytes=%u breaks=%u fe=%u oe=%u pe=%u frames=%u errFrames=%u altSc=%u lastAltSc=%u stray=%u presync=%u\n",
         s.bytes, s.breaks, s.fe, s.oe, s.pe, s.frames, s.errFrames, s.altSc, s.lastAltSc, s.stray, s.presync);
  printf("slotsMin=%u slotsMax=%u perMinUs=%u perMaxUs=%u perN=%u losses=%u\n",
         s.slotsMin, s.slotsMax, s.perMinUs, s.perMaxUs, s.perN, s.losses);
  printf("patOk=%u patPad=%u patTear=%u patBad=%u patNone=%u seqChanges=%u lastSeq=%u\n",
         s.patOk, s.patPad, s.patTear, s.patBad, s.patNone, s.seqChanges, s.lastSeq);
  printf("badSlot=%u badGot=%u badExp=%u badSeq=%u lastN=%u\n",
         s.haveBad ? s.firstBad.firstBad : 0, s.firstBad.got, s.firstBad.exp, s.firstBad.seq, d.lastN);
  return 0;
}
