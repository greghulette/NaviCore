#pragma once
// =============================================================================
//  navicore_hil.h — hardware-in-the-loop test hooks (NAVICORE_HIL_HOOKS builds ONLY)
//
//  NaviCore.ino includes this file only when the build defines NAVICORE_HIL_HOOKS
//  (--build-property compiler.cpp.extra_flags=-DNAVICORE_HIL_HOOKS=1, which the WCB
//  repo's tests/hil/hil/ncflash.py passes for build(..., hooks=True)). CI, the
//  released bins and the config tool's flasher never set it, and they must not:
//  these are fault injectors. A normal image answers "#L90" exactly as it answers
//  any other unknown code, which is how the harness tells the two apart — so the
//  "Valid:" list of that reply deliberately does not grow here.
//
//  What a hook build adds (WCB repo docs/hil_plan/NAVICORE.md INF9 b; the verbs and
//  the [WIRE] format are in docs/PROTOCOLS.md §2 and §3):
//    DBG_WIRE     SET_DEBUG_FLAGS bit 7: a "[WIRE] ..." hex line for every block
//                 handed to S3, S4, S5, Serial2 (the local Maestro bus) or the
//                 broadcast WCBStream, through the non-blocking vlogf()
//    #L90,<ms>    stall loop() once, for <ms> ms (0-60000), at the top of its next pass
//    #L91         cut /config.json to its first half, after copying the whole file to
//                 /config.json.hil — the present-but-unreadable file of the
//                 load-fallback path; #L91,R moves the copy back over /config.json
//    #L92         the next GET_CONFIG over USB or the WebSocket takes
//                 rcConfigToJSON()'s overflow branch
//    #L93         the next rcConfigSaveLFS() fails before it touches flash
//  Every armed state is RAM and clear at boot. #L91 is the exception by nature: it
//  edits flash, and its copy survives a restart on purpose, so a run that dies
//  between the fault and its restore can still put the user's config back.
//
//  Everything here runs on the loop task (Core 1): the CLI (USB, WebSocket and
//  relayed lines alike) is dispatched from loop(), and every port write it taps
//  is a loop-task write (ARCHITECTURE.md §8).
// =============================================================================
#ifndef NAVICORE_HIL_HOOKS
#error "navicore_hil.h belongs to NAVICORE_HIL_HOOKS builds only"
#endif

#include <Arduino.h>
#include <LittleFS.h>
#include <WCBStream.h>
#include "rc_config.h"   // RC_CFG_PATH, g_lfsReady, g_hilJsonOverflowNow, g_hilFailNextSave

// Defined in NaviCore.ino ahead of the include: g_dbgFlags and DBG_WIRE, the aux ports
// s3/s4/s5 and maestroBroadcast. vlogf() is defined further down that file.
static void vlogf(const char* fmt, ...);

namespace navihil {

// ── DBG_WIRE ──────────────────────────────────────────────────────────────────
// One line per 48 bytes of a block:  [WIRE] <port> <offset>/<length>: <hex>
//   port    S3 | S4 | S5 | Serial2 | WCBStream
//   hex     upper-case pairs, space-separated, e.g. [WIRE] S4 0/6: AA 04 04 05 70 2E
// A block is one write() or print() call on the port — a Maestro frame arrives as
// its 3-byte header and then its payload, an MP3 Trigger command as one byte at a
// time — the aux TX pump (auxTxPump) hands a line, and the device bytes auxDev() held
// behind one, over in one write per pass, logged as that block (the held bytes when
// they go out, not when a codec wrote them). So a reader joins the lines of a
// port in order; offset/length show when one of them was lost. 48 bytes keep a line
// (at most ~175 characters) inside vlogf()'s 192-byte buffer, and vlogf() drops a
// whole line rather than block when the USB TX ring is short.
constexpr size_t WIRE_PER_LINE = 48;

inline void wire(const char* port, const uint8_t* data, size_t len) {
  if (!(g_dbgFlags & DBG_WIRE) || len == 0) return;
  static const char digits[] = "0123456789ABCDEF";
  char hex[WIRE_PER_LINE * 3];
  for (size_t off = 0; off < len; off += WIRE_PER_LINE) {
    const size_t k = (len - off < WIRE_PER_LINE) ? len - off : WIRE_PER_LINE;
    for (size_t i = 0; i < k; i++) {
      hex[3 * i]     = digits[data[off + i] >> 4];
      hex[3 * i + 1] = digits[data[off + i] & 0x0F];
      hex[3 * i + 2] = ' ';
    }
    hex[3 * k - 1] = '\0';
    vlogf("[WIRE] %s %u/%u: %s\n", port, (unsigned)off, (unsigned)len, hex);
  }
}

// A pass-through Stream that logs what it forwards. Reads forward too, because the
// device codecs bound to a port (g_mp3, g_dfp) hold the Stream they were given.
// Logs only what the port accepted (write()'s return), which is what went out.
class WireTap : public Stream {
 public:
  explicit WireTap(const char* port) : _port(port) {}
  WireTap* over(Stream* s) { _s = s; return this; }
  using Print::write;
  size_t write(uint8_t b) override {
    const size_t n = _s->write(b);
    wire(_port, &b, n);
    return n;
  }
  size_t write(const uint8_t* buf, size_t len) override {
    const size_t n = _s->write(buf, len);
    wire(_port, buf, n);
    return n;
  }
  int  availableForWrite() override { return _s->availableForWrite(); }
  void flush() override             { _s->flush(); }
  int  available() override         { return _s->available(); }
  int  read() override              { return _s->read(); }
  int  peek() override              { return _s->peek(); }

 private:
  const char* _port;
  Stream*     _s = nullptr;
};

static WireTap tapS3("S3"), tapS4("S4"), tapS5("S5"), tapMaestro("Serial2"), tapStream("WCBStream");

// HIL_TAP(p): the tap over `p` when p is one of the five device ports, else p itself.
// nullptr stays nullptr, so a caller's own null check still means what it did. The
// ports are compared by identity, as bound at boot (s3/s4/s5 never move at run time:
// a boardType change needs a reboot).
inline Stream* tap(Stream* p) {
  if (!p) return p;
  if (p == s3) return tapS3.over(p);
  if (p == s4) return tapS4.over(p);
  if (p == s5) return tapS5.over(p);
  if (p == static_cast<Stream*>(&Serial2)) return tapMaestro.over(p);
  if (maestroBroadcast && p == static_cast<Stream*>(maestroBroadcast)) return tapStream.over(p);
  return p;
}

// ── #L90: stall loop() ────────────────────────────────────────────────────────
// delay(), so only the loop task stops: the WiFi task, the ESP-NOW callbacks and the
// UART/USB drivers keep running and queueing, which is what a test of a slow loop()
// wants to see (the SBUS backlog, mesh queues, heartbeats). The loop task has no task
// watchdog (setup() never subscribes it), so a long stall is safe; 60 s is the cap
// so a typo cannot wedge the board for good.
constexpr uint32_t STALL_MAX_MS = 60000;
static uint32_t stallMs = 0;

inline void loopStall() {
  if (!stallMs) return;
  const uint32_t ms = stallMs;
  stallMs = 0;
  const uint32_t t0 = millis();
  delay(ms);
  Serial.printf("[HIL] #L90: loop() resumed after %lu ms\n", (unsigned long)(millis() - t0));
}

// ── #L91: an unreadable /config.json, with the original kept ──────────────────
static const char* CFG_COPY = "/config.json.hil";

// Copy at most `limit` bytes of `from` into `to` (replaced) -> the size `to` has on
// flash afterwards when that is the byte count written, else -1. The size is read back
// after close(), as rcConfigSaveLFS() does: close() writes the last block and the core
// discards its error.
inline long copyFile(const char* from, const char* to, size_t limit) {
  File in = LittleFS.open(from, "r");
  if (!in) return -1;
  File out = LittleFS.open(to, "w");
  if (!out) { in.close(); return -1; }
  uint8_t buf[256];
  size_t done = 0;
  bool ok = true;
  while (done < limit) {
    const size_t want = (limit - done < sizeof(buf)) ? limit - done : sizeof(buf);
    const size_t got = in.read(buf, want);
    if (got == 0) break;
    if (out.write(buf, got) != got) { ok = false; break; }
    done += got;
  }
  in.close();
  out.close();
  File chk = LittleFS.open(to, "r");
  const size_t onFlash = chk ? chk.size() : 0;
  if (chk) chk.close();
  return (ok && onFlash == done) ? (long)done : -1;
}

// The whole file goes to CFG_COPY first and is checked; only then is /config.json cut
// to its first half, the torn write the load path must survive. A strict prefix of the
// saved JSON object never parses (IncompleteInput), so the next boot takes
// rcConfigLoadLFS()'s "parse failed — keeping file, using defaults this boot" branch.
// The running config is untouched until that restart, and any config save before it
// (SET_CONFIG, a mesh save) writes a whole file again.
inline void corruptConfig() {
  if (!g_lfsReady || !LittleFS.exists(RC_CFG_PATH)) {
    Serial.println("[HIL] #L91: no /config.json to corrupt — nothing changed");
    return;
  }
  File f = LittleFS.open(RC_CFG_PATH, "r");
  const size_t n = f ? f.size() : 0;
  if (f) f.close();
  if (n < 2) {
    Serial.printf("[HIL] #L91: /config.json is %u bytes — nothing to cut\n", (unsigned)n);
    return;
  }
  if (copyFile(RC_CFG_PATH, CFG_COPY, n) != (long)n) {
    LittleFS.remove(CFG_COPY);
    Serial.printf("[HIL] #L91: could not copy /config.json to %s — nothing changed\n", CFG_COPY);
    return;
  }
  const size_t half = n / 2;
  if (copyFile(CFG_COPY, RC_CFG_PATH, half) != (long)half) {
    Serial.printf("[HIL] #L91: cutting /config.json failed; the whole file is in %s (#L91,R restores it)\n",
                  CFG_COPY);
    return;
  }
  Serial.printf("[HIL] #L91: /config.json cut to %u of %u B, the whole file kept in %s (#L91,R restores it); "
                "restart to boot on defaults\n", (unsigned)half, (unsigned)n, CFG_COPY);
}

// #L91,R: the copy back over /config.json, atomically (LittleFS rename replaces the
// destination, as rcConfigSaveLFS() relies on). Takes effect at the next boot.
inline void restoreConfig() {
  if (!g_lfsReady || !LittleFS.exists(CFG_COPY)) {
    Serial.printf("[HIL] #L91,R: no %s to restore\n", CFG_COPY);
    return;
  }
  File f = LittleFS.open(CFG_COPY, "r");
  const size_t n = f ? f.size() : 0;
  if (f) f.close();
  if (!LittleFS.rename(CFG_COPY, RC_CFG_PATH)) {
    Serial.printf("[HIL] #L91,R: rename failed — %s kept\n", CFG_COPY);
    return;
  }
  Serial.printf("[HIL] #L91,R: /config.json restored from %s (%u bytes); restart to load it\n", CFG_COPY,
                (unsigned)n);
}

// ── #L92: the next GET_CONFIG overflows ───────────────────────────────────────
// Armed here, consumed by the GET_CONFIG handler (processInputLine), which hands it to
// its own rcConfigToJSON() call through g_hilJsonOverflowNow. Not a flag
// rcConfigToJSON() consumes by itself: a config save or a mesh GET_CONFIG in between
// would take the overflow instead of the GET_CONFIG the test sends.
static bool getConfigOverflowNext = false;

inline void armGetConfigOverflow() {
  if (!getConfigOverflowNext) return;
  getConfigOverflowNext = false;
  g_hilJsonOverflowNow = true;
}

// ── #L90-#L93 from execCliLine() ──────────────────────────────────────────────
// `line` is the whole command ("#L90,250", "#L91,R"); its argument follows the first ','.
inline void command(int fn, const String& line) {
  const int comma = line.indexOf(',');
  String arg = (comma > 0) ? line.substring(comma + 1) : String();
  arg.trim();
  switch (fn) {
    case 90: {
      long ms = arg.toInt();
      if (ms < 0) ms = 0;
      const bool capped = ms > (long)STALL_MAX_MS;
      if (capped) ms = STALL_MAX_MS;
      stallMs = (uint32_t)ms;
      Serial.printf("[HIL] #L90: loop() stalls %ld ms at its next pass%s\n", ms, capped ? " (capped)" : "");
      break;
    }
    case 91:
      if (arg.length() == 0)             corruptConfig();
      else if (arg.equalsIgnoreCase("R")) restoreConfig();
      else Serial.println("[HIL] #L91 usage: #L91 (cut /config.json, keeping a copy) or #L91,R (restore the copy)");
      break;
    case 92:
      getConfigOverflowNext = true;
      Serial.println("[HIL] #L92: the next GET_CONFIG reports an overflow");
      break;
    case 93:
      g_hilFailNextSave = true;
      Serial.println("[HIL] #L93: the next config save fails");
      break;
  }
}

}  // namespace navihil

#define HIL_TAP(p) navihil::tap(p)
