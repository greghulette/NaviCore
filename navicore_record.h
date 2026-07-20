#pragma once
#include <Arduino.h>
#include <FS.h>
#include <stdlib.h>       // qsort — chronological re-sort after a timeline-editor upload
#include "rc_config.h"    // RcAction, actionToJson/actionFromJson (shared with the button/knob editors)

// ─────────────────────────────────────────────────────────────────────────────
//  NaviCore record / replay — PHASE 1 (in-PSRAM clip, no persistence yet)
// ─────────────────────────────────────────────────────────────────────────────
// Captures every dispatched droid action as a timestamped event, into a single
// in-PSRAM clip, and replays it by re-dispatching with the original timing.
// See docs/RECORD_REPLAY_DESIGN.md. This module is DISPATCH-AGNOSTIC: NaviCore's
// dispatch helpers are `static`, so we reach them through callbacks registered
// from NaviCore.ino (recBegin).
//
// CONCURRENCY (the load-bearing part): the action tap (rcExecuteActionNow) runs
// on BOTH cores — Core 0 for a remote ESP-NOW TRIGGER (onWCBCommand ->
// rcTelemetry::handle -> rcDispatch, no defer) and Core 1 for local input. So
// captures go through a FreeRTOS queue (non-blocking, drop-if-full, small POD,
// noinline enqueue — same discipline as the OTA/remote-CLI queues that fixed the
// prior WiFi-task stack overflow). loop() drains the queue into the PSRAM buffer
// on Core 1 as the SOLE writer. The lastPos servo shadow is a single atomic
// 16-bit store, safe across cores without a lock.
namespace navirec {

enum RecKind : uint8_t { REC_ACTION = 0, REC_KF_MAESTRO = 1, REC_KF_HCRVOL = 2 };
enum State    : uint8_t { ST_IDLE = 0, ST_RECORDING = 1, ST_REPLAYING = 2, ST_EDITING = 3 };

struct RecEvent {
  uint32_t tMs;     // ms from record start
  uint8_t  kind;    // RecKind
  union {
    RcAction act;                                    // REC_ACTION
    struct { uint8_t slot, ch; uint16_t pos; } km;   // REC_KF_MAESTRO
    struct { uint8_t chan, vol; } kv;                // REC_KF_HCRVOL
  } u;
};

static const uint16_t LASTPOS_NONE   = 0xFFFF;       // never-commanded sentinel
static const uint32_t REC_MAX_MS     = 60000;        // 60 s capture cap (backstop); buffer sized to match (see recBegin cap)

// Callbacks into NaviCore.ino's (static) dispatch layer.
typedef void (*DispatchActionFn)(const RcAction&);          // re-dispatch a captured action
typedef void (*EmitMaestroFn)(uint8_t slot, uint8_t ch, uint16_t pos);
typedef void (*EmitHcrVolFn)(uint8_t chan, uint8_t vol);    // RAW (bypasses dispatchHcrVolume cache)
typedef void (*ResetChanFn)(uint8_t slot, uint8_t ch);      // speed=0/accel=0

// ── Module state ─────────────────────────────────────────────────────────────
inline RecEvent*     _buf      = nullptr;
inline uint32_t      _cap      = 0;          // max events
inline uint32_t      _count    = 0;          // events in clip
inline volatile uint8_t _state = ST_IDLE;
inline volatile bool _capturing = false;     // gate read in the tap (single volatile)
inline uint32_t      _recStart = 0;
inline char          _recordName[41] = {0};  // name latched when the CURRENT take started — immune to a
                                             // later Play/Record trigger overwriting _pendingName mid-record
inline uint8_t       _mode     = 1;          // mode at capture (context)
inline QueueHandle_t _queue    = nullptr;
inline volatile uint16_t _lastPos[8][32];    // [slot-1][ch], LASTPOS_NONE = unset

// Per-channel replay interpolation curve (phase 1b — see _buildCurveIndex below
// for the full explanation). Declared here (not next to _buildCurveIndex) so
// recBegin() below can allocate _curveNext before first use.
struct MaestroCurve {
  uint16_t pPrev;        // qus of the last finalized keyframe (or servoHome) — ease-from anchor
  uint32_t tPrev;        // its tMs
  int16_t  firstIdx;     // this channel's first keyframe in the clip, -1 = none (set once, for loop rewind)
  int16_t  nextIdx;      // next NOT-YET-finalized keyframe; walked forward as replay advances
  uint16_t lastEmitted;  // de-dupes a redundant _cbEmitMaestro call when the curve hasn't moved
  bool     active;       // channel has a servoHome or at least one keyframe to follow this replay
  bool     reanchor;     // pose is unknown (no home, or a goHome fired mid-replay) — suppress
                          // emission until the next keyframe finalizes and re-anchors the curve
};
inline MaestroCurve _curve[8][32];
inline int16_t*     _curveNext = nullptr;   // parallel to _buf; sized to _cap in recBegin()

inline uint32_t      _replayStart = 0;
inline uint32_t      _cursor   = 0;          // replay event cursor
inline uint32_t      _drops    = 0;          // queue-full drops (diagnostic)
inline volatile bool _doneFlag = false;      // set when a replay completes (loop drains it)
inline volatile bool _loop     = false;      // replay repeats on completion (idle-animation mode)
// Deferred control: a Record/Play trigger can dispatch on Core 0 (remote
// ESP-NOW TRIGGER), but startReplay does Maestro serial writes (ease-in) that
// must run on Core 1. So triggers just set a request here; pollControl() runs it
// from loop(). A single volatile byte is a lock-free edge (last-writer-wins).
enum Ctl : uint8_t { CTL_NONE = 0, CTL_REC_TOGGLE = 1, CTL_PLAY = 2, CTL_STOP = 3 };
inline volatile uint8_t _pendingCtl  = CTL_NONE;
inline volatile uint8_t _pendingMode = 1;    // mode captured at a record request
inline volatile bool    _pendingLoop = false;

inline DispatchActionFn _cbDispatch   = nullptr;
inline EmitMaestroFn    _cbEmitMaestro = nullptr;
inline EmitHcrVolFn     _cbEmitHcrVol  = nullptr;
inline ResetChanFn      _cbResetChan   = nullptr;

// ── Init (Core 1, setup() — BEFORE wcb->begin() so the recv callback can't hit
//    a null queue). RecEvent = 136 B, so cap 24000 ≈ 3.1 MB PSRAM (of ~8 MB;
//    ~6.7 MB was free). Hard ceiling = 32767 (the _curveNext replay index is
//    int16_t); beyond that, widen _curveNext to int32_t. ─────────────────────────
inline void recBegin(uint32_t cap, DispatchActionFn d, EmitMaestroFn m,
                     EmitHcrVolFn v, ResetChanFn r) {
  _cap = cap;
  _buf = (RecEvent*) ps_malloc((size_t)cap * sizeof(RecEvent));
  if (!_buf) _cap = 0;                       // OOM → recorder disabled, never crashes
  _queue = xQueueCreate(32, sizeof(RecEvent));
  // Curve-chain index for replay interpolation (indices must fit int16_t; cap is
  // a fixed 8192 today). OOM here degrades gracefully — replayTick() null-checks
  // _curveNext and each channel just holds at its first keyframe instead of
  // chaining through the rest, rather than crashing.
  _curveNext = (cap && cap <= 32767) ? (int16_t*) ps_malloc((size_t)cap * sizeof(int16_t)) : nullptr;
  _cbDispatch = d; _cbEmitMaestro = m; _cbEmitHcrVol = v; _cbResetChan = r;
  for (int s = 0; s < 8; s++) for (int c = 0; c < 32; c++) _lastPos[s][c] = LASTPOS_NONE;
}

// ── Servo last-position shadow (called from maestroSetTarget — ALL moves) ─────
// Single naturally-aligned 16-bit store → lock-free, torn-write-free on Xtensa
// even though maestroSetTarget runs on both cores. NO bitmap/counter RMW here.
inline void shadowSetTarget(uint8_t slot, uint8_t ch, uint16_t pos) {
  if (slot >= 1 && slot <= 8 && ch < 32) _lastPos[slot - 1][ch] = pos;
}
inline void shadowInvalidateSlot(uint8_t slot) {            // goHome/stopScript: pose unknown
  if (slot < 1 || slot > 8) return;
  for (int c = 0; c < 32; c++) {
    _lastPos[slot - 1][c] = LASTPOS_NONE;
    // Mid-replay this is a curve discontinuity (design §3.3): the servo just
    // snapped to a Maestro-side home position we can't read back, so the
    // interpolation curve's anchor is stale. _updateCurves() (Core 1, same
    // context replay dispatches goHome from) sees reanchor and holds silent
    // until the next keyframe for this channel lands, then snaps straight to
    // it instead of easing from a position that's no longer physically true.
    if (_state == ST_REPLAYING) _curve[slot - 1][c].reanchor = true;
  }
}
// Single-channel pose-unknown: one channel just went limp (auto-release Set Target 0 —
// the Maestro stopped pulsing it, so its true pose is unknown). Same effect as
// shadowInvalidateSlot but for ONE channel, so a later replay reanchors + snaps to the
// next keyframe instead of sweeping the servo up from a stale/zero anchor. Lock-free
// single 16-bit store like shadowSetTarget.
inline void shadowInvalidate(uint8_t slot, uint8_t ch) {
  if (slot < 1 || slot > 8 || ch >= 32) return;
  _lastPos[slot - 1][ch] = LASTPOS_NONE;
  if (_state == ST_REPLAYING) _curve[slot - 1][ch].reanchor = true;
}

// ── Capture taps (noinline + small POD: keep the event off the Core-0 ESP-NOW
//    callback frame, exactly the queueRemoteCli lesson). Non-blocking, drop-if-
//    full — a dropped keyframe just thins one frame; a stall would brick. ───────
inline void __attribute__((noinline)) captureAction(const RcAction& a) {
  if (!_capturing) return;
  if (a.type == RA_RECORD || a.type == RA_PLAY || a.type == RA_STOP) return;   // meta — never record the record/play/stop triggers
  RecEvent ev; ev.tMs = millis() - _recStart; ev.kind = REC_ACTION; ev.u.act = a;
  if (xQueueSend(_queue, &ev, 0) != pdTRUE) _drops++;
}
inline void __attribute__((noinline)) captureMaestroKf(uint8_t slot, uint8_t ch, uint16_t pos) {
  if (!_capturing) return;
  RecEvent ev; ev.tMs = millis() - _recStart; ev.kind = REC_KF_MAESTRO;
  ev.u.km.slot = slot; ev.u.km.ch = ch; ev.u.km.pos = pos;
  if (xQueueSend(_queue, &ev, 0) != pdTRUE) _drops++;
}
inline void __attribute__((noinline)) captureHcrVolKf(uint8_t chan, uint8_t vol) {
  if (!_capturing) return;
  RecEvent ev; ev.tMs = millis() - _recStart; ev.kind = REC_KF_HCRVOL;
  ev.u.kv.chan = chan; ev.u.kv.vol = vol;
  if (xQueueSend(_queue, &ev, 0) != pdTRUE) _drops++;
}

// ── Drain (loop, Core 1 — SOLE writer of the PSRAM buffer) ────────────────────
inline void drain() {
  if (!_queue) return;
  RecEvent ev;
  while (xQueueReceive(_queue, &ev, 0) == pdTRUE) {
    if (_state != ST_RECORDING) continue;
    if (_count < _cap) _buf[_count++] = ev;
    else _drops++;   // PSRAM clip buffer full — account the dropped keyframe so info() is truthful
  }
  // NOTE: the runaway-capture backstop lives in checkRecordBackstop() (called
  // from loop), NOT here — drain() is also invoked from stopRecord()/stop(),
  // and firing an auto-SAVE from an abort would save a take the user discarded.
}

// ── Record control ───────────────────────────────────────────────────────────
inline bool startRecord(uint8_t mode) {
  if (_state != ST_IDLE || !_buf) return false;
  _recordName[0] = '\0';    // default (e.g. CLI ?REC,START): no latched name → auto "rec_N" on save.
                            // The trigger path sets _recordName after this returns true.
  _mode = mode; _count = 0; _drops = 0;
  _recStart = millis();
  _state = ST_RECORDING;
  _capturing = true;                          // arm AFTER recStart so tMs is sane
  return true;
}
inline void stopRecord() {
  if (_state != ST_RECORDING) return;
  _capturing = false;
  drain();                                    // flush in-flight events
  _state = ST_IDLE;
}
inline void clearClip() { if (_state == ST_IDLE) _count = 0; }
inline uint32_t clipDurationMs() { return _count ? _buf[_count - 1].tMs : 0; }
inline uint32_t eventCount()     { return _count; }

// Abort whatever is active — stop a recording OR an in-progress replay. Returns
// what it stopped so the CLI can report it. Replay-abort hands live control back
// next loop (the isReplaying() gates clear).
inline uint8_t stop() {
  uint8_t was = _state;
  if (_state == ST_RECORDING) { _capturing = false; drain(); }
  _state = ST_IDLE;
  return was;
}

// ── Deferred control (set by RA_RECORD/RA_PLAY dispatch, run in loop) ─────────
inline char _pendingName[41] = {0};   // clip name carried by a Record/Play trigger
inline void requestRecordToggle(uint8_t mode, const char* name) {
  _pendingMode = mode; strlcpy((char*)_pendingName, name ? name : "", sizeof(_pendingName)); _pendingCtl = CTL_REC_TOGGLE;
}
inline void requestPlay(const char* name, bool loop) {
  _pendingLoop = loop; strlcpy((char*)_pendingName, name ? name : "", sizeof(_pendingName)); _pendingCtl = CTL_PLAY;
}
inline void requestStop() { _pendingCtl = CTL_STOP; }

// ── Maestro keyframe interpolation (phase 1b) ──────────────────────────────────
// Capture is dense (every knob-deadband keyframe kept), but firing each one AT
// its timestamp still steps servos at capture cadence — this smooths replay by
// emitting a linearly-interpolated qus position for every active (slot,ch)
// EVERY replayTick(), decoupling smoothness from both the original capture
// cadence and loop() jitter (design: docs/RECORD_REPLAY_DESIGN.md §3).
//
// Because a keyframe is only "known" once its own timestamp arrives, true
// look-ahead interpolation needs a channel-scoped chain PARALLEL to the flat,
// time-ordered event buffer: `_curveNext[i]` = index of the next REC_KF_MAESTRO
// event sharing (slot,ch) with event i (-1 = chain end). Built once per
// startReplay() with a single O(n) forward pass (cheap — capped at `_cap`
// events). Each channel eases from `pPrev`@`tPrev` (the last FINALIZED
// keyframe) toward `_buf[nextIdx]` (the next, still-future one); once elapsed
// reaches that keyframe's time it's finalized (becomes the new pPrev/tPrev)
// and the chain advances — so the anchor is always in the past and the target
// always in the future, the invariant real tweening needs. (MaestroCurve and
// _curve/_curveNext are declared up in ── Module state ── so recBegin() can
// allocate _curveNext before this point.)

// Initial build, called once from startReplay(). Also resets speed/accel (0 =
// unlimited) on every touched channel — the Maestro's OWN latched limits live
// device-side and can't be read back, so we force-reset rather than snapshot,
// else they'd double-smooth our interpolated stream — then eases known-home
// channels toward their current position (a no-op if nothing drifted).
inline void _buildCurveIndex() {
  int16_t lastIdx[8][32];
  for (int s = 0; s < 8; s++) {
    for (int c = 0; c < 32; c++) {
      lastIdx[s][c] = -1;
      MaestroCurve& cv = _curve[s][c];
      bool known = (_lastPos[s][c] != LASTPOS_NONE);
      cv.pPrev = known ? _lastPos[s][c] : 0;
      cv.tPrev = 0;
      cv.firstIdx = cv.nextIdx = -1;
      cv.lastEmitted = cv.pPrev;
      cv.active = known;
      cv.reanchor = !known;   // no known home → the first keyframe snaps instead of easing from 0
    }
  }
  for (uint32_t i = 0; i < _count; i++) {
    if (_buf[i].kind != REC_KF_MAESTRO) continue;
    uint8_t s = _buf[i].u.km.slot - 1, c = _buf[i].u.km.ch;
    if (s >= 8 || c >= 32) continue;
    MaestroCurve& cv = _curve[s][c];
    if (lastIdx[s][c] < 0) cv.firstIdx = cv.nextIdx = (int16_t)i;   // first keyframe for this channel
    else if (_curveNext)   _curveNext[lastIdx[s][c]] = (int16_t)i;
    lastIdx[s][c] = (int16_t)i;
    cv.active = true;
  }
  for (int s = 0; s < 8; s++) {
    for (int c = 0; c < 32; c++) {
      if (lastIdx[s][c] >= 0 && _curveNext) _curveNext[lastIdx[s][c]] = -1;   // terminate the chain
      MaestroCurve& cv = _curve[s][c];
      if (!cv.active) continue;
      if (_cbResetChan) _cbResetChan((uint8_t)(s + 1), (uint8_t)c);
      if (!cv.reanchor && _cbEmitMaestro) _cbEmitMaestro((uint8_t)(s + 1), (uint8_t)c, cv.pPrev);
    }
  }
}

// Idle-animation loop restart: rewind each active channel's cursor to its first
// keyframe (cheap — no event-buffer rescan) while KEEPING pPrev at wherever the
// previous lap actually left the servo, so the loop seam eases rather than
// snapping; tPrev resets to 0 to re-anchor against the new lap's elapsed clock.
inline void _rewindCurves() {
  for (int s = 0; s < 8; s++) {
    for (int c = 0; c < 32; c++) {
      MaestroCurve& cv = _curve[s][c];
      if (!cv.active) continue;
      cv.tPrev = 0;
      cv.nextIdx = cv.firstIdx;
      cv.reanchor = false;
    }
  }
}

// Advance + emit every active curve for the current elapsed time. Called once
// per replayTick(), independent of the discrete-event cursor.
inline void _updateCurves(uint32_t elapsed) {
  for (int s = 0; s < 8; s++) {
    for (int c = 0; c < 32; c++) {
      MaestroCurve& cv = _curve[s][c];
      if (!cv.active) continue;
      while (cv.nextIdx >= 0 && _buf[cv.nextIdx].tMs <= elapsed) {
        cv.pPrev = _buf[cv.nextIdx].u.km.pos;
        cv.tPrev = _buf[cv.nextIdx].tMs;
        cv.reanchor = false;               // finalized — pose is known again
        cv.nextIdx = _curveNext ? _curveNext[cv.nextIdx] : (int16_t)-1;
      }
      if (cv.reanchor) continue;           // pose still unknown — hold silent until the next keyframe lands
      uint16_t pos;
      if (cv.nextIdx < 0) {
        pos = cv.pPrev;                    // no more keyframes for this channel — hold position
      } else {
        uint32_t tNext = _buf[cv.nextIdx].tMs;
        uint16_t pNext = _buf[cv.nextIdx].u.km.pos;
        if (tNext <= cv.tPrev) {
          pos = pNext;                     // degenerate/zero-length segment — snap
        } else {
          float frac = (float)(elapsed - cv.tPrev) / (float)(tNext - cv.tPrev);
          pos = (uint16_t)((int32_t)cv.pPrev + (int32_t)(((int32_t)pNext - (int32_t)cv.pPrev) * frac));
        }
      }
      if (pos != cv.lastEmitted) {
        if (_cbEmitMaestro) _cbEmitMaestro((uint8_t)(s + 1), (uint8_t)c, pos);
        cv.lastEmitted = pos;
      }
    }
  }
}

// ── Replay ───────────────────────────────────────────────────────────────────
inline bool startReplay(bool loop = false) {
  if (_state != ST_IDLE || _count == 0) return false;
  _capturing = false;                         // never re-record replayed events
  _loop = loop;
  _buildCurveIndex();                         // resets speed/accel + eases every touched Maestro channel
  _cursor = 0;
  _replayStart = millis();
  _state = ST_REPLAYING;
  return true;
}

// ── Replay tick (loop, Core 1). Discrete actions/HCR-volume fire at their exact
//    timestamp; Maestro motion is driven continuously by _updateCurves(). ──────
inline void replayTick() {
  if (_state != ST_REPLAYING) return;
  uint32_t elapsed = millis() - _replayStart;
  while (_cursor < _count && _buf[_cursor].tMs <= elapsed) {
    const RecEvent& ev = _buf[_cursor++];
    switch (ev.kind) {
      case REC_ACTION:     if (_cbDispatch)   _cbDispatch(ev.u.act); break;
      case REC_KF_MAESTRO: break;   // driven by the interpolated curve follower, not fired directly
      case REC_KF_HCRVOL:  if (_cbEmitHcrVol)  _cbEmitHcrVol(ev.u.kv.chan, ev.u.kv.vol); break;
    }
  }
  _updateCurves(elapsed);
  // Complete when all events have fired, or as a safety net 1 s past the clip's
  // length (so a bad timestamp can never wedge replay + lock out live control).
  if (_cursor >= _count || elapsed > clipDurationMs() + 1000) {
    if (_loop && _count) {                 // idle-animation loop — restart the event stream
      _cursor = 0; _replayStart = millis();
      _rewindCurves();
    } else {
      _state = ST_IDLE;
      _doneFlag = true;   // loop() prints a "playback complete" line on the transition
    }
  }
}

// One-shot "replay just finished" edge — drained by loop() so the message prints
// from Core-1 context (the player completes asynchronously, not inside a CLI call).
inline bool takeReplayDone() { if (_doneFlag) { _doneFlag = false; return true; } return false; }

inline bool isReplaying() { return _state == ST_REPLAYING; }   // gate live dispatch

// ── Persistence: clip library on the dedicated `clips` LittleFS ───────────────
// The host FS is injected from NaviCore.ino (dispatch-agnostic). File format:
// ClipFileHeader + raw RecEvent[count]. All flash I/O runs in loop() context
// (save on record-stop, load on play, both via pollControl / CLI) — never Core 0.
inline fs::FS* _clipFS = nullptr;
inline void setClipFS(fs::FS* fs) { _clipFS = fs; }
inline bool clipFsReady() { return _clipFS != nullptr; }

#pragma pack(push, 1)
struct ClipFileHeader {
  char     magic[4];   // "NCR1"
  uint16_t version;    // 1
  uint16_t mode;       // FunctionSwState at record time (clip context)
  uint32_t count;      // RecEvent entries following the header
  uint32_t durationMs;
};
#pragma pack(pop)

// Sanitize a user clip name into a path "/<name>.ncr" (alnum + _-, <=32 chars).
inline bool _clipPath(char* out, size_t n, const char* name) {
  if (!name || !name[0]) return false;
  char clean[33]; size_t j = 0;
  for (size_t i = 0; name[i] && j < sizeof(clean) - 1; i++) {
    char ch = name[i];
    if (isalnum((int)ch) || ch == '_' || ch == '-') clean[j++] = ch;
  }
  clean[j] = 0;
  if (!j) return false;
  snprintf(out, n, "/%s.ncr", clean);
  return true;
}

inline bool saveClip(const char* name) {
  // Each failure prints a specific reason to the CLI/terminal so a rejected save
  // (record-stop, ?REC,SAVE, or a timeline EDITEND) says WHY, not just "failed".
  if (!_clipFS)          { Serial.println("[REC] save aborted: clips FS not mounted (needs the custom-partition full flash)"); return false; }
  if (_state != ST_IDLE) { Serial.printf ("[REC] save aborted: recorder busy (state=%u)\n", (unsigned)_state); return false; }
  if (_count == 0)       { Serial.println("[REC] save aborted: clip is empty"); return false; }
  char path[48];
  if (!_clipPath(path, sizeof(path), name)) { Serial.printf("[REC] save aborted: bad clip name '%s'\n", name ? name : "(null)"); return false; }
  File f = _clipFS->open(path, "w", true);
  if (!f) { Serial.printf("[REC] save aborted: could not open %s for writing\n", path); return false; }
  ClipFileHeader h; memcpy(h.magic, "NCR1", 4); h.version = 1;
  h.mode = _mode; h.count = _count; h.durationMs = clipDurationMs();
  size_t bodyN = (size_t)_count * sizeof(RecEvent);
  bool ok = (f.write((const uint8_t*)&h, sizeof(h)) == sizeof(h)) &&
            (f.write((const uint8_t*)_buf, bodyN) == bodyN);
  f.close();
  if (!ok) Serial.printf("[REC] save aborted: write to %s failed (flash full?)\n", path);
  return ok;
}

inline int _cmpEventByTime(const void* a, const void* b);   // defined below; used by loadClip's re-sort

inline bool loadClip(const char* name) {
  if (!_clipFS || _state != ST_IDLE || !_buf) return false;
  char path[48]; if (!_clipPath(path, sizeof(path), name)) return false;
  File f = _clipFS->open(path, "r");
  if (!f) return false;
  ClipFileHeader h;
  if (f.read((uint8_t*)&h, sizeof(h)) != (int)sizeof(h) || memcmp(h.magic, "NCR1", 4) != 0) { f.close(); return false; }
  uint32_t n = (h.count > _cap) ? _cap : h.count;
  size_t got = f.read((uint8_t*)_buf, (size_t)n * sizeof(RecEvent));
  f.close();
  _count = got / sizeof(RecEvent);
  _mode  = h.mode;
  // Force non-decreasing tMs regardless of on-flash order. replayTick()'s
  // event cursor, clipDurationMs() (reads the LAST slot), and the phase-1b
  // curve-chain builder all assume time-ordered events, so a corrupt or
  // externally-authored .ncr can't drive replay off the rails. Same re-sort
  // editEnd() performs after a timeline upload; idempotent for a good clip.
  if (_count > 1) qsort(_buf, _count, sizeof(RecEvent), _cmpEventByTime);
  return _count > 0;
}

inline bool deleteClip(const char* name) {
  if (!_clipFS) return false;
  char path[48]; if (!_clipPath(path, sizeof(path), name)) return false;
  return _clipFS->remove(path);
}

inline bool renameClip(const char* from, const char* to) {
  if (!_clipFS) return false;
  char pf[48], pt[48];
  if (!_clipPath(pf, sizeof(pf), from) || !_clipPath(pt, sizeof(pt), to)) return false;
  if (_clipFS->exists(pt)) return false;              // don't clobber an existing clip
  return _clipFS->rename(pf, pt);
}

// ── Phase 2: timeline editor transport ─────────────────────────────────────────
// Ground truth is still the on-device .ncr binary — the editor reuses `_buf`/
// `_count` exactly as LOAD/SAVE already do (only while ST_IDLE), so no second
// buffer, no new persistence format. Two independent one-shot operations:
//   DOWNLOAD (device→browser): ?REC,EDITLOAD,<name> loadClip()s, then dumps every
//     event as one small tagged JSON line — [CLIPDL:BEGIN]/[CLIPDL:EV]/[CLIPDL:END]
//     — same "small self-contained JSON line" shape as the [CLIPITEM] fix, so a
//     line can't be split mid-JSON by the WCB RTERM sink's 160-byte hard-wrap
//     (navicore_rterm.h). Read-only against `_buf` and doesn't hold any state —
//     a lost/incomplete download is trivially retried (config tool checks the
//     BEGIN-declared count against events actually received).
//   UPLOAD (browser→device, saving edits): needs real integrity, since a lost
//     event would silently corrupt the saved clip. So it's per-event ACK/NAK
//     (mirrors naviota's OTA DATA/ACK contract) over three CLI verbs:
//       ?REC,EDITBEGIN            → [CLIPUL:BEGIN,OK|ERR]
//       ?REC,EDITEV,<event json>  → [CLIPUL:ACK,<count>] | [CLIPUL:NAK,<reason>]
//       ?REC,EDITEND,<name>       → [CLIPUL:END,OK|ERR,<reason>]
//     `ST_EDITING` (new state) blocks a concurrent live Record/Play trigger
//     mid-upload from stomping `_buf`/`_count` out from under it — every other
//     entry point already gates on `_state==ST_IDLE`, so adding one more state
//     value automatically fences it out (no other call site needs to change).
//     A JSON event can carry a long WCB command (cmd[] is 95 chars) and, same as
//     download, could exceed 160 B — the config tool applies the SAME per-source
//     fragment-reassembly it already uses for [CLIPLIST] to any [CLIPDL:EV] that
//     arrives split.

inline int _cmpEventByTime(const void* a, const void* b) {
  uint32_t ta = ((const RecEvent*)a)->tMs, tb = ((const RecEvent*)b)->tMs;
  return (ta > tb) - (ta < tb);
}

// DOWNLOAD: stream the currently-loaded clip (call loadClip() first) as tagged
// JSON lines. This runs in loop() (Core 1), so it BLOCKS everything else the
// loop services (SBUS, WCB heartbeats) for its duration — pacing matters:
//  * paced=true (Via-WCB relay): a light delay(1) every 4 lines spaces the
//    [CLIPDL:EV] RTERM packets; the capture sink ALREADY retries with its own
//    delay on real ESP-NOW backpressure (navicore_rterm.h flushLine), so
//    heavier per-line pacing here would only stall loop() (worst case at the
//    3000-event relay cap: ~0.75 s vs 6 s with per-line delay(2)). The CLI
//    layer refuses relayed downloads above that cap (EDITLOAD in NaviCore.ino).
//  * paced=false (direct USB): full speed, with a brief yield every 16 lines so
//    the RTOS idle/WiFi tasks stay fed during a large clip.
inline void editStream(Print& out, bool paced = true) {
  out.printf("[CLIPDL:BEGIN]{\"count\":%lu,\"durationMs\":%lu,\"mode\":%u}\n",
             (unsigned long)_count, (unsigned long)clipDurationMs(), (unsigned)_mode);
  for (uint32_t i = 0; i < _count; i++) {
    const RecEvent& ev = _buf[i];
    if (ev.kind == REC_ACTION) {
      StaticJsonDocument<384> doc;
      JsonObject obj = doc.to<JsonObject>();
      actionToJson(ev.u.act, obj);           // same serializer the button/knob/switch editors use
      obj["t"] = ev.tMs; obj["k"] = (uint8_t)REC_ACTION;
      out.print("[CLIPDL:EV]");
      serializeJson(doc, out);
      out.println();
    } else if (ev.kind == REC_KF_MAESTRO) {
      out.printf("[CLIPDL:EV]{\"t\":%lu,\"k\":%u,\"slot\":%u,\"ch\":%u,\"pos\":%u}\n",
                 (unsigned long)ev.tMs, (unsigned)REC_KF_MAESTRO, ev.u.km.slot, ev.u.km.ch, ev.u.km.pos);
    } else if (ev.kind == REC_KF_HCRVOL) {
      out.printf("[CLIPDL:EV]{\"t\":%lu,\"k\":%u,\"chan\":%u,\"vol\":%u}\n",
                 (unsigned long)ev.tMs, (unsigned)REC_KF_HCRVOL, ev.u.kv.chan, ev.u.kv.vol);
    }
    if (paced) { if ((i & 3) == 3) delay(1); }
    else if ((i & 15) == 15) delay(1);
  }
  out.println("[CLIPDL:END]");
}

// UPLOAD step 1: stage a fresh edit session. Reuses `_buf`/`_count` (guarded to
// ST_IDLE, same as every other clip operation) so there's no second buffer.
inline bool editBegin() {
  if (_state != ST_IDLE || !_buf) return false;
  _count = 0;
  _state = ST_EDITING;
  return true;
}

// UPLOAD step 2: parse ONE event and write it AT AN INDEX — idempotent, so the
// tool's timeout-retry can resend the same event without appending a duplicate
// (the original hazard: the board processes an EDITEV, the ACK is lost, the tool
// retries, and the clip silently gains a copy). idx must be an overwrite or the
// next append (no gaps — the tool uploads sequentially). Self-contained (parses
// its own JSON rather than taking a pre-parsed JsonObject) so the
// StaticJsonDocument's lifetime never crosses a function boundary.
inline bool editAddEvent(uint32_t idx, const char* json) {
  if (_state != ST_EDITING || !_buf || idx >= _cap || idx > _count) return false;
  StaticJsonDocument<384> doc;
  if (deserializeJson(doc, json)) return false;
  JsonObject obj = doc.as<JsonObject>();     // mutable view — actionFromJson's signature requires it
  uint32_t t = obj["t"] | 0;
  uint8_t  k = (uint8_t)(obj["k"] | 0xFF);
  RecEvent ev{}; ev.tMs = t;
  bool ok = false;
  if (k == REC_ACTION) {
    ev.kind = REC_ACTION;
    ok = actionFromJson(obj, ev.u.act);      // same parser the button/knob/switch editors use
  } else if (k == REC_KF_MAESTRO) {
    ev.kind = REC_KF_MAESTRO;
    ev.u.km.slot = (uint8_t)(obj["slot"] | 0);
    ev.u.km.ch   = (uint8_t)(obj["ch"]   | 0);
    ev.u.km.pos  = (uint16_t)(obj["pos"] | 0);
    ok = true;
  } else if (k == REC_KF_HCRVOL) {
    ev.kind = REC_KF_HCRVOL;
    ev.u.kv.chan = (uint8_t)(obj["chan"] | 0);
    ev.u.kv.vol  = (uint8_t)(obj["vol"]  | 0);
    ok = true;
  }
  if (!ok) return false;
  _buf[idx] = ev;
  if (idx + 1 > _count) _count = idx + 1;
  return true;
}

// UPLOAD step 3: finalize. Edits (drag/insert/delete on the timeline) can leave
// events out of chronological order, which every downstream consumer assumes
// (clipDurationMs() reads the LAST buffer slot; replayTick()'s discrete-event
// cursor and the phase-1b curve-chain builder both walk _buf assuming
// non-decreasing tMs) — so re-sort before it's ever saved or played.
// Returns nullptr on success, else a short machine-readable failure reason the
// CLI forwards as "[CLIPUL:END,ERR,<reason>]" and the config tool shows.
inline const char* editEnd(const char* name) {
  if (_state != ST_EDITING) return "not-editing";
  // Drop to ST_IDLE *before* saving: saveClip() guards on `_state == ST_IDLE`
  // (it's meant to refuse a save mid-record/replay). The upload is complete and
  // _buf/_count are stable now, and this whole function runs synchronously in the
  // CLI handler on Core 1 — nothing can race between here and the save — so the
  // transition is safe. (Leaving it ST_EDITING is what made every timeline save
  // fail — the original bug.)
  _state = ST_IDLE;
  if (_count == 0) return "empty-clip";
  qsort(_buf, _count, sizeof(RecEvent), _cmpEventByTime);
  return saveClip(name) ? nullptr : "save-failed";   // saveClip prints the specific reason to the terminal
}

// Abort an in-progress upload (browser closed the editor, or a step NAK'd) —
// discards whatever was staged; nothing was written to flash.
inline void editCancel() { if (_state == ST_EDITING) _state = ST_IDLE; }

// Emits the clip library as one small "[CLIPITEM]{...}" line per clip, bracketed
// by "[CLIPLIST:BEGIN]" / "[CLIPLIST:END]", which the config-tool Clips panel
// accumulates.  Each line stays well under the 160-byte RTERM packet limit, so it
// survives the WCB bridge intact — a single long "[CLIPLIST]{...}" line would be
// hard-wrapped mid-JSON at 160 bytes (navicore_rterm.h) into two [TERM:] lines
// and fail to parse.  Per-clip lines also parse independently, so relayed log
// noise landing between them can't corrupt a neighbouring clip's name.  Clip names
// are already sanitised to [A-Za-z0-9_-] on save, so no JSON escaping is needed.
inline void listClips(Print& out) {
  out.println("[CLIPLIST:BEGIN]");
  if (_clipFS) {
    File dir = _clipFS->open("/");
    for (File e = dir ? dir.openNextFile() : File(); e; e = dir.openNextFile()) {
      if (e.isDirectory()) continue;
      String nm = e.name();
      int slash = nm.lastIndexOf('/'); if (slash >= 0) nm = nm.substring(slash + 1);
      if (nm.endsWith(".ncr")) nm = nm.substring(0, nm.length() - 4);
      uint32_t dur = 0; ClipFileHeader h;
      if (e.read((uint8_t*)&h, sizeof(h)) == (int)sizeof(h) && memcmp(h.magic, "NCR1", 4) == 0) dur = h.durationMs;
      out.printf("[CLIPITEM]{\"name\":\"%s\",\"bytes\":%u,\"dur\":%lu}\n",
                 nm.c_str(), (unsigned)e.size(), (unsigned long)dur);
    }
  }
  out.println("[CLIPLIST:END]");
}

// Lowest-numbered "rec_N" clip name not already on the FS — used when a Record
// trigger fires with no configured name, so a take is always saved (never lost).
inline void _autoClipName(char* out, size_t n) {
  if (!_clipFS) { strlcpy(out, "rec_1", n); return; }
  for (int i = 1; i < 10000; i++) {
    snprintf(out, n, "rec_%d", i);
    char path[48];
    if (_clipPath(path, sizeof(path), out) && !_clipFS->exists(path)) return;
  }
  strlcpy(out, "rec_x", n);   // pathological (10000 takes) — reuse a slot rather than fail
}

// Runaway-capture backstop (loop, Core 1). Auto-stop a recording that has run
// past REC_MAX_MS AND SAVE it, so a long take is never silently lost — mirrors
// pollControl()'s CTL_REC_TOGGLE save (configured slot name, or an auto "rec_N"
// when blank). Kept separate from drain() (which also runs from stopRecord()/
// stop()) so it can only fire from loop and never turns an abort into a save.
// Resolve the clip name for saving the CURRENT recording: the name latched when
// THIS take started (immune to a later Play/Record trigger writing _pendingName),
// else an auto "rec_N". Used by every save-a-recording path so a poisoned
// _pendingName can never redirect a take onto another clip's slot.
inline const char* _takeName(char* autoBuf, size_t n) {
  if (_recordName[0]) return (const char*)_recordName;
  _autoClipName(autoBuf, n);
  return autoBuf;
}

// Call right AFTER drain() so in-flight events are already flushed into _buf.
inline void checkRecordBackstop() {
  if (_state != ST_RECORDING || (millis() - _recStart) <= REC_MAX_MS) return;
  _capturing = false;
  _state = ST_IDLE;                          // saveClip() guards on ST_IDLE — drop first
  char autoName[16];
  const char* nm = _takeName(autoName, sizeof(autoName));
  if (saveClip(nm))
    Serial.printf("[REC] capped at %lus — saved clip '%s'\n",
                  (unsigned long)(REC_MAX_MS / 1000), nm);
}

// Run a deferred Record/Play trigger from loop() (Core 1). Play toggles: press
// while playing → stop, so one button both starts and stops (incl. a loop).
inline void pollControl() {
  uint8_t c = _pendingCtl;
  if (c == CTL_NONE) return;
  _pendingCtl = CTL_NONE;
  switch (c) {
    case CTL_REC_TOGGLE: {
      if (_state == ST_RECORDING) {
        stopRecord();
        // ALWAYS save on stop so a take is never lost: use the name latched at
        // START (_takeName, not the possibly-poisoned _pendingName), or auto-name
        // "rec_N" when blank. stopRecord() already dropped us to ST_IDLE.
        char autoName[16];
        const char* nm = _takeName(autoName, sizeof(autoName));
        if (saveClip(nm)) Serial.printf("[REC] saved clip '%s'\n", nm);
      } else if (_state == ST_IDLE) {
        if (startRecord(_pendingMode))
          strlcpy((char*)_recordName, (const char*)_pendingName, sizeof(_recordName));  // latch THIS take's name
      }
      break;
    }
    case CTL_PLAY:
      if (_state == ST_REPLAYING) {
        stop();                                                      // toggle off
      } else if (_state == ST_IDLE) {
        // Named clip: load it first; if it's missing, don't play the stale RAM clip.
        if (_pendingName[0] && !loadClip((const char*)_pendingName)) break;
        startReplay(_pendingLoop);
      }
      break;
    case CTL_STOP:
      if (_state == ST_RECORDING) {
        // A STOP trigger during recording SAVES the take (the documented
        // recording->save contract) — it must not discard it like a replay-abort.
        stopRecord();                                  // flush + drop to ST_IDLE
        char autoName[16];
        const char* nm = _takeName(autoName, sizeof(autoName));
        if (saveClip(nm)) Serial.printf("[REC] stopped — saved clip '%s'\n", nm);
      } else {
        stop();                                        // replay-abort (or idle) — nothing to save
      }
      break;
  }
}

inline void info(Print& out) {
  uint32_t durMs = _count ? _buf[_count - 1].tMs : 0;
  const char* st = _state == ST_RECORDING ? "RECORDING" : _state == ST_REPLAYING ? "REPLAYING" :
                   _state == ST_EDITING   ? "EDITING"   : "idle";
  out.printf("[REC] state=%s  events=%lu/%lu  dur=%lums  drops=%lu  buf=%s\n",
             st, (unsigned long)_count, (unsigned long)_cap, (unsigned long)durMs,
             (unsigned long)_drops, _buf ? "ok" : "OOM");
}

}  // namespace navirec
