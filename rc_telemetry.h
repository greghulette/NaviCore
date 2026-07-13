// =============================================================================
//  rc_telemetry.h
//
//  WCB-network-side bridge for remote management of the NaviCore.  The
//  user-facing UI stays in config_tool/index.html (the existing RC config
//  tool); the WCB ESP-NOW network is just a TRANSPORT that lets the tool
//  reach a remote RC through a USB-tethered "bridge" WCB.
//
//  Architecture overview:
//
//    [Browser: config_tool/index.html]
//          │
//          │ Web Serial  (Direct USB)                 (Via WCB)
//          ▼                                              ▼
//    [USB NaviCore]              [USB WCB] ─ ESP-NOW ─ [remote RC]
//
//  In "Via WCB" mode the config tool sends the SAME JSON messages it already
//  uses over direct USB; the bridge WCB relays them as ESP-NOW packets to
//  the target RC by deviceId.  Replies and live telemetry come back the
//  same way.  The RC firmware doesn't care which transport carried a
//  given JSON message — it dispatches identically.
//
//  This header provides three things:
//
//    1. OUTBOUND TELEMETRY  (rcTelemetry::tick / emitTrig / emitMode)
//       Periodic + event-driven JSON broadcasts the bridge WCB forwards
//       to the config tool.  Mirror of the existing PWM_UPDATE stream
//       but ESP-NOW-sized (≤250 bytes).
//
//         {"type":"rc_hb",  "id", "fw", "up", "mode", "model"}   0.5 Hz
//         {"type":"rc_ch",  "id", "ch":[24 values]}              5 Hz
//         {"type":"rc_trig","id", "mode", "btn", "tap"}          event
//         {"type":"rc_mode","id", "mode"}                        event
//
//    2. INBOUND COMMAND HANDLER  (rcTelemetry::handle)
//       Called from onWCBCommand().  Detects JSON payloads (leading '{')
//       addressed to this RC and routes them to the existing dispatch /
//       mode paths.  Accepts the same message types the config tool
//       already sends over Web Serial:
//
//         {"type":"PING"}                                   → PONG
//         {"type":"TRIGGER","mode","btn","tap"}             → rcDispatch
//         {"type":"SET_MODE","mode"}                        → FunctionSwState
//
//       SET_CONFIG / GET_CONFIG are intentionally NOT handled over the
//       WCB transport — a full config easily exceeds the 250-byte ESP-NOW
//       packet limit.  The config tool surfaces those as "use Direct USB
//       for full-config push/pull" in its UI.
//
//  Required externs (defined in NaviCore.ino):
//    extern WCB_Client*   wcb;
//    extern int           FunctionSwState;
//    extern int           sbusValues[24];     // signed int per the .ino's declaration
//    void                 rcDispatch(int buttonId, uint8_t tapCount);
// =============================================================================

#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>
#include <WCB_Client.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include "rc_config.h"
#include "fw_version.h"

// Forward declarations from NaviCore.ino.  Type signature for
// sbusValues MUST match the .ino's `int sbusValues[24]` declaration or
// the linker pulls the wrong symbol layout.
extern WCB_Client*  wcb;
extern int          FunctionSwState;
extern int          sbusValues[24];
// SBUS health metrics from the local reader — surface them in rc_hb so the
// Via-WCB config tool can show the ACTUAL receive rate the RC is seeing,
// not the rate at which we're broadcasting rc_ch over ESP-NOW (5 Hz).
extern int           sbusFps;            // frames/sec from the local SBUS reader
extern unsigned long sbusLastFrameMs;    // millis() at last received frame (0 = none yet)
extern bool          lostFrameOld;       // SBUS lost-frame flag (no signal)
extern bool          sbusFailsafe;       // SBUS failsafe flag (TX lost / disarmed)
extern bool          wcbReady;           // true only after wcb->begin() succeeded (ESP-NOW usable)
void                rcDispatch(int buttonId, uint8_t tapCount);
void                queueRemoteTrigger(int mode, int btn, uint8_t tap);   // Core-0 → loop hop for remote TRIGGER (defined in .ino)
void                queueForgetPeer(uint8_t id);   // Core-0 → loop hop for FORGET_PEER; id 0 = all (defined in .ino)
void                resetModeAwareKnobs();   // re-arm mode-aware knobs on a mode change (defined in .ino)

// Forward declarations from rc_config.h — needed for SET_CONFIG / GET_CONFIG
// fragmentation handlers below.  Both already exist; declared here so
// rc_telemetry.h doesn't have to #include rc_config.h's huge implementation.
String rcConfigToJSON();
bool   rcConfigFromJSON(const String& json);
bool   rcConfigFromJSON(const JsonObject& doc);   // JsonObject overload — skip the double-parse
bool   rcConfigSaveNVS();   // legacy NVS save (migration source only)
bool   rcConfigSaveLFS();   // primary: persist config to /config.json on LittleFS

namespace rcTelemetry {

// ── App-level fragmentation for oversize JSON over WCB transport ────────────
// ESP-NOW packets max out at 252 bytes; SET_CONFIG / GET_CONFIG payloads are
// kilobytes.  Sender slices the JSON into ~160-byte chunks and wraps each in
// a fragment envelope:
//
//     {"f":1,"of":3,"sid":42,"s":"…slice of original JSON…"}
//
// Receiver buffers by sid, concatenates `s` fields in fragment order, parses
// the result as JSON, then routes through the normal dispatch.  A small
// session pool + 5-second timeout handles concurrent transfers and dropped
// packets (the requester re-issues if no response).
// Why 80 (not 160): WCB_Client's _sendPacket() copies the envelope into a
// 200-byte structCommand buffer and snprintf's "|CRC%08X" (12 bytes) onto
// the end.  If envelope > 187 bytes the CRC suffix gets truncated and the
// bridge's CRC verifier silently drops the fragment.  The envelope wraps
// the slice in `{"f":N,"of":M,"sid":S,"s":"…"}` and JSON-escapes every
// quote / backslash, so an 80-byte slice can grow to ~130 bytes once
// escaped + wrapped — comfortably under 187.  18 fragments × 160 = 2.88 KB
// previously fit a typical config; at 80 bytes/fragment we need ~36 for
// the same config, still well under FRAG_MAX_PARTS.
constexpr size_t   FRAG_CHUNK_BYTES  = 80;   // underlying-JSON bytes per fragment
// Bumped 64 → 192 so a fully-populated RcConfig (mappings + per-button
// action chains + maestro slots + ...) fits.  Worst-case envelope size
// per session: 192 × sizeof(String) ≈ 3 KB pointer overhead until any
// fragment actually arrives; reasonable for the ESP32-S3's heap.
constexpr uint8_t  FRAG_MAX_PARTS    = 192;  // 192 × 80 = 15 KB max payload
constexpr uint8_t  FRAG_POOL_SIZE    = 3;    // concurrent reassembly sessions
constexpr uint32_t FRAG_TIMEOUT_MS   = 5000;

// Verbose Core-0 logging gate.  handle() runs in the ESP-NOW receive
// callback (WiFi task, Core 0).  Serial.printf there can (a) block up to
// the HWCDC tx timeout if a USB host is attached but not draining, and
// (b) interleave/garble with the main loop's Serial output on Core 1.
// The per-fragment "frag sid=..." line is the highest-frequency offender
// (~40 prints per config transfer, all on Core 0).  Keep it OFF by default;
// flip to true only when actively debugging the fragment handshake.
constexpr bool     RC_TELEM_VERBOSE  = false;

struct FragSession {
  uint16_t sid;          // 0 = slot free
  uint8_t  total;        // expected fragment count
  uint8_t  got;          // received so far
  uint8_t  senderID;     // who sent the fragments (for the SET_CONFIG ack)
  uint32_t expireAt;     // millis() when this session is reclaimed
  String   parts[FRAG_MAX_PARTS];
  // Explicit "have we received this fragment yet?" flags.  Previously we
  // used parts[i].length() == 0 as the proxy, but if a sender ever
  // produced an empty `s` field (legitimately empty slice) the duplicate
  // check would treat every duplicate as new, inflate `got`, and
  // potentially complete the session with garbage.  Separate flag array
  // is unambiguous.  192 bools = 192 bytes — fine.
  bool     received[FRAG_MAX_PARTS];
};

inline FragSession _fragPool[FRAG_POOL_SIZE] = {};
inline uint16_t    _nextOutSid = 1;            // sender-side sid generator

inline FragSession* _findOrAllocSession(uint16_t sid, uint8_t total, uint8_t senderID) {
  const uint32_t now = millis();
  FragSession* freeSlot = nullptr;
  // First pass: reclaim expired slots BEFORE the sid-match check.  If we
  // checked sid first, a stale session whose `expireAt` is in the past
  // but whose sid happens to equal the new sid (likely after _nextOutSid
  // wraps from 65535 → 1) would be returned with old `parts[]` content,
  // causing the new fragments to merge into stale data.  Reclaim-first
  // guarantees that any returned slot is either fresh-claimed or genuine.
  for (auto& s : _fragPool) {
    if (s.sid != 0 && (int32_t)(now - s.expireAt) >= 0) {
      s = FragSession{};
    }
  }
  for (auto& s : _fragPool) {
    // Match on (sid, senderID), not sid alone: two config tools each start
    // _nextOutSid at 1, so without the senderID check their sid=1 sessions would
    // collide and interleave fragments into one corrupt reassembly.
    if (s.sid == sid && s.senderID == senderID) return &s;
    if (!freeSlot && s.sid == 0)                freeSlot = &s;
  }
  if (!freeSlot) return nullptr;                // pool exhausted — drop
  freeSlot->sid      = sid;
  freeSlot->total    = total;
  freeSlot->got      = 0;
  freeSlot->senderID = senderID;
  freeSlot->expireAt = now + FRAG_TIMEOUT_MS;
  // FragSession{} default-construction already zeroed `received[]`, but
  // when we re-claim a slot that previously had a different sid we want
  // to be sure no stale flags carry over.  Cheap belt-and-braces.
  for (uint8_t i = 0; i < FRAG_MAX_PARTS; i++) freeSlot->received[i] = false;
  return freeSlot;
}

// Forward declaration so handle() can recurse on the reassembled message.
inline bool handle(uint8_t senderID, const char* command);

// ── Outbound: non-blocking GET_CONFIG fragment sender (state machine) ───────
// Ships a CONFIG response (rcConfigToJSON output) back to a requester,
// fragmented into ESP-NOW-sized unicast packets the receiver reassembles
// by sid.
//
// Unicast (wcb->send) not broadcast: WCBClient's send() path is ETM, so
// every fragment gets an ACK and is retried automatically on loss.
// Broadcast measured ~90% loss on a 38-fragment payload — sessions timed
// out and the config never loaded.
//
// WHY A STATE MACHINE (the reliability core of this whole feature):
// The previous version looped `delay(150)` x N fragments inside ONE tick()
// call.  For a ~40-fragment config that froze loop() for ~6 SECONDS — and
// since processSbus() lives in the same loop(), SBUS RX overflowed, the
// SBUS-OUT passthrough went dead, and transmitter button presses were
// dropped for the entire 6 s.  Now tick() sends AT MOST ONE fragment per
// call and returns immediately; pacing is enforced by comparing millis()
// to the last-send timestamp.  loop() keeps spinning between fragments, so
// SBUS read / dispatch / passthrough stay fully live during a config pull.
// loop()'s own wcb->update() (top of loop) drives ACK/retry processing
// continuously — far more often than the old once-per-fragment pump.
//
// While a send is active, periodic rc_hb / rc_ch telemetry is suppressed
// (tick() returns early).  This preserves the clean one-unicast-every-150ms
// ESP-NOW traffic pattern the pacing was tuned around; mixing in broadcasts
// would add MAC contention and risk the "[SEND CB] MAC-layer FAILED" drops.
// The config tool's live panel just goes briefly stale during the load —
// cosmetic, and the droid stays fully responsive throughout.

// Requester slot — set by handle() (Core 0 WiFi-callback) when a GET_CONFIG
// arrives, consumed/cleared by tick() (Core 1 main loop).  A plain uint8_t
// is an atomic read/write on Xtensa, so no mutex is needed.  Stays set for
// the ENTIRE duration of the send (request + in-progress) so handle()'s
// "already busy?" guard rejects overlapping requests; cleared only when the
// send completes or aborts (in _pumpGetConfigSend).  Worst-case collision
// (a new request arriving in the same instant the old one clears) just
// drops one request — the config tool re-issues after its 5 s timeout.
inline uint8_t _pendingGetConfigSender = 0;

// Inter-fragment pacing.  40 ms saturated the ESP-NOW MAC layer in testing
// (queue-overflow drops on the receiving WCB); 150 ms gives the MAC + ETM
// ACK round-trip comfortable headroom.  Matches the config tool's outbound
// SET_CONFIG cadence.  A real config is ~40 fragments => ~6 s wall-clock,
// now spread across thousands of non-blocking loop() iterations.
constexpr uint32_t FRAG_PACING_MS = 150;

// In-progress send state.  One outstanding send at a time — the requester
// re-issues GET_CONFIG if it times out, so we never need to queue two.
struct OutboundSend {
  bool     active     = false;
  uint8_t  targetID   = 0;
  uint16_t sid        = 0;
  uint16_t total      = 0;   // total fragment count
  uint16_t next       = 0;   // index of the next fragment to send (0-based)
  uint32_t lastSendMs = 0;   // millis() of the last fragment sent
  String   payload;          // full wrapped CONFIG JSON, held for the send
  uint16_t chunkOff[FRAG_MAX_PARTS + 1] = {0};   // codepoint-safe byte boundaries: slice idx = [chunkOff[idx], chunkOff[idx+1])
};
inline OutboundSend _outSend;

// Build + unicast fragment `idx` of the active job.  Returns false ONLY on a
// fatal envelope-size error (caller aborts the job); MAC-layer loss is
// handled by ETM retries, not reported here.
inline bool _sendFragment(uint16_t idx) {
  if (idx >= _outSend.total) return false;                     // out of range — shouldn't happen
  // Slice on the precomputed codepoint-safe boundaries (see _startGetConfigSend)
  // so a multi-byte UTF-8 char is never split across two fragments — the tool
  // reassembles by concatenating slices and would otherwise show U+FFFD.
  const size_t off = _outSend.chunkOff[idx];
  const size_t end = _outSend.chunkOff[idx + 1];
  String slice = _outSend.payload.substring(off, end);

  // ArduinoJson escapes quotes / backslashes / control chars in the slice.
  StaticJsonDocument<300> env;
  env["f"]   = (int)(idx + 1);
  env["of"]  = (int)_outSend.total;
  env["sid"] = _outSend.sid;
  env["s"]   = slice;

  char buf[252];
  size_t n = serializeJson(env, buf, sizeof(buf));
  // 187-byte cap: WCB_Client appends "|CRC%08X" (12 bytes) into a 200-byte
  // structCommand buffer; >187 truncates the CRC and the bridge drops the
  // fragment.  Keep in sync with FRAG_CHUNK_BYTES.
  constexpr size_t MAX_ENV_BYTES = 187;
  if (n == 0 || n > MAX_ENV_BYTES) {
    Serial.printf("[RC] CONFIG send: fragment %u/%u envelope %u > %u-byte cap — aborting\n",
                  (unsigned)(idx + 1), (unsigned)_outSend.total,
                  (unsigned)n, (unsigned)MAX_ENV_BYTES);
    return false;
  }
  if (wcb) wcb->send(_outSend.targetID, buf);
  return true;
}

// Arm a fragmented CONFIG send to `target`.  Returns false if the payload
// can't be sent (empty / too large for the fragment budget); true if armed.
// Does NOT send the first fragment — _pumpGetConfigSend() does, same tick.
inline bool _startGetConfigSend(uint8_t target) {
  if (!wcb || target == 0) return false;
  String body = rcConfigToJSON();
  String wrapped;
  wrapped.reserve(body.length() + 40);
  wrapped += "{\"type\":\"CONFIG\",\"id\":";
  wrapped += rcConfig.wcbNetwork.deviceId;
  wrapped += ",\"data\":";
  wrapped += body;
  wrapped += "}";

  // Split into codepoint-safe chunks (each <= FRAG_CHUNK_BYTES bytes, never cutting
  // a multi-byte UTF-8 char) and record the byte boundaries. `total` is the chunk
  // count; _sendFragment slices on these boundaries so a multi-byte value in the
  // CONFIG can't be torn across two fragments (the receiver positionally join()s).
  size_t total = 0;
  {
    size_t i = 0; const size_t N = wrapped.length();
    _outSend.chunkOff[0] = 0;
    while (i < N && total < FRAG_MAX_PARTS) {
      size_t take = 0;
      while (i + take < N) {
        unsigned char c = (unsigned char)wrapped[i + take];
        size_t cp = (c < 0x80) ? 1 : ((c >> 5) == 0x6) ? 2 : ((c >> 4) == 0xE) ? 3 : ((c >> 3) == 0x1E) ? 4 : 1;
        if (take + cp > FRAG_CHUNK_BYTES) break;
        take += cp;
      }
      if (take == 0) take = 1;                        // pathological lone continuation byte — make progress
      i += take; total++;
      _outSend.chunkOff[total] = (uint16_t)i;
    }
    if (i < N) total = (size_t)FRAG_MAX_PARTS + 1;    // didn't fit within the part budget — force the bail
  }
  if (total == 0 || total > FRAG_MAX_PARTS) {
    Serial.printf("[RC] CONFIG send bailed: %u bytes needs > %u fragments. "
                  "Use Direct USB for this size.\n",
                  (unsigned)wrapped.length(), (unsigned)FRAG_MAX_PARTS);
    return false;
  }

  uint16_t sid = _nextOutSid++;
  if (_nextOutSid == 0) _nextOutSid = 1;

  _outSend.active     = true;
  _outSend.targetID   = target;
  _outSend.sid        = sid;
  _outSend.total      = (uint16_t)total;
  _outSend.next       = 0;
  // Bias the timestamp back one full interval so the first pump fires the
  // first fragment immediately.  Unsigned subtraction makes this correct
  // even if millis() < FRAG_PACING_MS (can't happen post-boot, but free).
  _outSend.lastSendMs = millis() - FRAG_PACING_MS;
  _outSend.payload    = wrapped;

  Serial.printf("[RC] CONFIG send START: %u bytes → %u fragments to W%u (sid=%u)\n",
                (unsigned)wrapped.length(), (unsigned)total, (unsigned)target, (unsigned)sid);
  return true;
}

// Advance the in-progress send by at most one fragment, paced at
// FRAG_PACING_MS.  Returns true while a send is active (tick() then
// suppresses telemetry); false when idle or on the tick the send finishes.
// Clears _pendingGetConfigSender when the send completes or aborts so
// handle()'s "already busy" guard reopens for the next request.
inline bool _pumpGetConfigSend() {
  if (!_outSend.active) return false;
  const uint32_t now = millis();
  if ((now - _outSend.lastSendMs) < FRAG_PACING_MS) {
    return true;   // pacing — active, but nothing to send this tick
  }
  if (!_sendFragment(_outSend.next)) {
    Serial.println("[RC] CONFIG send ABORTED (fragment build error)");
    _outSend = OutboundSend{};        // free payload + reset
    _pendingGetConfigSender = 0;
    return false;
  }
  _outSend.lastSendMs = now;
  _outSend.next++;
  if (_outSend.next >= _outSend.total) {
    Serial.printf("[RC] CONFIG send COMPLETE: %u fragments (sid=%u)\n",
                  (unsigned)_outSend.total, (unsigned)_outSend.sid);
    _outSend = OutboundSend{};        // free payload + reset
    _pendingGetConfigSender = 0;
    return false;
  }
  return true;   // more fragments remain
}

// ── Timing ───────────────────────────────────────────────────────────────────
// Heartbeat at 0.5 Hz (2 s) — low-overhead "I'm alive" beacon driving the
// config tool / Wizard's online-RC list.  Channels at 5 Hz (200 ms) — fast
// enough for the live SBUS visualizer to feel responsive, slow enough that
// 24-channel JSON packets (~150 B) don't saturate ESP-NOW airtime.
constexpr uint32_t HB_INTERVAL_MS = 2000;
constexpr uint32_t CH_INTERVAL_MS = 200;

inline uint32_t _lastHb = 0;
inline uint32_t _lastCh = 0;

// ── Subscription gate for high-rate broadcasts ───────────────────────────────
// `rc_hb` is a low-cost (0.5 Hz) presence beacon that ALWAYS runs so the WCB
// Wizard's discovery feature can find this RC even when no config tool is
// open.  `rc_ch` is 10× the bandwidth (5 Hz × 128 B = ~640 B/s of ESP-NOW
// airtime) and only useful when a config tool is actively live-monitoring,
// so we gate it on "have we received an inbound JSON message from a WCB
// sender recently?".  Any inbound JSON (PING / TRIGGER / SET_MODE / fragment)
// counts as a subscription renewal; if 15 s passes with no inbound, we stop
// emitting rc_ch.  The config tool sends a periodic PING in Via WCB mode to
// keep the subscription alive during passive monitoring.
//
// Direct-USB-only setups never PING over WCB, so rc_ch broadcasts simply
// never start — no airtime wasted on networks where no one's listening.
inline uint32_t _lastWcbInbound = 0;
constexpr uint32_t WCB_SUBSCRIPTION_MS = 15000;   // rc_ch off after 15 s idle
inline bool _hasWcbSubscriber(uint32_t now) {
  return (_lastWcbInbound != 0) &&
         (now - _lastWcbInbound < WCB_SUBSCRIPTION_MS);
}

// ── Deferred SET_CONFIG queue (callback → main loop) ────────────────────────
// rcTelemetry::handle() runs in the WCB_Client receive-callback chain, which
// fires from the ESP-NOW WiFi-task context on Core 0.  rcConfigFromJSON()
// can be a multi-KB JSON parse and rcConfigSaveLFS() blocks 100+ ms doing
// a flash erase + write — running either inline blocks the WiFi task and
// trips Core 0's interrupt watchdog ("Guru Meditation Error: Core 0
// panic'ed (Interrupt wdt timeout on CPU0)").
//
// Pattern: handle() stashes the reassembled JSON + sender ID here, then
// tick() (called from loop()) picks it up and applies it on the main task
// where blocking flash I/O is safe.  ACK is also deferred to tick() so it
// reports the actual apply result, not an optimistic "we got it" pre-apply.
//
// We stash the WHOLE reassembled payload (not just the `data` field)
// because the StaticJsonDocument<512> in handle() can't parse multi-KB
// JSON — tick() does the parse with a properly-sized DynamicJsonDocument
// instead.
// ── Cross-core synchronization for the deferred queues ──────────────────────
// `handle()` runs in the WCB_Client receive-callback context (WiFi task
// on Core 0).  `tick()` runs in the main loop (Core 1).  Both read AND
// write `_pendingReassembled` (an Arduino String).  String assignment is
// multi-step (allocate buffer → set length → copy bytes), and an
// interleaved read between steps can yield a length pointing past the
// end of the buffer — `_applyReassembled` then parses garbage or crashes
// in deserializeJson.
//
// The mutex is acquired briefly on each side around the read/write of
// the slot.  Cost is microseconds; safety is absolute.  Initialized
// lazily by `init()` (called from NaviCore.ino setup() before
// WCB_Client init brings the receive callback online).
inline SemaphoreHandle_t _pendingMutex = nullptr;

// Call from NaviCore.ino setup() BEFORE wcb->begin() so the mutex
// is ready when the first ESP-NOW packet arrives.  Idempotent.
inline void init() {
  if (!_pendingMutex) _pendingMutex = xSemaphoreCreateMutex();
}

inline String  _pendingReassembled;          // full JSON of a reassembled fragmented payload
inline uint8_t _pendingReassembledSender = 0;

// (_pendingGetConfigSender is declared up in the outbound-send state-machine
//  block above, since the pump functions there reference it.)

// ── Deferred apply for reassembled fragmented payloads ──────────────────────
// Called from tick() (main loop context) when _pendingReassembled is set.
// Uses a DynamicJsonDocument sized to 2× the input length — enough for the
// expected SET_CONFIG / CONFIG envelopes without the StaticJsonDocument<512>
// truncation that breaks the inline-recursion path.  Safe to call
// rcConfigFromJSON + rcConfigSaveLFS here — flash I/O blocks the main task
// but the WiFi/ESP-NOW task on Core 0 stays unblocked.
inline void _applyReassembled(uint8_t senderID, const String& json) {
  // 2x heuristic: ArduinoJson docs need roughly the input size plus overhead
  // for the parsed-object tree.  SET_CONFIG over WCB normally carries a
  // DIFF (one or two changed branches → small), but a first-save with no
  // baseline can ship a full config (~3 KB → ~6 KB doc).  Cap at 16 KB so
  // a full config has headroom; the fragment layer can deliver up to 15 KB
  // (FRAG_MAX_PARTS × FRAG_CHUNK_BYTES) so this matches the transport's
  // ceiling.  On overflow deserializeJson returns NoMemory and we bail
  // WITHOUT applying — we never persist a partially-parsed config.
  size_t cap = json.length() * 2;
  if (cap < 1024)  cap = 1024;
  if (cap > 16384) cap = 16384;
  DynamicJsonDocument doc(cap);
  DeserializationError err = deserializeJson(doc, json);
  if (err) {
    Serial.printf("[RC] reassembled JSON parse failed: %s (size=%u cap=%u) — NOT applied\n",
                  err.c_str(), (unsigned)json.length(), (unsigned)cap);
    return;
  }
  const char* type = doc["type"] | "";
  if (!strcmp(type, "SET_CONFIG")) {
    if (!doc["data"].is<JsonObject>()) {
      Serial.println("[RC] reassembled SET_CONFIG missing 'data' object");
      return;
    }
    // ── Self-preservation: strip wcbNetwork.deviceId before applying ──
    // A SET_CONFIG arriving over the WCB transport must NEVER change
    // wcbNetwork.deviceId — doing so would change our broadcast sender
    // ID mid-session, the bridge's special-peer (slot 20) would stop
    // recognising us, and the config tool's `msg.id !== 20` filter
    // would silently drop every subsequent rc_hb / rc_ch / rc_trig.
    // Symptom: "SBUS stopped showing on the webpage after a Save."
    // The config tool's default `config.wcbNetwork.deviceId` is 3, so
    // any Save before a successful Load would otherwise demote us
    // from the special-peer slot.  We also strip macOct2/macOct3/
    // password/quantity for the same reason — those define which
    // network we live on; changing them mid-session is suicide.
    JsonObject data = doc["data"].as<JsonObject>();
    if (data["wcbNetwork"].is<JsonObject>()) {
      JsonObject wnet = data["wcbNetwork"].as<JsonObject>();
      if (wnet.containsKey("deviceId")) {
        Serial.printf("[RC] SET_CONFIG: ignoring incoming wcbNetwork.deviceId=%d "
                      "(WCB-transport saves can't change our own slot)\n",
                      (int)wnet["deviceId"]);
        wnet.remove("deviceId");
      }
      if (wnet.containsKey("macOct2"))  wnet.remove("macOct2");
      if (wnet.containsKey("macOct3"))  wnet.remove("macOct3");
      if (wnet.containsKey("password")) wnet.remove("password");
      if (wnet.containsKey("quantity")) wnet.remove("quantity");
      // If we stripped every field, drop the empty `wcbNetwork: {}` so
      // rcConfigFromJSON's containsKey("wcbNetwork") check short-circuits
      // and we skip the whole "apply each field from JSON with fallback"
      // loop.  Tiny perf win; avoids touching strlcpy(password) etc.
      // with the same values they already hold.
      if (wnet.size() == 0) data.remove("wcbNetwork");
    }
    // Pass the JsonObject directly instead of re-serializing → re-parsing.
    // The old String round-trip allocated 3 KB for `dataJson` AND another
    // ~6 KB inside rcConfigFromJSON(String) to re-parse, on top of the 6 KB
    // doc we're already holding here — heap fragmentation at that point
    // is a real crash risk on the ESP32-S3.  rcConfigFromJSON has a
    // JsonObject overload that reads `data` in-place from our parsed tree.
    Serial.printf("[RC] SET_CONFIG → applying via rcConfigFromJSON(JsonObject) "
                  "(heap=%u bytes free)\n", (unsigned)ESP.getFreeHeap());
    bool ok = rcConfigFromJSON(data);
    Serial.printf("[RC] SET_CONFIG → rcConfigFromJSON returned %s "
                  "(heap=%u bytes free)\n", ok ? "true" : "false",
                  (unsigned)ESP.getFreeHeap());
    if (ok) {
      bool saved = rcConfigSaveLFS();
      if (!saved) ok = false;   // surface the save failure in the ACK below
      Serial.println(saved ? "[RC] SET_CONFIG → applied + saved to LittleFS"
                           : "[RC] SET_CONFIG → applied but SAVE FAILED (not persisted)");
    } else {
      Serial.println("[RC] SET_CONFIG → rcConfigFromJSON returned false");
    }
    char ack[100];
    snprintf(ack, sizeof(ack),
      "{\"type\":\"ACK\",\"of\":\"SET_CONFIG\",\"id\":%u,\"ok\":%s}",
      rcConfig.wcbNetwork.deviceId, ok ? "true" : "false");
    if (wcb) wcb->send(senderID, ack);
    return;
  }
  Serial.printf("[RC] reassembled payload had unexpected type '%s' — dropping\n", type);
}

// ── Outbound: heartbeat + channel snapshot (periodic) ───────────────────────
// Called from loop().  Emits at most one HB and one CH packet per call,
// throttled to the constants above.  Setting _lastHb = 0 / _lastCh = 0
// forces an immediate emit on the next tick (used by PING handler).
inline void tick() {
  // No WCB / ESP-NOW down → nothing to send or pump. Guard so a failed
  // wcb->begin() can't drive sends into an uninitialized WCB_Client.
  if (!wcb || !wcbReady) return;
  // Drain any pending reassembled fragment payload BEFORE the broadcast
  // work — applying SET_CONFIG blocks 100+ ms doing flash I/O, and we'd
  // rather skip one heartbeat than risk a watchdog by mixing flash and
  // ESP-NOW TX in the same tick.  Clearing the slot before the apply
  // means a new reassembly can start landing while the apply runs.
  // ── Drain pending reassembled payload (mutex-guarded) ─────────────────
  // We take the mutex, swap out the slot contents, release the mutex,
  // THEN call _applyReassembled outside the lock — the apply runs flash
  // I/O for 100+ ms and we don't want to block handle() (Core 0 WiFi
  // task) waiting on the lock for that long.
  String  drainedPayload;
  uint8_t drainedSender = 0;
  if (_pendingMutex && xSemaphoreTake(_pendingMutex, portMAX_DELAY) == pdTRUE) {
    if (_pendingReassembled.length() > 0) {
      drainedPayload            = _pendingReassembled;
      drainedSender             = _pendingReassembledSender;
      _pendingReassembled       = String();   // free heap before apply
      _pendingReassembledSender = 0;
    }
    xSemaphoreGive(_pendingMutex);
  }
  if (drainedPayload.length() > 0) {
    _applyReassembled(drainedSender, drainedPayload);
    return;   // skip heartbeat/channel emit this tick — next call resumes
  }

  // CONFIG-send state machine.  Arm a requested send (if idle), then pump
  // it — at most ONE fragment per tick(), paced internally.  tick() returns
  // fast every iteration, so loop() keeps running processSbus() throughout
  // and SBUS / passthrough stay live during a config pull (vs. the old
  // ~6 s freeze).  While a send is active, _pumpGetConfigSend() returns true
  // and we skip the periodic telemetry below to keep the airwaves clear for
  // fragment ACKs.  See the OutboundSend block above for the full rationale.
  if (_pendingGetConfigSender != 0 && !_outSend.active) {
    if (!_startGetConfigSend(_pendingGetConfigSender)) {
      _pendingGetConfigSender = 0;    // couldn't start (too big / empty) — clear so we don't wedge
    }
  }
  if (_pumpGetConfigSend()) return;   // a send is in progress this tick — suppress telemetry

  if (!wcb) return;
  const uint32_t now = millis();
  const uint8_t  id  = rcConfig.wcbNetwork.deviceId;

  if (now - _lastHb >= HB_INTERVAL_MS) {
    _lastHb = now;
    // SBUS health: surface what the LOCAL reader is seeing so the config
    // tool's SBUS RECEIVER block shows the true ~70-100 Hz frame rate
    // rather than our 5 Hz rc_ch broadcast cadence.  ageMs is the time
    // since the last SBUS frame arrived at the RC; the tool uses this to
    // distinguish "transmitter on, no signal" from "transmitter off".
    const unsigned long sbusAgeMs =
      (sbusLastFrameMs == 0) ? 99999UL : (now - sbusLastFrameMs);
    char buf[220];
    snprintf(buf, sizeof(buf),
      "{\"type\":\"rc_hb\",\"id\":%u,\"fw\":\"%s\",\"up\":%lu,"
       "\"mode\":%d,\"model\":%u,"
       "\"sbusFps\":%d,\"sbusAge\":%lu,\"sbusLost\":%s,\"sbusFail\":%s}",
      id, FW_VERSION,
      (unsigned long)(now / 1000UL),
      FunctionSwState,
      rcConfig.txModel,
      sbusFps,
      sbusAgeMs,
      lostFrameOld ? "true" : "false",
      sbusFailsafe ? "true" : "false");
    // best-effort: liveness beat; a miss is covered by the next one (0.5 Hz)
    wcb->broadcast(buf, false);   // broadcast() not send(0,...) — send() bails on target_wcb<1
  }

  // rc_ch is gated on subscription — see _hasWcbSubscriber comment above.
  // Without a recent inbound, we don't emit channel updates because they're
  // useless to anyone not actively monitoring (10× the cost of rc_hb).
  if (now - _lastCh >= CH_INTERVAL_MS && _hasWcbSubscriber(now)) {
    _lastCh = now;
    char buf[240];
    int  len = snprintf(buf, sizeof(buf),
      "{\"type\":\"rc_ch\",\"id\":%u,\"ch\":[", id);
    if (len < 0 || len >= (int)sizeof(buf)) return;   // header itself overflowed — bail
    // Always emit 24 channels.  sbusValues[] is normally 0..2047 (11-bit
    // SBUS), but it's a signed int with no hard clamp, so guard against a
    // glitched/garbage value blowing the buffer.  snprintf returns the
    // would-have-written count, which can exceed the remaining space on
    // truncation; checking that return BEFORE advancing `len` prevents the
    // classic "len walks past the buffer, next sizeof(buf)-len underflows
    // to a huge size_t" stack-smash.
    bool ok = true;
    for (int i = 0; i < 24; i++) {
      int rem = (int)sizeof(buf) - len;
      if (rem <= 8) { ok = false; break; }            // leave room for "]}\0"
      int w = snprintf(buf + len, rem, "%s%d", (i ? "," : ""), sbusValues[i]);
      if (w < 0 || w >= rem) { ok = false; break; }    // truncated — stop, don't advance past buf
      len += w;
    }
    if (ok && len + 2 < (int)sizeof(buf)) {
      buf[len++] = ']';
      buf[len++] = '}';
      buf[len]   = '\0';
      // best-effort: superseded by the next frame 200 ms later; ensured would
      // thrash the ETM pending table at 5 Hz retrying already-stale channels.
      wcb->broadcast(buf, false);
    }
    // If !ok (garbage channel data overflowed the buffer) we simply skip
    // this rc_ch frame rather than emit a malformed/unterminated packet.
  }
}

// ── Outbound: event-driven emits ────────────────────────────────────────────
// Call from rcDispatch() right after argument validation so EVERY trigger
// — local matrix press, Web-Serial JSON TRIGGER, remote ESP-NOW TRIGGER —
// surfaces on the network.  The config tool's "Via WCB" mode treats this as
// "something fired, no matter who fired it".
inline void emitTrig(int mode, int btn, int tap) {
  if (!wcb || !wcbReady) return;
  char buf[120];
  snprintf(buf, sizeof(buf),
    "{\"type\":\"rc_trig\",\"id\":%u,\"mode\":%d,\"btn\":%d,\"tap\":%d}",
    rcConfig.wcbNetwork.deviceId, mode, btn, tap);
  wcb->broadcast(buf, false);   // best-effort: monitor display telemetry, not a must-land command
}

// Call from the FunctionSwState-update path whenever the active mode
// changes.  Lets the config tool update its mode indicator immediately
// instead of waiting up to 2 s for the next heartbeat.
inline void emitMode(int mode) {
  if (!wcb || !wcbReady) return;
  char buf[80];
  snprintf(buf, sizeof(buf),
    "{\"type\":\"rc_mode\",\"id\":%u,\"mode\":%d}",
    rcConfig.wcbNetwork.deviceId, mode);
  wcb->broadcast(buf, false);   // best-effort: monitor display telemetry (sibling of rc_trig)
}

// ── Inbound parser ──────────────────────────────────────────────────────────
// Called from onWCBCommand() for every command this RC's WCB stack delivers
// to the application (unicasts addressed to our deviceId + broadcasts).
// Returns true if the message was a JSON command directed at us and was
// dispatched (caller should NOT route it further); false if it's something
// the caller still needs to handle (e.g. another RC's heartbeat we want
// to ignore, or a non-JSON legacy WCB command intended for some other path).
// Cache of each WCB's friendly alias (its ?ALIAS name), learned from the
// {"type":"wcb_alias"} reply a board sends when we query it with "?WHOAMI".
// Index = WCB board number (1..20); "" = not yet known. Read by GET_WCB_STATUS
// to label the config tool's WCB-status panel. Written in the receive callback
// (Core 0) and read from loop() (Core 1) — a torn read at worst shows a
// momentarily-garbled name for one 3 s poll, so no lock is warranted.
static char _wcbAlias[21][25] = {{0}};
inline const char* wcbAlias(int id) { return (id >= 1 && id <= 20) ? _wcbAlias[id] : ""; }
// Parallel cache of whether each learned node is a CLIENT (mesh monitor / other
// controller) vs a WCB board. Held like the alias so a learned client still reads
// as a client on the status panel after its WDP advert ages out (a client never
// heartbeats, so it can't be re-identified while offline). Written on Core 0
// (onNeighbor), read on Core 1 (buildWcbStatus) — a single byte, benign to race.
static bool _wcbIsClient[21] = {false};
inline bool wcbIsClient(int id) { return (id >= 1 && id <= 20) ? _wcbIsClient[id] : false; }
inline void setWcbClient(int id, bool isClient) { if (id >= 1 && id <= 20) _wcbIsClient[id] = isClient; }
// Set/refresh a board's cached friendly name — from EITHER a "?WHOAMI" reply OR a
// live WDP advert (see onWcbNeighbor). The library rebuilds a neighbor's facts
// wholesale on every advert, so feeding the WDP name through here is what lets a
// RENAMED board update its status-panel name (the ?WHOAMI cache alone is held
// once set). Sanitizes chars that would break the UNescaped JSON buildWcbStatus
// emits. Call ONLY from the Core-0 receive path (onWCBCommand / onNeighbor) so
// every writer to _wcbAlias stays on one core.
inline void setWcbAlias(int id, const char* name) {
  if (id < 1 || id > 20 || !name) return;
  char*  dst = _wcbAlias[id];
  size_t cap = sizeof(_wcbAlias[id]) - 1, j = 0;
  dst[0] = '\0';   // terminate first so a racing Core-1 %s read during the rewrite
                   // sees an empty/short-but-terminated string, never a torn long one
                   // (the last byte dst[24] is invariantly '\0', so no row over-read either)
  for (size_t k = 0; name[k] && j < cap; k++) {
    char c = name[k];
    if (c == '"' || c == '\\' || (unsigned char)c < 0x20) continue;   // JSON-hostile chars
    dst[j++] = c;
  }
  dst[j] = '\0';
}

// "?WHOAMI" is re-sent each status poll only until a board's alias is cached.
// A board with NO alias configured never replies with a name, which used to
// mean we polled it forever (~every 3 s). Cap the attempts per online session
// so an alias is OPTIONAL: ask a few times when a board appears, then give up.
// Re-armed on an offline→online transition, so an alias configured later (after
// the board reconnects) is still picked up.
static const uint8_t ALIAS_MAX_TRIES = 4;
static uint8_t _aliasTries[21] = {0};
static bool    _aliasWasUp[21] = {false};

// Build the WCB_STATUS reply (board liveness + friendly aliases) into buf — the
// same payload NaviCore.ino emits to USB, but reusable over the WCB bridge.
// relayId = the WCB that relayed the query (0 = direct USB; non-zero lets the
// config tool badge the relay node). includeAliases=false omits names so a
// large network's reply still fits one ESP-NOW packet (structCommand[200]).
// Also lazily fires "?WHOAMI" at online-but-unnamed boards so names fill in.
// Returns the JSON length (clamped to n-1 if it overflowed buf).
// Dynamic WCB roster for the status panel: a board is "known" (shown) if it's
// within the pre-registered floor (wcb_quantity — always listed, even offline),
// OR currently online, OR we've heard its WDP advert (auto-joined beyond the
// floor). This surfaces auto-discovered boards without wcb_quantity having to
// cover them. Self appears only if it falls within the floor (unchanged).
inline bool wcbBoardKnown(int i, int floor) {
  if (i >= 1 && i <= floor) return true;
  if (!wcb) return false;
  // Show a board if it's live (heartbeat) OR advertising (WDP neighbor) OR a
  // PERMANENT learned peer — the last keeps an auto-joined node (esp. a client,
  // which never heartbeats) on the panel even while it's powered off / its WDP
  // advert has aged out, and on a fresh boot where it's restored but not yet heard.
  return wcb->isOnline((uint8_t)i)
      || (wcb->getNeighbor((uint8_t)i) != nullptr)
      || wcb->isLearnedPeer((uint8_t)i);
}
inline int wcbHighestKnown(int floor) {
  int hi = (floor > WCB_MAX_BOARDS) ? WCB_MAX_BOARDS : (floor < 0 ? 0 : floor);
  for (int i = WCB_MAX_BOARDS; i > hi; i--)
    if (wcbBoardKnown(i, floor)) { hi = i; break; }
  return hi;
}

inline size_t buildWcbStatus(char* buf, size_t n, uint8_t relayId, bool includeAliases) {
  if (!buf || n == 0) return 0;
  int q = rcConfig.wcbNetwork.quantity;   // pre-registered floor (still reported as "quantity")
  if (q < 0) q = 0;
  if (q > WCB_MAX_BOARDS) q = WCB_MAX_BOARDS;
  const int selfId = rcConfig.wcbNetwork.deviceId;
  const int hi = wcbHighestKnown(q);       // extend past the floor to cover auto-discovered boards
  size_t len = snprintf(buf, n,
      "{\"type\":\"WCB_STATUS\",\"quantity\":%d,\"self\":%d,\"relay\":%d,\"online\":[",
      q, selfId, (int)relayId);
  for (int i = 1; i <= hi && len < n; i++) {
    const WCBNeighbor* nb = wcb ? wcb->getNeighbor(i) : nullptr;
    const bool client = nb ? nb->isClient : wcbIsClient(i);
    // "live now": heartbeat for a WCB; a fresh WDP advert for a CLIENT (a client
    // never heartbeats, so a present monitor would otherwise always read offline).
    // self is always live.
    const bool up = (i == selfId) ? true
                  : (client ? (nb != nullptr) : (wcb && wcb->isOnline(i)));
    if (i != selfId && !client) {   // ?WHOAMI only to heartbeat boards — a client is named from its advert
      if (up && !_aliasWasUp[i]) _aliasTries[i] = 0;   // came online → re-arm the alias query
      _aliasWasUp[i] = up;
      // Ask for the friendly name only until it's cached AND only a few times —
      // an un-aliased board is fine, we just stop pestering it every poll.
      if (up && wcb && wcbReady && !wcbAlias(i)[0] && _aliasTries[i] < ALIAS_MAX_TRIES) {
        wcb->send((uint8_t)i, "?WHOAMI");
        _aliasTries[i]++;
      }
    }
    len += snprintf(buf + len, n - len, "%s%s", (i > 1) ? "," : "", up ? "1" : "0");
  }
  // known[] tells the tool which slots to actually render (floor + discovered),
  // so gaps above the floor that aren't real boards aren't drawn as offline chips.
  if (len < n) len += snprintf(buf + len, n - len, "],\"known\":[");
  for (int i = 1; i <= hi && len < n; i++)
    len += snprintf(buf + len, n - len, "%s%s", (i > 1) ? "," : "", wcbBoardKnown(i, q) ? "1" : "0");
  if (len < n) len += snprintf(buf + len, n - len, "]");
  // clients[] — 1 = this slot is a client device (mesh monitor / other controller),
  // not a WCB board. Prefer the live WDP neighbor; fall back to the cache so a
  // learned client still tags correctly after its advert has aged out.
  if (len < n) len += snprintf(buf + len, n - len, ",\"clients\":[");
  for (int i = 1; i <= hi && len < n; i++) {
    const WCBNeighbor* nb = wcb ? wcb->getNeighbor(i) : nullptr;
    const bool client = nb ? nb->isClient : wcbIsClient(i);
    len += snprintf(buf + len, n - len, "%s%s", (i > 1) ? "," : "", client ? "1" : "0");
  }
  if (len < n) len += snprintf(buf + len, n - len, "]");
  if (includeAliases) {
    if (len < n) len += snprintf(buf + len, n - len, ",\"aliases\":[");
    for (int i = 1; i <= hi && len < n; i++)
      len += snprintf(buf + len, n - len, "%s\"%s\"", (i > 1) ? "," : "",
                      (i == selfId) ? "" : wcbAlias(i));
    if (len < n) len += snprintf(buf + len, n - len, "]");
  }
  if (len < n) len += snprintf(buf + len, n - len, "}");
  // Return the would-be length (snprintf semantics): a value >= n means the JSON
  // was truncated. The per-statement `len < n` guards above prevent any overrun,
  // so this is used only by the caller to detect overflow (test: l >= bufsize).
  return len;
}

inline bool handle(uint8_t senderID, const char* command) {
  if (!wcb || !wcbReady) return false;   // WCB down — can't be receiving anyway
  if (!command || command[0] != '{') return false;

  // Parse the JSON payload.  Use a doc large enough to hold fragment
  // envelopes (`s` field carries up to FRAG_CHUNK_BYTES of escaped JSON)
  // plus the normal single-packet commands (PING / TRIGGER / SET_MODE).
  StaticJsonDocument<512> doc;
  DeserializationError err = deserializeJson(doc, command);
  if (err) {
    // Not valid JSON — silently ignore.  This is normal traffic (a legacy
    // WCB ;-command from a stage-cue device or another peer) and not an
    // error condition.
    return false;
  }

  // Renew the rc_ch subscription on real inbound COMMANDS — but NOT on raw
  // fragment envelopes. ETM retransmits a fragment until it's ACK'd; if a
  // fragment (or a late retry arriving after the tool already closed) renewed
  // the subscription, rc_ch would keep broadcasting for another full
  // WCB_SUBSCRIPTION_MS past the tool's disconnect, saturating the network. A
  // completed multi-fragment payload still counts as activity: it recurses
  // into handle() as reassembled JSON (which has no "f" field) and renews here.
  if (!doc.containsKey("f")) _lastWcbInbound = millis();

  // ── Fragment envelope ────────────────────────────────────────────────────
  // {"f":<n>,"of":<N>,"sid":<S>,"s":"<chunk>"} — slice of a larger JSON
  // payload.  Buffer by sid; when all N parts arrive, concatenate the `s`
  // fields in order and recurse with the reassembled JSON so the normal
  // dispatch handles it (SET_CONFIG, etc.).
  if (doc.containsKey("f") && doc.containsKey("of") && doc.containsKey("sid")) {
    int f     = doc["f"]   | 0;
    int total = doc["of"]  | 0;
    int sid   = doc["sid"] | 0;
    const char* s = doc["s"] | "";
    if (f < 1 || total < 1 || total > FRAG_MAX_PARTS || f > total || sid == 0) {
      return true;   // malformed envelope — drop silently
    }
    FragSession* sess = _findOrAllocSession((uint16_t)sid, (uint8_t)total, senderID);
    if (!sess) {
      Serial.println("[RC] Fragment pool exhausted — dropping");
      return true;
    }
    // The session's total is fixed by its FIRST fragment. A later envelope for
    // the same sid that disagrees on 'of' is malformed/spoofed — its 'f' could
    // index past the real part set and inflate 'got' to false-complete the
    // session with missing/stale parts. Drop the mismatched fragment.
    if ((uint8_t)total != sess->total) return true;
    // Use the explicit received[] flag rather than parts[].length() so an
    // empty slice (legitimately possible if the sender ever produces one)
    // can't masquerade as "not yet received" on duplicate arrivals.
    bool isNewFragment = !sess->received[f - 1];
    if (isNewFragment) {
      sess->parts[f - 1]    = String(s);
      sess->received[f - 1] = true;
      sess->got++;
      sess->expireAt = millis() + FRAG_TIMEOUT_MS;
    }
    // Visibility — log every fragment receive so the user can see in the
    // RC's serial monitor whether a SET_CONFIG / GET_CONFIG handshake is
    // actually arriving over the WCB bridge.  Gated: this runs on Core 0
    // and is the hottest print on that core (see RC_TELEM_VERBOSE).
    if (RC_TELEM_VERBOSE) {
      Serial.printf("[RC] frag sid=%u f=%d/%d got=%d%s\n",
                    (unsigned)sid, f, total, sess->got,
                    isNewFragment ? "" : " (dup)");
    }
    // Complete only when EVERY part 0..total-1 is actually present — don't trust
    // the got counter alone to decide reassembly is ready.
    bool complete = sess->got >= sess->total;
    if (complete)
      for (uint8_t i = 0; i < sess->total; i++)
        if (!sess->received[i]) { complete = false; break; }
    if (complete) {
      // Reassemble — concatenate all parts in order.
      String full;
      full.reserve(sess->total * FRAG_CHUNK_BYTES);
      for (uint8_t i = 0; i < sess->total; i++) full += sess->parts[i];
      uint8_t fromId = sess->senderID;
      *sess = FragSession{};   // free the slot before stashing
      Serial.printf("[RC] frag sid=%u COMPLETE, deferring %u bytes to main loop\n",
                    (unsigned)sid, (unsigned)full.length());
      // ── DEFER to main loop ────────────────────────────────────────────
      // Do NOT recurse into handle() here.  We're in the WCB_Client
      // receive-callback chain (WiFi-task context on Core 0); doing the
      // 1–4 KB JSON parse + rcConfigFromJSON + rcConfigSaveLFS inline
      // blocks the WiFi task long enough to trip Core 0's interrupt
      // watchdog.  Stash the payload; tick() picks it up from loop()
      // where blocking flash I/O is safe.  If a second large message
      // arrives before the first one applies, we drop the new one —
      // SET_CONFIG isn't expected to fire faster than ~1 Hz.
      // Mutex-guarded write — see _pendingMutex declaration for why.
      // We assign the String inside the critical section so a concurrent
      // tick() can't observe a partial buffer.
      bool accepted = false;
      if (_pendingMutex && xSemaphoreTake(_pendingMutex, portMAX_DELAY) == pdTRUE) {
        if (_pendingReassembled.length() == 0) {
          _pendingReassembled       = full;
          _pendingReassembledSender = fromId;
          accepted = true;
        }
        xSemaphoreGive(_pendingMutex);
      }
      if (!accepted) {
        Serial.println("[RC] previous reassembled payload still pending — dropping new one");
      }
    }
    return true;
  }

  const char* type = doc["type"] | "";
  if (!type[0]) return false;

  // Filter out our own broadcasts echoing back — `id` field on outbound
  // telemetry equals our deviceId; if we see that, it's something WE sent.
  // (Optional defensive check — WCB stack usually filters self-echoes.)
  if (doc["id"].is<int>() &&
      (uint8_t)(doc["id"] | 0) == rcConfig.wcbNetwork.deviceId &&
      type[0] == 'r' && type[1] == 'c' && type[2] == '_') {
    return false;
  }

  // ── PING ─────────────────────────────────────────────────────────────────
  // Unicast a PONG back to the sender, then force an immediate heartbeat +
  // channel burst on the next tick so the asker gets a full state snapshot.
  if (!strcmp(type, "PING")) {
    char buf[180];
    snprintf(buf, sizeof(buf),
      "{\"type\":\"PONG\",\"id\":%u,\"version\":\"%s\",\"model\":%u,\"mode\":%d}",
      rcConfig.wcbNetwork.deviceId, FW_VERSION,
      rcConfig.txModel, FunctionSwState);
    wcb->send(senderID, buf);
    _lastHb = 0;   // force re-broadcast
    _lastCh = 0;
    return true;
  }

  // ── wcb_alias ─────────────────────────────────────────────────────────────
  // A WCB's reply to our "?WHOAMI": cache its friendly name so GET_WCB_STATUS
  // can label it in the config tool. id = the replying board's WCB number.
  if (!strcmp(type, "wcb_alias")) {
    setWcbAlias(doc["id"] | 0, doc["alias"] | "");   // range-checks + sanitizes
    return true;
  }

  // ── TRIGGER ──────────────────────────────────────────────────────────────
  // Same shape the config tool already sends over Web Serial. It must reach the
  // SAME rcDispatch path as a local matrix press — but handle() runs in the
  // ESP-NOW receive callback on Core 0, and rcDispatch runs the full action
  // chain (Maestro Serial2 writes, HCR/MP3 aux-serial, WCB sends, speed/accel
  // caches) which ALSO runs from processSbus() on Core 1. Two cores hitting the
  // same UART/caches with no lock interleaves bytes and corrupts frames — the
  // exact cross-core hazard every other heavy handler here defers. So enqueue
  // and let drainRemoteTriggers() (loop, Core 1) dispatch it: remote and local
  // triggers then share the single-core dispatch path the Maestro / tap-timing /
  // record-replay subsystems assume. Dispatch-side telemetry still fires exactly
  // once, indistinguishable from a local press.
  if (!strcmp(type, "TRIGGER")) {
    int mode = doc["mode"] | 0;
    int btn  = doc["btn"]  | 0;
    int tap  = doc["tap"]  | 1;
    if (mode >= 1 && mode <= 3 &&
        btn  >= 1 && btn  <= RC_NUM_THRESHOLDS) {
      if (tap < 1) tap = 1; else if (tap > 3) tap = 3;
      queueRemoteTrigger(mode, btn, (uint8_t)tap);   // defer to Core 1 (see above)
    }
    return true;
  }

  // ── SET_MODE ─────────────────────────────────────────────────────────────
  // Forces FunctionSwState immediately.  NOTE: the next SBUS frame's
  // mode-switch decode will overwrite this — so SET_MODE is most useful
  // when the user has the modeSwitch parked at a steady position, or
  // for transient testing.  Persistent mode override would need a separate
  // "muted modeSwitch" config flag — out of scope for Phase 1.
  if (!strcmp(type, "SET_MODE")) {
    int mode = doc["mode"] | 0;
    if (mode >= 1 && mode <= 3 && mode != FunctionSwState) {
      FunctionSwState = mode;
      resetModeAwareKnobs();   // snap mode-aware knob servos to the stick in the new mode
      emitMode(mode);
    }
    return true;
  }

  // ── GET_CONFIG ────────────────────────────────────────────────────────────
  // Just record the requester here (we're in the WiFi-task callback context,
  // Core 0).  tick() on the main loop (Core 1) arms a non-blocking fragment
  // send and ships one fragment per iteration — see the OutboundSend state
  // machine.  The single _pendingGetConfigSender slot is the "busy" guard:
  // it stays set for the whole send and is cleared by the pump on
  // completion, so a second GET_CONFIG arriving mid-send is rejected (the
  // requester re-issues if its own 5 s reassembly window lapses).
  //
  // We deliberately guard on ONLY _pendingGetConfigSender here, NOT on
  // _outSend.active.  _outSend is a struct containing an Arduino String and
  // is owned exclusively by Core 1 (tick/pump write and reset it); reading
  // it from this Core-0 callback would be an unsynchronized cross-core read
  // of a non-atomic object.  Because _pendingGetConfigSender (an atomic
  // uint8_t) remains set for the entire duration that _outSend.active is
  // true, gating on it alone is equivalent AND keeps _outSend strictly
  // single-core.
  if (!strcmp(type, "GET_CONFIG")) {
    if (_pendingGetConfigSender == 0) {
      _pendingGetConfigSender = senderID;
      Serial.printf("[RC] GET_CONFIG → queued for W%u, will send from main loop\n",
                    (unsigned)senderID);
    } else {
      Serial.println("[RC] GET_CONFIG dropped — a CONFIG response is already pending/sending");
    }
    return true;
  }

  // ── SET_CONFIG ────────────────────────────────────────────────────────────
  // Always defer to main loop — rcConfigFromJSON + rcConfigSaveLFS block
  // 100+ ms on flash I/O and we're in the WCB_Client receive-callback
  // chain (WiFi-task on Core 0).  Stash the WHOLE message (caller's JSON
  // string, not our 512-byte truncated parse) so tick() can re-parse with
  // a DynamicJsonDocument sized for the real payload.  This branch fires
  // for a single-packet SET_CONFIG too (rare but possible if the config
  // ever shrinks below the ~200 byte threshold).
  if (!strcmp(type, "SET_CONFIG")) {
    // Mutex-guarded — same reasoning as the fragment-complete branch.
    bool accepted = false;
    if (_pendingMutex && xSemaphoreTake(_pendingMutex, portMAX_DELAY) == pdTRUE) {
      if (_pendingReassembled.length() == 0) {
        _pendingReassembled       = String(command);
        _pendingReassembledSender = senderID;
        accepted = true;
      }
      xSemaphoreGive(_pendingMutex);
    }
    if (accepted) {
      Serial.println("[RC] SET_CONFIG → deferred to main loop");
    } else {
      Serial.println("[RC] SET_CONFIG dropped — apply queue full");
    }
    return true;
  }

  // ── GET_WCB_STATUS — answer it over the WCB bridge too ─────────────────
  // Direct USB builds this same WCB_STATUS payload in NaviCore.ino; when the
  // config tool is bridged through a WCB the query arrives here instead, so
  // build the reply and unicast it back to the relaying WCB (senderID).  We
  // tag senderID as "relay" so the tool can badge that node, and fall back to
  // a names-less reply if a big network wouldn't fit one ESP-NOW packet.
  if (!strcmp(type, "GET_WCB_STATUS")) {
    char wbuf[200];
    size_t l = buildWcbStatus(wbuf, sizeof(wbuf), senderID, true);
    if (l >= sizeof(wbuf))                           // names overflowed one packet (would-be len >= buf size)
      buildWcbStatus(wbuf, sizeof(wbuf), senderID, false);
    if (wcb) wcb->send(senderID, wbuf);
    return true;
  }

  // ── Direct-USB-only commands that mean nothing over the WCB transport ──
  // The config tool fires these on every connect/poll cycle (live-monitor
  // start/stop, debug-flag bitmask, calibration mute toggle).  Over USB they
  // have local effects (gating PWM_UPDATE emission, etc.).  Over WCB the
  // equivalent live-state already streams via rc_hb / rc_ch broadcasts, so
  // just silently accept these so the config tool's connect sequence runs
  // cleanly without filling the RC's serial log with "Unknown inbound type"
  // noise on every PING.
  if (!strcmp(type, "START_MONITOR")  || !strcmp(type, "STOP_MONITOR") ||
      !strcmp(type, "SET_DEBUG_FLAGS") || !strcmp(type, "CALIB")) {
    return true;
  }

  // ── REBOOT (JSON form — mirror of the ;<id>,r short-form above) ──────
  if (!strcmp(type, "REBOOT")) {
    Serial.println("[RC] Remote REBOOT requested via WCB");
    delay(100);
    ESP.restart();
    return true;
  }

  // ── FORGET_PEER — drop a learned (auto-joined) peer. {"id":N} forgets one,
  //    {"all":true} forgets all. Deferred to Core 1 (drainForgetPeer) because
  //    forgetPeer/clearLearnedPeers do esp_now_del_peer + an NVS write that can't
  //    run on this Core-0 ESP-NOW callback stack.
  if (!strcmp(type, "FORGET_PEER")) {
    bool all = doc["all"] | false;
    int  id  = doc["id"]  | 0;
    if (all)                                 queueForgetPeer(0);
    else if (id >= 1 && id <= WCB_MAX_BOARDS) queueForgetPeer((uint8_t)id);
    return true;
  }

  // Unknown JSON type — log and don't consume so the caller can keep
  // logging it for visibility.
  Serial.printf("[RC] Unknown inbound type '%s' from WCB%d\n", type, senderID);
  return false;
}

}   // namespace rcTelemetry
