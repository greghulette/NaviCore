// =============================================================================
//  navicore_wsserver.h — WebSocket command endpoint over the optional SoftAP
// =============================================================================
//
//  A second mouth for the SAME protocol the USB serial port speaks. Every line
//  that arrives here goes to processInputLine(), the identical dispatcher
//  handleSerialInput() feeds, so there is exactly one implementation of the
//  command surface and the two transports cannot drift.
//
//  Deliberately NOT a web server. The config tool is ~1.17 MB and the app slot
//  has ~840 KB free, so hosting the UI here was never possible. This serves one
//  WebSocket and nothing else, which is also why it costs ~33 KB instead of the
//  ~120 KB a static-file server would.
//
//  ── THE CONCURRENCY RULE THIS FILE EXISTS TO OBEY ──────────────────────────
//  The httpd task is NOT the loop task. Its handler runs on Core 0 alongside
//  the ESP-NOW receive callback, and the same prohibition applies: nothing
//  touching flash, NVS, or droid hardware may run there. processInputLine()
//  does all three — SET_CONFIG writes LittleFS and NVS, TRIGGER drives servos
//  and bit-banged SoftwareSerial.
//
//  So the handler does the minimum: copy the frame, enqueue, return. drain()
//  runs the command from loop() on Core 1. This is the same split
//  drainRemoteCli() already uses for mesh-relayed CLI lines, for the same
//  reason, and it is why the reply comes back through httpd_ws_send_frame_async
//  (the API documented for sends "out of the scope of current request") rather
//  than the in-request send.
//
//  ── WHY A POINTER QUEUE, NOT A VALUE QUEUE ─────────────────────────────────
//  RemoteCliMsg carries char cmd[200] by value because a mesh payload cannot
//  exceed 187 B. There is no such cap here: a SET_CONFIG line approaches 98 KB
//  (see the serialInputBuf cap in handleSerialInput). Copying that into a queue
//  slot is impossible, so the handler allocates from PSRAM and drain() frees.
//  Ownership transfers with the pointer — if the queue send fails, the HANDLER
//  frees it, because drain() will never see it.
// =============================================================================
#pragma once

#include <esp_http_server.h>
#include <WiFi.h>
#include <lwip/sockets.h>   // close() - the session-close hook must close the socket itself
#include <errno.h>          // wsSendAll tells a send timeout from a dead socket
#include "rc_serial.h"   // rcSerial — the capture tee this borrows

// Defined in NaviCore.ino. Declared here because this header is included near the
// top of the sketch, long before the definition, and Arduino's auto-prototyping
// only covers the .ino itself.
bool processInputLine(const String& line);

namespace naviws {

// One command in flight at a time is the honest ceiling: rcSerial's capture tee
// is a single slot (rc_serial.h), so two commands cannot have their output
// separated anyway. The depth only has to absorb a burst.
//
// Depth 3 did not. drain() runs ONE command per loop() pass, so a client that put
// several lines in a single frame overran it immediately: measured at 3 of 6 PINGs
// answered, with the other three discarded in the handler and NOTHING sent back.
// Silent loss is the worst possible failure here — the client waits forever for a
// reply to a command the board already threw away.
static const uint8_t  WS_QUEUE_DEPTH = 8;
// How long the handler will wait for room rather than discard a command. This is
// the httpd task on Core 0, NOT the loop task, so blocking here costs a little
// latency on other sockets and nothing on SBUS. It converts a silent drop into
// ordinary backpressure, which is what TCP is for.
static const uint32_t WS_ENQUEUE_WAIT_MS = 50;
// Flush the reply in ~MSS-sized pieces. A GET_CONFIG reply is tens of KB; one
// frame per output line would be hundreds of TCP writes, and buffering the whole
// thing would need another 98 KB of PSRAM for no benefit.
static const size_t   WS_TX_CHUNK    = 1400;
// Keep in step with httpd_config_t::max_open_sockets in begin(). More clients than
// the server will accept is just dead slots; fewer means a connected client the sink
// never writes to, which is the silent-deafness bug this array exists to prevent.
static const uint8_t  WS_MAX_CLIENTS = 3;
// How long a client's socket may take NO byte before NaviCore gives up on it and shuts
// it down (wsSendAll). Not shorter: a WiFi hiccup stalls a healthy link for seconds - 3 s
// with nothing moving, measured on the bench in HIL ncwifi.ws_ping_soak - and a 1 s bound
// closed that healthy client. 5 s is the per-send timeout httpd shipped with, which that
// soak always survived; it is also short enough that one stalled client holds the httpd
// task (and so every other client) about 5 s once, then never again.
static const uint32_t WS_STALL_MS    = 5000;
// The outbound queue (wsTxHead): a line that STARTS while it holds this many bytes is
// dropped whole (WsSink::write's admission). Sized well past what a stall can pile up:
// the monitor adds ~13 KB a second, and a stalled client holds the queue at most about
// WS_STALL_MS before it is shut down, so this is ~20 s of it - only clients that take
// nothing for that long see a line refused, and a reply to a healthy client never is. A
// line already started is always queued whole, past the cap if need be. PSRAM (8 MB).
static const size_t   WS_TX_MAX_QUEUED = 256 * 1024;
// Frames one wsDrainWork run sends before it re-queues itself, so httpd serves its
// sockets (a client's PING, its next command) between runs of a long backlog.
static const uint8_t  WS_DRAIN_BATCH   = 8;
// A wsDrainWork queued this long ago that has not started was lost (httpd's control
// socket drops what it cannot hold, without an error); wsKick then queues another.
static const uint32_t WS_KICK_LOST_MS  = 2000;

struct WsCmd {
  int   fd;      // client socket, for the async reply
  char* line;    // ps_malloc'd, owned by whoever holds this struct
};

inline QueueHandle_t  wsQueue  = nullptr;
inline httpd_handle_t wsServer = nullptr;
// True only while drain() runs a line that came in on a socket - the transport a
// handler can ask about. NOT the capture tee: that stays armed for a client's whole
// session (below), so it says "a client is connected", never "this line came from one".
inline bool           wsLineRunning = false;
inline bool lineFromSocket() { return wsLineRunning; }

// ── Reply sink ──────────────────────────────────────────────────────────────
// Print sink armed around processInputLine() so everything the command prints to
// Serial is ALSO shipped to the WebSocket client. Same trick navicore_rterm.h
// uses for the mesh terminal — the command handlers stay transport-unaware and
// keep printing to Serial exactly as they always have.
// The capture stays ARMED for as long as a client is connected, not just around a
// command. It has to: PWM_UPDATE (the live monitor, ~20 Hz), WCB_STATUS pushes and
// every terminal line are emitted from loop(), OUTSIDE any command. Arming only
// around processInputLine() gives working request/response and a completely dead
// monitor — no SBUS, no button presses, no stick movement — which is most of what
// the tool is for.
//
// BUFFER, THEN SEND FROM pump(). Never send inside write(). write() runs in the
// middle of arbitrary Serial.printf calls on the loop task, and a TCP send there
// would put network I/O between two halves of a print — on the core that has to
// service SBUS at ~111 fps. Buffering here and flushing at one known point in
// loop() keeps the blocking where it can be reasoned about.
//
// A LINE GOES OUT WHOLE OR NOT AT ALL. Truncating mid-line would hand the tool half
// a JSON object, which fails at JSON.parse and (per the tool's own notes) silently
// freezes a panel. Dropping a whole line loses a sample instead, which for 20 Hz
// telemetry is invisible. See the admission in write().

// One frame of the outbound stream. Queued on wsTxHead/wsTxTail by pump() on the loop
// task; sent and freed by wsDrainWork() on the httpd task. `fds` are the clients that
// were connected when its bytes were written.
struct WsTx {
  WsTx*  next;
  char*  data;
  size_t len;
  int    fds[WS_MAX_CLIENTS];
};

// ── The outbound queue: OURS, drained by ONE httpd work item at a time ──────
// pump() used to hand httpd one work item per frame. httpd's control socket holds 6
// (CONFIG_LWIP_UDP_RECVMBOX_SIZE) and drops the rest without an error
// (CONFIG_HTTPD_QUEUE_WORK_BLOCKING is off in core 3.3.4), so any burst the httpd task
// could not keep up with - a reply over ~6 frames, or anything written while a send was
// slow - lost frames out of its middle, each lost item leaking its PSRAM copy, and
// httpd's own control messages could be lost with them. Holding the bytes back instead
// (a 2 KB buffer behind a slow send) cut GET_CONFIG at 2048 characters on a healthy
// socket (HIL ncwifi.ws_parity). Now frames wait here, in PSRAM, for as long as they
// need, and at most one wsDrainWork item of ours is ever in httpd's queue.
// The list and the flags are shared by the loop task (pump, wsKick) and the httpd task
// (wsDrainWork, wsKick): every access holds wsTxMux, and nothing inside it calls a
// socket function or allocates.
inline portMUX_TYPE wsTxMux        = portMUX_INITIALIZER_UNLOCKED;
inline WsTx*        wsTxHead       = nullptr;
inline WsTx*        wsTxTail       = nullptr;
inline size_t       wsTxQueued     = 0;       // bytes in the list, frames being sent included
inline bool         wsKickPending  = false;   // a wsDrainWork is in httpd's queue, not yet started
inline bool         wsDrainRunning = false;   // a wsDrainWork is running (it re-queues itself)
inline uint32_t     wsKickAtMs     = 0;       // when the pending one was queued
inline uint32_t     wsDroppedLines = 0;       // lines refused whole (loop task); drain() says so on USB

// Defined below WsSink: they need the sink to drop a dead fd.
void wsDrainWork(void* arg);
void wsKick();

// Free every queued frame. ONLY on the httpd task, and only while no client is live:
// then nothing writes to the queue (write() returns at !live()) and no drain is running
// (drains run on this same task). WsSink::begin() calls it as the first client of a new
// session arrives, so a frame written for a session that has ended is never delivered to
// a newcomer that happens to get one of its socket numbers back.
inline void wsTxDiscard() {
  portENTER_CRITICAL(&wsTxMux);
  WsTx* tx   = wsTxHead;
  wsTxHead   = nullptr;
  wsTxTail   = nullptr;
  wsTxQueued = 0;
  portEXIT_CRITICAL(&wsTxMux);
  while (tx) { WsTx* next = tx->next; free(tx->data); free(tx); tx = next; }
}

class WsSink : public Print {
 public:
  // MULTIPLE CLIENTS, not one.
  //
  // A single fd meant every new connection silently STOLE the stream from the
  // existing one: the old client's TCP socket stayed open and healthy while it went
  // permanently deaf, so it showed "connected" and waited forever. Any second
  // window, any diagnostic probe, any reconnect racing its own predecessor would do
  // it — and the victim had no way to tell.
  //
  // Sized to httpd's max_open_sockets so we can never track more clients than the
  // server will hold.
  void begin(int fd) {
    for (int i = 0; i < WS_MAX_CLIENTS; i++) if (_fds[i] == fd) return;   // already known
    // FIRST client after a quiet period: start from a clean sheet. _dropping (a line
    // being refused) can be left set as the last client leaves mid-line - and nothing
    // cleared it again, because write() returns at !live() BEFORE reaching the reset,
    // and end() has no caller. It therefore survived the disconnect and ate the first
    // whole line the next client was sent. Usually that is one 20 Hz telemetry frame and
    // invisible; when it is the ~14 KB GET_CONFIG reply the tool comes up with no config
    // at all, which reads as "the droid lost my settings". Frames still queued for the
    // ended session go too (wsTxDiscard). Only on the transition to live: clearing while
    // another client is already connected would discard output buffered for it.
    if (!live()) {
      _len = 0; _dropping = false; _atLineStart = true; _lineQueued = false; _lineStartInBuf = 0;
      wsTxDiscard();
    }
    for (int i = 0; i < WS_MAX_CLIENTS; i++) if (_fds[i] < 0) { _fds[i] = fd; return; }
    _fds[0] = fd;   // full: evict the oldest rather than refuse the newcomer
  }
  void drop(int fd) { for (int i = 0; i < WS_MAX_CLIENTS; i++) if (_fds[i] == fd) _fds[i] = -1; }
  bool has(int fd) const { for (int i = 0; i < WS_MAX_CLIENTS; i++) if (_fds[i] == fd) return true; return false; }
  void end() { for (int i = 0; i < WS_MAX_CLIENTS; i++) _fds[i] = -1; _len = 0; _dropping = false; _atLineStart = true; _lineQueued = false; _lineStartInBuf = 0; }
  bool live() const {
    for (int i = 0; i < WS_MAX_CLIENTS; i++) if (_fds[i] >= 0) return true;
    return false;
  }

  size_t write(uint8_t c) override {
    if (!live()) return 1;
    if (_atLineStart) {
      // ADMISSION: ONCE PER LINE, AT ITS FIRST BYTE, never again mid-line. A line that
      // starts while the outbound queue holds WS_TX_MAX_QUEUED is dropped whole, and
      // counted for drain()'s note on USB. Any other line is queued whole, however long
      // it is and however far past the cap it takes the queue - GET_CONFIG's ~14 KB, a
      // cmdlib, a clip list - so a slow client still gets every reply it can take, and
      // nothing is ever cut in the middle by a full queue.
      _atLineStart    = false;
      _lineQueued     = false;
      _lineStartInBuf = _len;
      if (wsTxQueued >= WS_TX_MAX_QUEUED) { _dropping = true; wsDroppedLines++; }
    }
    if (_dropping) { if (c == '\n') { _dropping = false; _atLineStart = true; } return 1; }
    // FULL: move the buffer into the outbound queue and go on. A line longer than the
    // buffer - a config, a cmdlib - leaves in buffer-sized frames, every one queued.
    if (_len >= sizeof(_buf) && !_stage()) {
      // ...unless there is no PSRAM for the frame - never seen: the queue is capped far
      // below it. This line can then no longer go out whole, so drop what of it is still
      // here and the rest of it, and if its head was already queued, end that head with
      // a newline so no client gets it glued to the next line: the one way a line can
      // still arrive cut.
      _len = _lineStartInBuf;
      if (_lineQueued && _len < sizeof(_buf)) _buf[_len++] = '\n';
      _dropping = true; wsDroppedLines++;
      return 1;
    }
    _buf[_len++] = (char)c;
    if (c == '\n') _atLineStart = true;
    return 1;
  }
  size_t write(const uint8_t* b, size_t n) override {
    for (size_t i = 0; i < n; i++) write(b[i]);
    return n;
  }
  using Print::write;

  // Called from loop() (and from write() when the buffer fills). Broadcasts to
  // every connected client; one that has gone away is dropped individually rather
  // than taking the others down with it. Returns false only when nobody is left.
  // Sends are MARSHALLED ONTO THE HTTPD TASK, never issued from here.
  //
  // esp_http_server answers a client's PING with a PONG from its own task, on
  // Core 0. pump() runs on Core 1 (loop). Nothing serialises the two, so a PONG
  // and a data frame could be written to the same TCP socket at once and their
  // bytes interleaved -- the client then reads a frame header out of the middle
  // of a payload and kills the connection with 1002 "invalid opcode".
  //
  // Measured: with client pings on, the link died every ~90 s; with pings off,
  // zero drops in 5.5 minutes. A read-only raw client that never pinged (so the
  // droid never ponged) ran 7,393 frames / 2.5 MB clean through the same window
  // in which the pinging client dropped three times.
  //
  // Turning pings off is not the fix -- ANY compliant client may ping, and the
  // keepalive is what detects a vanished AP in ~3 s instead of ~34 s.
  // wsDrainWork() runs the sends on the httpd task, so every write to the socket
  // (ours and the server's own control frames) is issued by one task.
  bool pump() {
    if (!wsServer) return live();
    _stage();   // a frame PSRAM cannot take stays buffered for the next pass
    wsKick();
    return live();
  }

 private:
  // Move the buffered bytes onto the outbound queue as one frame. False only when
  // PSRAM has no room for it; the bytes then stay in _buf.
  bool _stage() {
    if (!_len) return true;
    // NEVER CUT THROUGH A UTF-8 SEQUENCE. This is a TEXT frame, and RFC 6455 8.1
    // requires each one to be valid UTF-8 on its own. The buffer is flushed on a
    // BYTE count (write() at _len >= sizeof(_buf), and flushHook at arbitrary
    // points), so a multi-byte character in a servo or sound name straddles the
    // boundary and the frame ends in a bare continuation byte. A conforming client
    // does not tolerate that - Python's `websockets` fails the connection with 1007,
    // the reconnect replays the same bytes at the same alignment, and the link dies
    // in a loop that looks like flaky WiFi. Hold the partial character back for the
    // next frame instead; it is at most 3 bytes.
    size_t send = _utf8SafeLen(_buf, _len);
    if (!send) send = _len;          // cannot improve it - send rather than stall

    // The frame owns its own copy: _buf is reused the moment this returns, and the
    // send happens later on the other task.
    WsTx* tx = (WsTx*)ps_malloc(sizeof(WsTx));
    char* copy = tx ? (char*)ps_malloc(send) : nullptr;
    if (!tx || !copy) {
      if (copy) free(copy);
      if (tx) free(tx);
      return false;
    }
    memcpy(copy, _buf, send);
    tx->next = nullptr;
    tx->data = copy;
    tx->len  = send;
    for (int i = 0; i < WS_MAX_CLIENTS; i++) tx->fds[i] = _fds[i];
    portENTER_CRITICAL(&wsTxMux);
    if (wsTxTail) wsTxTail->next = tx; else wsTxHead = tx;
    wsTxTail    = tx;
    wsTxQueued += send;
    portEXIT_CRITICAL(&wsTxMux);

    const size_t left = _len - send;
    if (left) memmove(_buf, _buf + send, left);
    _len = left;
    // Where the unfinished line starts, and whether its head is now queued (write()).
    if (_lineStartInBuf >= send) _lineStartInBuf -= send;
    else { _lineQueued = true; _lineStartInBuf = 0; }
    return true;
  }

  // Longest prefix of b[0..n) that does not end part-way through a UTF-8 sequence.
  // Returns n when the tail is already complete, which is the overwhelmingly common
  // case (pure ASCII), so this costs a couple of compares per flush.
  static size_t _utf8SafeLen(const char* b, size_t n) {
    if (!n) return 0;
    size_t i = n, back = 0;
    while (i > 0 && back < 3 && ((uint8_t)b[i - 1] & 0xC0) == 0x80) { i--; back++; }
    if (i == 0) return n;                   // nothing but continuations: not UTF-8
    const uint8_t lead = (uint8_t)b[i - 1];
    size_t need = 1;
    if      ((lead & 0x80) == 0x00) need = 1;
    else if ((lead & 0xE0) == 0xC0) need = 2;
    else if ((lead & 0xF0) == 0xE0) need = 3;
    else if ((lead & 0xF8) == 0xF0) need = 4;
    else                            need = 1;   // stray continuation: treat as done
    return ((n - (i - 1)) >= need) ? n : i - 1;
  }

  int    _fds[WS_MAX_CLIENTS] = { -1, -1, -1 };
  size_t _len      = 0;
  bool   _dropping = false;        // dropping the current line, to its newline
  bool   _atLineStart    = true;   // the next byte starts a line (admission is decided then)
  bool   _lineQueued     = false;  // part of the current line is already on the queue
  size_t _lineStartInBuf = 0;      // where the current line begins in _buf
  // One PWM_UPDATE is ~640 B and they arrive at ~20 Hz; loop() pumps far faster
  // than that, so this only has to absorb one busy pass, not a backlog.
  char   _buf[2048];
};

inline WsSink wsSink;

// The core drain() runs on, i.e. the loop task. Recorded there and used to gate
// the direct emitters below.
//
// It deliberately does NOT ask rcSerial whether its capture is armed: the tee is
// disarmed and re-armed constantly (drainRemoteCli() disarms unconditionally),
// and printlnDirect() exists precisely to be independent of that state. Gating on
// "is the tee armed right now" silently dropped every PWM_UPDATE whose loop pass
// happened to fall on the disarmed side -- measured as a completely dead monitor.
inline int wsLoopCore = -1;

// Make sure a wsDrainWork is coming while the outbound queue holds frames. Called by
// pump() (loop task) and by wsDrainWork as it ends (httpd task). Queues nothing while a
// drain runs - that drain re-queues itself if it leaves frames - or while one is already
// queued, unless it was queued WS_KICK_LOST_MS ago and never started: then httpd's
// control socket dropped it, and another is the only way the queue moves again.
inline void wsKick() {
  if (!wsServer) return;
  const uint32_t now = millis();
  bool kick = false;
  portENTER_CRITICAL(&wsTxMux);
  if (wsTxHead && !wsDrainRunning &&
      (!wsKickPending || (uint32_t)(now - wsKickAtMs) >= WS_KICK_LOST_MS)) {
    wsKickPending = true;
    wsKickAtMs    = now;
    kick          = true;
  }
  portEXIT_CRITICAL(&wsTxMux);
  if (kick && httpd_queue_work(wsServer, wsDrainWork, nullptr) != ESP_OK) {
    portENTER_CRITICAL(&wsTxMux);
    wsKickPending = false;                     // refused outright: the next pump() asks again
    portEXIT_CRITICAL(&wsTxMux);
  }
}

// One frame to every client it was written for that is still connected. RUNS ON THE
// HTTPD TASK. Every socket write goes through here, which is the whole point: the
// server's own control frames (a PONG answering a client PING) are issued by this same
// task, so they can no longer interleave with ours.
inline void wsSendFrame(WsTx* tx) {
  if (wsServer && tx->len) {
    httpd_ws_frame_t f = {};
    f.final   = true;
    f.type    = HTTPD_WS_TYPE_TEXT;
    f.payload = (uint8_t*)tx->data;
    f.len     = tx->len;
    for (int i = 0; i < WS_MAX_CLIENTS; i++) {
      if (tx->fds[i] < 0) continue;
      // Skip a client an EARLIER frame already dropped: frames queued while it held the
      // task still list its fd, and each would block on it again for the full stall
      // bound, holding every other client up behind a socket we know is dead.
      if (!wsSink.has(tx->fds[i])) continue;
      if (httpd_ws_send_frame_async(wsServer, tx->fds[i], &f) != ESP_OK) {
        // That client is gone or stalled: stop writing to it AND end its session. Only
        // dropping it left the socket open - it could still send lines that ran, and
        // never received another, since only a handshake adds a client. Ended, it sees
        // its connection close and can reconnect. wsSendAll has already shut a stalled
        // socket down; a send that failed any other way is shut down here.
        //
        // shutdown(), NEVER httpd_sess_trigger_close(). That queues the close as a work
        // item on httpd's control socket, which holds 6 and drops the rest without an
        // error - and it is full exactly when a client has stalled the httpd task. The
        // lost close left the stalled client open and deaf: its lines ran, and no reply
        // ever reached it (HIL ncwifi.ws_stalled_client, run 20261004-213430). httpd's
        // select() sees the shut socket and deletes the session (close_fn = wsClose).
        wsSink.drop(tx->fds[i]);
        shutdown(tx->fds[i], SHUT_RDWR);
      }
    }
  }
}

// RUNS ON THE HTTPD TASK, queued by wsKick. Sends up to WS_DRAIN_BATCH frames in order,
// then, if any are left, queues itself again rather than looping on: httpd serves its
// sockets (a client's PING, its next command) between the runs of a long backlog. A
// frame stays counted in wsTxQueued until it is sent, so write()'s admission sees it.
inline void wsDrainWork(void* arg) {
  (void)arg;
  portENTER_CRITICAL(&wsTxMux);
  wsKickPending  = false;
  wsDrainRunning = true;
  portEXIT_CRITICAL(&wsTxMux);
  for (uint8_t n = 0; n < WS_DRAIN_BATCH; n++) {
    portENTER_CRITICAL(&wsTxMux);
    WsTx* tx = wsTxHead;
    if (tx) { wsTxHead = tx->next; if (!wsTxHead) wsTxTail = nullptr; }
    portEXIT_CRITICAL(&wsTxMux);
    if (!tx) break;
    wsSendFrame(tx);
    portENTER_CRITICAL(&wsTxMux);
    wsTxQueued -= tx->len;
    portEXIT_CRITICAL(&wsTxMux);
    free(tx->data);
    free(tx);
  }
  portENTER_CRITICAL(&wsTxMux);
  wsDrainRunning = false;
  portEXIT_CRITICAL(&wsTxMux);
  wsKick();   // frames left, or written meanwhile: come back after httpd's own turn
}

// Deliver a line to a connected WebSocket client WITHOUT going through Serial.
//
// For the hot-path emitters that are gated on Serial.availableForWrite(): PWM_UPDATE,
// wcbStreamLog(), vlogf(). That guard is right and must stay — an unguarded USB write
// blocks up to HWCDC's 50 ms tx timeout and starves the SBUS decode in loop(). But it
// asks "can USB take this?" when the real question is "can the DESTINATION take this?",
// and with no USB host attached (every WiFi session) availableForWrite() is 0, so the
// line is dropped before it can even reach the capture tee.
//
// That is why command replies worked over the WebSocket while the live monitor was
// completely dead: replies are unguarded Serial.println, PWM_UPDATE is not.
//
// Non-blocking by construction — the sink only appends to its buffer, and drops whole
// lines rather than truncating when full, so this can never stall loop().
inline void printlnDirect(const char* s) {
  // Only the loop core may touch the sink buffer -- the same rule rc_serial.h's
  // tee enforces, for the same reason: a Core-0 (WiFi / ESP-NOW) caller would race
  // the single-threaded buffer. Gated on the CORE, not on the tee's armed state.
  if (!wsSink.live() || (wsLoopCore >= 0 && xPortGetCoreID() != wsLoopCore)) return;
  wsSink.write((const uint8_t*)s, strlen(s));
  wsSink.write((uint8_t)'\n');
}

// As above, but writes EXACTLY n bytes and appends nothing. For callers whose
// text already carries its own newline -- vlogf()'s format strings do, and a
// second one would show as a blank line in the terminal for every debug message.
inline void writeDirect(const char* s, size_t n) {
  if (!n || !wsSink.live() || (wsLoopCore >= 0 && xPortGetCoreID() != wsLoopCore)) return;
  wsSink.write((const uint8_t*)s, n);
}

inline bool clientConnected() { return wsSink.live(); }

// Registered with rcSerial.armCapture() so Serial.flush() reaches this sink.
// Callers flush before something stalls the CPU for seconds (the OTA slot erase),
// and a sink that waits for loop() to drain would never get that chance — loop()
// is precisely what is about to stop running.
inline void flushHook() { wsSink.pump(); }

// -- Per-client line accumulators --------------------------------------------
// ONE PER SOCKET. A single shared accumulator lets any client corrupt any other's
// command, and the interleaving is not rare - the config tool splits every line
// over 512 B into several frames, so an OTA DATA line sits half-accumulated for
// milliseconds at a time. A discovery PING landing in that window fuses into the
// middle of the base64 payload. Measured on hardware: BOTH lines were destroyed,
// which is "[OTA] DATA base64 error -44" arriving by a second route.
//
// A client that closed mid-line was worse still: nothing owned the tail, so it sat
// in the buffer and ate the FIRST command of whoever connected next - including a
// brand new client that had done nothing wrong.
//
// buf/cap are deliberately KEPT when a slot is released. The next client in that
// slot reuses the allocation instead of churning PSRAM; len = 0 is what guarantees
// none of the previous client's bytes can ever be read.
struct WsAcc {
  int    fd  = -1;
  char*  buf = nullptr;
  size_t len = 0, cap = 0;
};
inline WsAcc wsAcc[WS_MAX_CLIENTS];

// The slot for `fd`, claiming a free one if this socket is new. The eviction policy
// is deliberately identical to WsSink::begin() - if the two ever disagree, a client
// ends up holding a sink slot with no accumulator, or the reverse.
inline WsAcc* accFor(int fd) {
  for (int i = 0; i < WS_MAX_CLIENTS; i++) if (wsAcc[i].fd == fd) return &wsAcc[i];
  for (int i = 0; i < WS_MAX_CLIENTS; i++)
    if (wsAcc[i].fd < 0) { wsAcc[i].fd = fd; wsAcc[i].len = 0; return &wsAcc[i]; }
  wsAcc[0].fd = fd; wsAcc[0].len = 0; return &wsAcc[0];   // full: evict, as begin() does
}

inline void accRelease(int fd) {
  for (int i = 0; i < WS_MAX_CLIENTS; i++)
    if (wsAcc[i].fd == fd) { wsAcc[i].fd = -1; wsAcc[i].len = 0; }
}

// Session teardown - httpd calls this when a client closes AND when lru_purge
// evicts one. Before it existed WsSink::drop() had no caller at all: a departed
// client kept its sink slot until some later send happened to fail on it, and its
// half-line kept poisoning the accumulator indefinitely.
//
// Runs on the httpd task (Core 0). Writing _fds[i] = -1 races pump() on Core 1 the
// same way begin() already does; it is a single aligned int store, and the only
// consequence of losing the race is one send to a closed fd, which fails harmlessly
// and clears the slot anyway.
//
// MANDATORY: httpd hands the socket over entirely here, so this must close it.
inline void wsClose(httpd_handle_t hd, int sockfd) {
  (void)hd;
  accRelease(sockfd);
  wsSink.drop(sockfd);
  close(sockfd);
}

// The send function of every WebSocket session, installed at the handshake (wsHandler).
// RUNS ON THE HTTPD TASK, for our frames (wsSendFrame) and the server's own (PONG, close).
//
// A FRAME GOES OUT WHOLE, OR IT IS THE LAST THING ON THAT SOCKET. httpd's default send
// is one send(), and with SO_SNDTIMEO set (send_wait_timeout) lwIP ends a write that
// times out after making progress with a SHORT count, which httpd_ws_send_frame_async
// takes as success: a frame went out cut short and the next frame's header followed it,
// so the client read headers out of the middle of payloads (HIL ncwifi.ws_ping_soak:
// opcodes 3, 12 and 2 after a 3 s WiFi stall, then a reset). Here a short write is
// continued until the buffer is out. Only when no byte has gone for WS_STALL_MS - the
// client has stopped reading, or is gone - does it give up, and then it shuts the
// socket down at once, so a frame that may now be cut short is never followed by
// another. httpd's select() sees the shut socket and deletes the session itself
// (close_fn = wsClose), and the client sees its connection end and can reconnect.
inline int wsSendAll(httpd_handle_t hd, int sockfd, const char* buf, size_t len, int flags) {
  (void)hd;
  if (!buf) return HTTPD_SOCK_ERR_INVALID;
  size_t   done     = 0;
  uint32_t lastByte = millis();
  while (done < len) {
    const int n = send(sockfd, buf + done, len - done, flags);   // returns within send_wait_timeout
    if (n > 0) { done += (size_t)n; lastByte = millis(); continue; }
    const int  e     = errno;
    const bool retry = (n == 0) || e == EAGAIN || e == EWOULDBLOCK || e == EINTR || e == ENOMEM;
    if (!retry || (uint32_t)(millis() - lastByte) >= WS_STALL_MS) {
      shutdown(sockfd, SHUT_RDWR);
      return retry ? HTTPD_SOCK_ERR_TIMEOUT : HTTPD_SOCK_ERR_FAIL;
    }
    if (n == 0) vTaskDelay(1);   // a send() that returns at once with nothing done must not spin
  }
  return (int)len;
}

// ── Handler — RUNS ON THE HTTPD TASK (Core 0). Enqueue only. ────────────────
inline esp_err_t wsHandler(httpd_req_t* req) {
  // GET is the opening handshake; esp_http_server completes it for us. Remember the
  // socket: from here on the sink mirrors Serial to it continuously, which is what
  // makes the live monitor work rather than only command replies. And give the
  // session wsSendAll, so every frame on it goes out whole (the handshake reply,
  // already sent, is the only write it does not cover).
  if (req->method == HTTP_GET) {
    const int sockfd = httpd_req_to_sockfd(req);
    httpd_sess_set_send_override(req->handle, sockfd, wsSendAll);
    wsSink.begin(sockfd);
    return ESP_OK;
  }

  // Two-step receive: length first (len = 0), then the payload.
  httpd_ws_frame_t frame = {};
  frame.type = HTTPD_WS_TYPE_TEXT;
  esp_err_t e = httpd_ws_recv_frame(req, &frame, 0);
  if (e != ESP_OK) return e;
  if (frame.len == 0) return ESP_OK;
  // Same ceiling handleSerialInput enforces on serialInputBuf. A frame larger
  // than the dispatcher could ever accept is refused here rather than allocated.
  if (frame.len >= 98304) return ESP_FAIL;

  // PSRAM, not heap: internal SRAM is the scarce budget (~257 KB free) and a
  // SET_CONFIG line is tens of KB.
  char* buf = (char*)ps_malloc(frame.len + 1);
  if (!buf) return ESP_ERR_NO_MEM;
  frame.payload = (uint8_t*)buf;
  e = httpd_ws_recv_frame(req, &frame, frame.len);
  if (e != ESP_OK) { free(buf); return e; }
  buf[frame.len] = '\0';

  // A MESSAGE BOUNDARY IS NOT A LINE BOUNDARY.
  //
  // The protocol is newline-delimited; WebSocket is message-framed. A client that
  // writes in fixed-size chunks (which the config tool does — it splits anything
  // over 512 B) delivers one line as several frames, and treating each frame as a
  // command turns a 1391 B OTA DATA line into three fragments. Observed exactly
  // that: "[OTA] DATA base64 error -44", the decoder handed a fragment.
  //
  // So frame here, as handleSerialInput() does for the serial byte stream: append
  // and dispatch only on a newline. Cheap (a memcpy on Core 0) and it makes the
  // endpoint robust against ANY client's chunking rather than relying on ours.
  //
  // PER SOCKET - see WsAcc. One accumulator shared across every client was measured
  // on hardware to destroy BOTH sides of any interleave.
  const int sockfd = httpd_req_to_sockfd(req);
  WsAcc* ac = accFor(sockfd);

  if (ac->len + frame.len + 1 > ac->cap) {
    size_t want = ac->len + frame.len + 1024;
    if (want > 98304 + 2048) { ac->len = 0; free(buf); return ESP_FAIL; }   // runaway
    char* grown = (char*)ps_realloc(ac->buf, want);
    if (!grown) { free(buf); return ESP_ERR_NO_MEM; }
    ac->buf = grown; ac->cap = want;
  }
  memcpy(ac->buf + ac->len, buf, frame.len);
  ac->len += frame.len;
  free(buf);                       // the accumulator owns the bytes now

  // Dispatch every COMPLETE line in what we have; keep any tail for the next frame.
  size_t start = 0;
  for (size_t i = 0; i < ac->len; i++) {
    if (ac->buf[i] != '\n' && ac->buf[i] != '\r') continue;
    size_t len = i - start;
    if (len > 0) {
      char* line = (char*)ps_malloc(len + 1);
      if (line) {
        memcpy(line, ac->buf + start, len);
        line[len] = '\0';
        WsCmd m = { sockfd, line };
        // Wait briefly for room instead of discarding. Only a genuinely sustained
        // overload reaches the free() now, and it says so on the console — a Core-0
        // print is not teed to the client (rcSerial gates the tee by the arming
        // core), so USB is the only place this can be reported.
        if (!wsQueue ||
            xQueueSend(wsQueue, &m, pdMS_TO_TICKS(WS_ENQUEUE_WAIT_MS)) != pdTRUE) {
          Serial.println("[WS] command queue full - dropped a line");
          free(line);
        }
      }
    }
    start = i + 1;
  }
  if (start) { memmove(ac->buf, ac->buf + start, ac->len - start); ac->len -= start; }
  return ESP_OK;
}

// ── Start ───────────────────────────────────────────────────────────────────
// Call AFTER the SoftAP is up. Returns false and stays entirely inert on failure —
// a WebSocket that will not start must never take the droid down with it.
inline bool begin() {
  wsQueue = xQueueCreate(WS_QUEUE_DEPTH, sizeof(WsCmd));
  if (!wsQueue) { Serial.println("[WS] queue alloc failed — endpoint disabled"); return false; }

  httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
  // Pin to Core 0 with the other network work, leaving Core 1 for loop(): SBUS is
  // serviced at ~111 fps there and must not contend with TCP.
  cfg.core_id          = 0;
  // Leave task_priority at the default 5. NEVER 0 — that is tskIDLE_PRIORITY, and
  // the idle task on Core 0 is watched by the task WDT.
  cfg.max_open_sockets = 3;    // a config channel, not a hotspot
  cfg.lru_purge_enable = true; // a stale client must not permanently consume a slot
  // How often wsSendAll regains control while a send is blocked - NOT how long a client
  // may stall (that is WS_STALL_MS, counted across these). Each send() returns within it,
  // with a short count or nothing, and wsSendAll decides whether to go on. 1 s, the
  // config's floor (whole seconds; 0 means no timeout). Never rely on it alone to bound a
  // send: a timeout that ends a write part-way leaves a frame cut short (see wsSendAll).
  cfg.send_wait_timeout = 1;
  // Release the sink slot and the line accumulator the moment a session ends,
  // rather than leaving both to be noticed later, or never. See wsClose().
  cfg.close_fn         = wsClose;

  if (httpd_start(&wsServer, &cfg) != ESP_OK) {
    Serial.println("[WS] httpd_start failed — endpoint disabled");
    vQueueDelete(wsQueue); wsQueue = nullptr; wsServer = nullptr;
    return false;
  }

  httpd_uri_t uri = {};
  uri.uri = "/ws";
  uri.method = HTTP_GET;
  uri.handler = wsHandler;
  uri.is_websocket = true;
  httpd_register_uri_handler(wsServer, &uri);

  Serial.printf("[WS] command endpoint ready — ws://%s/ws\n",
                WiFi.softAPIP().toString().c_str());
  return true;
}

// ── Drain — RUNS ON THE LOOP TASK (Core 1). Safe to do real work. ───────────
// One command per call, mirroring drainRemoteCli(): loop() keeps SBUS, the mesh
// heartbeat and the aux-serial pump alive between commands.
inline void drain() {
  // ── Keep the tee armed, and flush ─────────────────────────────────────────
  // Runs every loop() pass, not just when a command is pending. Two jobs:
  //
  // 1. RE-ARM. rcSerial's capture is a SINGLE slot, and drainRemoteCli() takes it
  //    for mesh-relayed CLI lines then disarms unconditionally. Without re-arming
  //    here, one relayed command silently ends the WebSocket's live monitor for
  //    good. Re-arming is cheap (two stores) and idempotent.
  // 2. FLUSH. The sink buffers during arbitrary Serial.printf calls; this is the
  //    one place the bytes actually go out, so network I/O stays at a known point
  //    on the core that must also service SBUS.
  wsLoopCore = xPortGetCoreID();   // this IS the loop task; see wsLoopCore
  if (wsSink.live()) {
    rcSerial.armCapture(&wsSink, flushHook);
    if (!wsSink.pump()) rcSerial.disarmCapture();   // client gone — stop teeing
  }
  // Lines the sink refused whole (its admission: the clients have taken nothing for a
  // while) are said so on USB, at most once a second. Straight to HWCDCSerial: through
  // Serial the note would be teed back into the same full sink, and refused in its turn.
  // Only when USB has room, like vlogf(): a USB write must never stall loop().
  { static uint32_t told = 0, toldAt = 0;
    const uint32_t dropped = wsDroppedLines;
    if (dropped != told && (uint32_t)(millis() - toldAt) >= 1000) {
      char note[112];
      const int n = snprintf(note, sizeof(note),
                             "[WS] %lu line(s) dropped whole: the socket(s) took nothing (%u KB queued)\n",
                             (unsigned long)(dropped - told), (unsigned)(wsTxQueued / 1024));
      if (n > 0 && HWCDCSerial.availableForWrite() >= n) {
        HWCDCSerial.write((const uint8_t*)note, (size_t)n);
        told = dropped; toldAt = millis();
      }
    } }

  if (!wsQueue) return;
  WsCmd m;
  if (xQueueReceive(wsQueue, &m, 0) != pdTRUE) return;

  // Deliberately NOT re-begin()ing the sink and NOT disarming afterwards. The tee
  // is a standing arrangement for the whole session now: begin() would clear
  // whatever loop() has already buffered this pass, and disarming at the end of a
  // command is precisely the bug that left the live monitor dead.
  //
  // The String copy is safe even for a ~98 KB SET_CONFIG: the toolchain sets
  // CONFIG_SPIRAM_USE_MALLOC=y with CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL=4096, so any
  // allocation over 4 KB is served from PSRAM, not the ~256 KB of internal SRAM.
  // Worth knowing before "optimising" this copy away — it is not on the scarce heap.
  //
  // TRIMMED, and skipped when that leaves it empty - exactly what handleSerialInput()
  // does to every USB line. processInputLine() switches on the first character, so an
  // untrimmed socket line with a leading space or tab was dropped with no reply, and a
  // trailing one made "?version " an unknown command: the same protocol only if the
  // framing matches too (HIL ncwifi.ws_line_trim).
  String line(m.line);
  line.trim();
  wsLineRunning = true;                        // lineFromSocket(): this line's transport
  if (line.length()) processInputLine(line);   // the SAME dispatcher the USB path uses
  wsLineRunning = false;

  // Push the reply now rather than waiting for the next pass, so a command feels
  // immediate instead of picking up one loop() of latency.
  if (wsSink.live() && !wsSink.pump()) rcSerial.disarmCapture();

  free(m.line);                      // ownership ends here

  // NOTE (same limitation as the RTERM path): only output printed on THIS core,
  // inside the call above, is captured — rcSerial gates the tee by the core that
  // armed it. A command whose reply arrives asynchronously (a mesh Maestro read)
  // answers after disarmCapture() and goes to Serial only. drainRemoteCli() solves
  // this with g_rtermRelay; if it matters here, that is the pattern to copy.
}

}  // namespace naviws
