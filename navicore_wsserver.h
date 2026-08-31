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

struct WsCmd {
  int   fd;      // client socket, for the async reply
  char* line;    // ps_malloc'd, owned by whoever holds this struct
};

inline QueueHandle_t  wsQueue  = nullptr;
inline httpd_handle_t wsServer = nullptr;

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
// OVERFLOW DROPS WHOLE LINES. Truncating mid-line would hand the tool half a JSON
// object, which fails at JSON.parse and (per the tool's own notes) silently
// freezes a panel. Dropping to the next newline loses a sample instead, which for
// 20 Hz telemetry is invisible.
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
    // FIRST client after a quiet period: start from a clean sheet. _dropping is set
    // when a pump() fails, which is exactly what happens as the last client leaves
    // mid-line - and nothing cleared it again, because write() returns at !live()
    // BEFORE reaching the reset, and end() has no caller. It therefore survived the
    // disconnect and ate the first whole line the next client was sent. Usually that
    // is one 20 Hz telemetry frame and invisible; when it is the ~14 KB GET_CONFIG
    // reply the tool comes up with no config at all, which reads as "the droid lost
    // my settings". Only on the transition to live: clearing while another client is
    // already connected would discard output buffered for it this pass.
    if (!live()) { _len = 0; _dropping = false; }
    for (int i = 0; i < WS_MAX_CLIENTS; i++) if (_fds[i] < 0) { _fds[i] = fd; return; }
    _fds[0] = fd;   // full: evict the oldest rather than refuse the newcomer
  }
  void drop(int fd) { for (int i = 0; i < WS_MAX_CLIENTS; i++) if (_fds[i] == fd) _fds[i] = -1; }
  void end() { for (int i = 0; i < WS_MAX_CLIENTS; i++) _fds[i] = -1; _len = 0; _dropping = false; }
  bool live() const {
    for (int i = 0; i < WS_MAX_CLIENTS; i++) if (_fds[i] >= 0) return true;
    return false;
  }

  size_t write(uint8_t c) override {
    if (!live()) return 1;
    if (_dropping) { if (c == '\n') _dropping = false; return 1; }
    if (_len >= sizeof(_buf)) {
      // FULL — flush right here rather than dropping. A single CONFIG reply is one
      // ~14 KB line, so a drop-on-full policy discarded the whole thing and the tool
      // sat there with no config at all ("Configure WCB Network..."), while the
      // ~640 B PWM_UPDATE fitted and made the monitor look healthy.
      //
      // Yes, this puts a TCP send inside a Serial.printf on the SBUS core — the very
      // thing pump() exists to avoid. It is the lesser evil and it is RARE: only a
      // line longer than the buffer reaches here, i.e. a config or cmdlib dump, not
      // the 20 Hz telemetry that motivated the buffering. Losing the config is a
      // hard failure; a few ms of jitter on an explicit user action is not.
      if (!pump()) { _dropping = true; return 1; }   // send failed: client is gone
    }
    _buf[_len++] = (char)c;
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
  bool pump() {
    if (!_len || !wsServer) return live();
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
    httpd_ws_frame_t f = {};
    f.final   = true;
    f.type    = HTTPD_WS_TYPE_TEXT;
    f.payload = (uint8_t*)_buf;
    f.len     = send;
    for (int i = 0; i < WS_MAX_CLIENTS; i++) {
      if (_fds[i] < 0) continue;
      if (httpd_ws_send_frame_async(wsServer, _fds[i], &f) != ESP_OK) _fds[i] = -1;
    }
    const size_t left = _len - send;
    if (left) memmove(_buf, _buf + send, left);
    _len = left;
    return live();
  }

 private:
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
  bool   _dropping = false;
  // One PWM_UPDATE is ~640 B and they arrive at ~20 Hz; loop() pumps far faster
  // than that, so this only has to absorb one busy pass, not a backlog.
  char   _buf[2048];
};

inline WsSink wsSink;

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
  if (!wsSink.live()) return;
  wsSink.write((const uint8_t*)s, strlen(s));
  wsSink.write((uint8_t)'\n');
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

// ── Handler — RUNS ON THE HTTPD TASK (Core 0). Enqueue only. ────────────────
inline esp_err_t wsHandler(httpd_req_t* req) {
  // GET is the opening handshake; esp_http_server completes it for us. Remember the
  // socket: from here on the sink mirrors Serial to it continuously, which is what
  // makes the live monitor work rather than only command replies.
  if (req->method == HTTP_GET) {
    wsSink.begin(httpd_req_to_sockfd(req));
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
  if (wsSink.live()) {
    rcSerial.armCapture(&wsSink, flushHook);
    if (!wsSink.pump()) rcSerial.disarmCapture();   // client gone — stop teeing
  }

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
  processInputLine(String(m.line));  // the SAME dispatcher the USB path uses

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
