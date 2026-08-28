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
#include "rc_serial.h"   // rcSerial — the capture tee this borrows

// Defined in NaviCore.ino. Declared here because this header is included near the
// top of the sketch, long before the definition, and Arduino's auto-prototyping
// only covers the .ino itself.
bool processInputLine(const String& line);

namespace naviws {

// One command in flight at a time is the honest ceiling: rcSerial's capture tee
// is a single slot (rc_serial.h), so two commands cannot have their output
// separated anyway. Depth 3 just absorbs a burst without dropping it.
static const uint8_t  WS_QUEUE_DEPTH = 3;
// Flush the reply in ~MSS-sized pieces. A GET_CONFIG reply is tens of KB; one
// frame per output line would be hundreds of TCP writes, and buffering the whole
// thing would need another 98 KB of PSRAM for no benefit.
static const size_t   WS_TX_CHUNK    = 1400;

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
  void begin(int fd) { _fd = fd; _len = 0; _dropping = false; }
  void end()         { _fd = -1; _len = 0; _dropping = false; }
  bool live() const  { return _fd >= 0; }
  int  fd()   const  { return _fd; }

  size_t write(uint8_t c) override {
    if (_fd < 0) return 1;
    if (_dropping) { if (c == '\n') _dropping = false; return 1; }
    if (_len >= sizeof(_buf)) { _dropping = true; return 1; }   // drop the rest of THIS line
    _buf[_len++] = (char)c;
    return 1;
  }
  size_t write(const uint8_t* b, size_t n) override {
    for (size_t i = 0; i < n; i++) write(b[i]);
    return n;
  }
  using Print::write;

  // Called from loop() only. Returns false once the client has gone.
  bool pump() {
    if (_fd < 0 || !_len || !wsServer) return _fd >= 0;
    httpd_ws_frame_t f = {};
    f.final   = true;
    f.type    = HTTPD_WS_TYPE_TEXT;
    f.payload = (uint8_t*)_buf;
    f.len     = _len;
    esp_err_t e = httpd_ws_send_frame_async(wsServer, _fd, &f);
    _len = 0;
    if (e != ESP_OK) { end(); return false; }   // client vanished
    return true;
  }

 private:
  int    _fd       = -1;
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

  WsCmd m = { httpd_req_to_sockfd(req), buf };
  if (!wsQueue || xQueueSend(wsQueue, &m, 0) != pdTRUE) {
    // Nobody took ownership, so it is still ours to release. Dropping under load
    // is the same policy queueRemoteCli uses; the client sees no reply and can retry.
    free(buf);
  }
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
    rcSerial.armCapture(&wsSink);
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
