// =============================================================================
//  NaviCore.ino — WCB-based RC Controller (formerly RC-Controller / HyperCore)
//  Target hardware: NaviCore v2 PCB (default) or WCB HW 3.2 — ESP32-S3,
//  selected at runtime by rcConfig.boardType (see applyBoardProfile()).
// 
//  Features:
//    • SBUS input (16 or 24 channel, auto-detected) on shared UART1 RX
//    • SBUS output passthrough on the SAME shared UART1 TX (per board profile:
//      v2 GPIO5 / WCB 3.2 GPIO4) — byte-streamed re-emit of the RX frame so a
//      downstream device sees the same channel data
//    • Local Pololu Maestro on Serial2 @ 115200 baud (GPIO6 TX)
//    • Up to 8 remote Maestros via WCBStream broadcast over ESP-NOW
//    • WCB unicast and broadcast command dispatch
//    • Three aux serial ports S3/S4/S5 (S3 = hardware UART0; S4/S5 SoftwareSerial)
//    • Multi-mode RC button/switch/knob mapping (NVS-backed, GUI-configurable)
//    • USB Serial JSON protocol for config tool and CLI debugging
//      (open config_tool/index.html on your PC, connect via Web Serial API)
//
//  USB Serial JSON Protocol (newline-delimited):
//    PING                        → {"type":"PONG"}
//    GET_CONFIG                  → {"type":"CONFIG","data":{...full config JSON...}}
//    {"type":"SET_CONFIG","data":{...}} → {"type":"ACK","ok":true}
//    {"type":"START_MONITOR"}    → streams PWM_UPDATE every 50 ms until STOP_MONITOR
//    {"type":"STOP_MONITOR"}     → {"type":"ACK","ok":true}
//    {"type":"RESET_DEFAULTS"}   → reloads factory defaults, replies ACK
//    {"type":"REBOOT"}           → ACKs then restarts the board after 250 ms
//    {"type":"TRIGGER","mode":1,"btn":3,"tap":1} → fires virtual button press
//    {"type":"WCB_SEND","target":2,"cmd":":PP100"} → manually fires WCB command
//
//  Required libraries (Library Manager):
//    • ArduinoJson        (Benoit Blanchon) v6.x
//    • Adafruit NeoPixel
//
//  Local libraries (in libraries/ folder):
//    • WCB_Client + WCBStream  (from the greghulette/WCBClient repo)
//    • PololuMaestro
// =============================================================================

#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <LittleFS.h>     // config persists as /config.json (replaces the NVS 4000-byte/value limit)
#include <SoftwareSerial.h>
#include <Adafruit_NeoPixel.h>
#include "esp_timer.h"          // one-shot boot-guard timer (cold-boot auto-recovery)
#include "esp_ota_ops.h"        // esp_ota_get_bootloader_description (boot banner)
#include "rom/rtc.h"            // rtc_get_reset_reason (low-level boot telemetry)
#include <WCB_Client.h>   // header in greghulette/WCBClient is WCB_Client.h
#include <WcbCmd.h>       // shared device-command translators (Maestro/MP3/WLED/HCR) —
                          // ONE source of the device wire bytes across WCB + NaviCore
#include <WCBStream.h>
// HCR (Human Cyborg Relations Vocalizer): no library dependency — we format
// the same byte string the upstream HCRVocalizer would have written and push
// it directly to the bound aux-serial port.  See hcrFormatCommand() below.
#include "sbus_reader.h"
#include "rc_serial.h"     // USB-CDC tee — MUST precede project headers that print (remote-terminal capture)
#include "rc_config.h"
#include "wcb_config.h"
#include "fw_version.h"     // FW_VERSION_BASE / FW_VERSION_DTG / FW_VERSION
#include "rc_telemetry.h"   // WCB-network remote-management bridge (Phase 1 — see file header)
#include "navicore_ota.h"   // firmware OTA — ?OTALOCAL (direct USB) + ?OTA (ESP-NOW relay)
#include "navicore_rterm.h" // remote terminal — mirror CLI output back over the WCB bridge (RTERM)
#include "navicore_record.h" // record/replay — capture dispatched droid actions to a clip + replay
#include "navicore_wsserver.h" // optional WebSocket command endpoint over the SoftAP (rcConfig.wifiEnabled)

// USB-CDC tee instance backing the `#define Serial rcSerial` in rc_serial.h.
// Defined once here; setup()'s Serial.begin()/setRxBufferSize() drive it.
RcSerial rcSerial;

// Record/replay clip library filesystem — a SECOND LittleFS mounted on the
// dedicated 12 MB `clips` partition (partitions.csv), separate from the config
// LittleFS (`spiffs` label). Its own instance + label so a first-boot format is
// scoped to clips and can never touch /config.json.
fs::LittleFSFS clipsFS;
bool           g_clipsReady = false;

// Remote-terminal state.  A raw ?.../#... CLI line arriving over the WCB mesh
// (onWCBCommand, Core 0) is stashed here and run in loop() with its Serial
// output tee'd to rtermSink, which ships each line back to the relay (bridge)
// as an RTERM packet that surfaces on the config-tool terminal.
navirterm::CaptureSink rtermSink;
// Single cross-core hop for relayed CLI lines: onWCBCommand (Core 0, WiFi task)
// enqueues; drainRemoteCli (loop, Core 1) dequeues and runs. A FreeRTOS queue
// provides the memory barrier a bare volatile flag would not — same pattern as
// the OTA packet queue.
struct RemoteCliMsg { uint8_t relay; char cmd[200]; };
QueueHandle_t remoteCliQueue = nullptr;
// Relay id of the CLI line drainRemoteCli() is running right now; 0 while a locally
// typed (USB) line runs. A command whose answer arrives ASYNCHRONOUSLY — long after
// the tee is disarmed — latches this so it can re-arm the tee for its own reply (see
// maeLatchRemoteRelay / maePumpRemoteEmits).
static uint8_t g_rtermRelay = 0;

// ── New-peer detection → configured action + passive alert ──────────────────
// wcb->onNeighbor() fires on the WiFi/ESP-NOW task (Core 0) whenever a WDP advert
// is decoded. That callback only ENQUEUES the board number; all the real work
// (dedup, boot-grace suppression, action dispatch, LED flash, terminal line)
// runs in drainPeerEvents() from loop() (Core 1) where flash/serial I/O is safe.
QueueHandle_t peerEventQueue   = nullptr;
uint32_t      g_peerSeenMask   = 0;   // bit (id-1) set once a board's new-peer event has been handled this session
uint32_t      g_peerGraceUntil = 0;   // millis(): boards first heard before this are recorded silently (the boot fleet)
uint32_t      g_peerFlashUntil = 0;   // millis(): show the new-peer LED pulse until then (0 = not flashing)
#define PEER_GRACE_MS 8000            // suppress the initial fleet-discovery burst for 8 s after wcb->begin()

// ── Inbound mesh COMMAND counters ───────────────────────────────────────────
// The receive-side half of the delivery statistics. WCB_Client 1.13.0 counts
// only what this board SENDS (getPeerStats / getAggregateStats / getBroadcastSent
// are all outbound), so the inbound side is counted here in onWCBCommand — the
// single funnel every mesh COMMAND reaches. Written on Core 0, read on Core 1;
// see the comment at the increment for why that needs no lock.
//
// RAM-only and never persisted: a reboot zeroes them, which is the intent — the
// counters describe this session's link health, not the board's history.
uint32_t g_meshRxCount = 0;                   // total COMMANDs delivered to onWCBCommand
uint32_t g_meshRxFrom[WCB_MAX_BOARDS] = {0};  // per-sender breakdown, index = id-1
// Deferred mesh-stats reset. WCB_Client::resetStats() takes the pending-table
// lock — it races the RX task's `ackd` increment — so the library requires it be
// called from loop(), NEVER from inside a receive callback. A bridged request
// arrives on the Core-0 ESP-NOW callback, so both transports just raise this flag
// and drainMeshStatsReset() does the work on Core 1. One-shot with no payload, so
// a volatile bool is the whole queue (last-writer-wins is correct here).
volatile bool g_meshStatsResetPending = false;

// ── Boot roll call ──────────────────────────────────────────────────────────
// One shot, ROLL_CALL_MS after wcb->begin(): name every board in the configured
// floor (1..quantity) that has still never been heard from. This is distinct
// from the online/offline transition log — a board that comes up and later
// drops gets a transition line, but a board that was never there at all
// produces NO evidence on this console at all without this.
// It is a ROLL CALL, NOT AN ALARM: "WCB3 absent" is worth one line whether the
// answer is "the dome is on the bench" or "check its power".
uint32_t g_rollCallAt   = 0;      // millis() deadline; 0 = disarmed/already run
#define  ROLL_CALL_MS 30000       // long enough for ETM heartbeats from a slow-booting board

// ── Remote TRIGGER → deferred dispatch (Core 0 → Core 1) ────────────────────
// A remote {"type":"TRIGGER"} arrives in onWCBCommand → rcTelemetry::handle() on
// Core 0 (WiFi task). Dispatching it there would run the full action chain
// (Maestro Serial2, HCR/MP3 aux-serial, WCB sends, speed/accel caches) on the
// wrong core, racing processSbus() on Core 1 over the same UART/caches with no
// lock. The TRIGGER handler instead enqueues here and drainRemoteTriggers()
// (loop, Core 1) dispatches it, so remote and local triggers share the one
// single-core dispatch path the Maestro/tap-timing/record-replay code assumes.
struct RemoteTrigger { uint8_t mode, btn, tap; };
QueueHandle_t remoteTriggerQueue = nullptr;

// ── Forget-learned-peer → deferred dispatch (Core 0 → Core 1) ────────────────
// A {"type":"FORGET_PEER"} arriving over the WCB bridge runs in onWCBCommand on
// Core 0. forgetPeer()/clearLearnedPeers() do an esp_now_del_peer + an NVS write
// that can't run on the small ESP-NOW callback stack, so the Via-WCB path enqueues
// here and drainForgetPeer() (loop, Core 1) does the work. id 0 = clear ALL
// learned peers; 1..WCB_MAX_BOARDS = forget that one. (USB + CLI are already on
// Core 1, so they call doForgetPeer() directly.)
QueueHandle_t forgetPeerQueue = nullptr;

// =============================================================================
//  Pin assignments — RUNTIME, selected by the active board profile
// =============================================================================
//  These were compile-time #defines for the WCB HW 3.2 layout. They are now
//  plain globals set at boot by applyBoardProfile() from rcConfig.boardType, so
//  ONE firmware image runs on both the WCB 3.2 hardware and the NaviCore v2 PCB.
//  Every use site is a runtime .begin()/printf(), so a variable is a drop-in.
//  Initialised to the NaviCore v2 defaults; applyBoardProfile() overwrites them
//  AFTER the config loads and BEFORE any port opens (see setup()).
//  (STATUS_LED_PIN stays a constant — GPIO48 onboard NeoPixel on both boards.)
//
//    Board 0 = NaviCore v2 PCB (default)      Board 1 = WCB HW 3.2
//      SBUS in 4  / SBUS out 5                   SBUS in 5  / SBUS out 4
//      Maestro TX6 / RX7                         Maestro TX6 / RX7
//      S3 (v2 "Serial 1") TX8  / RX9             S3 TX15 / RX16  (WCB "Serial 3")
//      S4 (v2 "Serial 2") TX10 / RX21            S4 TX17 / RX18  (WCB "Serial 4")
//      S5 (v2 "Serial 3") TX38 / RX47            S5 TX9  / RX10  (WCB "Serial 5")
// =============================================================================
enum BoardType : uint8_t { BOARD_NAVICORE_V2 = 0, BOARD_WCB_HW_32 = 1 };

uint8_t SBUS_RX_PIN    = 4;   // Serial1/UART1 RX — SBUS from RC receiver
uint8_t SBUS_OUT_PIN   = 5;   // SBUS OUT TX (UART0/Serial0 dedicated, or shared UART1 TX)
uint8_t MAESTRO_TX_PIN = 6;   // Serial2 TX — local Maestro command bus
uint8_t MAESTRO_RX_PIN = 7;   // Serial2 RX — optional Maestro feedback
uint8_t S3_TX_PIN      = 8;   // Aux serial S3 TX (v2 "Serial 1")
uint8_t S3_RX_PIN      = 9;   // Aux serial S3 RX
uint8_t S4_TX_PIN      = 10;  // Aux serial S4 TX (v2 "Serial 2")
uint8_t S4_RX_PIN      = 21;  // Aux serial S4 RX
uint8_t S5_TX_PIN      = 38;  // Aux serial S5 TX (v2 "Serial 3"; WCB 3.2 "Serial 5" GPIO9/10)
uint8_t S5_RX_PIN      = 47;  // Aux serial S5 RX

// SBUS layout is RUNTIME via the `sbusSharedUart` flag (set by applyBoardProfile();
// replaces the old compile-time SBUS_SHARED_UART):
//   • BOTH current boards (WCB 3.2 and NaviCore v2) use SHARED: SBUS IN + OUT on ONE
//     full-duplex UART (UART1/Serial1), RX SBUS_RX_PIN + TX SBUS_OUT_PIN, 100k 8E2
//     inverted (the invert flag is right for both directions; a non-blocking TX
//     buffer keeps the byte-tee from stalling). That frees UART0 to be the hardware
//     S3 aux port (>57600 capable). NOTE: the shared path is not yet bench-validated.
//   • sbusSharedUart=false is the DEDICATED fallback (SBUS OUT on its own UART0/Serial0,
//     TX-only; S3 then bit-banged SoftwareSerial). Not used by either current board —
//     flip a board's flag in applyBoardProfile() if shared SBUS proves unreliable.

#define STATUS_LED_PIN  48    // onboard NeoPixel — GPIO48 on both WCB 3.2 and NaviCore v2
#define STATUS_LED_COUNT 1

// =============================================================================
//  WCB Client + Streams
//
//  Single shared broadcast stream:  All remote Maestro bytes go out on one
//  WCBStream targeting `broadcast` (target_wcb=0). The Kyber path delivers
//  them to every WCB on the network. Each receiving WCB with Kyber_Remote
//  forwards the raw bytes to its configured Maestro port — no per-slot
//  WCB/port routing needed on the sender.
//
//  No compile-time Maestro instances: Maestro slots (IDs 1-8) are configured
//  at RUNTIME via the GUI's Maestro Locations panel (rcConfig.maestros[]).
//  Each slot decides whether bytes go to Serial2 (local) or the broadcast
//  stream (remote), and which Pololu device # to embed.  See maestroWrite()
//  below for the dispatch.
// =============================================================================
// Heap-allocated in setup() AFTER NVS config loads, so the values come from
// rcConfig.wcbNetwork (GUI-editable) instead of the wcb_config.h #defines.
// The #defines are still the factory defaults on a fresh device — see
// rcConfigLoadDefaults() in rc_config.h.
WCB_Client* wcb = nullptr;
// True ONLY after wcb->begin() succeeds. ESP-NOW is unusable until then, and
// calling send/broadcast/update on a WCB_Client whose begin() failed is
// undefined behavior — so every wcb-> call is gated on this flag.
bool wcbReady = false;

// One stream, broadcast to all WCBs.  target_port is ignored for broadcast.
//
// MUST be constructed AFTER `wcb` (in setup()): WCBStream's constructor
// self-registers with the WCB_Client singleton via WCB_Client::instance(), and
// that registration is what makes wcb->update() drive this stream's flush.
// If it were a global it would construct at static-init time — before the
// heap-allocated WCB_Client exists — and silently fail to register, so no
// Maestro bytes would ever leave the board over ESP-NOW.
WCBStream* maestroBroadcast = nullptr;

// =============================================================================
//  Aux serial ports S3, S4, S5  (shared-SBUS layout)
//
//  ESP32-S3 has 3 hardware UARTs (UART0/1/2).  Allocation once the debug
//  console moved to native USB CDC (USBMode=hwcdc, CDCOnBoot=cdc):
//    • Serial  → native USB CDC (debug + config tool)  — no UART consumed
//    • Serial1 → UART1 → SBUS IN + OUT (shared, 100k 8E2 inverted)
//    • Serial2 → UART2 → local Maestro TX
//    • Serial0 → UART0 → aux S3 (freed because SBUS IN+OUT share UART1)
//  Both current boards use the shared-SBUS layout (sbusSharedUart=true), so the
//  first aux port S3 runs on HARDWARE UART0/Serial0 (can exceed 57600, up to
//  115200); S4 and S5 are bit-banged via SoftwareSerial (≤57600 baud; they never
//  run the 100k SBUS rate). The dedicated-SBUS-out-on-UART0 layout
//  (sbusSharedUart=false) is a kept-but-unused fallback — see applyBoardProfile().
// =============================================================================
// Aux command ports — their TYPE is RUNTIME board-dependent, so they're reached
// through Stream* pointers bound in setup() AFTER applyBoardProfile(). On both
// current (shared-SBUS) boards: s3 = HARDWARE Serial0/UART0, s4 + s5 =
// SoftwareSerial. v2 silkscreen = "Serial 1/2/3"; WCB 3.2 = its "Serial 3/4/5"
// headers (S5 = GPIO9/10). The dedicated-SBUS fallback would make s3 + s4
// SoftwareSerial with s5 = nullptr.
// Two SoftwareSerial instances back whichever slots are software on the active
// board; the core's Serial0 backs s3 on v2. s3IsHw tells applySerialBauds which
// .begin() overload to use. (This replaces the old compile-time #if SBUS_SHARED_UART
// HardwareSerial&/SoftwareSerial alias — that choice is now runtime, per board.)
SoftwareSerial swAux0;      // backing SoftwareSerial A
SoftwareSerial swAux1;      // backing SoftwareSerial B
Stream* s3 = nullptr;       // aux "S3"  (v2 "Serial 1")
Stream* s4 = nullptr;       // aux "S4"  (v2 "Serial 2")
Stream* s5 = nullptr;       // aux "S5"  (v2 "Serial 3" GPIO38/47; WCB 3.2 "Serial 5" GPIO9/10)
bool    s3IsHw = false;     // true when s3 points at the hardware UART0 (Serial0)

// Board-aware aux-serial port label for logs/UI. idx 0/1/2 = s3/s4/s5.
// NaviCore v2 PCB silkscreens "Serial 1/2/3"; WCB HW 3.2 = "Serial 3/4/5".
// Static buffer — the caller uses it immediately (before the next call).
static const char* auxPortLabel(int idx) {
  static char lbl[12];
  int n = ((int)rcConfig.boardType == 0) ? (idx + 1) : (idx + 3);
  snprintf(lbl, sizeof(lbl), "Serial %d", n);
  return lbl;
}

// Open/close an aux port, dispatching on hardware-UART vs bit-banged SoftwareSerial.
static inline void auxBegin(Stream* p, bool isHw, uint32_t baud, uint8_t rxPin, uint8_t txPin) {
  if (!p) return;
  if (isHw) ((HardwareSerial*)p)->begin(baud, SERIAL_8N1, rxPin, txPin);
  else      ((SoftwareSerial*)p)->begin(baud, SWSERIAL_8N1, rxPin, txPin, false, 95);
}
static inline void auxEnd(Stream* p, bool isHw) {
  if (!p) return;
  if (isHw) ((HardwareSerial*)p)->end();
  else      ((SoftwareSerial*)p)->end();
}
// SBUS OUT uses the real UART0 (Serial0) — see setup() and SBUS_OUT_PIN above.

// The Stream backing a FIRMWARE port number (3/4/5). nullptr when the port doesn't
// exist on the active board. Mesh port numbers must go through rcFwPortForMesh()
// first — this takes firmware numbering only.
static inline Stream* auxStreamFor(int fwPort) {
  switch (fwPort) {
    case 3:  return s3;
    case 4:  return s4;
    case 5:  return s5;
    default: return nullptr;
  }
}

// =============================================================================
//  Mesh ↔ serial bridge
// =============================================================================
// Makes NaviCore's aux ports behave like a WCB's on the mesh, in both directions:
//
//   TARGETED   `;w20,;s<n><cmd>` on any WCB → we write <cmd> out mesh port n.
//              Always accepted, no config needed — the same one-hop route a WCB
//              honors for `;w2;s4<cmd>`.
//   BROADCAST  A mesh broadcast (a command with no `;` / `?` / `#` prefix) is
//              written out every port with bcastOut set; a line read from a port
//              with bcastIn set is broadcast to the mesh AND out the other
//              bcastOut ports. Both are per-port opt-in (rcConfig.serialBcast*).
//
// EVERY write is deferred to loop() through this queue. onWCBCommand runs on the
// Core-0 WiFi task, and S4/S5 are bit-banged SoftwareSerial — a write there blocks
// with interrupts off for the whole frame time (~1 ms per 10 chars at 9600), which
// on the WiFi task would stall ESP-NOW and on either core would jitter the ~111 fps
// SBUS path. The queue is the same cross-core hop pattern as remoteCliQueue.
//
// text[] is sized to the 200-char mesh command payload (WCB_Client's structCommand
// cap), so a forwarded command is never truncated in transit.
struct SerialFwdMsg { uint8_t fwPort; char text[201]; };
QueueHandle_t serialFwdQueue = nullptr;

// =============================================================================
//  Mesh → local Maestro  (inbound ";M")
// =============================================================================
// A WCB holding a remote-Maestro proxy that points at us forwards the command as
// PLAIN TEXT, verbatim — `;M<id><seq>` for a subroutine, `;M<dev>,<verb>[,args]`
// for everything else (WCB_Maestro.cpp builds those strings and unicasts them).
// So NaviCore accepts exactly the grammar a WCB accepts, and it does that by
// handing the text straight to WcbMaestro::build() rather than re-implementing
// the verb table — one parser, shared by both firmwares, no drift.
//
// Deferred to Core 1 for two reasons: Serial2 writes race the dispatch caches
// that processSbus() touches, and a get* query blocks up to 25 ms waiting on the
// Maestro's reply (maestroLocalQuery). Neither belongs on the Core-0 WiFi task.
//
// text[] comfortably holds the longest legal line — ";MG127,20,getPosition,127"
// is 25 chars. An over-long line is DROPPED at enqueue rather than truncated: a
// clipped command could otherwise re-parse as a different, still-valid one.
struct MaestroCmdMsg { uint8_t sender; char text[48]; };
QueueHandle_t maestroCmdQueue = nullptr;

// =============================================================================
//  HCR Vocalizer routing
//
//  HCR command strings are the same regardless of transport (Serial or
//  WCB-ESP-NOW) — short ASCII frames like "<SH3,QEH,QT>\n".  hcrFormatCommand()
//  builds them; executeHcrAction() picks the destination (Serial3, Serial4,
//  or a remote WCB) and writes the formatted payload there.
//
//  We deliberately do NOT use the upstream HCRVocalizer library:
//    • Its begin() is the only place it touches Serial — and we never call it
//      (Serial3/4 baud comes from rcConfig.auxBaud via setup()'s begin call).
//    • Its update()/receive() are no-ops without setRefreshSpeed() — which
//      this sketch doesn't call.
//    • Its I2C transport path has known upstream bugs that don't compile on
//      modern ESP32 + GCC 14, and we never use I2C anyway.
//  Eliminating the dependency keeps CI builds clean and removes a third-party
//  library from the install chain.
//
//  S3/S4/S5 are all valid local HCR destinations (S5 = v2 "Serial 3" / WCB 3.2 "Serial 5").
// =============================================================================

// =============================================================================
//  SBUS + RC Config
// =============================================================================
SbusReader sbusRx;
// rcConfig is heap-allocated in EXTERNAL PSRAM (ESP32-S3-WROOM-1 N16R8 = 8 MB).
// With the 15 logical-button slots the struct is ~210 KB — it will NOT fit in
// internal DRAM alongside the WCB stack + the ~96 KB SET_CONFIG JSON parse.
//
// NOTE: EXT_RAM_BSS_ATTR does NOT help here — the stock Arduino-ESP32 core is
// built without CONFIG_SPIRAM_ALLOW_BSS_SEG_EXTERNAL_MEMORY, so the attribute is
// a no-op and the struct lands in .bss → linker "dram0_0_seg overflowed". The
// reliable way on the stock core is to allocate from the PSRAM heap at runtime
// (ps_calloc, at the top of setup()).  The global below is just a 4-byte pointer;
// `rcConfig` is a macro alias (see rc_config.h) so all rcConfig.xxx access is
// unchanged.  Mapping/threshold arrays aren't a hot path (read on button events,
// not per-SBUS-byte), so PSRAM latency is irrelevant.
// REQUIRES Arduino IDE: Tools → PSRAM → "OPI PSRAM" (the N16R8 is OCTAL PSRAM).
// If PSRAM is off, ps_calloc() returns null and setup() halts with a message.
RcConfig* g_rcConfig = nullptr;

// =============================================================================
//  Status LED
// =============================================================================
Adafruit_NeoPixel statusLed(STATUS_LED_COUNT, STATUS_LED_PIN, NEO_GRB + NEO_KHZ800);

void setStatusLed(uint32_t color, uint8_t brightness = 20) {
  statusLed.setBrightness(brightness);
  statusLed.setPixelColor(0, color);
  statusLed.show();
}

static const uint32_t C_RED    = 0xFF0000;
static const uint32_t C_GREEN  = 0x00FF00;
static const uint32_t C_BLUE   = 0x0000FF;
static const uint32_t C_YELLOW = 0xFFFF00;
static const uint32_t C_ORANGE = 0xFF8000;
static const uint32_t C_CYAN   = 0x00FFFF;
static const uint32_t C_OFF    = 0x000000;

// LED fault latch — tiny arbiter so a fault outranks the routine SBUS status.
// A non-zero color here means "a persistent fault was detected" (currently:
// wcb->begin() failure in setup; future faults can latch their own color).
// updateStatusLed() displays a latched fault STEADY and skips the SBUS
// indication, so last-writer-wins can't silently erase a fault. 0 = no fault.
uint32_t g_ledFaultColor = 0;

// =============================================================================
//  SBUS state
// =============================================================================
int           sbusValues[24]        = {0};
unsigned long sbusFrameCount        = 0;
unsigned long sbusLastFrameMs       = 0;
unsigned long sbusFpsLastSecond     = 0;
unsigned long sbusFpsCounter        = 0;
int           sbusFps               = 0;
bool          sbusFailsafe          = false;
bool          lostFrameOld          = false;
bool          sbusLiveDump          = false;
unsigned long sbusLiveDumpLastMs    = 0;

int oldValueMatrix  = 100;   // raw matrix SBUS value — kept fresh for status/diagnostics
int oldValueMode    = 100;

// ── Matrix-button edge state machine ─────────────────────────────────────────
// Replaces the old raw-value-delta gate (abs(mxVal-oldValueMatrix) >= 5) that
// silently dropped fast re-presses of the same button (the value was identical
// across the two sampled SBUS frames so no "edge" was seen).
//
// Three-state debounced edge machine (NEUTRAL · TRANSITION · BUTTON):
//   • A press is COMMITTED only after the decoded button is stable in-band for
//     rcConfig.matrixDebounceFrames consecutive frames — rejects single-frame
//     noise and analog resistor-ladder sweep transients.
//   • A release/re-arm is COMMITTED only after NEUTRAL (decoded == 0) is stable
//     for matrixDebounceFrames consecutive frames — so a brief 1-frame neutral
//     dip mid-press (sweep slew, contact bounce) can no longer falsely re-arm
//     and split one press into a phantom double.
//   • Anything not yet stable (short neutral blips, unsettled band) is the
//     implicit TRANSITION state: it changes nothing — no fire, no re-arm.
// matrixDebounceFrames is runtime-configurable (Config modal): 1 = fastest
// (clean digital SBUS source), 2-4 for a noisy analog transmitter matrix.
// Only a true sub-frame tap (press+release inside one ~9-14 ms frame interval)
// is unrecoverable — a hard SBUS-protocol limit, not logic.
bool matrixArmed       = false; // must be re-armed by a confirmed neutral before the first fire
                                // (starting true let a button already in-band at boot fire a phantom press)
int  matrixCandidate   = 0;    // decoded button currently being debounced
int  matrixCandCount   = 0;    // consecutive frames matrixCandidate has held in-band
int  matrixNeutralCount = 0;   // consecutive NEUTRAL (decoded==0) frames — for release debounce

// =============================================================================
//  Mode state
// =============================================================================
int FunctionSwState = 1;   // 1=down, 2=mid, 3=up

// =============================================================================
//  Pending action queue
// =============================================================================
struct PendingAction {
  bool          active;
  unsigned long fireAt;
  RcAction      action;
};
#define PENDING_ACTION_SLOTS 8
PendingAction pendingActions[PENDING_ACTION_SLOTS];

// =============================================================================
//  USB Serial WebSerial monitor state
// =============================================================================
bool          wsMonitorActive   = false;
unsigned long wsMonitorLastSent = 0;
// While the config tool's calibration wizard is open the operator deliberately
// wiggles every stick/knob/switch/button. Suppress ALL action dispatch while
// this is set so nothing (HCR/Maestro/MP3/WCB/Serial) fires during cal.
// Set/cleared via {"type":"CALIB","on":bool}; also force-cleared by
// STOP_MONITOR and a fresh PING so a crashed/closed page can never leave the
// board permanently muted.
bool          calibrationActive = false;

// ── Debug-category bitmask ───────────────────────────────────────────────
// Set by the GUI via {"type":"SET_DEBUG_FLAGS","flags":N}. Default 0 = all
// [DISPATCH] logs silenced (no formatting, no USB-CDC bytes spent). Each
// dispatch log site uses dlog(BIT, ...) which is a no-op when the bit is
// off. Lets the config tool's terminal debug chips actually GATE the
// firmware's output instead of just hiding it client-side.
static uint32_t g_dbgFlags = 0;
#define DBG_MAESTRO    (1u << 0)
#define DBG_WCB        (1u << 1)   // covers both unicast and broadcast sends
#define DBG_WLED       (1u << 2)   // WLED ;L<id> dispatch (local WcbWled::emit + remote forward)
#define DBG_HCR        (1u << 3)
#define DBG_MP3        (1u << 4)
#define DBG_SERIAL     (1u << 5)
#define DBG_DFP        (1u << 6)   // DFPlayer Mini ;D dispatch (local frames + remote forwards)
// Category-gated log. ##__VA_ARGS__ swallows the trailing comma when only
// a fmt is passed. Wraps vlogf so it inherits the same non-blocking USB
// back-pressure handling (see vlogf() definition).
#define dlog(catBit, fmt, ...) do { if (g_dbgFlags & (catBit)) vlogf(fmt, ##__VA_ARGS__); } while (0)
#define WS_MONITOR_INTERVAL_MS  50

// =============================================================================
//  Tap detection state
// =============================================================================
struct TapState {
  int           lastBtn         = 0;   // most recent button that was tapped (sticky across release)
  uint8_t       tapCount        = 0;
  unsigned long lastTapMs       = 0;
  bool          deferredPending = false;
  unsigned long deferredFireAt  = 0;
  int           deferredBtn     = 0;
  uint8_t       deferredTaps    = 0;
  // ── Long-press (tap tier 4) tracking ──────────────────────────────────────
  // A press stays "held" from its debounced commit until processSbus() confirms
  // a debounced NEUTRAL. While held, checkDeferredTap() parks the tap dispatch:
  // holdMs is by definition longer than tapWindowMs, so firing on the tap window
  // alone would dispatch the tap tier before the hold could ever be recognised.
  bool          holdActive      = false;  // a committed press is still physically down
  int           holdBtn         = 0;      // matrix slot (1..RC_NUM_THRESHOLDS) being held
  unsigned long holdStartMs     = 0;      // millis() at the press that opened the hold
  bool          holdFired       = false;  // tier 4 already dispatched for this hold
};
TapState tapState;

// Last-seen switch positions for change detection
int switchPrevPos[RC_NUM_SWITCHES];

// ── Switch settle state ──────────────────────────────────────────────────────
// A 3-position switch swept end-to-end passes THROUGH the middle band for a
// frame or two. Firing on the raw edge dispatched that middle tier in full on
// the way past — a sound, a script, an easing change the pilot never chose. So
// a new position is only a CANDIDATE until it has held for switchSettleMs;
// only the position the switch comes to rest in dispatches.
//   switchCandPos   = position currently being timed (-1 = none pending)
//   switchCandSince = millis() the candidate first appeared
int           switchCandPos  [RC_NUM_SWITCHES];
unsigned long switchCandSince[RC_NUM_SWITCHES];

// =============================================================================
//  Helpers
// =============================================================================
// Decode SBUS value (172..1811 for FrSky -100%..+100%) into a switch
// position. 3-pos toggles cluster at ~172 / ~992 / ~1811, so the
// midpoints between min↔mid and mid↔max are the right thresholds:
//   (172+992)/2 = 582,  (992+1811)/2 = 1401.
// (Earlier 340/680 thresholds were for a 0-1023 range and mis-decoded
// the middle position as "up".)
static inline int readSwitchPos(int sbusVal, uint8_t positions) {
  if (positions == 2) return (sbusVal > 900) ? 2 : 0;
  if (sbusVal < 582)  return 0;
  if (sbusVal > 1401) return 2;
  return 1;
}

static inline int readBoundSwitchSbus(int8_t swIdx) {
  if (swIdx < 0 || swIdx >= RC_NUM_SWITCHES) return -1;
  int ch = rcConfig.switches[swIdx].channel;
  if (ch < 1 || ch > 24) return -1;
  return sbusValues[ch - 1];
}

int pwmToButton(int val) {
  for (int i = 0; i < RC_NUM_THRESHOLDS; i++) {
    // A 0/0 band is the "Unassigned" sentinel — treat it as inert so it
    // can never match (otherwise pwmToButton(0) returns that slot, e.g. a
    // pre-signal/garbage 0 would decode as a real button press).
    if (rcConfig.thresholds[i].minPwm == 0 && rcConfig.thresholds[i].maxPwm == 0)
      continue;
    if (val >= rcConfig.thresholds[i].minPwm && val <= rcConfig.thresholds[i].maxPwm)
      return i + 1;
  }
  return 0;
}

static inline uint16_t sbusToRange(int sbusVal, uint16_t outMin, uint16_t outMax) {
  long mapped = (long)(sbusVal - 172) * (outMax - outMin) / (1811 - 172) + outMin;
  // Clamp to the numeric range regardless of endpoint order. A reversed output
  // (posMin > posMax, e.g. a mirrored servo) makes the map produce a correct
  // DESCENDING ramp; order-dependent clamps (outMin as floor, outMax as ceil)
  // would then fight and pin every in-range value to one extreme. lo/hi fixes it
  // and is behavior-identical in the normal outMin <= outMax case.
  long lo = outMin < outMax ? outMin : outMax;
  long hi = outMin < outMax ? outMax : outMin;
  if (mapped < lo) mapped = lo;
  if (mapped > hi) mapped = hi;
  return (uint16_t)mapped;
}

// Like sbusToRange, but only the UPPER HALF of stick travel is used: the joystick
// CENTER (rest) maps to outMin and full-deflection to outMax; anything at or below
// center pins to outMin. Lets a passthrough servo rest CLOSED at stick-center (no
// half-open panels) at the cost of the lower half of the stick — see
// RcKnobOutput.midClosed. SBUS center = (172+1811)/2. Order-independent clamp so a
// reversed (outMin > outMax) endpoint pair still ramps correctly.
static inline uint16_t sbusToRangeMidClosed(int sbusVal, uint16_t outMin, uint16_t outMax) {
  const int center = (172 + 1811) / 2;          // ≈ 991 — joystick rest position
  if (sbusVal <= center) return outMin;         // lower half → closed (outMin)
  long mapped = (long)(sbusVal - center) * (outMax - outMin) / (1811 - center) + outMin;
  long lo = outMin < outMax ? outMin : outMax;
  long hi = outMin < outMax ? outMax : outMin;
  if (mapped < lo) mapped = lo;
  if (mapped > hi) mapped = hi;
  return (uint16_t)mapped;
}

// =============================================================================
//  Maestro byte-level dispatcher
//
//  Writes a Pololu Maestro command to the destination stream for slot
//  `id` (1-8).  The slot's runtime location (type + device #) is looked up
//  in rcConfig.maestros[].
//
//    type = 1 → bytes go to Serial2 (local wired Maestro)
//    type = 2 → bytes go to the broadcast WCBStream (every WCB forwards)
//    type = 0 → slot disabled, command silently dropped
//
//  This firmware ALWAYS uses Pololu protocol — every Maestro on the network
//  is assumed to have its own device number (0-127), set in Maestro Control
//  Center. Compact protocol is not supported because in our broadcast model
//  multiple Maestros can share the same physical bus; the device-number
//  filter is what keeps them from all responding to every command.
//
//  Pololu protocol frame: 0xAA <device> <cmd_compact & 0x7F> <payload...>
// =============================================================================
// Returns true only if the command was actually emitted on the wire. Callers
// that cache channel state (passthrough smoothing) MUST gate their cache write
// on this, else a dropped write (disabled/invalid slot, or a remote stream not
// yet up) would poison the cache and defeat the self-healing re-apply.
static bool maestroWrite(uint8_t id, uint8_t cmd_compact,
                         const uint8_t* payload, size_t plen) {
  if (id < 1 || id > RC_NUM_MAESTROS) return false;
  const RcMaestroSlot& slot = rcConfig.maestros[id - 1];
  if (slot.type == 0) return false;    // disabled (expected — not an error)
  if (slot.device > 127) {             // invalid Pololu device # (config error)
    dlog(DBG_MAESTRO, "[DISPATCH] Maestro %u: invalid Pololu device # %u (must be 0-127) — skipped\n",
         id, slot.device);
    return false;
  }

  Stream* dest = (slot.type == 1) ? (Stream*)&Serial2 : (Stream*)maestroBroadcast;
  if (!dest) return false;              // remote slot but stream not yet up

  // ;M subroutine-trigger (0xA7) frame bytes now come from the shared WcbCmd library,
  // so the wire frame is one source of truth across the WCB firmware + NaviCore.
  // Byte-identical to the generic path below — {0xAA, slot.device, 0x27, sub} — with the
  // same flush-on-frame-boundary behaviour. NOTE the library arg is the POLOLU DEVICE
  // number (slot.device), NOT NaviCore's slot id.
  if (cmd_compact == 0xA7 && payload && plen == 1) {
    uint8_t frame[4];
    const size_t n = WcbMaestro::buildSubroutineFrame(slot.device, payload[0], frame);  // n == 4
    if (slot.type != 1 && maestroBroadcast && maestroBroadcast->bytesFree() < n)
      maestroBroadcast->flushNow();
    return dest->write(frame, n) == n;
  }

  uint8_t hdr[3] = { 0xAA, slot.device, (uint8_t)(cmd_compact & 0x7F) };
  const size_t frameLen = 3 + ((payload && plen) ? plen : 0);
  // Remote (broadcast WCBStream) path: if this whole Pololu frame won't fit in
  // the stream's buffer, flush first so a burst that programs many channels is
  // split on FRAME boundaries into multiple ESP-NOW packets — never truncated
  // mid-frame (which would forward a corrupt byte-stream to the remote Maestro).
  if (slot.type != 1 && maestroBroadcast && maestroBroadcast->bytesFree() < frameLen)
    maestroBroadcast->flushNow();
  size_t wrote = dest->write(hdr, 3);
  if (payload && plen) wrote += dest->write(payload, plen);
  return wrote == frameLen;             // false if the sink dropped any byte (truncated)
}

// ── Maestro 2-way query (LOCAL slots only — Phase 1) ─────────────────────────
// NaviCore is otherwise fire-and-forget; this is the ONE Maestro READ path. A Pololu
// query (Get Position / Moving State / Errors) makes the Maestro reply with a fixed
// number of RAW, header-less bytes on its TX line (Serial2 RX = GPIO7). The reply
// carries NO address or echo, so we drain stale RX, send exactly ONE request, then
// read exactly `nReply` bytes — the caller knows which query it issued. A LOCAL slot
// (type 1, Serial2) answers synchronously; a REMOTE slot (type 2) instead broadcasts the
// query and its WCB relays the reply back over the mesh as :MQR (consumed async). Runs in
// loop()/Core-1 (execCliLine) where Serial2 I/O is race-free; the ~25 ms deadline
// bounds the stall so a missing Maestro or an unwired RX line can't starve SBUS.
//   return: bytes read (== nReply on success, < nReply on timeout), or
//           -2 = send failed (disabled/bad device#), -3 = REMOTE query sent (reply async).
// Send a REMOTE Maestro read as the shared ";M<dev>,<verb>[,<ch>]" TEXT verb (broadcast), NOT
// raw Pololu bytes — so the hosting WCB's get-relay (maestroRewriteInboundGet → handleMaestroGet)
// reads the Maestro and unicasts ":MQR,<id>,<chan>,<KIND>,<value>" home. <dev> is the slot's
// Pololu device # (must equal the WCB's ?MAESTRO,M<n> id). ch is used only for getPosition.
static bool maestroBroadcastReadVerb(uint8_t id, uint8_t cmd_compact, uint8_t ch) {
  if (!wcb || id < 1 || id > RC_NUM_MAESTROS) return false;
  const char* verb = (cmd_compact == 0x90) ? "getPosition"
                   : (cmd_compact == 0x93) ? "getMovingState"
                   : (cmd_compact == 0xA1) ? "getErrors" : nullptr;
  if (!verb) return false;
  const uint8_t dev = rcConfig.maestros[id - 1].device;
  // The WCB's get-relay only services device 1-8 (handleMaestroGet rejects the rest,
  // and 0/9 are its fan-out ids — a query needs a single answer). A slot outside that
  // range can never get a :MQR back, so say so instead of putting a verb on the mesh
  // that will be dropped and then timing out three seconds later with no explanation.
  if (dev < 1 || dev > 8) {
    dlog(DBG_MAESTRO, "[DISPATCH] Maestro %u: remote read needs device 1-8 (device is %u) — not sent\n", id, dev);
    return false;
  }
  char q[40];
  if (cmd_compact == 0x90) snprintf(q, sizeof(q), ";M%u,%s,%u", dev, verb, ch);
  else                     snprintf(q, sizeof(q), ";M%u,%s", dev, verb);
  wcb->broadcast(q);
  return true;
}

static int maestroLocalQuery(uint8_t id, uint8_t cmd_compact,
                             const uint8_t* payload, size_t plen,
                             uint8_t* reply, size_t nReply) {
  if (id < 1 || id > RC_NUM_MAESTROS) return -2;
  const uint8_t t = rcConfig.maestros[id - 1].type;
  if (t == 0) return -2;                                   // disabled slot
  if (t == 2) {                                            // Remote → send the ;M<dev>,<verb> TEXT (not raw bytes)
    const uint8_t ch = (payload && plen >= 1) ? payload[0] : 0;  // so the hosting WCB's get-relay reads the Maestro
    bool sent = maestroBroadcastReadVerb(id, cmd_compact, ch);   // and unicasts :MQR home (handleMaestroGet). async.
    dlog(DBG_MAESTRO, "[DISPATCH] Maestro %u remote read 0x%02X %s — awaiting :MQR\n",
         id, cmd_compact, sent ? "verb-sent" : "SEND-FAILED");
    return sent ? -3 : -2;                                 // -3 = sent (maeConsumeRemoteReply handles the reply)
  }
  while (Serial2.available()) Serial2.read();              // drop stale RX so the decode aligns
  if (!maestroWrite(id, cmd_compact, payload, plen)) return -2;
  size_t got = 0; const uint32_t t0 = millis();
  while (got < nReply && (millis() - t0) < 25) {
    if (Serial2.available()) reply[got++] = (uint8_t)Serial2.read();
  }
  return (int)got;
}

// Emit a Maestro query result as a compact machine marker [MAE:<slot>]{…} — the
// config tool intercepts it (handleBoardMessage → _maeReadFeed) to fill that slot's
// inline readout and echo a friendly terminal line; a direct USB user just sees the
// JSON. kind: 0=position (ch used), 1=moving-state, 2=errors. `n` = maestroLocalQuery()
// return (bytes read, -1 remote, -2 disabled, -3 remote read sent — answer emits later
// from maePumpRemoteEmits(), so this prints nothing).
static void maestroReportQuery(uint8_t slot, uint8_t kind, uint8_t ch, int n, uint16_t val) {
  if (n == -3) return;                                      // Remote query sent async — the :MQR reply emits later
  const int   want = (kind == 1) ? 1 : 2;
  const char* qk   = (kind == 0) ? "pos" : (kind == 1) ? "mov" : "err";
  const char* err  = (n == -1) ? "remote" : (n == -2) ? "disabled" : (n < want) ? "timeout" : nullptr;
  if (err) {
    if (kind == 0) Serial.printf("[MAE:%u]{\"q\":\"pos\",\"ch\":%u,\"err\":\"%s\"}\n", slot, ch, err);
    else           Serial.printf("[MAE:%u]{\"q\":\"%s\",\"err\":\"%s\"}\n", slot, qk, err);
  } else if (kind == 0) {
    Serial.printf("[MAE:%u]{\"q\":\"pos\",\"ch\":%u,\"val\":%u}\n", slot, ch, val);
  } else if (kind == 1) {
    Serial.printf("[MAE:%u]{\"q\":\"mov\",\"val\":%u}\n", slot, val);
  } else {
    Serial.printf("[MAE:%u]{\"q\":\"err\",\"val\":%u}\n", slot, val);
  }
}

// ── Remote-Maestro read cache + skip-if-running gate ─────────────────────────
// A REMOTE Maestro's read reply (getMovingState/Position/Errors) can't be read
// synchronously — it comes back over the mesh via the WCB relay (see
// docs/WCB_NATIVE_MAESTRO_DESIGN.md §10). onWCBCommand parses the relayed ":MQR,…"
// text into this per-slot cache, which the skip-if-running gate reads.
// pendMask bits (1=pos, 2=mov, 4=err) flag a just-arrived reply for maePumpRemoteEmits()
// to surface as a [MAE:] marker on Core 1; pendCh carries the channel a POS reply is for.
// `relay` is the bridge the read was ASKED over (0 = USB), latched at query time so
// maePumpRemoteEmits() can re-arm the RTERM tee for the answer.
struct MaeRemoteCache { uint16_t pos; uint8_t moving; uint16_t err; uint32_t ms; bool valid;
                        uint8_t pendMask; uint8_t pendCh; uint8_t relay; };
static MaeRemoteCache g_maeRemote[RC_NUM_MAESTROS] = {};
// The skip-if-running freshness window is configurable: rcConfig.maeGateMs (default 250).
// A busy-state older than that — or none at all — is not trusted: the REMOTE gate reads it
// as "unknown" and fails open, the LOCAL gate re-queries (g_maeLocalGate, below). Set it in
// the config tool's Maestro Locations panel.

// Which REMOTE slot carries Pololu device `dev`? The cache is SLOT-keyed (every
// consumer below reads it that way), but the wire speaks DEVICE numbers — we address a
// remote read as ";M<dev>" and the WCB echoes that same <dev> back in its :MQR. The two
// diverge freely: the config tool lets any slot carry any device 0-127, so slot 2 with
// device 7 would otherwise file its answer under slot 7 — poisoning that slot's readout
// and leaving slot 2's gate permanently stale. Returns 0 when no remote slot claims it.
static uint8_t maeRemoteSlotForDevice(uint8_t dev) {
  for (uint8_t s = 1; s <= RC_NUM_MAESTROS; s++)
    if (rcConfig.maestros[s - 1].type == 2 && rcConfig.maestros[s - 1].device == dev) return s;
  return 0;
}

// Remember WHERE a remote read was asked from. Called with maestroLocalQuery()'s return,
// so it only latches on -3 (verb sent, answer coming back async over the mesh). Always
// overwrites — a USB-asked read latches 0, which is what sends its answer to USB only.
static inline void maeLatchRemoteRelay(uint8_t slot, int n) {
  if (n == -3 && slot >= 1 && slot <= RC_NUM_MAESTROS) g_maeRemote[slot - 1].relay = g_rtermRelay;
}

// Parse ":MQR,<dev>,<chan>,<KIND>,<value>" (KIND = POS|MOV|ERR) into g_maeRemote.
// Runs on Core 0 (WiFi RX task) — parse + store only, never any I/O.
static void maeConsumeRemoteReply(const char* body) {
  int id = atoi(body);                                      // body = "<dev>,<chan>,<KIND>,<value>" — <dev>, NOT a slot
  if (id < 1 || id > 255) return;
  const uint8_t slot = maeRemoteSlotForDevice((uint8_t)id); // → the slot that asked
  if (!slot) return;                                        // a reply for a device we don't host remotely
  const char* p = strchr(body, ','); if (!p) return; p++;   // → chan
  int chan = atoi(p);
  const char* k = strchr(p, ',');    if (!k) return; k++;   // → KIND
  const char* v = strchr(k, ',');    if (!v) return;        // v points at the comma before value
  long val = atol(v + 1);
  MaeRemoteCache& c = g_maeRemote[slot - 1];
  if      (strncmp(k, "MOV", 3) == 0) { c.moving = (uint8_t)(val != 0);                  c.pendMask |= 0x02; }
  else if (strncmp(k, "POS", 3) == 0) { c.pos = (uint16_t)val; c.pendCh = (uint8_t)chan; c.pendMask |= 0x01; }
  else if (strncmp(k, "ERR", 3) == 0) { c.err = (uint16_t)val;                           c.pendMask |= 0x04; }
  else return;
  c.ms = millis(); c.valid = true;
}

// Core-1 pump (loop): surface any just-arrived remote reply (flagged by pendMask in
// maeConsumeRemoteReply on Core 0) as a [MAE:<slot>] marker for the config tool's
// "Read live" readout — kept off Core 0 because it does Serial I/O. No-op when idle.
static void maePumpRemoteEmits() {
  for (uint8_t i = 0; i < RC_NUM_MAESTROS; i++) {
    MaeRemoteCache& c = g_maeRemote[i];
    const uint8_t m = c.pendMask;
    if (!m) continue;
    c.pendMask &= ~m;                                        // clear only the bits we're emitting now
    // A remote read answers long after drainRemoteCli() disarmed the capture tee, so
    // without re-arming it here the [MAE:] marker goes to USB only and a read asked
    // over the bridge sits at "…" until the tool gives up. NOT cleared on emit: a
    // second reply for the same slot can land in a later loop() pass, and the next
    // read re-latches it (0 for a USB-asked read). Core 1 — loop() only.
    const uint8_t relay = c.relay;
    if (relay) { rtermSink.begin(wcb, relay); rcSerial.armCapture(&rtermSink); }
    if (m & 0x01) maestroReportQuery(i + 1, 0, c.pendCh, 2, c.pos);
    if (m & 0x02) maestroReportQuery(i + 1, 1, 0,        1, c.moving);
    if (m & 0x04) maestroReportQuery(i + 1, 2, 0,        2, c.err);
    if (relay) { rtermSink.finish(); rcSerial.disarmCapture(); }
  }
}

// LOCAL gate cache, same maeGateMs freshness window the REMOTE branch uses. The local
// read is BLOCKING: maestroLocalQuery spins up to 25 ms on Serial2 when nothing answers,
// and a tap tier fires up to RC_ACTIONS_PER_TIER gated actions back-to-back in ONE loop()
// pass (more still on a non-exclusive multi-tap), so an uncached gate could stall loop()
// for 125 ms+ with SBUS queued behind it. An unanswered read is not an edge case — a Micro
// 6 has no getMovingState at all, and a Maestro whose TX isn't wired back to Serial2 RX is
// a supported install, so on those boards EVERY gated action pays the full timeout.
// Cache it: a whole tier then costs one query at most.
struct MaeLocalGate { uint32_t ms; bool valid; bool moving; };
static MaeLocalGate g_maeLocalGate[RC_NUM_MAESTROS] = {};
// Drop the cached busy-state when WE change what the slot is doing (script start/stop,
// goHome). Without this a second gated action ~immediately after the first would read the
// pre-start "not busy" answer and fire anyway — the exact retrigger skipRunning exists to
// prevent. NOT called from maestroSetTarget: that runs on every passthrough SBUS frame,
// and invalidating there would restore the per-action blocking read this cache removes.
static inline void maeGateInvalidateSlot(uint8_t id) {
  if (id >= 1 && id <= RC_NUM_MAESTROS) g_maeLocalGate[id - 1].valid = false;
}

// Is Maestro `id` mid-movement (a servo still moving)? The skip-if-running gate uses this.
// LOCAL slot → query getMovingState (bounded ~25 ms read), cached per g_maeLocalGate above.
// REMOTE slot → read the WDP-relayed cache; if stale/missing, FAIL OPEN (return false =
// "not busy, go ahead"). getMovingState is a PROXY for "sequence running" — it misses a
// paused / instant-move script, but it's the only native running-signal the Maestro offers.
static bool maestroSequenceBusy(uint8_t id) {
  if (id < 1 || id > RC_NUM_MAESTROS) return false;
  const RcMaestroSlot& slot = rcConfig.maestros[id - 1];
  if (slot.type == 1) {                                     // LOCAL — ask now (bounded raw Serial2 read)
    MaeLocalGate& g = g_maeLocalGate[id - 1];
    if (g.valid && (millis() - g.ms) <= rcConfig.maeGateMs) return g.moving;   // fresh → don't re-block
    uint8_t rb[1]; uint16_t mv = 0;                         // 0x93 = getMovingState (1-byte reply)
    int n = maestroLocalQuery(id, 0x93, nullptr, 0, rb, WcbMaestro::replyLen(WcbMaestro::ReplyKind::MOV));
    const bool busy = WcbMaestro::decodeReply(WcbMaestro::ReplyKind::MOV, rb, (n > 0) ? (size_t)n : 0, mv) && mv != 0;
    g.moving = busy; g.ms = millis(); g.valid = true;
    return busy;
  }
  if (slot.type == 2) {                                     // REMOTE — read the mesh-relayed cache
    const MaeRemoteCache& c = g_maeRemote[id - 1];
    if (c.valid && (millis() - c.ms) <= rcConfig.maeGateMs) return c.moving != 0;   // fresh → trust it
    maestroBroadcastReadVerb(id, 0x93, 0);                  // stale/unknown → warm via ;M<dev>,getMovingState (async :MQR) …
    // Nobody asked for this read — it's the gate warming itself. Drop any relay a
    // previous CLI read latched so the :MQR's [MAE:] marker isn't mirrored to a
    // bridge as if it were an answer to a "Read live" click.
    g_maeRemote[id - 1].relay = 0;
    dlog(DBG_MAESTRO, "[DISPATCH] Maestro %u gate: cache stale → sent getMovingState, fail-open\n", id);
    return false;                                           // … and fail open right now
  }
  return false;                                             // disabled → never "busy"
}

// Extract the mesh Maestro id from a ;M verb for the skip-if-running gate: the comma form
// ;M<id>,verb uses the digits before the comma; the short form ;M<id><seq> uses the FIRST
// digit. Returns 0 when it's not a ;M verb or the id is 0/9/out-of-range (all/local — not a
// single gate-able Maestro).
static uint8_t maeVerbDeviceId(const char* cmd) {
  if (!cmd) return 0;
  const char* p = cmd;
  while (*p && *p != 'M' && *p != 'm') p++;                 // ;M… → the 'M'
  if (!*p) return 0; p++;
  if (*p < '0' || *p > '9') return 0;
  const char* d = p; while (*d >= '0' && *d <= '9') d++;    // run of digits
  int id = (*d == ',') ? atoi(p) : (*p - '0');              // comma form: full number ; short: first digit
  return (id >= 1 && id <= RC_NUM_MAESTROS) ? (uint8_t)id : 0;
}

// Skip-if-running gate for a via-WCB Maestro verb (RA_WCB_UNICAST/BROADCAST). Reads the mesh
// reply cache for the verb's Maestro; if stale, warms it by sending getMovingState to the same
// WCB (wcbId, or broadcast when wcbId==0) and FAILS OPEN. True only on a fresh "moving" reply.
static bool maestroVerbBusy(const char* cmd, uint8_t wcbId) {
  const uint8_t id = maeVerbDeviceId(cmd);                  // DEVICE number, as written in the verb
  if (!id || !wcb) return false;                            // not a single gate-able Maestro → never skip
  // g_maeRemote is SLOT-keyed (see maeRemoteSlotForDevice). A hand-written ";M<dev>"
  // verb may address a Maestro that has no remote slot on this board at all — there is
  // then no cache to consult and nowhere for a reply to land, so fail open silently
  // rather than warming a request whose :MQR would be discarded on arrival.
  const uint8_t slot = maeRemoteSlotForDevice(id);
  if (!slot) return false;
  const MaeRemoteCache& c = g_maeRemote[slot - 1];
  if (c.valid && (millis() - c.ms) <= rcConfig.maeGateMs) return c.moving != 0;
  char q[24]; snprintf(q, sizeof(q), ";M%u,getMovingState", id);
  if (wcbId) wcb->send(wcbId, q); else wcb->broadcast(q);   // warm the cache (async :MQR) …
  dlog(DBG_MAESTRO, "[DISPATCH] Maestro %u via-WCB gate: cache stale → sent %s, fail-open\n", id, q);
  return false;                                             // … and fail open right now
}

// Valid Maestro channel guard (0-31 covers Micro/Mini Maestro 6/12/18/24).
// An out-of-range channel is a config error; warn + skip so it isn't a silent
// no-op the user has to debug by guessing the servo/wiring is broken.
static bool maestroChanOk(uint8_t id, uint8_t ch) {
  if (ch <= 31) return true;
  dlog(DBG_MAESTRO, "[DISPATCH] Maestro %u: channel %u out of range (0-31) — skipped\n", id, ch);
  return false;
}

// Per-(Maestro id 1-8, channel 0-31) cache of the last Speed/Accel WE wrote.
// Passthrough smoothing uses it so Set Speed/Accel are sent ONCE (not every
// SBUS frame) and self-heal: maestroSetSpeed/maestroSetAccel below both update
// this, so if a script action (or ?MAE,FREE) changes a channel, the next
// passthrough stick move notices the mismatch and re-applies the knob's
// profile smoothing. Zero-initialised = "nothing sent yet" (matches unmanaged
// channels; a profile channel with speed>0 always applies on its first move
// since 0 != want).
static uint16_t g_maeSpeed[RC_NUM_MAESTROS][32] = {};
static uint16_t g_maeAccel[RC_NUM_MAESTROS][32] = {};
static const uint16_t MAE_SMOOTH_UNKNOWN = 0xFFFF;   // sentinel: force re-apply on next move (out of the 0-16383/0-255 range)

// ── Auto-release (idle servo de-energize) runtime state, per Maestro channel ──
// A KF_MAESTRO_PASSTHROUGH output with releaseIdleMs>0 opts a servo channel into
// auto-release: the passthrough only writes a target on stick MOVEMENT, so a
// resting servo holds its last target and can buzz/hunt. maestroIdleReleaseTick()
// (loop, Core 1) sends Set Target 0 — the Maestro stops pulsing that channel, so
// the servo goes limp and silent — once it has been idle releaseIdleMs long. The
// next stick move writes a real target (re-energizing) and clears the flag.
//   g_maeReleaseMs   = the active output's releaseIdleMs, copied on each dispatch
//                      (0 = channel not opted in; nothing to release).
//   g_maeLastMoveMs  = millis() of the last movement-driven target write.
//   g_maeReleased    = the channel is currently de-energized (Set Target 0 sent).
// All three are (id-1, ch)-keyed like g_maeSpeed. Not persisted; boot = zeroed.
static uint16_t g_maeReleaseMs [RC_NUM_MAESTROS][32] = {};
static uint32_t g_maeLastMoveMs[RC_NUM_MAESTROS][32] = {};
static bool     g_maeReleased  [RC_NUM_MAESTROS][32] = {};

// Wipe ALL auto-release runtime state (per-channel policy, idle timers, released
// flags). Call after a live config apply / reset: SET_CONFIG applies to RAM without a
// reboot, so a just-disabled or remapped releaseIdleMs would otherwise leave a STALE
// non-zero policy that fires one spurious idle release. Cleared → the policy re-derives
// from the fresh config on the next passthrough dispatch (mirrors reapplyMaestroEasing).
static void resetMaestroReleaseState() {
  memset(g_maeReleaseMs,  0, sizeof(g_maeReleaseMs));
  memset(g_maeLastMoveMs, 0, sizeof(g_maeLastMoveMs));
  memset(g_maeReleased,   0, sizeof(g_maeReleased));
}

// A whole-slot servo-state change happened OUTSIDE the passthrough setTarget path
// (goHome / stopScript re-pose or re-energize channels). Treat it as a "move" for
// auto-release: clear the released flags and re-arm the idle timers, so a channel that
// had been de-energized is considered energized again and only auto-releases after a
// fresh idle period — instead of leaving a stale "released" flag that would keep the
// tick from ever re-releasing it. Only release-enabled channels (g_maeReleaseMs>0) are
// ever acted on by the tick, so this is a no-op for channels that never opted in.
static void maeReleaseArmSlot(uint8_t id) {
  if (id < 1 || id > RC_NUM_MAESTROS) return;
  const uint32_t now = millis();
  for (uint8_t ch = 0; ch < 32; ch++) { g_maeReleased[id - 1][ch] = false; g_maeLastMoveMs[id - 1][ch] = now; }
}

// Per-Maestro "active easing" the SWITCH sets (a "Set active easing" action). It is
// the FALLBACK, not the boss: local easing (a knob's own profile / a script action's
// profile) wins by default; the switch only applies where local has NONE — unless a
// per-context "allow switch to override" opt-in flips it. Not persisted; boot =
// EASE_RELEASED; every change logs (DBG_MAESTRO) so what's in effect is visible.
#define EASE_RELEASED (-1)   // switch imposes nothing → local-only (also = "leave alone")
#define EASE_OFF      (-2)   // switch imposes "no easing" = full speed (drive 0)
static int8_t g_switchEasing[RC_NUM_MAESTROS] = { EASE_RELEASED, EASE_RELEASED, EASE_RELEASED, EASE_RELEASED,
                                                  EASE_RELEASED, EASE_RELEASED, EASE_RELEASED, EASE_RELEASED };

// Resolve the EFFECTIVE easing for a passthrough knob on Maestro `mid`. PRIORITY:
//   knob has its OWN profile  →  that profile wins, UNLESS the knob opts in
//                                (easeSwitchOverride) AND the switch is active
//   knob has NO profile       →  follow the switch (EASE_RELEASED = none/leave alone)
// Returns a profile 0-5, EASE_OFF (-2 = force full speed), or EASE_RELEASED (-1 = leave alone).
static int8_t resolveKnobEasing(const RcKnob& kn, uint8_t mid) {
  const int8_t sw  = (mid >= 1 && mid <= RC_NUM_MAESTROS) ? g_switchEasing[mid - 1] : EASE_RELEASED;
  const int8_t own = kn.smoothProfile;                     // -1 = none, 0-5 = the knob's own profile
  if (own >= 0) return (kn.easeSwitchOverride && sw != EASE_RELEASED) ? sw : own;  // local wins unless opted-in override
  return sw;                                               // no local easing → follow the switch
}

// Invalidate a whole slot's smoothing cache so the next passthrough stick move
// re-applies each channel's speed/accel. Called on Maestro-internal script
// start/stop: a device-side script can change a channel's speed/accel that the
// ESP32 never sees, so without this the cache would read "already applied" and
// the knob's smoothing would never be restored.
static void maeSmoothInvalidateSlot(uint8_t id) {
  if (id < 1 || id > RC_NUM_MAESTROS) return;
  for (uint8_t ch = 0; ch < 32; ch++) { g_maeSpeed[id - 1][ch] = MAE_SMOOTH_UNKNOWN; g_maeAccel[id - 1][ch] = MAE_SMOOTH_UNKNOWN; }
}

// (The old global smoothing-override latch was retired — smoothing is now driven
//  by per-knob / per-script smoothing PROFILES, see rcConfig.smoothProfiles.)

// Pololu Maestro command byte values (compact protocol).
//   0x84 SET_TARGET   · 0x87 SET_SPEED  · 0x89 SET_ACCEL
//   0xA2 GO_HOME      · 0xA4 STOP_SCRIPT · 0xA7 RESTART_SCRIPT_AT_SUB
static void maestroSetTarget(uint8_t id, uint8_t ch, uint16_t pos) {
  if (!maestroChanOk(id, ch)) return;
  if (pos > 16383) pos = 16383;   // target is 14-bit; clamp so an out-of-range value
                                  // is capped, not wrapped to its low 14 bits (wrong servo pos)
  uint8_t p[3] = { ch, (uint8_t)(pos & 0x7F), (uint8_t)((pos >> 7) & 0x7F) };
  maestroWrite(id, 0x84, p, 3);
  if (pos == 0) {
    // Target 0 = the Maestro stops pulsing this channel → servo goes limp, so its TRUE
    // pose is now UNKNOWN. Invalidate the record/replay pose shadow (NOT a shadow of 0,
    // which would make a later replay anchor at the min extreme and sweep the servo up
    // from there). This is the auto-release path (or an explicit disable) — do NOT re-arm
    // the idle timer here (that is what the release IS).
    navirec::shadowInvalidate(id, ch);
  } else {
    navirec::shadowSetTarget(id, ch, pos);   // record/replay last-position shadow (all real moves)
    // Auto-release idle-timer re-arm. ANY real (non-zero) move — passthrough stick,
    // scene/trigger setTarget action, remote, or replay step — restarts the timer and
    // re-energizes a released channel. Centralized here so EVERY mover re-arms, not just
    // passthrough (a scene action holding a servo must not be dropped by a stale stick
    // timer). The per-output releaseIdleMs POLICY is still set at the passthrough
    // dispatch (only it knows the timeout). See maestroIdleReleaseTick().
    if (id >= 1 && id <= RC_NUM_MAESTROS && ch < 32) {
      g_maeLastMoveMs[id - 1][ch] = millis();
      g_maeReleased  [id - 1][ch] = false;
    }
  }
}
static void maestroSetSpeed(uint8_t id, uint8_t ch, uint16_t spd) {
  if (!maestroChanOk(id, ch)) return;
  if (spd > 16383) spd = 16383;   // speed is 14-bit; saturate instead of aliasing (16384 -> 0 = unlimited)
  uint8_t p[3] = { ch, (uint8_t)(spd & 0x7F), (uint8_t)((spd >> 7) & 0x7F) };
  // Cache only what actually went out — a dropped write must NOT mark the
  // channel "applied" (id validity is guaranteed once maestroWrite returns true).
  if (maestroWrite(id, 0x87, p, 3) && ch < 32) {
    g_maeSpeed[id - 1][ch] = spd;
    dlog(DBG_MAESTRO, "[DISPATCH] Maestro %u ch %u  SetSpeed %u\n", id, ch, spd);
  }
}
static void maestroSetAccel(uint8_t id, uint8_t ch, uint8_t accel) {
  if (!maestroChanOk(id, ch)) return;
  // Acceleration (0-255) is sent as TWO 7-bit bytes, same framing as
  // speed/target. The old single-byte form set the high bit for accel>127
  // (corrupting the Pololu data stream) and was one byte short of the frame
  // the Maestro expects for 0x89.
  uint8_t p[3] = { ch, (uint8_t)(accel & 0x7F), (uint8_t)((accel >> 7) & 0x7F) };
  if (maestroWrite(id, 0x89, p, 3) && ch < 32) {   // cache only what actually went out
    g_maeAccel[id - 1][ch] = accel;
    dlog(DBG_MAESTRO, "[DISPATCH] Maestro %u ch %u  SetAccel %u\n", id, ch, accel);
  }
}
static void maestroGoHome(uint8_t id)        { maestroWrite(id, 0xA2, nullptr, 0); navirec::shadowInvalidateSlot(id); maeGateInvalidateSlot(id); maeReleaseArmSlot(id); }
static void maestroStopScript(uint8_t id)    { maestroWrite(id, 0xA4, nullptr, 0); navirec::shadowInvalidateSlot(id); maeSmoothInvalidateSlot(id); maeGateInvalidateSlot(id); maeReleaseArmSlot(id); }
static void maestroRestartScript(uint8_t id, uint8_t sub) {
  maestroWrite(id, 0xA7, &sub, 1);
  maeSmoothInvalidateSlot(id);   // a device-side script may change speed/accel we can't see — re-apply on next stick move
  maeGateInvalidateSlot(id);     // the slot is (re)started — the skip-if-running gate must re-ask
}
// Restart Script at Subroutine WITH parameter (Pololu 0x28) — the value is pushed on
// the Maestro's script stack. Shared by the local ";M<id>,subParam" action and the
// inbound-mesh handler so there is ONE implementation of the frame.
static void maestroSubParam(uint8_t id, uint8_t sub, uint16_t param) {
  if (param > 16383) param = 16383;
  uint8_t p[3] = { sub, (uint8_t)(param & 0x7F), (uint8_t)((param >> 7) & 0x7F) };
  maestroWrite(id, 0xA8, p, 3);  // 0xA8 & 0x7F = 0x28 → {0xAA,dev,0x28,sub,pl,ph} == WcbMaestro::buildSubParam
  maeSmoothInvalidateSlot(id);   // a device-side script may change speed/accel we can't see
  maeGateInvalidateSlot(id);     // the slot is (re)started — the skip-if-running gate must re-ask
}

// Zero speed/accel on every passthrough channel of Maestro `id` that has
// smoothing enabled, so a Maestro script runs at full / its own speed instead
// of being throttled by leftover passthrough smoothing. maestroSetSpeed/Accel
// update the cache to 0, so the knob restores its smoothing on the next stick
// move. Used by the restartScript "reset smoothing first" option.
static void maestroResetSmoothedChannels(uint8_t id) {
  if (id < 1 || id > RC_NUM_MAESTROS) return;
  for (uint8_t ch = 0; ch < 32; ch++) {
    bool managed = false;
    for (int p = 0; p < RC_NUM_SMOOTH_PROFILES && !managed; p++) {
      const RcSmoothEntry& e = rcConfig.smoothProfiles[p].entries[id - 1][ch];
      if (e.speed || e.accel) managed = true;
    }
    if (managed) { maestroSetSpeed(id, ch, 0); maestroSetAccel(id, ch, 0); }
  }
}

// Load a smoothing profile's speed/accel onto Maestro `id`'s channels — used by
// a script action to set the desired smoothing before the script runs.
static void maestroApplySmoothProfile(uint8_t id, int8_t prof) {
  if (id < 1 || id > RC_NUM_MAESTROS || prof < 0 || prof >= RC_NUM_SMOOTH_PROFILES) return;
  const RcSmoothProfile& P = rcConfig.smoothProfiles[prof];
  for (uint8_t ch = 0; ch < 32; ch++) {
    const RcSmoothEntry& e = P.entries[id - 1][ch];
    if (e.speed || e.accel) {
      maestroSetSpeed(id, ch, e.speed > 16383 ? 16383 : e.speed);
      maestroSetAccel(id, ch, e.accel);
    }
  }
}

// Apply an EFFECTIVE easing to a Maestro's channels imperatively (used before a
// script runs): a profile 0-5 → its speed/accel; EASE_OFF → full speed (reset);
// EASE_RELEASED/none → leave the channels as-is.
static void applyScriptEasing(uint8_t id, int8_t easing) {
  if      (easing >= 0 && easing < RC_NUM_SMOOTH_PROFILES) maestroApplySmoothProfile(id, easing);
  else if (easing == EASE_OFF)                             maestroResetSmoothedChannels(id);
}

// Re-drive every passthrough channel on Maestro `id` to its EFFECTIVE easing —
// the profile's speed/accel, or 0 (full speed) when the effective easing is
// EASE_OFF or none (resolveKnobEasing < 0). Called when the SWITCH's active easing
// changes so it takes effect immediately AND channels snap back when the switch
// releases / goes Off. Steady-state stick moves only RE-apply a positive limit (see
// processKnobs) so scripts/EEPROM on unmanaged channels are left alone; this is the
// one place that actively drives 0.
static void reapplyMaestroEasing(uint8_t id) {
  if (id < 1 || id > RC_NUM_MAESTROS) return;
  for (int i = 0; i < RC_NUM_KNOBS; i++) {
    const RcKnob& kn = rcConfig.knobs[i];
    if (kn.function != KF_MAESTRO_PASSTHROUGH) continue;
    const int8_t eff = resolveKnobEasing(kn, id);
    for (int m = 1; m <= 3; m++) {
      const uint8_t       cnt  = rcKnobOutCount(kn, m);
      const RcKnobOutput* outs = rcKnobOuts(kn, m);
      for (uint8_t o = 0; o < cnt && o < RC_KNOB_MAX_OUTPUTS; o++) {
        if (outs[o].target != id || outs[o].maestroCh >= 32) continue;
        const uint8_t ch = outs[o].maestroCh;
        uint16_t wantSpd = 0, wantAcc = 0;
        if (eff >= 0 && eff < RC_NUM_SMOOTH_PROFILES) {
          const RcSmoothEntry& e = rcConfig.smoothProfiles[eff].entries[id - 1][ch];
          wantSpd = e.speed > 16383 ? 16383 : e.speed; wantAcc = e.accel;
        }
        if (g_maeSpeed[id - 1][ch] != wantSpd) maestroSetSpeed(id, ch, wantSpd);
        if (g_maeAccel[id - 1][ch] != wantAcc) maestroSetAccel(id, ch, (uint8_t)wantAcc);
      }
      if (!kn.modeAware) break;
    }
  }
}

// ── Easing retransmit burst ──────────────────────────────────────────────────
// Set Speed / Set Acceleration are FIRE-AND-FORGET in every sense that matters:
// the Pololu protocol has no readback for either (WcbMaestro's reply table is
// POS/MOV/ERR and nothing else), and maestroWrite() reports success as soon as
// the bytes are QUEUED — HardwareSerial::write() returns the byte count
// unconditionally, and a remote slot only buffers into a WCBStream whose actual
// unacked-broadcast result is discarded. So a lost easing write is invisible AND
// never retried: the cache records it as applied and the compare skips it forever.
//
// Since we cannot verify, we repeat. Two extra sends after each easing change,
// spaced far enough apart to clear whatever burst was congesting the link.
// Bounded on purpose — this is a retry, not a heartbeat, so an idle droid puts
// nothing on the mesh.
#define EASE_REPEATS      2      // extra sends after the initial one
#define EASE_REPEAT_MS  500      // spacing
static uint8_t       g_easeRepeatLeft[RC_NUM_MAESTROS] = {};
static unsigned long g_easeRepeatAt  [RC_NUM_MAESTROS] = {};

static void scheduleEasingRepeat(uint8_t id) {
  if (id < 1 || id > RC_NUM_MAESTROS) return;
  g_easeRepeatLeft[id - 1] = EASE_REPEATS;
  g_easeRepeatAt  [id - 1] = millis() + EASE_REPEAT_MS;
}

// Re-send any owed easing repeats. Called from loop() (Core 1, same as every
// other maestroWrite) — see the Core-0 rule above processSbus().
// Re-send ONLY positive easing limits for slot `id`, bypassing the write cache.
//
// This is deliberately NOT reapplyMaestroEasing. That function's job includes
// driving speed/accel to ZERO — it is how "easing Off" resets a channel to full
// speed — and it is correct at the moment the easing actually changes. But a
// REPEAT must never do that: the repeat exists because a write may have been
// lost, and a channel with no profile has no NaviCore-owned value to restore. On
// such a channel the Maestro's own EEPROM speed/accel limits are the truth, and
// writing 0 over them silently removes limits the user configured in Control
// Center. (reapplyMaestroEasing paired with maeSmoothInvalidateSlot did exactly
// that: invalidation makes the cache 0xFFFF, so the 0 always got written.)
//
// So: same predicate as the SBUS hot path — an effective profile, and a channel
// the profile actually defines — and unconditional writes for those only.
static void reassertMaestroEasing(uint8_t id) {
  if (id < 1 || id > RC_NUM_MAESTROS) return;
  for (int i = 0; i < RC_NUM_KNOBS; i++) {
    const RcKnob& kn = rcConfig.knobs[i];
    if (kn.function != KF_MAESTRO_PASSTHROUGH) continue;
    const int8_t eff = resolveKnobEasing(kn, id);
    if (eff < 0 || eff >= RC_NUM_SMOOTH_PROFILES) continue;   // nothing to impose — leave the device alone
    for (int m = 1; m <= 3; m++) {
      const uint8_t       cnt  = rcKnobOutCount(kn, m);
      const RcKnobOutput* outs = rcKnobOuts(kn, m);
      for (uint8_t o = 0; o < cnt && o < RC_KNOB_MAX_OUTPUTS; o++) {
        if (outs[o].target != id || outs[o].maestroCh >= 32) continue;
        const uint8_t ch = outs[o].maestroCh;
        const RcSmoothEntry& e = rcConfig.smoothProfiles[eff].entries[id - 1][ch];
        if (!(e.speed || e.accel)) continue;                  // 0/0 = unset; not ours to drive
        maestroSetSpeed(id, ch, e.speed > 16383 ? 16383 : e.speed);
        maestroSetAccel(id, ch, e.accel);
      }
      if (!kn.modeAware) break;
    }
  }
}

static void easingRepeatTick() {
  // A clip owns the outputs while replaying, and calibration mutes dispatch;
  // same pair of gates processKnobs() applies before touching hardware.
  if (calibrationActive || navirec::isReplaying()) return;
  const unsigned long now = millis();
  for (uint8_t id = 1; id <= RC_NUM_MAESTROS; id++) {
    if (!g_easeRepeatLeft[id - 1]) continue;
    if ((int32_t)(now - g_easeRepeatAt[id - 1]) < 0) continue;   // signed: millis() wrap
    g_easeRepeatLeft[id - 1]--;
    g_easeRepeatAt  [id - 1] = now + EASE_REPEAT_MS;
    reassertMaestroEasing(id);
  }
}

// Adopt the ACTIVE EASING a switch tier selects, WITHOUT executing the tier.
//
// g_switchEasing is otherwise only ever written by an actually-executed setEasing
// action, and processSwitches() seeds a switch's position without firing (at boot
// and after every config apply). So the firmware booted believing "released" no
// matter where the switch physically sat, `resolveKnobEasing` returned < 0, and
// the SBUS hot path skipped easing entirely until the pilot happened to flick the
// switch. This closes that gap at the seed.
//
// Executing the tier instead is NOT an option and must not be "simplified" to it:
// the seed exists precisely so a power-up cannot fire scripts, sounds or servo
// moves. Easing verbs only — they set a variable and touch nothing else. The three
// verbs mirrored here are the same set executeMaestroCmd() accepts (setEasing plus
// the legacy applyProfile / snappy); keep them in sync.
static void seedSwitchEasingFromTier(const RcTier& tier) {
  for (int ai = 0; ai < tier.count && ai < RC_ACTIONS_PER_TIER; ai++) {
    const RcAction& a = tier.a[ai];
    if (a.type != RA_MAESTRO_REMOTE && a.type != RA_MAESTRO_LOCAL) continue;
    const int id = (a.type == RA_MAESTRO_LOCAL) ? 1 : atoi(a.target);
    if (id < 1 || id > RC_NUM_MAESTROS) continue;

    // Parse without strtok: a.cmd is const here, and strtok would also clobber
    // any tokenizer state a caller further up the stack is relying on.
    const char* c = a.cmd;
    const char* arg = strchr(c, ',');
    arg = arg ? arg + 1 : nullptr;
    int8_t v;
    if (!strncmp(c, "setEasing", 9)) {
      v = EASE_RELEASED;
      if (arg) {
        if      (arg[0] == 'p' || arg[0] == 'P') { int p = atoi(arg + 1); if (p >= 0 && p < RC_NUM_SMOOTH_PROFILES) v = (int8_t)p; }
        else if (arg[0] == 'o' || arg[0] == 'O')   v = EASE_OFF;
      }
    } else if (!strncmp(c, "applyProfile", 12)) {          // legacy
      v = EASE_RELEASED;
      if (arg) {
        if      (arg[0] == 'p' || arg[0] == 'P') { int p = atoi(arg + 1); if (p >= 0 && p < RC_NUM_SMOOTH_PROFILES) v = (int8_t)p; }
        else if (arg[0] == 'u' || arg[0] == 'U')   v = EASE_OFF;
      }
    } else if (!strncmp(c, "snappy", 6)) {                 // legacy: bare/on → Off (full speed)
      v = EASE_OFF;
      if (arg && (((arg[0]=='o'||arg[0]=='O') && (arg[1]=='f'||arg[1]=='F')) || arg[0]=='0')) v = EASE_RELEASED;
    } else continue;

    g_switchEasing[id - 1] = v;
    dlog(DBG_MAESTRO, "[DISPATCH] Maestro %d active easing seeded from switch position → %s\n", id,
         v == EASE_RELEASED ? "Release (local-only)" : v == EASE_OFF ? "Off (full speed)" : "profile");
  }
}

// Parse and execute a Maestro action command string against slot `id` (1-8).
// cmd: "setTarget,ch,pos" | "goHome" | "stopScript" | "restartScript,n[,pN][,o]"
//      | "setSpeed,ch,spd" | "setAccel,ch,acc" | "setSpeedAccel,ch,spd,acc"
//      | "setEasing,<pN|off|release>" — the SWITCH sets this Maestro's ACTIVE easing
//        (the fallback the profile priority uses; see resolveKnobEasing).
//   restartScript: "pN" = the action's OWN easing (wins by default); ",o" = allow the
//   switch's active easing to override it. No pN = follow the switch; nothing = leave
//   as-is. (Legacy "applyProfile,<spec>" and "snappy,<state>" still parse → setEasing.)
static void executeMaestroCmd(uint8_t id, const char* cmd) {
  char buf[36];
  strlcpy(buf, cmd, sizeof(buf));
  char* tok = strtok(buf, ",");
  if (!tok) return;
  if      (strcmp(tok, "goHome")        == 0) maestroGoHome(id);
  else if (strcmp(tok, "stopScript")    == 0) maestroStopScript(id);
  else if (strcmp(tok, "setTarget")     == 0) {
    char* sCh = strtok(nullptr, ","); char* sPos = strtok(nullptr, ",");
    if (sCh && sPos) maestroSetTarget(id, (uint8_t)atoi(sCh), (uint16_t)atoi(sPos));
  }
  else if (strcmp(tok, "setSpeed")      == 0) {
    char* sCh = strtok(nullptr, ","); char* sSpd = strtok(nullptr, ",");
    if (sCh && sSpd) maestroSetSpeed(id, (uint8_t)atoi(sCh), (uint16_t)atoi(sSpd));
  }
  else if (strcmp(tok, "setAccel")      == 0) {
    char* sCh = strtok(nullptr, ","); char* sAcc = strtok(nullptr, ",");
    if (sCh && sAcc) maestroSetAccel(id, (uint8_t)atoi(sCh), (uint8_t)atoi(sAcc));
  }
  else if (strcmp(tok, "setSpeedAccel") == 0) {   // both in one action (smoothing)
    char* sCh  = strtok(nullptr, ",");
    char* sSpd = strtok(nullptr, ",");
    char* sAcc = strtok(nullptr, ",");
    if (sCh && sSpd && sAcc) {
      uint8_t ch = (uint8_t)atoi(sCh);
      maestroSetSpeed(id, ch, (uint16_t)atoi(sSpd));
      maestroSetAccel(id, ch, (uint8_t) atoi(sAcc));
    }
  }
  else if (strcmp(tok, "setEasing") == 0) {       // the SWITCH sets this Maestro's active easing (the fallback)
    char* s = strtok(nullptr, ",");               // "pN" = profile | "off" = full speed | "release"/absent = local-only
    if (id >= 1 && id <= RC_NUM_MAESTROS) {
      int8_t v = EASE_RELEASED;
      if (s) {
        if      (s[0] == 'p' || s[0] == 'P') { int p = atoi(s + 1); if (p >= 0 && p < RC_NUM_SMOOTH_PROFILES) v = (int8_t)p; }
        else if (s[0] == 'o' || s[0] == 'O')   v = EASE_OFF;        // "off" → full speed
        // "release" / anything else → EASE_RELEASED (switch imposes nothing)
      }
      g_switchEasing[id - 1] = v;
      reapplyMaestroEasing(id); scheduleEasingRepeat(id);
      dlog(DBG_MAESTRO, "[DISPATCH] Maestro %u active easing → %s\n", id,
           v == EASE_RELEASED ? "Release (local-only)" : v == EASE_OFF ? "Off (full speed)" : "profile");
    }
  }
  else if (strcmp(tok, "applyProfile") == 0) {    // LEGACY master latch → setEasing (pN→profile, u→Off, r/absent→Release)
    char* s = strtok(nullptr, ",");
    if (id >= 1 && id <= RC_NUM_MAESTROS) {
      int8_t v = EASE_RELEASED;
      if (s) { if (s[0]=='p'||s[0]=='P') { int p=atoi(s+1); if (p>=0 && p<RC_NUM_SMOOTH_PROFILES) v=(int8_t)p; }
               else if (s[0]=='u'||s[0]=='U') v = EASE_OFF; }
      g_switchEasing[id - 1] = v; reapplyMaestroEasing(id); scheduleEasingRepeat(id);
    }
  }
  else if (strcmp(tok, "snappy") == 0) {          // LEGACY snappy toggle → setEasing (on/bare→Off full speed, off→Release)
    char* s = strtok(nullptr, ",");
    if (id >= 1 && id <= RC_NUM_MAESTROS) {
      int8_t v = EASE_OFF;                                                         // bare / "on" / "1" → full speed
      if (s && ((((s[0]=='o'||s[0]=='O') && (s[1]=='f'||s[1]=='F'))) || s[0]=='0')) v = EASE_RELEASED;  // "off"/"0" → local-only
      g_switchEasing[id - 1] = v; reapplyMaestroEasing(id); scheduleEasingRepeat(id);
    }
  }
  else if (strcmp(tok, "restartScript") == 0) {
    char* sN    = strtok(nullptr, ",");
    char* sSpec = strtok(nullptr, ",");   // optional "p<N>" = the ACTION's OWN easing (local — wins by default)
    char* sOvr  = strtok(nullptr, ",");   // optional "o"    = allow the switch's active easing to override the action's
    uint8_t sub = sN ? (uint8_t)atoi(sN) : 0;
    // PRIORITY (mirrors the joystick rule): the action's own easing wins; the switch's
    // active easing is the fallback when the action has none — or overrides when the
    // action opts in (",o"). Nothing set anywhere = leave the channels as-is.
    int8_t local = EASE_RELEASED;
    if (sSpec && (sSpec[0] == 'p' || sSpec[0] == 'P')) { int p = atoi(sSpec + 1); if (p >= 0 && p < RC_NUM_SMOOTH_PROFILES) local = (int8_t)p; }
    const bool allowOvr = (sOvr && (sOvr[0] == 'o' || sOvr[0] == 'O'));
    const int8_t sw = (id >= 1 && id <= RC_NUM_MAESTROS) ? g_switchEasing[id - 1] : EASE_RELEASED;
    const int8_t use = (local >= 0) ? ((allowOvr && sw != EASE_RELEASED) ? sw : local)   // has local: local wins unless override
                                    : sw;                                                // no local: follow the switch
    applyScriptEasing(id, use);
    maestroRestartScript(id, sub);
  }
  else if (strcmp(tok, "subParam") == 0) {   // Restart Script at Subroutine WITH parameter (Pololu 0x28)
    char* sN = strtok(nullptr, ",");         // subroutine 0-127
    char* sP = strtok(nullptr, ",");         // parameter 0-16383, pushed on the Maestro's script stack
    if (sN && sP) maestroSubParam(id, (uint8_t)atoi(sN), (uint16_t)atoi(sP));
  }
}

// =============================================================================
//  Serial port write helpers (S3, S4, S5 aux ports)
// =============================================================================
// Write the payload + a trailing CR in as few SoftwareSerial calls as
// possible.  The old form `for (char c : (s + '\r'))` allocated a fresh
// String every call AND wrote one byte at a time; on a bit-banged port
// each write blocks ~1 byte-time, so a long command stalled loop() (and
// thus SBUS) for many ms.  One block write + one CR minimizes the hit.
void writeS3(const String& s) { if (s3) { s3->write((const uint8_t*)s.c_str(), s.length()); s3->write('\r'); } }
void writeS4(const String& s) { if (s4) { s4->write((const uint8_t*)s.c_str(), s.length()); s4->write('\r'); } }
void writeS5(const String& s) { if (s5) { s5->write((const uint8_t*)s.c_str(), s.length()); s5->write('\r'); } }

// =============================================================================
//  HCR command formatter
//
//  Reimplements the byte-string format used by HCRVocalizer::sendCommand() so
//  we can ship the same payload through wcb.sendRaw() for WCB transport. Keep
//  this in sync with the HCR library if its protocol ever changes.
//
//  fn / chan / track follow the same convention as BC firmware's HCRFunction()
//  dispatcher:
//     2  SetEmotion(e, v)
//     3  Trigger(e, v)         (same payload as Stimulate)
//     4  Stimulate(e, v)
//     5  Overload()
//     6  Muse()                (single muse)
//     7  Muse(min, max)        (auto-muse gap in seconds — min=chan, max=track)
//     8  Stop all              (Stops V/A/B audio + emotes)
//     9  StopEmote()
//    10  OverrideEmotions(v)   (v=chan, 0=off/1=on — locks emotion normalization)
//    11  ResetEmotions()
//    13  SetMuse(v)            (v=track, 0=off/1=on — continuous idle musing)
//    14  PlayWAV(ch, track)
//    16  StopWAV(ch)
//    17  SetVolume(ch, vol)
//
//  fn numbers + parameter positions match the BC firmware's HCRFunction()
//  dispatcher exactly (RA_HCR shares rc_config.h with the Body Controller), so a
//  saved action means the same thing on both. The query/poll functions (1, 18+)
//  are intentionally NOT implemented — NaviCore is fire-and-forget.
//
//  Returns an empty String for unknown fn or bad parameters.
// =============================================================================
// Validate + normalize an HCR fn/chan/track triplet IN PLACE. This is the ONE
// place that owns the parameter ranges AND the "emotion 4 = Overload" shortcut
// (fn 3/4 + chan 4 → fn 5). Both transports — local serial (hcrFormatCommand)
// and WCB (hcrFormatWcbCommand) — normalize through here first, so they can
// never drift apart on what an action means. Returns false for an unknown fn
// or out-of-range params (caller skips the action).
static bool hcrNormalizeAction(uint8_t& fn, int& chan, int& track) {
  // The parameter ranges AND the "emotion 4 = Overload" shortcut (fn 3/4 + chan 4
  // → fn 5) now live in the shared WcbCmd HcrCodec (byte-identical extraction of
  // this exact switch). Forwarding here keeps BOTH NaviCore transports — local
  // serial (hcrFormatCommand) and WCB (hcrFormatWcbCommand) — and the WCB
  // firmware validating against ONE source of truth, so they can never drift.
  return HcrCodec::normalize(fn, chan, track);
}

// Local-serial HCR device-wire formatter — a thin wrapper over the shared WcbCmd
// HcrCodec, so the exact bytes an HCR consumes are IDENTICAL whether NaviCore
// formats them here for a locally-wired UART or a WCB formats them off the mesh
// (both compile the same WcbHcr.cpp). The per-channel commanded-volume shadow
// for VolumeUp/DownAll (fn 18/19 — the local HCR protocol has no relative-step
// command and no volume readback) lives inside g_hcr: seeded to a mid default,
// exact after the first SetVolume on this transport. See WcbHcr.h.
static HcrCodec g_hcr;

// Non-blocking per-channel audio-volume fade (shared WcbCmd HcrFade). Drives the
// LOCAL HCR: a FadeIn/FadeOut action calls g_hcrFade.start(...) and loop() ticks it —
// the same ramp a WCB runs for a ;H,FADEIN/FADEOUT, so a locally-wired HCR fades
// identically. Every step writes through g_hcr (fn 17 SetVolume) so the volume shadow
// stays authoritative.
static HcrFade g_hcrFade;

// Pre-fade volume per HcrCodec channel (0=V, 1=A, 2=B); -1 = nothing remembered.
// Every ramp step writes through g_hcr, so DURING a fade g_hcr.getVol(ch) is the live
// ramp value, NOT the level the channel belongs at. A fade started while another is in
// flight must therefore never read the shadow for its anchor: a FadeIn that did latched
// the channel silent for good, because HcrFade completes by writing its own `to` back as
// the commanded volume, so the next FadeIn read 0 as its target too. Latched here at the
// start of a fade and consulted only while g_hcrFade.active(ch) — after a fade ends the
// shadow is authoritative again and this is never read, so it cannot go stale.
static int8_t g_hcrPreFade[3] = { -1, -1, -1 };

// Resolve the LOCAL HCR aux-serial port (S3/S4/S5) from the global HCR destination,
// or nullptr if unbound. Shared by executeHcrAction and the loop() fade tick so both
// resolve the same port.
static Stream* hcrLocalSerial() {
  if      (!strcmp(rcConfig.hcrDest.target, "S3")) return s3;
  else if (!strcmp(rcConfig.hcrDest.target, "S4")) return s4;
  else if (!strcmp(rcConfig.hcrDest.target, "S5")) return s5;
  return nullptr;
}

static String hcrFormatCommand(uint8_t fn, int chan, int track) {
  return g_hcr.format(fn, chan, track);
}

// Build the ";H,FN,<fn>,<chan>,<track>" command an HCR action sends over the
// WCB network. The receiving WCB strips the ';', routes 'H,...' to its
// processHCRRuntimeCommand() FN handler (the "RC-Controller numeric
// convention"), and that drives its locally-wired HCRVocalizer to the WCB's
// own configured HCR port. Gating on hcrFormatCommand() returning non-empty
// reuses the SAME fn/param validation as the local-serial path, so both
// transports accept exactly the same actions. Returns "" for an unknown fn or
// out-of-range params. No trailing newline — this is a WCB command, not a raw
// serial forward (mirrors mp3FormatCommand()).
static String hcrFormatWcbCommand(uint8_t fn, int chan, int track) {
  // Shared validation + normalization (incl. the emotion-4→Overload shortcut)
  // lives in hcrNormalizeAction() — one source of truth for both transports.
  if (!hcrNormalizeAction(fn, chan, track)) return "";
  // Emit the WCB's READABLE ";H,<VERB>" forms (WCB_HCR.cpp processHCRRuntimeCommand)
  // rather than the numeric ";H,FN,fn,chan,track" convention, so the ETM log is
  // self-documenting. EVERY fn NaviCore supports has a verb; params map 1:1 —
  // emotion 0..3 → H/S/M/C, Trigger/Stimulate level 0/1 → MOD/STRONG, audio channel
  // 0/1/2 → V/A/B (fn 17 chan 3 = ALL → channel omitted; the WCB's ;H,VOL loops
  // V/A/B when no channel is named). A WCB too old to speak these verbs must update —
  // that is the intended contract (it only ever spoke a subset of the numeric switch).
  static const char EMO[] = "HSMC";   // NaviCore emotion enum: 0=Happy 1=Sad 2=Mad 3=Scared
  static const char VAB[] = "VAB";    // NaviCore audio-chan enum: 0=V 1=A 2=B
  const char emo = (chan >= 0 && chan <= 3) ? EMO[chan] : '?';   // used by fn 2/3/4
  const char vab = (chan >= 0 && chan <= 2) ? VAB[chan] : 'V';   // used by fn 14/16/17
  switch (fn) {
    case 2:  return String(";H,SETEMOTION,") + emo + "," + track;                 // SetEmotion(e, 0-100)
    case 3:  return String(";H,TRIGGER,")    + emo + "," + (track >= 1 ? "STRONG" : "MOD");  // Trigger(e, MOD|STRONG)
    case 4:  return String(";H,STIM,")       + emo + "," + (track >= 1 ? "STRONG" : "MOD");  // Stimulate(e, MOD|STRONG)
    case 5:  return String(";H,OVERLOAD");
    case 6:  return String(";H,MUSE");                                            // trigger one muse
    case 7:  return String(";H,MUSE,GAP,") + chan + "," + track;                  // Muse(min,max) seconds
    case 8:  return String(";H,STOP");                                            // stop all audio + emotes
    case 9:  return String(";H,STOPEMOTE");
    case 10: return String(";H,OVERRIDE,") + chan;                               // OverrideEmotions(0|1)
    case 11: return String(";H,RESETEMOTIONS");
    // Defensive fallback only: fades are intercepted in executeHcrAction() and normalize()
    // rejects fn 12/15, so this is unreachable — but keep its channel mapping consistent
    // with the live fade path (1=A, 2=B) rather than the wrong 2=B/else=A it had.
    case 12:
    case 15: return String(";H,") + (fn == 12 ? "FADEIN" : "FADEOUT") + "," + (char)(chan == 1 ? 'A' : 'B') + "," + track;
    case 13: return String(";H,MUSE,") + track;                                  // SetMuse(0|1)
    // PlayWAV/StopWAV verbs are A|B only — the V (voice) channel has no verb. Keep the
    // numeric form for chan 0 (V): the WCB's FN handler still delivers PlayWAV/StopWAV on V
    // there, so an existing V-channel WAV action keeps working (no regression from verbs).
    case 14: return (chan == 0) ? (String(";H,FN,14,0,") + track)
                                : (String(";H,PLAY,")    + vab + "," + track);   // PlayWAV(A|B, file)
    case 16: return (chan == 0) ? (String(";H,FN,16,0,") + track)
                                : (String(";H,STOPWAV,") + vab);                 // StopWAV(A|B)
    // SetVolume: a single channel → ;H,VOL,<V|A|B>,<v>; chan 3 = ALL → ;H,VOL,<v>
    // (channel omitted, one message sets V+A+B).
    case 17: return (chan == 3) ? (String(";H,VOL,") + track)
                                : (String(";H,VOL,") + vab + "," + track);
    // Volume Up/Down. chan 0 = ALL (channel omitted, the WCB loops V/A/B); chan 1/2/3 =
    // a single V/A/B channel. track = step (0 → the WCB's default 5).
    case 18:
    case 19: {
      String out = String(";H,") + (fn == 18 ? "VOLUP" : "VOLDN");
      if (chan >= 1 && chan <= 3) out += String(",") + (char)(chan == 1 ? 'V' : chan == 2 ? 'A' : 'B');
      if (track > 0) out += String(",") + track;
      return out;
    }
    default: return String(";H,FN,") + (int)fn + "," + chan + "," + track;       // unreachable: every valid fn has a verb
  }
}

// ── Non-blocking hot-path logging ───────────────────────────────────────────
// loop() must NEVER stall on Serial. On WCB HW 3.2 the USB serial path
// back-pressures when no host is draining it, so a plain Serial.printf() on
// the per-action / per-flush hot path freezes the whole controller whenever
// the config tool isn't connected — heartbeats stop and the WCBs see this
// board go OFFLINE. It also caused the webpage Disconnect hang (firmware
// blocked mid-write while the browser stopped reading).
//
// vlogf() formats into a small stack buffer and writes ONLY if the TX buffer
// currently has room; otherwise the line is silently dropped. It therefore
// can never block, so standalone operation — and a mid-stream disconnect —
// can't freeze loop(). Use it for everything on the hot path; reserve plain
// Serial.* for one-shot setup output and JSON replies to host commands (a
// host is by definition present and draining when it sent the command).
static void vlogf(const char* fmt, ...) {
  char buf[192];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  if (n <= 0) return;
  // vsnprintf returns the length the line WOULD have been, and on truncation it
  // wrote sizeof(buf)-1 chars + a NUL. Clamp to the char count, not the buffer
  // size, or the terminating NUL ships as a payload byte on the console.
  if (n > (int)sizeof(buf) - 1) n = (int)sizeof(buf) - 1;
  if (Serial.availableForWrite() >= n) Serial.write((const uint8_t*)buf, (size_t)n);
  else naviws::writeDirect(buf, (size_t)n);
  // THE THIRD INSTANCE of the availableForWrite() trap, after PWM_UPDATE and
  // rc_trig. Everything dlog() emits comes through here -- every DBG_MAESTRO,
  // DBG_WCB and friend -- so with no USB host attached (i.e. every WiFi session)
  // the entire debug terminal was silent. Turning a category on did nothing at
  // all, which reads as "debug flags are broken" rather than "the transport
  // dropped it".
  //
  // The guard above still stands and is still right: an unguarded USB write
  // blocks up to HWCDC's 50 ms tx timeout and starves the SBUS decode in loop().
  // writeDirect() only appends to the sink buffer, so it can never stall loop(),
  // and on overflow the sink drops whole lines rather than truncating one.
}

// Dispatch an HCR action.
//
// The destination is GLOBAL — pulled from rcConfig.hcrDest rather than from
// the action itself. This lets every HCR action share one configured
// vocalizer wiring; the action only carries fn/chan/track.
static void executeHcrAction(const RcAction& a) {
  const RcHcrDest& dest = rcConfig.hcrDest;

  // transport 2 = the user has this device switched off in the tool's Audio
  // section. Refuse here rather than at the port: a disabled device must not
  // emit anything, even if a stale target still names a live serial port.
  if (dest.transport == 2) {
    dlog(DBG_HCR, "[DISPATCH] HCR is disabled in config — action skipped\n");
    return;
  }

  if (dest.transport == 1) {
    // ── WCB transport (ETM command via ESP-NOW) ────────────────────────────
    // HCR over WCB is UNICAST ONLY and rides the normal WCB command path: we
    // send ";H,FN,<fn>,<chan>,<track>" and the receiving WCB strips the ';',
    // routes 'H,...' to processHCRRuntimeCommand()'s FN handler (the
    // "RC-Controller numeric convention"), which drives its locally-wired
    // HCRVocalizer to the WCB's OWN configured HCR port. So the WCB owns the
    // HCR serial port now — NaviCore no longer specifies it (dest.wcbPort is
    // unused on this path; the receiving WCB must have ?HCR,PORT configured).
    // This replaces the old raw-serial forward (sendRaw, targetID 97), which
    // was best-effort only: wcb->send() defaults to ETM, so a dropped HCR
    // command is retried until ACK'd — HCR can't tolerate a miss. Mirrors the
    // MP3-over-WCB pattern (";A,..." → processMP3AudioCommand). Broadcast is
    // unsupported — an HCR vocalizer is a single device at a known WCB.
    if (!wcb || !wcbReady) { dlog(DBG_HCR, "[DISPATCH] HCR-WCB: WCB not ready — skipped\n"); return; }
    String cmd;
    if (a.fn == 12 || a.fn == 15) {
      // Fade over the mesh — the WCB runs its own HcrFade. Readable verb (NOT the
      // numeric ;H,FN; the WCB's numeric switch has no fade). A/B channels only.
      if (a.chan != 1 && a.chan != 2) {
        dlog(DBG_HCR, "[DISPATCH] HCR-WCB: fade chan must be A(1)/B(2), got %d — skipped\n", a.chan);
        return;
      }
      cmd = String(";H,") + (a.fn == 12 ? "FADEIN" : "FADEOUT") + "," + (char)(a.chan == 1 ? 'A' : 'B') + "," + (int)a.track;
    } else {
      cmd = hcrFormatWcbCommand(a.fn, a.chan, a.track);
    }
    if (cmd.length() == 0) {
      dlog(DBG_HCR, "[DISPATCH] HCR-WCB: bad/unsupported fn=%u chan=%d track=%d — skipped\n",
            a.fn, a.chan, a.track);
      return;
    }
    uint8_t target = (uint8_t)atoi(dest.target);
    if (target < 1 || target > WCB_MAX_BOARDS) {
      dlog(DBG_HCR, "[DISPATCH] HCR-WCB: target '%s' invalid — HCR over WCB must be a "
            "unicast WCB ID 1-%d (broadcast is not supported). Fix the HCR "
            "Destination in the config tool. Not sent.\n",
            dest.target, WCB_MAX_BOARDS);
      return;
    }
    bool ok = wcb->send(target, cmd.c_str());   // ETM by default — HCR can't tolerate a miss
    dlog(DBG_HCR, "[DISPATCH] HCR→WCB%u  %s  %s\n", target, cmd.c_str(), ok ? "OK" : "FAIL");
    return;
  }

  // ── Local serial transport ───────────────────────────────────────────────
  // We format the same byte string the HCRVocalizer library would have
  // written and push it directly to the bound aux-serial port.  No library
  // dependency required — see hcrFormatCommand() for the formatter that
  // mirrors HCRVocalizer's protocol.  S3/S4/S5 are all valid local HCR
  // destinations; an unbound/unknown port falls through to the "unknown
  // port" log below.
  Stream* hcrSerial = hcrLocalSerial();
  if (!hcrSerial) {
    dlog(DBG_HCR, "[DISPATCH] HCR: unknown serial port '%s' — skipped\n", dest.target);
    return;
  }

  // ── Fade (fn 12/15): drive the shared HcrFade on THIS board; loop() ticks it. ──
  if (a.fn == 12 || a.fn == 15) {
    if (a.chan != 1 && a.chan != 2) {
      dlog(DBG_HCR, "[DISPATCH] HCR-Serial: fade chan must be A(1)/B(2), got %d — skipped\n", a.chan);
      return;
    }
    const int ch = a.chan, sec = a.track;
    // Where the channel belongs when the audio is "up". Read the shadow only when no
    // fade owns it; mid-fade the shadow is the ramp's live value (see g_hcrPreFade) and
    // using it as a target would ratchet the channel down — to silence, permanently, if
    // the ramp is still near 0. `from` still tracks the live level so a fade that
    // supersedes another continues from where the audio actually is, without a jump.
    const int base = (g_hcrFade.active(ch) && g_hcrPreFade[ch] >= 0)
                   ? (int)g_hcrPreFade[ch] : g_hcr.getVol(ch);
    g_hcrPreFade[ch] = (int8_t)base;                        // survives this fade being superseded
    if (a.fn == 12) g_hcrFade.start(g_hcr, *hcrSerial, ch, 0, base, sec, false, 0);   // FadeIn: 0 → pre-fade level
    else { const int cur = g_hcr.getVol(ch); g_hcrFade.start(g_hcr, *hcrSerial, ch, cur, 0, sec, true, base); }  // FadeOut: cur → 0, StopWAV, restore pre-fade level
    dlog(DBG_HCR, "[DISPATCH] HCR→%s  Fade%s ch=%d %ds\n", dest.target, a.fn == 12 ? "In" : "Out", a.chan, sec);
    return;
  }

  // A manual audio command supersedes any in-flight fade on the channel(s) it touches —
  // mirror the WCB (WCB_HCR.cpp calls hcrCancelFade in PLAY/STOPWAV/VOL/VOLUP/VOLDN),
  // else loop()'s fade tick would overwrite the manual change within STEP_MS.
  if (a.fn == 14 || a.fn == 16 || a.fn == 17 || a.fn == 18 || a.fn == 19) {
    if (a.fn == 18 || a.fn == 19) {              // Vol chan: 0=ALL, 1/2/3=V/A/B
      if (a.chan == 0) { g_hcrFade.cancel(0); g_hcrFade.cancel(1); g_hcrFade.cancel(2); }
      else             { g_hcrFade.cancel(a.chan - 1); }
    } else if (a.chan == 3) {                    // fn 14/16/17 audio enum: 0=V,1=A,2=B; fn 17 chan 3 = ALL
      g_hcrFade.cancel(0); g_hcrFade.cancel(1); g_hcrFade.cancel(2);
    } else if (a.chan >= 0 && a.chan <= 2) {
      g_hcrFade.cancel(a.chan);
    }
  }

  // ── Per-channel Vol +/- (fn 18/19, chan 1/2/3 = V/A/B): HcrCodec's 18/19 are
  //    ALL-channel, so synthesize one channel from the shadow + an absolute SetVolume. ──
  String payload;
  if ((a.fn == 18 || a.fn == 19) && a.chan >= 1 && a.chan <= 3) {
    const int ch = a.chan - 1;                          // 1/2/3 → V/A/B (0/1/2)
    int step = (a.track > 0) ? a.track : 5;
    if (a.fn == 19) step = -step;
    int nv = g_hcr.getVol(ch) + step;
    if (nv < 0) nv = 0; else if (nv > 99) nv = 99;
    payload = g_hcr.format(17, ch, nv);                 // SetVolume(ch, nv) — updates the shadow
  } else {
    payload = hcrFormatCommand(a.fn, a.chan, a.track);  // chan 0 = ALL + every other fn (byte-identical to today)
  }
  if (payload.length() == 0) {
    dlog(DBG_HCR, "[DISPATCH] HCR-Serial: bad/unsupported fn=%u chan=%d track=%d — skipped\n",
          a.fn, a.chan, a.track);
    return;
  }
  dlog(DBG_HCR, "[DISPATCH] HCR→%s  fn=%u chan=%d track=%d  %s",
        dest.target, a.fn, a.chan, a.track, payload.c_str());
  hcrSerial->print(payload);
}

// Build the ";A,<CMD>" MP3 verb for an RA_MP3 action — the SINGLE producer for
// BOTH transports: sent verbatim to a WCB (remote), and ALSO fed to g_mp3.handle()
// for the local-serial path (see executeMp3Action), so one command string drives
// the MP3 Trigger identically over the mesh or on a local UART. Mirrors the WCB's
// ";A,<CMD>" set (WCB_MP3.cpp). Returns "" for an unknown fn or out-of-range arg.
// No trailing newline: as a WCB command the receiver strips the ';' and routes
// 'A...' to its MP3 driver — not a raw serial forward.
static String mp3FormatCommand(uint8_t fn, int16_t arg) {
  // One set of ranges for both transports (local g_mp3.handle + WCB relay), so an
  // action that's valid locally is valid remotely and vice-versa — never forward
  // a garbage track/index/volume one path would have rejected.
  switch (fn) {
    case MP3_PLAY:
      if (arg < 1 || arg > 255) return "";                // track 1-255
      return String(";A,PLAY,")   + arg;
    case MP3_PLAYFS:
      if (arg < 0 || arg > 255) return "";                // index 0-255
      return String(";A,PLAYFS,") + arg;
    case MP3_STOP:   return ";A,STOP";
    case MP3_NEXT:   return ";A,NEXT";
    case MP3_PREV:   return ";A,PREV";
    case MP3_VOL:
      if (arg < 0 || arg > 64) return "";                 // 0=loudest..64=inaudible
      return String(";A,VOL,")    + arg;
    case MP3_VOLUP:  return ";A,VOLUP";
    case MP3_VOLDN:  return ";A,VOLDN";
    default:         return "";
  }
}

// ── Local MP3 Trigger (v2.x) serial driver ──────────────────────────────────
// Used when rcConfig.mp3Dest.transport == 0 (MP3 Trigger wired to this board's
// S3/S4/S5). The native byte protocol —
//   play track    : 'v' <vol>  then 't' <track>
//   play file idx : 'v' <vol>  then 'p' <idx>
//   stop toggle   : 'O'        next: 'F'   prev: 'R'
//   set volume    : 'v' <vol>  (0=loudest .. 64=inaudible)
// — plus the pre-play volume shadow now live in the shared WcbCmd Mp3Codec, the
// SAME parser a WCB runs on its receive side. executeMp3Action() formats the ;A
// verb (mp3FormatCommand) and hands it to g_mp3.handle(), so a locally-wired MP3
// Trigger and one reached over the mesh consume byte-identical serial (WcbMp3.h).
// No g_mp3.poll(): the local path stays fire-and-forget — no ONFIN/RX — exactly
// as the old direct driver did. The shadow seeds to 20, matching the old default.
static Mp3Codec g_mp3;

// Dispatch an RA_MP3 action. Destination is GLOBAL (rcConfig.mp3Dest).
//   transport 0 = local serial (S3/S4/S5) — drive the MP3 Trigger directly here.
//   transport 1 = WCB unicast — ";A,..." command to one WCB whose own MP3
//                 driver (configured there via ?MP3,S<port>) does the serial.
static void executeMp3Action(const RcAction& a) {
  const RcMp3Dest& dest = rcConfig.mp3Dest;

  if (dest.transport == 2) {   // disabled in the tool's Audio section
    dlog(DBG_MP3, "[DISPATCH] MP3 Trigger is disabled in config — action skipped\n");
    return;
  }

  if (dest.transport == 0) {
    // ── Local serial transport ───────────────────────────────────────────
    Stream* p = nullptr;
    if      (!strcmp(dest.target, "S3")) p = s3;
    else if (!strcmp(dest.target, "S4")) p = s4;
    else if (!strcmp(dest.target, "S5")) p = s5;   // both boards (v2 GPIO38/47, WCB 3.2 GPIO9/10)
    if (!p) {
      dlog(DBG_MP3, "[DISPATCH] MP3-local: unknown serial port '%s' — skipped\n", dest.target);
      return;
    }
    // Format the ;A verb with the shared producer, then let g_mp3.handle() emit
    // the MP3 Trigger's native bytes — the SAME shared codec a WCB runs on its
    // receive side, so local and over-mesh drive the Trigger identically.
    String cmd = mp3FormatCommand(a.fn, a.track);
    if (cmd.length() == 0) {
      dlog(DBG_MP3, "[DISPATCH] MP3-local: bad/out-of-range fn=%u arg=%d — skipped\n", a.fn, a.track);
      return;
    }
    g_mp3.begin(*p);                          // rebind — the resolved port can change per action
    bool ok = g_mp3.handle(cmd.c_str() + 1);  // skip leading ';' (handle tolerates the 'A' verb)
    dlog(DBG_MP3, "[DISPATCH] MP3→%s  fn=%u arg=%d vol=%u  %s\n",
          dest.target, a.fn, a.track, g_mp3.volume(), ok ? "OK" : "FAIL");
    return;
  }

  // ── WCB unicast transport ──────────────────────────────────────────────
  if (!wcb || !wcbReady) { dlog(DBG_MP3, "[DISPATCH] MP3: WCB not ready — skipped\n"); return; }
  String cmd = mp3FormatCommand(a.fn, a.track);
  if (cmd.length() == 0) {
    dlog(DBG_MP3, "[DISPATCH] MP3: bad fn=%u — skipped\n", a.fn);
    return;
  }
  uint8_t target = (uint8_t)atoi(dest.target);
  if (target < 1 || target > WCB_MAX_BOARDS) {
    dlog(DBG_MP3, "[DISPATCH] MP3: target '%s' invalid — set MP3 Destination to a "
          "WCB ID 1-%d in the config tool. Not sent.\n",
          dest.target, WCB_MAX_BOARDS);
    return;
  }
  bool ok = wcb->send(target, cmd.c_str());
  dlog(DBG_MP3, "[DISPATCH] MP3→WCB%u  %s  %s\n", target, cmd.c_str(), ok ? "OK" : "FAIL");
}

// Build the ";D,<CMD>" DFPlayer verb for an RA_DFPLAYER action — the SINGLE
// producer for BOTH transports, exactly as mp3FormatCommand() is for ;A: sent
// verbatim to a WCB (remote), and ALSO fed to g_dfp.handle() for the local-serial
// path (see executeDfpAction), so one command string drives the player identically
// over the mesh or on a local UART. Mirrors WcbCmd's DfPlayerCodec::handle() verb
// set. Returns "" for an unknown fn or out-of-range arg.
//
// The bounds here MUST match DfPlayerCodec::handle()'s — an action that is valid
// locally has to be valid remotely and vice-versa, so a garbage folder/track/volume
// is never forwarded to a WCB that would only reject it. See
// docs/DFPLAYER_DESIGN.md §3 for the verb table.
static String dfpFormatCommand(uint8_t fn, int8_t chan, int16_t track) {
  switch (fn) {
    case DFP_PLAY:
      if (track < 1 || track > 2999) return "";                    // global track index
      return String(";D,PLAY,") + track;
    case DFP_FOLDER:
      if (chan < 1 || chan > 99 || track < 1 || track > 255) return "";
      return String(";D,FOLDER,") + (int)chan + "," + track;       // /01/002.mp3 layout
    case DFP_MP3FOLDER:
      if (track < 1 || track > 9999) return "";                    // the reserved /MP3 folder
      return String(";D,MP3FOLDER,") + track;
    case DFP_STOP:      return ";D,STOP";
    case DFP_NEXT:      return ";D,NEXT";
    case DFP_PREV:      return ";D,PREV";
    case DFP_PAUSE:     return ";D,PAUSE";
    case DFP_RESUME:    return ";D,RESUME";
    case DFP_VOL:
      // 0 = SILENT, 30 = LOUDEST — the INVERSE of the MP3 Trigger's scale above.
      // Do not "harmonise" these; each action type carries its device's own scale.
      if (track < 0 || track > 30) return "";
      return String(";D,VOL,") + track;
    case DFP_VOLUP:     return ";D,VOLUP";
    case DFP_VOLDN:     return ";D,VOLDN";
    case DFP_LOOP:
      if (track < 1 || track > 2999) return "";
      return String(";D,LOOP,") + track;
    case DFP_LOOPALL:
      if (chan < 0 || chan > 1) return "";
      return String(";D,LOOPALL,") + (int)chan;
    case DFP_LOOPFOLDER:
      if (chan < 1 || chan > 99) return "";
      return String(";D,LOOPFOLDER,") + (int)chan;
    case DFP_RANDOM:    return ";D,RANDOM";
    case DFP_EQ:
      if (chan < 0 || chan > 5) return "";                         // Normal/Pop/Rock/Jazz/Classic/Bass
      return String(";D,EQ,") + (int)chan;
    case DFP_DEVICE:
      if (chan < 1 || chan > 5) return "";                         // 1 USB 2 SD 3 AUX 4 sleep 5 flash
      return String(";D,DEVICE,") + (int)chan;
    case DFP_RESET:     return ";D,RESET";
    default:            return "";
  }
}

// ── Local DFPlayer Mini serial driver ───────────────────────────────────────
// Used when rcConfig.dfpDest.transport == 0 (DFPlayer wired to this board's
// S3/S4/S5). The 10-byte frame protocol (7E FF 06 CMD ACK PH PL CKH CKL EF, with
// checksum = -(sum of bytes 1..6)) lives in the shared WcbCmd DfPlayerCodec — the
// SAME parser a WCB runs on its receive side. executeDfpAction() formats the ;D
// verb (dfpFormatCommand) and hands it to g_dfp.handle(), so a locally-wired
// DFPlayer and one reached over the mesh consume byte-identical serial.
//
// No g_dfp.poll(), matching the local MP3 path: fire-and-forget, no ONFIN/RX. The
// DFRobot library is deliberately NOT a dependency — its sendStack() delay(10)
// (ACK off) or blocking wait (ACK on) would both wreck the ~9 ms SBUS cadence.
// At the DFPlayer's fixed 9600 baud a frame takes ~10.4 ms to shift out, so
// back-to-back commands in one loop() pass are paced by the UART itself.
static DfPlayerCodec g_dfp;

// Dispatch an RA_DFPLAYER action. Destination is GLOBAL (rcConfig.dfpDest).
//   transport 0 = local serial (S3/S4/S5) — drive the DFPlayer directly here.
//   transport 1 = WCB unicast — ";D,..." command to one WCB whose own DFPlayer
//                 driver (configured there via ?DFP,S<port>) does the serial.
static void executeDfpAction(const RcAction& a) {
  const RcDfpDest& dest = rcConfig.dfpDest;

  if (dest.transport == 2) {   // disabled in the tool's Audio section
    dlog(DBG_DFP, "[DISPATCH] DFPlayer is disabled in config — action skipped\n");
    return;
  }

  if (dest.transport == 0) {
    // ── Local serial transport ───────────────────────────────────────────
    Stream* p = nullptr;
    if      (!strcmp(dest.target, "S3")) p = s3;
    else if (!strcmp(dest.target, "S4")) p = s4;
    else if (!strcmp(dest.target, "S5")) p = s5;
    if (!p) {
      dlog(DBG_DFP, "[DISPATCH] DFP-local: unknown serial port '%s' — skipped\n", dest.target);
      return;
    }
    String cmd = dfpFormatCommand(a.fn, a.chan, a.track);
    if (cmd.length() == 0) {
      dlog(DBG_DFP, "[DISPATCH] DFP-local: bad/out-of-range fn=%u chan=%d track=%d — skipped\n",
            a.fn, a.chan, a.track);
      return;
    }
    g_dfp.begin(*p);                          // rebind — the resolved port can change per action
    bool ok = g_dfp.handle(cmd.c_str() + 1);  // skip leading ';' (handle tolerates the 'D' verb)
    dlog(DBG_DFP, "[DISPATCH] DFP→%s  fn=%u chan=%d track=%d vol=%u  %s\n",
          dest.target, a.fn, a.chan, a.track, g_dfp.volume(), ok ? "OK" : "FAIL");
    return;
  }

  // ── WCB unicast transport ──────────────────────────────────────────────
  if (!wcb || !wcbReady) { dlog(DBG_DFP, "[DISPATCH] DFP: WCB not ready — skipped\n"); return; }
  String cmd = dfpFormatCommand(a.fn, a.chan, a.track);
  if (cmd.length() == 0) {
    dlog(DBG_DFP, "[DISPATCH] DFP: bad fn=%u — skipped\n", a.fn);
    return;
  }
  uint8_t target = (uint8_t)atoi(dest.target);
  if (target < 1 || target > WCB_MAX_BOARDS) {
    dlog(DBG_DFP, "[DISPATCH] DFP: target '%s' invalid — set DFPlayer Destination to a "
          "WCB ID 1-%d in the config tool. Not sent.\n",
          dest.target, WCB_MAX_BOARDS);
    return;
  }
  bool ok = wcb->send(target, cmd.c_str());
  dlog(DBG_DFP, "[DISPATCH] DFP→WCB%u  %s  %s\n", target, cmd.c_str(), ok ? "OK" : "FAIL");
}

// Dispatch an RA_WLED action. The ";L<id>,<verb>" command in a.cmd (authored via
// the command library) selects a slot in the GLOBAL wledSlots[] routing table —
// NaviCore's mirror of the WCB's wledConfigs. Routing follows the WCB
// (WCB_WLED.cpp processWLEDRuntimeCommand):
//   • id 0 (bare ";L,")             → the lowest-id configured LOCAL slot
//   • local slot (serialPort 3/4/5) → WcbWled::emit(port, verb-body) — the SAME
//                                     shared translator a WCB runs, so a WLED on a
//                                     NaviCore aux port sees byte-identical JSON
//   • remote slot (remoteWCB 1-20)  → wcb->send(remoteWCB, full ";L<id>,…"); the
//                                     host WCB's own router drives its WLED
static void executeWledAction(const RcAction& a) {
  // ── Parse the id: digits immediately after 'L' (";L3,PS,2"→3; ";L,ON"→0) ──
  const char* s = a.cmd;
  while (*s == ' ' || *s == '\t') s++;      // tolerate leading whitespace (matches the WCB's body.trim())
  if (*s == ';') s++;                       // optional leading command char
  if (*s != 'L' && *s != 'l') {
    dlog(DBG_WLED, "[DISPATCH] WLED: '%s' is not a ;L command — skipped\n", a.cmd);
    return;
  }
  s++;                                      // past 'L'
  int id = 0;
  while (*s >= '0' && *s <= '9') { if (id < 100) id = id * 10 + (*s - '0'); s++; }  // cap: no int overflow on a corrupt cmd
  if (id > 9) {                             // valid WLED ids are 1-9 (0 = bare) — mirror the WCB's range check
    dlog(DBG_WLED, "[DISPATCH] WLED: id %d out of range (1-9) — skipped\n", id);
    return;
  }
  if (*s == ',') s++;                       // past the id/verb separator
  const char* body = s;                     // id-stripped verb body ("PS,2" / "ON")

  // ── Find the slot ──
  int slot = -1;
  if (id == 0) {
    // Bare ;L, — act on this board's lowest-id LOCAL WLED (mirror of the WCB).
    for (int i = 0; i < RC_NUM_WLED; i++) {
      const RcWledSlot& w = rcConfig.wledSlots[i];
      if (w.configured && w.remoteWCB == 0 && w.serialPort >= 3 && w.serialPort <= 5)
        if (slot < 0 || w.wledID < rcConfig.wledSlots[slot].wledID) slot = i;
    }
    if (slot < 0) { dlog(DBG_WLED, "[DISPATCH] WLED: bare ;L but no LOCAL WLED configured — skipped\n"); return; }
  } else {
    for (int i = 0; i < RC_NUM_WLED; i++)
      if (rcConfig.wledSlots[i].configured && rcConfig.wledSlots[i].wledID == (uint8_t)id) { slot = i; break; }
    if (slot < 0) { dlog(DBG_WLED, "[DISPATCH] WLED %d not configured — skipped\n", id); return; }
  }
  const RcWledSlot& w = rcConfig.wledSlots[slot];

  // ── Route ──
  if (w.remoteWCB == 0 && w.serialPort >= 3 && w.serialPort <= 5) {
    Stream* port = (w.serialPort == 3) ? s3 : (w.serialPort == 4) ? s4 : s5;
    if (!port) {
      dlog(DBG_WLED, "[DISPATCH] WLED %u: local S%u not available on this board — skipped\n", w.wledID, w.serialPort);
      return;
    }
    // nullptr diag sink, deliberately: WcbWled's parse-error paths write to the
    // Print* with unguarded println/printf — no availableForWrite() room check
    // and no g_dbgFlags gate — which is exactly the blocking-Serial hot-path
    // pattern vlogf() exists to prevent. The dlog() below already reports the
    // no-op through the non-blocking, category-gated path. (Matches g_mp3/g_dfp,
    // which both take the default diag = nullptr.)
    bool ok = WcbWled::emit(*port, body, nullptr);   // build ;L verb → WLED JSON, newline-framed
    dlog(DBG_WLED, "[DISPATCH] WLED %u→S%u  %s  %s\n", w.wledID, w.serialPort, body, ok ? "OK" : "no-op");
  } else if (w.remoteWCB >= 1 && w.remoteWCB <= WCB_MAX_BOARDS) {
    if (!wcb || !wcbReady) { dlog(DBG_WLED, "[DISPATCH] WLED %u: WCB not ready — skipped\n", w.wledID); return; }
    bool ok = wcb->send(w.remoteWCB, a.cmd);          // forward the full ";L<id>,…" string
    dlog(DBG_WLED, "[DISPATCH] WLED %u→WCB%u  %s  %s\n", w.wledID, w.remoteWCB, a.cmd, ok ? "OK" : "FAIL");
  } else {
    dlog(DBG_WLED, "[DISPATCH] WLED %u: slot has no valid destination — skipped\n", w.wledID);
  }
}

// =============================================================================
//  rcExecuteAction — single-action dispatcher
// =============================================================================
// Forward declaration: scheduleAction() (queue-full path) and
// checkPendingActions() fire actions through rcExecuteActionNow(), which is
// defined further below. Declared explicitly so the call sites don't depend on
// the Arduino auto-prototype generator handling this static function.
static void rcExecuteActionNow(const RcAction& a);

static void scheduleAction(const RcAction& action, unsigned long delayMs) {
  for (int i = 0; i < PENDING_ACTION_SLOTS; i++) {
    if (!pendingActions[i].active) {
      pendingActions[i] = { true, millis() + delayMs, action };
      return;
    }
  }
  // Queue full — fire the action NOW (dropping its delay) rather than losing
  // it. rcExecuteActionNow bypasses the delay check, so this can't recurse.
  vlogf("WARN: pendingActions full (%d slots) — firing now, delay dropped\n", PENDING_ACTION_SLOTS);
  rcExecuteActionNow(action);
}

// Dispatch an action's EFFECT immediately — no delay handling, no calibration
// gate (callers handle those). Split out from rcExecuteAction so the delayed
// path fires WITHOUT re-entering the delay check: previously checkPendingActions
// called rcExecuteAction, which saw delayMs>0 and re-queued the action forever,
// so a delayed action's real effect never ran.
static void rcExecuteActionNow(const RcAction& a) {
  // Record/replay capture tap. Gated internally (no-op unless recording), and a
  // non-blocking queue hop. All dispatch now lands on Core 1 (remote ESP-NOW
  // TRIGGERs are deferred here via drainRemoteTriggers); the queue hop stays as a
  // cheap cross-core safeguard for the capture buffer.
  navirec::captureAction(a);
  switch (a.type) {
    case RA_WCB_UNICAST: {
      uint8_t boardId = (uint8_t)atoi(a.target);
      if (boardId >= 1 && boardId <= WCB_MAX_BOARDS) {
        if (!wcb || !wcbReady) { dlog(DBG_WCB, "[DISPATCH] WCB→%d skipped — WCB not ready\n", boardId); break; }
        if (a.skipRunning && maestroVerbBusy(a.cmd, boardId)) { dlog(DBG_MAESTRO, "[DISPATCH] WCB→%d Maestro skipped — already running  %s\n", boardId, a.cmd); break; }
        dlog(DBG_WCB, "[DISPATCH] WCB→%d  %s\n", boardId, a.cmd);
        wcb->send(boardId, a.cmd);
      }
      break;
    }
    case RA_WCB_BROADCAST:
      if (!wcb || !wcbReady) { dlog(DBG_WCB, "[DISPATCH] WCB broadcast skipped — WCB not ready\n"); break; }
      if (a.skipRunning && maestroVerbBusy(a.cmd, 0)) { dlog(DBG_MAESTRO, "[DISPATCH] WCB broadcast Maestro skipped — already running  %s\n", a.cmd); break; }
      dlog(DBG_WCB, "[DISPATCH] WCB broadcast  %s\n", a.cmd);
      wcb->broadcast(a.cmd);
      break;

    case RA_MAESTRO_LOCAL:
      // Legacy "local Maestro" — treat as Maestro ID 1 for backward compat
      // with old configs.  The location of Maestro 1 (and whether it's
      // actually wired locally) is now defined in the Maestro Locations panel.
      dlog(DBG_MAESTRO, "[DISPATCH] Maestro (legacy local → ID 1)  %s\n", a.cmd);
      if (a.skipRunning && maestroSequenceBusy(1)) { dlog(DBG_MAESTRO, "[DISPATCH] Maestro 1 skipped — already running\n"); break; }
      executeMaestroCmd(1, a.cmd);
      break;

    case RA_MAESTRO_REMOTE: {
      // RA_MAESTRO_REMOTE is now the unified "Maestro" action — target holds
      // the Maestro slot ID (1-8). Wiring per ID is in rcConfig.maestros[].
      int id = atoi(a.target);
      if (id < 1 || id > RC_NUM_MAESTROS) {
        vlogf("WARN: Maestro action with invalid ID %d (target='%s')\n", id, a.target);
        break;
      }
      dlog(DBG_MAESTRO, "[DISPATCH] Maestro %d  %s\n", id, a.cmd);
      if (a.skipRunning && maestroSequenceBusy((uint8_t)id)) { dlog(DBG_MAESTRO, "[DISPATCH] Maestro %d skipped — already running\n", id); break; }
      executeMaestroCmd((uint8_t)id, a.cmd);
      break;
    }
    case RA_SERIAL: {
      String s(a.cmd);
      { int _pi = (a.target[0] == 'S' && a.target[1] >= '3' && a.target[1] <= '5') ? (a.target[1] - '3') : -1;
        dlog(DBG_SERIAL, "[DISPATCH] Serial TX [%s]  %s\n", (_pi >= 0) ? auxPortLabel(_pi) : a.target, a.cmd); }
      if      (!strcmp(a.target, "S3")) writeS3(s);
      else if (!strcmp(a.target, "S4")) writeS4(s);
      else if (!strcmp(a.target, "S5")) writeS5(s);   // both boards (v2 "Serial 3", WCB 3.2 "Serial 5")
      break;
    }
    case RA_HCR:
      executeHcrAction(a);
      break;
    case RA_MP3:
      executeMp3Action(a);
      break;
    case RA_DFPLAYER:
      executeDfpAction(a);
      break;
    case RA_WLED:
      executeWledAction(a);
      break;
    case RA_RECORD:
      // Toggle recording; save to a.cmd (clip name) on stop. Deferred to loop()
      // so the flash I/O never runs inline in a dispatch path. Mode = clip context.
      navirec::requestRecordToggle((uint8_t)FunctionSwState, a.cmd);
      break;
    case RA_PLAY:
      // Play (toggles): press → load a.cmd + play, press again → stop. fn = loop.
      navirec::requestPlay(a.cmd, a.fn != 0);
      break;
    case RA_STOP:
      navirec::requestStop();   // explicit halt (recording → save, or playback)
      break;
    // RA_SMOOTH_OVERRIDE retired — superseded by smoothing profiles. An old
    // config's "smooth" action is dropped at parse time (actionFromJson), so it
    // never reaches here; the enum value stays reserved to avoid renumbering.
    default: break;
  }
}

// Public entry: applies the calibration gate and per-action delay, then
// dispatches the effect via rcExecuteActionNow().
void rcExecuteAction(const RcAction& a) {
  // Calibration mode: the operator is intentionally moving every control to
  // set thresholds — drop every action (including delayed/scheduled ones, so
  // nothing queues up to fire the instant calibration ends).
  if (calibrationActive) return;
  // Replay owns the outputs — suppress LIVE button/switch/remote dispatch while
  // a clip plays (replay calls rcExecuteActionNow directly, bypassing this gate).
  // EXCEPT the record/play/stop controls themselves, so a mapped button can stop
  // a running clip (Play toggles / Stop) or start a recording.
  if (navirec::isReplaying() && a.type != RA_RECORD && a.type != RA_PLAY && a.type != RA_STOP) return;
  if (a.delayMs > 0) { scheduleAction(a, a.delayMs); return; }
  rcExecuteActionNow(a);
}

// Test-fire ONE action from the config tool's per-action Test button, without
// saving the config. Parses the action object with the SAME parser the editors
// use, then dispatches its effect NOW — bypassing the per-action delay (a test
// should fire the instant you click) and the calibration gate (you asked for it
// explicitly). Runs on Core 1 (USB handler / loop drain), so the dispatch's
// Maestro/ESP-NOW TX is safe. Returns false if the action JSON didn't parse.
static bool rcTestAction(JsonObject act) {
  if (act.isNull()) return false;
  RcAction a{};   // zero-init so any field the parser leaves unset (skipRunning/delayMs/…) is clean
  if (!actionFromJson(act, a)) return false;
  rcExecuteActionNow(a);
  return true;
}

// Drain a bridged TEST_ACTION deferred from rc_telemetry::handle() (Core 0).
// Called from loop() (Core 1); parses the stashed JSON, fires the action, then
// unicasts an ACK back to the relay so the bridged path mirrors the direct-USB
// reply (the tool's terminal shows confirmation — the only feedback for a
// silent action like a serial write).
void drainTestAction() {
  String  js;
  uint8_t sender = 0;
  if (!rcTelemetry::takeTestAction(js, sender)) return;
  bool ok = false;
  StaticJsonDocument<640> tdoc;      // one action object is small; 640 B is ample
  if (deserializeJson(tdoc, js) == DeserializationError::Ok)
    ok = rcTestAction(tdoc["action"].as<JsonObject>());
  if (sender >= 1 && sender <= WCB_MAX_BOARDS && wcb && wcbReady) {
    char ack[64];
    snprintf(ack, sizeof(ack), "{\"sys\":1,\"type\":\"ACK\",\"of\":\"TEST_ACTION\",\"ok\":%s}", ok ? "true" : "false");
    wcb->send(sender, ack);
  }
}

// ── Record/replay dispatch callbacks (the player reaches NaviCore's static
//    dispatch layer through these). ────────────────────────────────────────────
static void recCbDispatch(const RcAction& a)               { rcExecuteActionNow(a); }
static void recCbEmitMaestro(uint8_t slot, uint8_t ch, uint16_t pos) { maestroSetTarget(slot, ch, pos); }
static void recCbResetChan(uint8_t slot, uint8_t ch)       { maestroSetSpeed(slot, ch, 0); maestroSetAccel(slot, ch, 0); }
static void recCbEmitHcrVol(uint8_t chan, uint8_t vol) {
  // RAW HCR SetVolume (fn 17) — bypasses dispatchHcrVolume()'s static cache so
  // replayed volume keyframes aren't decimated by its 80 ms/value-dedupe gate.
  RcAction a{}; a.type = RA_HCR; a.fn = 17; a.chan = (int8_t)chan; a.track = (int16_t)vol;
  executeHcrAction(a);
}

void checkPendingActions() {
  unsigned long now = millis();
  for (int i = 0; i < PENDING_ACTION_SLOTS; i++) {
    // Signed difference, not `now >= fireAt`: fireAt is millis()+delayMs, which
    // wraps for actions scheduled in the last delayMs of the 49.7-day millis()
    // epoch. An absolute compare then fires them instantly with the delay dropped.
    if (pendingActions[i].active && (int32_t)(now - pendingActions[i].fireAt) >= 0) {
      const RcAction& pa = pendingActions[i].action;
      pendingActions[i].active = false;
      // Fire the EFFECT directly (NOT rcExecuteAction) so the action's own
      // delayMs can't re-queue it forever. Honor a calibration — or a replay —
      // that started after it was scheduled: rcExecuteAction applies both gates
      // at schedule time, and a delayed action must not step on the outputs a
      // clip now owns. Same record/play/stop exemption as that gate.
      bool recCtl = (pa.type == RA_RECORD || pa.type == RA_PLAY || pa.type == RA_STOP);
      if (!calibrationActive && (!navirec::isReplaying() || recCtl)) rcExecuteActionNow(pa);
    }
  }
}

// =============================================================================
//  RC dispatch by buttonId + tap count
// =============================================================================
void rcDispatch(int buttonId, uint8_t tapCount) {
  int mode = buttonId / 100, btn = buttonId % 100;
  if (mode < 1 || mode > 3 || btn < 1 || btn > RC_NUM_THRESHOLDS) return;
  if (tapCount < 1 || tapCount > RC_NUM_TAP_TIERS) return;   // 4 == RC_TAP_LONG (long press)

  // Broadcast a "this trigger fired" event over the WCB ESP-NOW network so
  // the config tool's "Via WCB" mode (and any listening Wizard) sees every
  // dispatch — local matrix press, Web-Serial JSON TRIGGER, or remote
  // ESP-NOW TRIGGER — uniformly.  Emitted BEFORE action execution so the
  // event arrives even if a synchronous action stalls.
  rcTelemetry::emitTrig(mode, btn, tapCount);

  // ...and the same event on USB. emitTrig() above only reaches the MESH (it
  // returns early without a ready WCB), so a tool on Direct USB saw nothing at
  // all for a local button press — no way to tell "the tier fired and did
  // nothing visible" from "the tier never fired". Same JSON shape as the mesh
  // telemetry so the tool parses one form on either transport. Human-rate, so
  // the added USB traffic is negligible.
  // GATED on TX room and DROPPED if there is none — same discipline as vlogf()
  // and sendPWMUpdate(). This fires on every button press, and an unguarded
  // write blocks up to the 50 ms HWCDC tx timeout whenever the host is not
  // draining (a backgrounded tab, a closed tool, a stalled read loop), stalling
  // loop() — and therefore SBUS — once per press. The event is a UI nicety (it
  // flashes the tier that fired); a dropped one costs a flash, nothing more.
  {
    char tb[96];
    // No trailing newline in the buffer: the USB path appends one, and the
    // WebSocket path goes through printlnDirect(), which appends its own.
    const int tn = snprintf(tb, sizeof(tb),
                            "{\"sys\":1,\"type\":\"rc_trig\",\"id\":%u,\"mode\":%d,\"btn\":%d,\"tap\":%d}",
                            rcConfig.wcbNetwork.deviceId, mode, btn, tapCount);
    if (tn > 0 && tn < (int)sizeof(tb)) {
      if (Serial.availableForWrite() >= tn + 1) {
        Serial.write((const uint8_t*)tb, (size_t)tn);
        Serial.write((uint8_t)'\n');
      } else {
        // SAME TRAP AS PWM_UPDATE, and it hid in exactly the same place. The
        // guard above asks "can USB take this?" when the question is "can the
        // DESTINATION take this?" — with no USB host attached (every WiFi
        // session) availableForWrite() is permanently 0, so the event was
        // dropped before it could ever reach the capture tee.
        //
        // The symptom is precise and easy to misread as a UI bug: the
        // transmitter SVG still glows, because that is driven by PWM_UPDATE,
        // while the assignment cards never flash, because only
        // flashAssignmentTier() needs rc_trig. It looks like the assignment
        // list broke; nothing was wrong with it.
        //
        // Unlike vlogf() and wcbStreamLog(), which stay USB-only on purpose
        // (debug chatter, see docs/PROTOCOLS.md), this one drives a panel the
        // user is actively watching, so it has to reach the socket too.
        naviws::printlnDirect(tb);
      }
    }
  }

  const RcMapping& mapping = rcConfig.mappings[rcMapIndex(mode, btn)];
  // A long press is a DIFFERENT GESTURE, not a 4th tap, so it is always
  // dispatched exclusively no matter how `exclusive` is set. Letting it fall
  // through to the cumulative branch would fire single+double+triple alongside
  // it — the tiers below a long press were never "passed through" on the way.
  if (mapping.exclusive || tapCount == RC_TAP_LONG) {
    // Exclusive: only the matched tier fires (e.g. double-tap fires t2 alone).
    const RcTier& tier = mapping.t[tapCount - 1];
    for (int i = 0; i < tier.count; i++) rcExecuteAction(tier.a[i]);
  } else {
    // Non-exclusive: cumulative — every tier up to and including the matched
    // one fires (e.g. double-tap fires t1 then t2; triple-tap fires t1+t2+t3).
    for (int ti = 0; ti < tapCount; ti++) {
      const RcTier& tier = mapping.t[ti];
      for (int i = 0; i < tier.count; i++) rcExecuteAction(tier.a[i]);
    }
  }
}

// =============================================================================
//  Matrix button tap detection
// =============================================================================
// Called exactly ONCE per discrete, debounced button press by processSbus()
// (the matrixArmed / neutral-frame logic there guarantees one invocation per
// press and a release between presses). So every call here is a genuine new
// press — there is no need to suppress "held" frames internally, and doing so
// (the old prevPollBtn edge check) wrongly dropped a second press of the SAME
// button when no other button was pressed in between.
void RCRadio_Matrix_Buttons(int val) {
  int btn = pwmToButton(val);
  if (btn == 0) return;                 // caller only calls for a real button; defensive
  unsigned long now = millis();

  if (btn != tapState.lastBtn) {
    // Different button than the last gesture — commit any pending fire for the
    // previous button now, then start a fresh single tap on this one.
    if (tapState.deferredPending) {
      tapState.deferredPending = false;
      rcDispatch(tapState.deferredBtn, tapState.deferredTaps);
    }
    tapState.lastBtn   = btn;
    tapState.tapCount  = 1;
    tapState.lastTapMs = now;
  } else {
    // Same button as the last gesture — within the tap window it's another tap
    // of a multi-tap; past the window it's a brand-new single tap.
    if ((now - tapState.lastTapMs) < (unsigned long)rcConfig.tapWindowMs) {
      tapState.tapCount++;
      // Deliberately 3, NOT RC_NUM_TAP_TIERS: tier 4 is the LONG PRESS, reached by
      // holding — never by a 4th tap. A 4-tap flurry saturates at triple, as before.
      if (tapState.tapCount > 3) tapState.tapCount = 3;
    } else {
      tapState.tapCount = 1;
    }
    tapState.lastTapMs = now;
  }

  // Arm (or refresh) the deferred fire — both exclusive and non-exclusive modes
  // wait the tap window before dispatching so multi-taps have time to register.
  tapState.deferredPending = true;
  tapState.deferredFireAt  = now + rcConfig.tapWindowMs;
  tapState.deferredBtn     = FunctionSwState * 100 + btn;
  tapState.deferredTaps    = tapState.tapCount;

  // Open a hold window ONLY on the first press of a gesture. A hold on the 2nd
  // or 3rd tap stays an ordinary double/triple tap: promoting it would make
  // "tap, tap, hold" ambiguous and turn this into a gesture parser for no gain.
  // (The press is still parked by checkDeferredTap until release either way, so
  // a held 2nd tap simply dispatches its tier when the button comes up.)
  tapState.holdActive  = (tapState.tapCount == 1);
  tapState.holdBtn     = btn;
  tapState.holdStartMs = now;
  tapState.holdFired   = false;
}

// Does this button's LONG PRESS tier actually have actions? A hold must not
// consume the gesture on a button that has no long press configured — the tap
// tier still has to fire on release, exactly as it did before tier 4 existed.
static bool rcHoldTierConfigured(int buttonId) {
  const int mode = buttonId / 100, btn = buttonId % 100;
  if (mode < 1 || mode > 3 || btn < 1 || btn > RC_NUM_THRESHOLDS) return false;
  return rcConfig.mappings[rcMapIndex(mode, btn)].t[RC_TAP_LONG - 1].count > 0;
}

// Long-press threshold reached with the button still down — dispatch tier 4 and
// consume the gesture so the parked tap never fires. Called from processSbus()
// (Core 1, same as every other matrix dispatch).
static void rcMatrixHoldFire() {
  tapState.holdFired       = true;
  tapState.deferredPending = false;   // the parked tap is superseded, not queued
  int btnId = tapState.deferredBtn;   // mode*100+btn, captured at the press
  tapState.tapCount = 0;
  tapState.lastBtn  = 0;
  rcDispatch(btnId, RC_TAP_LONG);
}

// Debounced NEUTRAL confirmed — the held button came up. Idempotent: processSbus
// keeps re-confirming neutral every frame while nothing is pressed.
static void rcMatrixRelease() {
  if (!tapState.holdActive) return;
  tapState.holdActive = false;
  tapState.holdBtn    = 0;

  if (tapState.holdFired) {
    // Long press already dispatched — the gesture is spent. Drop anything the
    // release might otherwise re-arm (covers a release racing the threshold).
    tapState.holdFired       = false;
    tapState.deferredPending = false;
    tapState.tapCount        = 0;
    tapState.lastBtn         = 0;
    return;
  }

  // Sub-threshold hold: this was an ordinary tap that simply took a while to let
  // go of. Re-time BOTH windows from the release instant — people space taps by
  // the gap between them, not press-to-press, so measuring from the press would
  // spend a 400 ms hold's worth of the tap window before the finger even lifts
  // and silently demote the next tap to a fresh single.
  unsigned long now = millis();
  tapState.lastTapMs = now;
  if (tapState.deferredPending) tapState.deferredFireAt = now + rcConfig.tapWindowMs;
}

void checkDeferredTap() {
  if (!tapState.deferredPending) return;
  // Park the dispatch while the button is still physically down. holdMs is by
  // definition longer than tapWindowMs, so firing on the tap window alone would
  // dispatch the tap tier BEFORE the hold could ever be recognised. A normal tap
  // is released long before this matters; the visible effect is limited to a
  // press-and-hold, which now resolves on release (or at holdMs) instead of
  // mid-hold. rcMatrixRelease() re-times the window when the button comes up.
  if (tapState.holdActive) return;
  // Signed difference, not `millis() >= deferredFireAt`: deferredFireAt is
  // now+tapWindowMs, which wraps in the last tapWindowMs of the millis() epoch.
  // An absolute compare fires the gesture on the next loop() pass, so a
  // multi-tap started there degrades into repeated tier-1 dispatches.
  if ((int32_t)(millis() - tapState.deferredFireAt) >= 0) {
    tapState.deferredPending = false;
    rcDispatch(tapState.deferredBtn, tapState.deferredTaps);
    tapState.tapCount = 0;
    tapState.lastBtn  = 0;
  }
}

// =============================================================================
//  Switch processing
// =============================================================================
void processSwitches() {
  // Seed switchPrevPos with each switch's actual decoded position WITHOUT firing
  // whenever a re-seed is pending: at boot (g_switchSeedPending starts true) and
  // after EVERY config apply (rcConfigFromJSON sets it), so changing a switch's
  // channel/positions via SET_CONFIG/RESET_DEFAULTS can't fire a phantom action
  // from a now-stale switchPrevPos[]. processSbus only reaches here on a valid,
  // non-failsafe frame, so the pending seed lands on the first valid frame after
  // the change. (Matches the matrix debounce, which re-arms on a confirmed neutral.)
  const unsigned long now = millis();
  for (int i = 0; i < RC_NUM_SWITCHES; i++) {
    RcSwitch& sw = rcConfig.switches[i];
    if (sw.channel < 1 || sw.channel > 24) continue;
    int pos = readSwitchPos(sbusValues[sw.channel - 1], sw.positions);
    if (g_switchSeedPending) {
      switchPrevPos[i]  = pos;          // seed only, don't fire
      switchCandPos[i]  = -1;           // drop any half-timed candidate from before the re-seed
      // Adopt the easing this position SELECTS, without executing the tier.
      // g_switchEasing is otherwise only ever written by an executed setEasing
      // action, so at boot the firmware believed "released" no matter where the
      // switch physically sat — and the pilot had to flick it to sync. Executing
      // the tier here instead is not an option: the seed exists precisely so a
      // power-up cannot fire scripts and sounds (see the comment above).
      seedSwitchEasingFromTier(sw.t[pos]);
      continue;
    }
    if (pos == switchPrevPos[i]) { switchCandPos[i] = -1; continue; }   // back to rest → cancel

    // New position: start (or continue) timing it.
    if (switchCandPos[i] != pos) { switchCandPos[i] = pos; switchCandSince[i] = now; }
    // Signed compare for the millis()-wrap reason documented on checkDeferredTap.
    if (rcConfig.switchSettleMs &&
        (int32_t)(now - (switchCandSince[i] + (unsigned long)rcConfig.switchSettleMs)) < 0) continue;

    switchPrevPos[i] = pos;
    switchCandPos[i] = -1;
    RcTier& tier = sw.t[pos];
    for (int ai = 0; ai < tier.count; ai++) rcExecuteAction(tier.a[ai]);
  }
  if (g_switchSeedPending) {
    // The seed just established every switch's easing. Drive it onto the hardware
    // once, here rather than in setup(): the seed lands on the first valid
    // non-failsafe SBUS frame, which is also the earliest point the positions are
    // actually known. Nothing else re-applies easing at boot.
    for (uint8_t mid = 1; mid <= RC_NUM_MAESTROS; mid++) {
      reapplyMaestroEasing(mid);
      // Boot is the WORST case for a lost easing write: the Maestro may still be
      // coming up (its rail is often switched separately) and would silently miss
      // a write NaviCore then records as applied. Burst it.
      scheduleEasingRepeat(mid);
    }
  }
  g_switchSeedPending = false;
}

// =============================================================================
//  Knob processing
//
//  Each knob/joystick-axis samples its SBUS channel once and fans the value
//  out to every configured output. The dispatch path is chosen by the knob's
//  function:
//
//    KF_MAESTRO_PASSTHROUGH  → each output.target is a Maestro ID (1-8) and
//                              maestroCh/posMin/posMax describe the servo.
//    KF_HCR_VOLUME           → each output.target is an HCR audio chan
//                              (0=V, 1=A, 2=B, 3=All); posMin/posMax are volume
//                              endpoints (0-99). Rate-limited to avoid
//                              saturating ESP-NOW / HCR serial input.
//
//  ⚠ At 8 sources × 8 outputs × ~70Hz SBUS rate this could emit up to 4480
//  Maestro.setTarget() calls/sec. Keep active outputs reasonable.
// =============================================================================

// HCR volume rate-limiter — only re-emit when value changes AND a minimum
// interval has elapsed.  One slot per audio channel: 0=V, 1=A, 2=B, 3=All.
struct HcrVolCache {
  int8_t        lastVol[4]  = { -1, -1, -1, -1 };
  unsigned long lastSent[4] = {  0,  0,  0,  0 };
  // Trailing-edge latch (-1 = nothing pending). A value that arrives inside the
  // throttle window is held here, not dropped — see dispatchHcrVolume().
  int8_t        pendVol[4]  = { -1, -1, -1, -1 };
};
static HcrVolCache hcrVolCache;
static const unsigned long HCR_VOLUME_MIN_INTERVAL_MS = 80;  // ~12 Hz max per chan

// Emit one HCR SetVolume and re-arm the rate limiter. audioChan/vol must already
// be validated and clamped by the caller.
static void hcrVolEmit(uint8_t audioChan, uint8_t vol) {
  hcrVolCache.lastVol[audioChan]  = (int8_t)vol;
  hcrVolCache.lastSent[audioChan] = millis();
  hcrVolCache.pendVol[audioChan]  = -1;
  // Build a synthetic RA_HCR action and reuse the existing dispatcher so
  // it automatically honors rcConfig.hcrDest (serial vs WCB transport).
  RcAction a{};
  a.type  = RA_HCR;
  a.fn    = 17;        // SetVolume
  a.chan  = (int8_t)audioChan;
  a.track = (int16_t)vol;
  executeHcrAction(a);
  navirec::captureHcrVolKf(audioChan, vol);   // record the EMITTED (post-gate) knob volume
}

static void dispatchHcrVolume(uint8_t audioChan, uint8_t vol) {
  if (rcConfig.hcrDest.transport == 2) return;   // HCR disabled — knob drives nothing
  if (audioChan > 3) {
    dlog(DBG_HCR, "[DISPATCH] HCR volume: audio channel %u out of range (0-3 = V/A/B/All) — "
         "check the knob's HCR output target; skipped\n", audioChan);
    return;
  }
  if (vol > 99) vol = 99;
  if ((int8_t)vol == hcrVolCache.lastVol[audioChan]) { hcrVolCache.pendVol[audioChan] = -1; return; }
  unsigned long now = millis();
  if ((uint32_t)(now - hcrVolCache.lastSent[audioChan]) < HCR_VOLUME_MIN_INTERVAL_MS) {
    // Latch, don't drop. processKnobs() commits lastKnobRaw[] before calling us,
    // so once the knob stops moving the deadband blocks every later frame and
    // nothing calls this again — a leading-edge-only throttle would leave the HCR
    // at whatever landed up to 80 ms before the end of the sweep. hcrVolFlushTick()
    // sends the latched value once the interval expires.
    hcrVolCache.pendVol[audioChan] = (int8_t)vol;
    return;
  }
  hcrVolEmit(audioChan, vol);
}

// Trailing edge of the HCR volume throttle — flush any latched value once its
// interval has elapsed, so the knob's final resting position always lands.
// Core 1 only (loop()); executeHcrAction touches serial/mesh.
static void hcrVolFlushTick() {
  // Same gates processKnobs() applies — a value latched just before a clip started
  // (or calibration opened) must not be flushed into outputs they now own.
  if (calibrationActive || navirec::isReplaying()) return;
  unsigned long now = millis();
  for (uint8_t c = 0; c < 4; c++) {
    if (hcrVolCache.pendVol[c] < 0) continue;
    if ((uint32_t)(now - hcrVolCache.lastSent[c]) < HCR_VOLUME_MIN_INTERVAL_MS) continue;
    int8_t v = hcrVolCache.pendVol[c];
    hcrVolCache.pendVol[c] = -1;
    if (v == hcrVolCache.lastVol[c]) continue;   // already there — nothing to send
    hcrVolEmit(c, (uint8_t)v);
  }
}

// Per-knob last-processed raw SBUS value. Knobs only re-dispatch when their
// channel moves past KNOB_CHANGE_DEADBAND counts — otherwise a stationary
// stick/knob would spam a Maestro/HCR command on every SBUS frame (~70+/s),
// and SBUS line jitter (±1-2 counts) would do the same. Matches the ≥5
// deadband the matrix/mode selectors already use. Sentinel 0xFFFF forces the
// first frame past the deadband so the baseline gets seeded immediately; the
// knobPrimed gate then suppresses that first DISPATCH so nothing twitches to the
// startup position on boot (see knobPrimed) — outputs move only on a real change.
#define KNOB_CHANGE_DEADBAND 5
static uint16_t lastKnobRaw[RC_NUM_KNOBS] = {
  0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF,    // S1, S2, LS, RS
  0xFFFF,                            // MS  (X20)
  0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF,    // J1-J4
  0xFFFF, 0xFFFF                     // J5, J6 (X20 3-axis)
};
// Last mode each knob was dispatched in (0 = uninitialized). A knob using a
// modeSwitchOverride follows its own switch, whose changes DON'T trip
// resetModeAwareKnobs() (that only watches the global mode switch) — so we
// detect the per-knob mode change here and re-arm.
static int8_t lastKnobMode[RC_NUM_KNOBS] = {0};
// Boot-twitch suppression. A knob's VERY FIRST processed frame only SEEDS its
// change-detection baseline (lastKnobRaw) — it does not dispatch — so servos and
// other outputs don't jump to the startup stick position on power-up. Nothing
// moves until the source actually changes (a real user action). This is distinct
// from the mode-change re-arm (lastKnobRaw = 0xFFFF), which DOES snap to position:
// by the time a mode switch is flipped the knob is already primed, so flipping it
// is itself the action that commands the move.
static bool knobPrimed[RC_NUM_KNOBS] = {false};

void processKnobs() {
  // The calibration wizard asks the operator to sweep every source to its
  // extremes. Dispatch is already muted while it runs, but passthrough is NOT a
  // dispatch — without this gate every passthrough servo tracks those sweeps and
  // slams its full mechanical travel while the user is just reading the wizard.
  if (calibrationActive) return;
  if (navirec::isReplaying()) return;   // replay owns the servos/volume during playback
  for (int i = 0; i < RC_NUM_KNOBS; i++) {
    RcKnob& kn = rcConfig.knobs[i];
    if (kn.channel < 1 || kn.channel > 24) continue;
    if (kn.function == KF_NONE) continue;
    // Which mode's output set this knob uses. Default = the global mode switch
    // (FunctionSwState); a mode-aware knob may instead follow its OWN switch via
    // modeSwitchOverride (0-7 = SA-SH) so it can be gated independently — e.g. a
    // joystick that drives Maestro passthrough only when SA is flipped, without
    // disturbing the droid's global mode.
    int m = 1;
    if (kn.modeAware) {
      if (kn.modeSwitchOverride >= 0) {
        // The override decodes the 3 modes from the switch's SBUS band. A
        // 2-POSITION switch only reads the low/high bands (sv<582 / sv>=1401),
        // so it can address modes 1 and 3 but never mode 2 — a mode override
        // needs a 3-position switch to reach all three. The config tool only
        // offers 3-position switches here for that reason.
        const int sv = readBoundSwitchSbus(kn.modeSwitchOverride);
        m = (sv < 0) ? FunctionSwState : (sv < 582 ? 1 : (sv < 1401 ? 2 : 3));
      } else {
        m = FunctionSwState;
      }
    }
    // Re-arm on a per-knob mode change so the new mode's servo snaps to the
    // current stick position next frame (resetModeAwareKnobs only fires on a
    // GLOBAL mode change, so an override knob's switch move needs this).
    if ((int8_t)m != lastKnobMode[i]) { lastKnobMode[i] = (int8_t)m; lastKnobRaw[i] = 0xFFFF; }
    const uint8_t         cnt  = rcKnobOutCount(kn, m);
    const RcKnobOutput*   outs = rcKnobOuts(kn, m);
    uint16_t raw = sbusValues[kn.channel - 1];

    // Per-knob direction reversal — invert around the SBUS centre so a
    // "stick up" reading maps to what would otherwise be "stick down" for
    // all downstream output computations.  SBUS valid range is ~172-1811;
    // reflecting around the midpoint = (172 + 1811) - raw = 1983 - raw.
    // The deadband / change-detection still uses the post-reverse value so
    // moving the source dispatches as expected.
    if (kn.reverse) {
      int flipped = 1983 - (int)raw;
      if (flipped < 0)     flipped = 0;
      if (flipped > 2047)  flipped = 2047;
      raw = (uint16_t)flipped;
    }

    // Only dispatch when the source actually moved past the deadband.
    if (abs((int)raw - (int)lastKnobRaw[i]) < KNOB_CHANGE_DEADBAND) continue;
    lastKnobRaw[i] = raw;
    // First frame after boot: seed the baseline only, don't drive anything — a
    // servo shouldn't twitch to the startup stick position on power-up (only
    // once the source is actually moved). See knobPrimed.
    if (!knobPrimed[i]) { knobPrimed[i] = true; continue; }
    if (cnt == 0) continue;   // this mode drives nothing (still tracked lastKnobRaw above)

    for (uint8_t o = 0; o < cnt && o < RC_KNOB_MAX_OUTPUTS; o++) {
      const RcKnobOutput& out = outs[o];
      // midClosed only reshapes the Maestro-passthrough mapping — never HCR volume.
      // (The tool checkbox is Maestro-only; this gate also stops a hand-edited config
      // from hijacking an HCR-volume output's mapping.)
      uint16_t mapped = (out.midClosed && kn.function == KF_MAESTRO_PASSTHROUGH)
                          ? sbusToRangeMidClosed(raw, out.posMin, out.posMax)
                          : sbusToRange(raw, out.posMin, out.posMax);
      if (kn.function == KF_MAESTRO_PASSTHROUGH) {
        // out.target is the Maestro slot ID (1-8)
        const uint8_t mid = out.target, mch = out.maestroCh;
        // Easing comes from resolveKnobEasing (the knob's own profile wins; the
        // switch's active easing is the fallback / opt-in override). No effective
        // profile (<0) or an unset channel (0/0) = untouched here, so scripts/EEPROM
        // stand; a managed channel re-applies its speed/accel via the g_maeSpeed/
        // g_maeAccel cache (self-heals after a script). A switch-easing change resets
        // channels via reapplyMaestroEasing() — the one place that drives 0.
        const int8_t eff = resolveKnobEasing(kn, mid);
        if (eff >= 0 && eff < RC_NUM_SMOOTH_PROFILES &&
            mid >= 1 && mid <= RC_NUM_MAESTROS && mch < 32) {
          const RcSmoothEntry& e = rcConfig.smoothProfiles[eff].entries[mid - 1][mch];
          if (e.speed || e.accel) {
            uint16_t wantSpd = e.speed > 16383 ? 16383 : e.speed;   // Maestro speed is 14-bit; also keeps 'want' off the 0xFFFF cache sentinel
            uint16_t wantAcc = e.accel;
            if (g_maeSpeed[mid - 1][mch] != wantSpd) maestroSetSpeed(mid, mch, wantSpd);
            if (g_maeAccel[mid - 1][mch] != wantAcc) maestroSetAccel(mid, mch, (uint8_t)wantAcc);
          }
        }
        maestroSetTarget(mid, mch, mapped);
        navirec::captureMaestroKf(mid, mch, mapped);   // knob keyframe
        // Carry THIS output's auto-release policy (the idle timeout) to the channel —
        // only the passthrough output knows releaseIdleMs. The idle-timer re-arm +
        // released-clear happen inside maestroSetTarget() above, which fires for every
        // real move regardless of source (mapped is a servo position, never 0).
        if (mid >= 1 && mid <= RC_NUM_MAESTROS && mch < 32)
          g_maeReleaseMs[mid - 1][mch] = out.releaseIdleMs;
      } else if (kn.function == KF_HCR_VOLUME) {
        // out.target is the HCR audio chan (0/1/2/3 = V/A/B/All); mapped is volume 0-99.
        // Clamp before the uint8 cast: a posMin/posMax above 255 would otherwise
        // wrap (e.g. 300 → 44) instead of pinning to the max volume.
        dispatchHcrVolume(out.target, (uint8_t)(mapped > 99 ? 99 : mapped));
      }
    }
  }
}

// On a mode change, re-arm each MODE-AWARE knob's change-detection sentinel so
// its new mode's output(s) re-dispatch on the very next frame at the current
// stick position — otherwise the servo would sit at the previous mode's target
// until the stick is nudged past the deadband. Called from both mode-commit
// sites (processSbus SBUS decode + rc_telemetry SET_MODE).
void resetModeAwareKnobs() {
  // A GLOBAL mode change must re-arm ONLY the knobs that actually follow the global mode.
  // A knob with a modeSwitchOverride (>=0) follows its OWN switch, so the global-mode
  // change did NOT change ITS mode — re-arming it forces a spurious re-dispatch that
  // snaps the servo to the current stick position on every mode-selector flip (and, for a
  // REMOTE Maestro, fires a needless mesh broadcast each time). Its own switch moving
  // still re-arms it via the per-knob lastKnobMode check in processKnobs().
  for (int i = 0; i < RC_NUM_KNOBS; i++)
    if (rcConfig.knobs[i].modeAware && rcConfig.knobs[i].modeSwitchOverride < 0)
      lastKnobRaw[i] = 0xFFFF;
}

// De-energize idle passthrough servos. A Maestro-passthrough output with
// releaseIdleMs>0 (see RcKnobOutput) opts its channel into auto-release: the
// passthrough only writes on stick MOVEMENT, so a resting servo keeps its last
// target and can buzz/hunt against it. Once a channel has been idle its
// releaseIdleMs, send Set Target 0 — the Maestro stops pulsing it, so the servo
// goes limp and silent (NO holding torque; opt-in per channel for a reason). The
// next stick move writes a real target (re-energizing) and clears the flag in the
// dispatch loop above. Runs in loop() on Core 1 — the SAME core as the passthrough
// dispatch and maestroWrite, so there is no cross-core race on the Maestro UART.
// Scans at ~20 Hz (the timeout is coarse, 100s-1000s of ms), keeping loop() cheap.
void maestroIdleReleaseTick() {
  // Replay owns the servos while a clip plays (processKnobs bails at its own isReplaying
  // gate). Without this, the tick would de-energize a channel the clip is actively
  // driving — and a recorded HOLD emits no keyframes, so it would stay limp for the rest
  // of the show. maestroSetTarget re-arms the timer on every replay step, so when
  // playback ends the idle countdown restarts cleanly from the last driven frame.
  if (navirec::isReplaying()) return;
  static uint32_t lastScan = 0;
  const uint32_t now = millis();
  if (now - lastScan < 50) return;
  lastScan = now;
  for (uint8_t id = 1; id <= RC_NUM_MAESTROS; id++)
    for (uint8_t ch = 0; ch < 32; ch++) {
      const uint16_t idle = g_maeReleaseMs[id - 1][ch];
      if (!idle || g_maeReleased[id - 1][ch]) continue;          // not opted in, or already released
      if ((uint32_t)(now - g_maeLastMoveMs[id - 1][ch]) >= idle) {
        maestroSetTarget(id, ch, 0);                             // Set Target 0 → stop pulses → servo limp + silent
        g_maeReleased[id - 1][ch] = true;
        dlog(DBG_MAESTRO, "[RELEASE] Maestro %u ch %u idle %ums → servo off\n", id, ch, idle);
      }
    }
}

// =============================================================================
//  SBUS frame processing
// =============================================================================
void processSbus() {
  if (!sbusRx.read()) return;

  sbusFrameCount++;
  sbusLastFrameMs = millis();
  sbusFpsCounter++;
  sbusFailsafe  = sbusRx.failsafe;
  lostFrameOld  = sbusRx.lostFrame;
  for (int i = 0; i < 24; i++) sbusValues[i] = sbusRx.channels[i];

  // ── Failsafe gate — DO NOT dispatch on a failsafe frame ─────────────────
  // When the transmitter powers off (or link is lost), many receivers keep
  // emitting frames with the failsafe bit set and channels parked at their
  // configured failsafe positions.  Acting on those would drive servos to
  // the failsafe pose and fire any switch/knob thresholds the parked values
  // happen to cross — unexpected motion on signal loss, which is dangerous
  // for an animatronic.  Instead we freeze: keep telemetry/FPS current (so
  // the config tool shows "FAILSAFE"), but skip mode/matrix/switch/knob
  // dispatch entirely, leaving all outputs holding their last commanded
  // state.  We also reset the matrix debounce so that when the link
  // recovers, a button physically held across the dropout requires a fresh
  // confirmed-neutral + press before it can fire (no recovery-transient
  // phantom press).
  // NOTE: this gates on `failsafe` only, NOT `lostFrame` — lostFrame is a
  // single-frame transient and gating on it would make control feel laggy.
  if (sbusRx.failsafe) {
    matrixArmed        = false;   // require a confirmed neutral to re-arm post-recovery
    matrixCandidate    = 0;
    matrixCandCount    = 0;
    matrixNeutralCount = 0;
    // Abandon any hold in progress. The link dropped mid-press, so we never saw
    // the release edge that would normally close it — left set, holdActive would
    // park checkDeferredTap() forever and the button would go dead after recovery.
    tapState.holdActive = false;
    tapState.holdFired  = false;
    tapState.holdBtn    = 0;
    return;
  }

  // Mode selector — same SBUS-cluster thresholds as readSwitchPos()
  // (582/1401) so the bound mode switch decodes its three positions
  // correctly. Earlier 340/680 incorrectly mapped middle (~992) → mode 3.
  int modeVal = readBoundSwitchSbus(rcConfig.funcBindings.modeSwitch);
  if (modeVal >= 0 && abs(modeVal - oldValueMode) > 5) {
    oldValueMode = modeVal;
    int newMode = (modeVal < 582) ? 1 : (modeVal < 1401 ? 2 : 3);
    if (newMode != FunctionSwState) {
      FunctionSwState = newMode;
      resetModeAwareKnobs();   // re-arm mode-aware knobs so their new-mode servos snap to the stick
      // Surface the mode change on the WCB network immediately — config
      // tools watching via the bridge get instant feedback instead of
      // waiting up to 2 s for the next rc_hb heartbeat.
      rcTelemetry::emitMode(FunctionSwState);
    }
  }

  // Button matrix — edge-detected with an asymmetric debounce (see
  // matrixArmed/Candidate state above). Decodes EVERY frame instead of
  // gating on raw value delta, so a fast re-press of the same button is no
  // longer dropped.
  int mxCh = rcConfig.matrixChannel;
  if (mxCh >= 1 && mxCh <= 24) {
    int mxVal = sbusValues[mxCh - 1];
    oldValueMatrix = mxVal;                 // keep fresh for status/diagnostics
    int decoded = pwmToButton(mxVal);       // 0 = neutral / between-band gap
    int debFrames = rcConfig.matrixDebounceFrames;
    if (debFrames < 1) debFrames = 1;            // safety clamp (config is 1-4)

    if (decoded == 0) {
      // NEUTRAL candidate. A real release sits at rest for many frames; a
      // sweep slew / contact bounce only dips out-of-band for 1-2 frames.
      // Only a *debounced* neutral run counts as a release → re-arm. This is
      // the key reliability fix: a transient neutral can no longer split one
      // physical press into a phantom double.
      matrixCandidate = 0;
      matrixCandCount = 0;
      if (matrixNeutralCount < debFrames) matrixNeutralCount++;
      if (matrixNeutralCount >= debFrames) {
        matrixArmed = true;      // release confirmed
        rcMatrixRelease();       // ...which is also the "button came up" edge the
                                 // long-press tracker needs. Idempotent — this
                                 // branch re-runs every frame while nothing is down.
      }
    } else {
      // BUTTON candidate. Any in-band reading breaks a neutral run, so a
      // mid-press sweep transient that briefly crosses a neighbor band does
      // not accumulate toward a release.
      matrixNeutralCount = 0;
      if (decoded == matrixCandidate) {
        if (matrixCandCount < debFrames) matrixCandCount++;
      } else {
        matrixCandidate = decoded;
        matrixCandCount = 1;
      }
      // Fire once the button has been stable in-band AND we're armed
      // (armed only by a confirmed neutral — never by a 1-frame blip).
      if (matrixArmed && matrixCandCount >= debFrames) {
        matrixArmed = false;             // consume — needs a CONFIRMED neutral to re-arm
        RCRadio_Matrix_Buttons(mxVal);
      }

      // Long-press promotion. Tested HERE rather than in loop() because
      // `decoded` is the authoritative "what is in-band right now" — requiring
      // it to still equal holdBtn means sliding onto a neighbouring band can't
      // fire the wrong button's long press. A 1-frame transient can't cancel a
      // hold either: only a debounced NEUTRAL clears it, matching how the
      // release debounce already refuses to split one press into a phantom double.
      // Signed compare for the same millis()-wrap reason as checkDeferredTap.
      // ...and ONLY when tier 4 actually has actions. Promoting unconditionally
      // consumed the gesture even on a button with no long press configured, so
      // holding it fired NOTHING where it previously fired the single-tap tier on
      // release — a silent regression on every existing mapping. An unconfigured
      // long press must leave the tap behaviour exactly as it was.
      if (tapState.holdActive && !tapState.holdFired && decoded == tapState.holdBtn &&
          (int32_t)(millis() - (tapState.holdStartMs + (unsigned long)rcConfig.holdMs)) >= 0 &&
          rcHoldTierConfigured(tapState.deferredBtn)) {
        rcMatrixHoldFire();
      }
    }
  }

  processSwitches();
  processKnobs();
}

// =============================================================================
//  SBUS FPS tracking
// =============================================================================
void trackSbusFps() {
  unsigned long now = millis();
  if (now - sbusFpsLastSecond >= 1000) {
    sbusFps           = (int)sbusFpsCounter;
    sbusFpsCounter    = 0;
    sbusFpsLastSecond = now;
  }
}

// =============================================================================
//  SBUS state dump (#L09)
// =============================================================================
void dumpSbusState() {
  unsigned long ageMs = (sbusLastFrameMs == 0) ? 0 : (millis() - sbusLastFrameMs);
  Serial.println("---- SBUS STATE ----");
  Serial.printf("  variant=%s (%d ch, %d-byte frame)\n",
                sbusRx.detectedChCount == 24 ? "SBUS-24" :
                sbusRx.detectedChCount == 16 ? "SBUS-16" : "(none yet)",
                sbusRx.detectedChCount, sbusRx.detectedFrameLen);
  Serial.printf("  frames=%lu  fps=%d  ageMs=%lu  lost=%s  failsafe=%s\n",
                sbusFrameCount, sbusFps, ageMs,
                lostFrameOld ? "YES" : "no", sbusFailsafe ? "YES" : "no");
  for (int r = 0; r < 3; r++) {
    int base = r * 8;
    if (base >= sbusRx.detectedChCount) break;
    Serial.printf("  CH%d-%d: ", base + 1, base + 8);
    for (int i = base; i < base + 8 && i < 24; i++)
      Serial.printf("%4d ", sbusValues[i]);
    Serial.println();
  }
}

// Enqueue a relayed CLI line for drainRemoteCli(). MUST stay noinline: the
// 200-byte RemoteCliMsg lives in THIS frame, not onWCBCommand's, so it isn't
// reserved on the Core-0 ESP-NOW callback stack while rcTelemetry::handle()
// runs — that extra 200 bytes was enough to overflow the WiFi-task stack and
// crash the board on every mesh command. (The weight in handle() is NOT the
// ArduinoJson parse, as this comment used to claim: JsonDocument is heap-pooled
// in ArduinoJson 7. It was the FragSession slot-clear temporary — see
// _fragClear() in rc_telemetry.h.)
static void __attribute__((noinline)) queueRemoteCli(uint8_t relay, const char* command) {
  RemoteCliMsg m;
  m.relay = relay;
  strlcpy(m.cmd, command, sizeof(m.cmd));
  xQueueSend(remoteCliQueue, &m, 0);   // non-blocking; drop under load
}

// ── Mesh → serial ────────────────────────────────────────────────────────────
// Enqueue one write for the aux port `fwPort` (firmware numbering). Safe to call
// from the Core-0 ESP-NOW callback; drainSerialFwd()/auxTxPump() do the actual
// (blocking, bit-banged) I/O on Core 1. noinline keeps the 201-byte SerialFwdMsg
// off the caller's frame, same discipline as queueRemoteCli.
static void __attribute__((noinline)) queueSerialFwd(uint8_t fwPort, const char* text) {
  if (!serialFwdQueue || !text || !text[0]) return;
  SerialFwdMsg m;
  m.fwPort = fwPort;
  strlcpy(m.text, text, sizeof(m.text));
  xQueueSend(serialFwdQueue, &m, 0);   // non-blocking; drop under load
}

// Enqueue an inbound ";M" line for drainMaestroCmd() (loop, Core 1). Safe to call from
// the Core-0 ESP-NOW callback; noinline keeps the MaestroCmdMsg off that frame (same
// discipline as queueRemoteCli/queueSerialFwd). An over-long line is dropped rather
// than truncated — see the MaestroCmdMsg comment.
static void __attribute__((noinline)) queueMaestroCmd(uint8_t sender, const char* text) {
  if (!maestroCmdQueue || !text || !text[0]) return;
  MaestroCmdMsg m;
  if (strlen(text) >= sizeof(m.text)) return;
  m.sender = sender;
  strlcpy(m.text, text, sizeof(m.text));
  xQueueSend(maestroCmdQueue, &m, 0);   // non-blocking; drop under load
}

// True when a device owns this aux port — an HCR, MP3 Trigger, DFPlayer or WLED is
// routed to it. rcSerialLabelAuto() already derives exactly that from the routing
// config, so a new device type is covered here the moment it is added there, and so
// it's the single source of truth for both the advertised label and this check.
// The broadcast fan-out skips these ports the way a WCB skips its own device ports:
// the owning driver frames its traffic, and broadcast chatter would corrupt it. A
// TARGETED `;s<n>` write is never skipped — that's an explicit operator instruction.
static bool auxPortHasDevice(int fwPort) {
  if (fwPort < 3 || fwPort > 5) return false;
  return rcSerialLabelAuto(fwPort - 3)[0] != '\0';   // slot RC_SLBL_S3..RC_SLBL_S5
}

// Fan a broadcast command out to every aux port that opted in. `exceptFwPort` is the
// port it arrived on (0 when it came off the mesh), so a line read from S3 is never
// echoed straight back out S3. Core-0 safe — it only enqueues.
static void queueSerialBroadcastOut(const char* text, int exceptFwPort) {
  for (int i = 0; i < 3; i++) {
    int fwPort = i + 3;
    if (fwPort == exceptFwPort)      continue;
    if (!rcConfig.serialBcastOut[i]) continue;   // port hasn't joined the broadcast domain
    if (!auxStreamFor(fwPort))       continue;   // not present on this board
    if (auxPortHasDevice(fwPort))    continue;   // HCR/MP3/WLED owns the wire
    queueSerialFwd((uint8_t)fwPort, text);
  }
}

// Enqueue a remote TRIGGER for drainRemoteTriggers() (loop, Core 1). Called from
// rcTelemetry::handle() on the Core-0 ESP-NOW callback; noinline keeps the POD
// off that callback's stack frame (same discipline as queueRemoteCli). Args are
// already range-validated by the caller. Non-static: rc_telemetry.h forward-
// declares it (like rcDispatch) since the header is compiled before this point.
void __attribute__((noinline)) queueRemoteTrigger(int mode, int btn, uint8_t tap) {
  if (!remoteTriggerQueue) return;
  RemoteTrigger t{ (uint8_t)mode, (uint8_t)btn, tap };
  xQueueSend(remoteTriggerQueue, &t, 0);   // non-blocking; drop under load
}

// Forget one learned peer (id 1..WCB_MAX_BOARDS) or ALL of them (id 0). MUST run
// on Core 1 (loop): forgetPeer()/clearLearnedPeers() do esp_now_del_peer + NVS
// writes. Uses isLearnedPeer() so the id path gives honest "nothing to forget"
// feedback for a non-learned slot. Called directly from the USB JSON + CLI paths
// (already Core 1) and from drainForgetPeer() for the deferred Via-WCB path.
void doForgetPeer(uint8_t id) {
  if (!wcb || !wcbReady) return;
  if (id == 0) {
    wcb->clearLearnedPeers();
    Serial.println("[WCB] cleared all learned peers");
  } else if (wcb->isLearnedPeer(id)) {
    wcb->forgetPeer(id);
    Serial.printf("[WCB] forgot learned WCB %u\n", id);
  } else {
    Serial.printf("[WCB] WCB %u is not a learned peer (nothing to forget)\n", id);
  }
}

// Enqueue a forget request from the Core-0 ESP-NOW callback (Via-WCB FORGET_PEER);
// drainForgetPeer() runs it on Core 1. noinline keeps the payload off the callback
// frame (same discipline as queueRemoteTrigger). Non-static: rc_telemetry.h
// forward-declares it. id 0 = clear all.
void __attribute__((noinline)) queueForgetPeer(uint8_t id) {
  if (!forgetPeerQueue) return;
  xQueueSend(forgetPeerQueue, &id, 0);   // non-blocking; a dropped request can be re-issued
}

// =============================================================================
//  WCB receive callback
// =============================================================================
void onWCBCommand(uint8_t senderID, const char* command) {
  // Inbound COMMAND counter. WCB_Client's statistics are outbound-only
  // (sent/ackd/retries/failed/unguaranteed are all about deliveries WE originate), so
  // "how much is this board actually hearing" has no library counter — count it
  // here, at the single funnel every mesh COMMAND passes through.
  //
  // Deliberately BEFORE the rcTelemetry::handle() delegation, so a management
  // message counts as received traffic like any other. Raw-packet paths (OTA,
  // bulk chunks) are NOT counted — they never reach this callback.
  //
  // Core 0, unlocked: one aligned 32-bit increment with a single writer, read on
  // Core 1 for reporting. Same rationale as the library's own counters — a lock
  // here would put the loop task in contention with the WiFi task for nothing.
  // Volatile-free on purpose: this is a statistic, and a reader that is one
  // packet stale is not wrong in any way that matters.
  g_meshRxCount++;
  if (senderID >= 1 && senderID <= WCB_MAX_BOARDS) g_meshRxFrom[senderID - 1]++;
  // Delegate to the telemetry/management bridge first — if it recognised
  // the command as a JSON management message addressed to us (PING /
  // TRIGGER / SET_MODE / GET_CONFIG / SET_CONFIG), it dispatches and
  // returns true.  Otherwise we just log the raw command for visibility
  // (legacy WCB ;-commands intended for other peers or droid hardware).
  if (rcTelemetry::handle(senderID, command)) return;

  // ── Remote terminal ─────────────────────────────────────────────────────
  // A raw ?.../#... CLI line relayed from the config tool over the bridge. It
  // isn't JSON management traffic, so rcTelemetry::handle() declined it. Stash
  // it and run it in loop() — this callback is on Core 0 and execCliLine does
  // flash I/O / long prints that must not stall the WiFi task. Its Serial
  // output is tee'd back to senderID (the bridge) as RTERM packets that surface
  // on the config-tool terminal. Queue is non-blocking — drop under load rather
  // than stall the WiFi task.
  if (remoteCliQueue && command && (command[0] == '?' || command[0] == '#')) {
    queueRemoteCli(senderID, command);   // noinline — keeps the 200 B buffer off this frame
    return;
  }

  // A WCB-relayed Maestro read reply — §10 format ":MQR,<id>,<chan>,<KIND>,<value>"
  // (WCB_NATIVE_MAESTRO_DESIGN.md). Parse into g_maeRemote (the skip-if-running gate +
  // remote Read-live read it). Core-0 safe: the consumer only parses + stores.
  if (command && strncmp(command, ":MQR,", 5) == 0) {
    dlog(DBG_MAESTRO, "[DISPATCH] Maestro RX reply  %s\n", command);   // gated + infrequent
    maeConsumeRemoteReply(command + 5); return;
  }

  // ── Mesh → local Maestro: inbound `;M...` ───────────────────────────────────
  // A WCB whose remote-Maestro proxy points at us forwards the command as plain
  // text, verbatim (`;M35`, `;M3,goHome`, `;MG3,2,getMovingState`). Enqueue only —
  // the parse, the Serial2 write and any query readback all run on Core 1.
  if (command && command[0] == ';' && (command[1] == 'M' || command[1] == 'm')) {
    queueMaestroCmd(senderID, command);   // noinline — keeps the buffer off this frame
    return;
  }

  // ── Mesh → serial: targeted `;s<n><payload>` ────────────────────────────────
  // A WCB routing `;w20,;s2<cmd>` delivers exactly ";s2<cmd>" here — the `;w20,`
  // is stripped by the sending board, the rest arrives verbatim. <n> is the MESH
  // port (S1-S3 = the NaviCore v2 silkscreen), converted to the firmware port the
  // Streams are keyed by. The payload is written raw + CR, no comma stripping —
  // byte-identical to what the same command would put on a WCB's port.
  if (command && command[0] == ';' && (command[1] == 's' || command[1] == 'S') &&
      command[2] >= '0' && command[2] <= '9') {
    int meshPort = command[2] - '0';
    int fwPort   = rcFwPortForMesh(meshPort);
    if (fwPort == 0) {
      dlog(DBG_SERIAL, "[DISPATCH] Serial route: S%d is not a NaviCore port (S1-S3)\n", meshPort);
    } else if (!auxStreamFor(fwPort)) {
      dlog(DBG_SERIAL, "[DISPATCH] Serial route: S%d not available on this board\n", meshPort);
    } else {
      queueSerialFwd((uint8_t)fwPort, command + 3);   // deferred to loop — see SerialFwdMsg
    }
    return;
  }

  // ── Mesh → serial: broadcast ────────────────────────────────────────────────
  // An unprefixed command is a mesh BROADCAST and goes out every port that opted in
  // (none by default). Never re-broadcast to the mesh: this arrived from there, and the
  // out-path is mesh→serial only, so there is no loop to break.
  //
  // '{' is excluded as firmly as ';'. rcTelemetry::handle() above declines several
  // classes of JSON it cannot act on — an unknown "type", a type it deliberately no-ops,
  // a missing "type", a self-echoed rc_* frame, a parse failure — and ALL of them start
  // with '{'. Without this test they would be written verbatim onto the operator's serial
  // device: a bridged Reset-to-Defaults, or a peer's rc_hb/rc_ch telemetry at several
  // hundred B/s against a 9600-baud port. This is the same rule the WCB firmware applies
  // (WCB.ino consumes a '{'-prefixed payload and returns before processBroadcastCommand).
  // Declined JSON is not silently dropped — the tail log below still shows it.
  if (command && command[0] && command[0] != ';' && command[0] != '{') {
    queueSerialBroadcastOut(command, 0);
    // fall through — the log below still shows it, which is what makes an
    // unexpected broadcast visible instead of silently swallowed.
  }

  // Unhandled (legacy/unknown) WCB command.  This runs in the ESP-NOW
  // receive callback on Core 0; a blocking Serial.printf here can stall the
  // WiFi task (if a USB host is attached but not draining) and interleave
  // with the main loop's Serial output on Core 1.  It's rare (almost all RC
  // traffic is JSON handled above), so gate it behind the same verbose flag
  // used for the fragment logging rather than printing unconditionally.
  if (rcTelemetry::RC_TELEM_VERBOSE || (g_dbgFlags & DBG_MAESTRO))
    Serial.printf("[WCB RX] from WCB%d: %s\n", senderID, command);   // Maestro chip also surfaces a malformed/other WCB reply
}

// =============================================================================
//  PWM monitor — streams to USB Serial while wsMonitorActive
// =============================================================================
void sendPWMUpdate() {
  if (!wsMonitorActive) return;
  unsigned long now = millis();
  if (now - wsMonitorLastSent < WS_MONITOR_INTERVAL_MS) return;
  wsMonitorLastSent = now;

  unsigned long ageMs = (sbusLastFrameMs == 0) ? 99999 : (now - sbusLastFrameMs);
  bool sbusOk = (sbusFps > 0) && !lostFrameOld && (ageMs < 500);

  char channelBuf[300] = "";
  int chCount = sbusRx.detectedChCount > 24 ? 24 : sbusRx.detectedChCount;
  for (int i = 0; i < chCount; i++) {
    char tmp[16];
    snprintf(tmp, sizeof(tmp), "%d%s", sbusValues[i], (i + 1 < chCount) ? "," : "");
    strncat(channelBuf, tmp, sizeof(channelBuf) - strlen(channelBuf) - 1);
  }

  int modeCh = 0;
  if (rcConfig.funcBindings.modeSwitch >= 0 &&
      rcConfig.funcBindings.modeSwitch < RC_NUM_SWITCHES)
    modeCh = rcConfig.switches[rcConfig.funcBindings.modeSwitch].channel;

  char buf[640];
  snprintf(buf, sizeof(buf),
    "{\"type\":\"PWM_UPDATE\",\"matrixCh\":%d,\"modeCh\":%d,\"matrixVal\":%d,"
    "\"modeVal\":%d,\"btn\":%d,\"mode\":%d,"
    "\"sbus\":{\"ok\":%s,\"fps\":%d,\"frames\":%lu,\"ageMs\":%lu,"
    "\"lost\":%s,\"failsafe\":%s,\"chCount\":%d,\"frameLen\":%d,\"channels\":[%s]}}",
    rcConfig.matrixChannel, modeCh, oldValueMatrix, oldValueMode,
    pwmToButton(oldValueMatrix), FunctionSwState,
    sbusOk ? "true" : "false", sbusFps, sbusFrameCount, ageMs,
    lostFrameOld ? "true" : "false", sbusFailsafe ? "true" : "false",
    sbusRx.detectedChCount, sbusRx.detectedFrameLen, channelBuf);
  // Best-effort, NEVER blocking: the live monitor must not stall loop() — SBUS decode and
  // the Maestro-passthrough broadcast run there, and a blocked USB write (HWCDC has a 50 ms
  // tx timeout) starves them, which shows up as choppy remote passthrough. If the host
  // isn't draining fast enough — or the tab/USB closed without a STOP_MONITOR, leaving this
  // flag stuck on — DROP this frame; the next is 50 ms out. Mirrors wcbStreamLog()/vlogf().
  const size_t _pwmLen = strlen(buf);              // +2 for println's CRLF
  if (Serial.availableForWrite() >= (int)(_pwmLen + 2)) {
    Serial.println(buf);          // tees to a WebSocket client too, when one is armed
  } else {
    // USB cannot take it — but that does NOT mean nobody can. With no USB host
    // attached (every WiFi session) availableForWrite() is permanently 0, so this
    // guard silently dropped EVERY monitor frame and the live panel was dead over
    // the WebSocket while command replies worked fine. Replies are unguarded
    // Serial.println; this one is not, and that difference was the whole bug.
    // Deliver straight to the socket, which has its own non-blocking backpressure.
    naviws::printlnDirect(buf);
  }
}

// =============================================================================
//  applyBoardProfile — load the selected board's pin map into the runtime pin
//  globals. Called ONCE in setup() after the config loads and BEFORE any port
//  is opened (SBUS / Maestro / aux). boardType 0 = NaviCore v2, 1 = WCB HW 3.2.
//  Status LED is GPIO48 on both, so it isn't profiled.
// =============================================================================
// Runtime equivalent of the old compile-time SBUS_SHARED_UART flag. BOTH current
// boards set this true in applyBoardProfile() (SBUS IN+OUT share UART1, freeing
// UART0 for the hardware S3 aux). The false branch is the kept-but-unused
// dedicated-UART0-SBUS-OUT fallback, selectable by flipping a board's flag.
bool sbusSharedUart = false;
// boardType that applyBoardProfile() actually applied at boot. SET_CONFIG uses it
// to detect a live boardType change — pins are assigned only at boot, so a changed
// profile must defer pin-dependent re-apply to a reboot.
static uint8_t appliedBoardType = 0xFF;

static void applyBoardProfile() {
  if (rcConfig.boardType == BOARD_WCB_HW_32) {
    sbusSharedUart = true;                    // shared SBUS on UART1 (in GPIO5 / out GPIO4, both on the WCB Serial1 header); UART0 freed → hardware S3
    SBUS_RX_PIN = 5;   SBUS_OUT_PIN = 4;      // SBUS OUT moved 9→4 (Serial1 TX) to free the Serial5 header for S5
    MAESTRO_TX_PIN = 6; MAESTRO_RX_PIN = 7;
    S3_TX_PIN = 15;    S3_RX_PIN = 16;
    S4_TX_PIN = 17;    S4_RX_PIN = 18;
    S5_TX_PIN = 9;     S5_RX_PIN = 10;        // WCB "Serial 5" header (freed by moving SBUS OUT to GPIO4)
    Serial.println("[BOARD] WCB HW 3.2 pin profile");
  } else {                                   // BOARD_NAVICORE_V2 (default)
    sbusSharedUart = true;                    // shared SBUS on UART1; S3 = hardware UART0
    SBUS_RX_PIN = 4;   SBUS_OUT_PIN = 5;
    MAESTRO_TX_PIN = 6; MAESTRO_RX_PIN = 7;
    S3_TX_PIN = 8;     S3_RX_PIN = 9;
    S4_TX_PIN = 10;    S4_RX_PIN = 21;
    S5_TX_PIN = 38;    S5_RX_PIN = 47;
    Serial.println("[BOARD] NaviCore v2 pin profile");
  }
  appliedBoardType = rcConfig.boardType;   // remember what was applied so SET_CONFIG can detect a live change
}

// =============================================================================
//  Serial-port baud (re)application
//  Serial2 (Maestro) and Serial3/4 (aux SWSerial) are opened from rcConfig.
//  Called once at boot AND again after every SET_CONFIG save, so a baud
//  change in the config tool takes effect immediately — no reboot needed.
//  Only a port whose baud actually changed is re-opened: re-begin() briefly
//  drops the line, so unrelated saves must not disturb live ports.
// =============================================================================
static uint32_t appliedMaestroBaud = 0;
static uint32_t appliedAuxBaud[3]  = { 0, 0, 0 };

// NEVER hand a raw config value to begin(). A stored 0 is representable — the JSON parse
// substitutes its default only when the key is ABSENT (ArduinoJson's `|` dispatches on
// is<T>(), so an explicit 0 passes straight through) — and a 0 is fatal at both port
// types: SoftwareSerial::begin divides by baud (Xtensa IntegerDivideByZero → panic), and
// HardwareSerial::begin reads 0 as "auto-detect" and blocks up to 20 s, overrunning
// BOOT_GUARD_TIMEOUT_MS. Either way the bad value is already on flash and is re-applied
// at every boot, so the board boot-loops with no serial window to fix it in — recovery
// would need an esptool erase. Clamping HERE (not only on the input path) is what
// guarantees an already-corrupt stored config still boots. The range matches the config
// tool's own limits (1200 … 115200 = WDP_BAUDS).
static uint32_t sanBaud(uint32_t b, uint32_t def, const char* what) {
  if (b >= 1200 && b <= 115200) return b;
  Serial.printf("[AUX] %s baud %lu out of range (1200-115200) — using %lu\n",
                what, (unsigned long)b, (unsigned long)def);
  return def;
}

static void applySerialBauds(bool initial) {
  const uint32_t maestroBaud = sanBaud(rcConfig.maestroBaud, LOCAL_MAESTRO_BAUD_RATE, "Maestro");
  const uint32_t auxBaud[3]  = { sanBaud(rcConfig.auxBaud[0], 9600, "S3"),
                                 sanBaud(rcConfig.auxBaud[1], 9600, "S4"),
                                 sanBaud(rcConfig.auxBaud[2], 9600, "S5") };
  if (initial || maestroBaud != appliedMaestroBaud) {
    if (!initial) Serial2.end();
    Serial2.begin(maestroBaud, SERIAL_8N1, MAESTRO_RX_PIN, MAESTRO_TX_PIN);
    appliedMaestroBaud = maestroBaud;
    Serial.printf("[Serial2] Local Maestro %s @ %lu baud  TX=GPIO%d\n",
                  initial ? "open" : "re-open",
                  (unsigned long)maestroBaud, MAESTRO_TX_PIN);
  }
  if (initial || auxBaud[0] != appliedAuxBaud[0]) {
    if (!initial) auxEnd(s3, s3IsHw);
    auxBegin(s3, s3IsHw, auxBaud[0], S3_RX_PIN, S3_TX_PIN);   // hw UART0 (v2) or SoftwareSerial (3.2)
    appliedAuxBaud[0] = auxBaud[0];
    Serial.printf("[AUX] S3 %s @ %lu baud (%s)\n",
                  initial ? "open" : "re-open", (unsigned long)auxBaud[0], s3IsHw ? "hw UART0" : "sw");
  }
  if (initial || auxBaud[1] != appliedAuxBaud[1]) {
    if (!initial) auxEnd(s4, false);
    auxBegin(s4, false, auxBaud[1], S4_RX_PIN, S4_TX_PIN);
    appliedAuxBaud[1] = auxBaud[1];
    Serial.printf("[AUX] S4 %s @ %lu baud\n",
                  initial ? "open" : "re-open", (unsigned long)auxBaud[1]);
  }
  // S5 — both boards (v2 GPIO38/47, WCB 3.2 GPIO9/10). s5 is nullptr only in the unused
  // dedicated-SBUS fallback, so the guard still applies.
  if (s5 && (initial || auxBaud[2] != appliedAuxBaud[2])) {
    if (!initial) auxEnd(s5, false);
    auxBegin(s5, false, auxBaud[2], S5_RX_PIN, S5_TX_PIN);
    appliedAuxBaud[2] = auxBaud[2];
    Serial.printf("[AUX] S5 %s @ %lu baud\n",
                  initial ? "open" : "re-open", (unsigned long)auxBaud[2]);
  }
}

// =============================================================================
//  SBUS OUT enable/disable  (rcConfig.sbusOutEnabled)
//  The SBUS-OUT passthrough re-emits every received frame on SBUS_OUT_PIN. Its
//  only recurring cost is the per-byte tee in SbusReader::read() (one sink
//  write per received SBUS byte). When OFF we drop the sink entirely — no tee,
//  no CPU — and, in the dedicated-UART layout, release UART0. Called once after
//  the config load AND after every SET_CONFIG, so the toggle applies live (no
//  reboot). A guard skips the work when the setting hasn't changed.
// =============================================================================
static bool sbusOutApplied = false;
static void applySbusOut(bool initial) {
  bool want = rcConfig.sbusOutEnabled;
  if (!initial && want == sbusOutApplied) return;   // no change since last apply
  if (want) {
    if (sbusSharedUart) {
      // v2 shared UART1: TX is already configured by sbusRx.begin(); enabling OUT
      // just starts teeing each received byte to it. (Serial0/UART0 is the aux S3.)
      sbusRx.setPassthroughSink(&Serial1);
    } else {
      // 3.2 dedicated UART0: bring it up (TX-only, 100k 8E2 inverted) and tee.
      Serial0.setTxBufferSize(256);                  // must precede begin()
      Serial0.begin(100000, SERIAL_8E2, /*rxPin=*/-1, /*txPin=*/SBUS_OUT_PIN, /*invert=*/true);
      sbusRx.setPassthroughSink(&Serial0);
    }
    Serial.printf("[SBUS] OUT enabled — re-emit on GPIO%d (100k 8E2 inverted)\n", SBUS_OUT_PIN);
  } else {
    sbusRx.setPassthroughSink(nullptr);              // stop the tee — zero CPU when unused
    // Release UART0 ONLY on 3.2 (dedicated). On v2, UART0 is the hardware aux S3,
    // NOT SBUS-out, so it must never be ended here.
    if (!sbusSharedUart && !initial) Serial0.end();
    Serial.println("[SBUS] OUT disabled (passthrough off — no CPU cost)");
  }
  sbusOutApplied = want;
}

// Live re-apply of everything a saved config can change at runtime.  Shared by
// BOTH save transports: the USB SET_CONFIG handler and rcTelemetry's Via-WCB
// _applyReassembled().  It exists as one function because the two paths drifted
// — the mesh path used to skip all of this, so a Save over the bridge left the
// board running the OLD baud / SBUS-OUT / easing / auto-release state until the
// next reboot, with no indication anything was pending.  Any new post-save
// fixup belongs HERE, not in one caller.
//
// Returns false when boardType changed: the pin profile is assigned only at boot
// (applyBoardProfile runs once in setup()), so re-opening ports now would use the
// OLD board's pins.  Everything pin-dependent defers to the reboot the GUI
// already prompts for after a board change; the caller reports that its own way.
bool applyConfigSideEffects() {
  if (rcConfig.boardType != appliedBoardType) return false;
  applySerialBauds(false);   // HCR / MP3 / Maestro pick up a new rate immediately
  applySbusOut(false);       // apply a flipped SBUS-OUT toggle live
  // Re-apply Maestro easing so a changed smoothing profile / knob assignment /
  // profile value takes effect IMMEDIATELY (matching the switch path) instead of
  // waiting for a stick move — and resets any channel the new profile leaves
  // uncovered (the steady-state hot path won't drive those down, so without this
  // a re-assigned profile could leave the OLD easing stuck on the servo).
  // Cache-gated, so a save that didn't touch easing is a no-op.
  //
  // NOT while a clip is playing: _buildCurveIndex() zeroed speed/accel (= unlimited) on
  // every channel the replay touches, on purpose — the Maestro's own latched limits can't
  // be read back, so the player force-resets them and owns the motion shaping itself. Any
  // save at all (a button label is enough) would re-latch the profile here and the servos
  // would additionally be rate-limited device-side, rounding off the recorded motion. A
  // LOOPING clip never rebuilds its curve index (_rewindCurves doesn't re-zero), so it
  // would stay double-smoothed for every remaining lap. loop() re-applies on the
  // takeReplayDone edge; a stopped replay self-heals on the next stick move via the
  // g_maeSpeed/g_maeAccel mismatch check.
  if (!navirec::isReplaying())
    for (uint8_t mid = 1; mid <= RC_NUM_MAESTROS; mid++) reapplyMaestroEasing(mid);
  resetMaestroReleaseState();   // re-derive auto-release policy (no stale idle release)
  return true;
}

// =============================================================================
//  USB Serial WebSerial protocol handler
//  Same JSON protocol as Body Controller so the config_tool HTML is compatible.
// =============================================================================
String serialInputBuf;

// Execute one CLI line (?OTALOCAL,* / ?OTA,* / #L*).  Shared by the USB serial
// console (handleSerialInput) and the remote terminal over the WCB bridge
// (drainRemoteCli) so a locally-typed command and one relayed from the config
// tool run identically.  All output goes to Serial; the remote path tees Serial
// to the RTERM capture sink so the lines surface on the config-tool terminal.
// Returns true if the line was a recognised CLI command.
bool execCliLine(const String& line) {
  if (line.startsWith("?OTALOCAL,")) { naviota::processOtaLocalCommand(line.substring(10)); return true; }
  if (line.startsWith("?OTA,"))      { naviota::processOtaRelayCommand(line.substring(5));  return true; }
  // ?FORGET,<id>  — drop learned WCB <id> from the peer table + NVS
  // ?FORGET,ALL   — drop every learned (auto-joined) peer
  if (line.length() >= 7 && line.substring(0, 7).equalsIgnoreCase("?FORGET")) {
    String arg = (line.length() > 8) ? line.substring(8) : "";   // after "?FORGET,"
    arg.trim();
    if (arg.equalsIgnoreCase("ALL")) {
      doForgetPeer(0);
    } else {
      int id = arg.toInt();
      if (id >= 1 && id <= WCB_MAX_BOARDS) doForgetPeer((uint8_t)id);
      else Serial.println("[WCB] usage: ?FORGET,<id 1-20>  or  ?FORGET,ALL");
    }
    return true;
  }
  // ── Direct Maestro control + 2-way queries. Fire-and-forget writes drive the
  //    config-tool timeline editor's LIVE scrub/preview (servos follow the yellow
  //    cursor); the GET/MOVING/ERR queries read a LOCAL Maestro's reply off Serial2
  //    and print a [MAE:<slot>]{…} marker (see maestroLocalQuery/maestroReportQuery).
  //    All run in loop()/Core-1 like the rest of the CLI, so the Maestro serial I/O
  //    is safe; queries bound the read so a missing/unwired Maestro can't stall SBUS.
  //      ?MAE,<slot>,<ch>,<pos>   set target (¼µs) on logical Maestro slot 1-8, ch 0-31
  //      ?MAE,FREE,<slot>,<ch>    speed=0/accel=0 so the preview tracks the cursor snappily
  //      ?MAE,GET,<slot>,<ch>     Get Position    → {"q":"pos",…}  (2-byte reply, ¼µs)
  //      ?MAE,MOVING,<slot>       Get Moving State → {"q":"mov",…}  (1-byte reply, 0/1)
  //      ?MAE,ERR,<slot>          Get Errors       → {"q":"err",…}  (2-byte reply; CLEARS errors on read)
  //    LOCAL slots reply synchronously off Serial2; REMOTE slots broadcast the query and
  //    the hosting WCB relays the reply back as :MQR → [MAE:] (async).
  if (line.length() >= 5 && line.substring(0, 5).equalsIgnoreCase("?MAE,")) {
    String a = line.substring(5); a.trim();
    int c1 = a.indexOf(','), c2 = (c1 >= 0) ? a.indexOf(',', c1 + 1) : -1;
    String verb = (c1 >= 0) ? a.substring(0, c1) : a;   // first field: FREE/GET/MOVING/ERR, or a numeric slot
    if (verb.equalsIgnoreCase("FREE")) {
      if (c1 > 0 && c2 > c1) {
        maestroSetSpeed((uint8_t)a.substring(c1 + 1, c2).toInt(), (uint8_t)a.substring(c2 + 1).toInt(), 0);
        maestroSetAccel((uint8_t)a.substring(c1 + 1, c2).toInt(), (uint8_t)a.substring(c2 + 1).toInt(), 0);
      }
    } else if (verb.equalsIgnoreCase("GET")) {          // ?MAE,GET,<slot>,<ch> — Get Position
      if (c1 > 0 && c2 > c1) {
        uint8_t slot = (uint8_t)a.substring(c1 + 1, c2).toInt();
        uint8_t ch   = (uint8_t)a.substring(c2 + 1).toInt();
        uint8_t rb[2]; uint16_t val = 0;
        int n = maestroLocalQuery(slot, 0x90, &ch, 1, rb, WcbMaestro::replyLen(WcbMaestro::ReplyKind::POS));
        if (n > 0) WcbMaestro::decodeReply(WcbMaestro::ReplyKind::POS, rb, (size_t)n, val);
        maeLatchRemoteRelay(slot, n);   // remote read → answer emits later; remember where to send it
        maestroReportQuery(slot, 0, ch, n, val);
      }
    } else if (verb.equalsIgnoreCase("MOVING")) {       // ?MAE,MOVING,<slot> — Get Moving State
      if (c1 > 0) {
        uint8_t slot = (uint8_t)a.substring(c1 + 1).toInt();
        uint8_t rb[1]; uint16_t val = 0;
        int n = maestroLocalQuery(slot, 0x93, nullptr, 0, rb, WcbMaestro::replyLen(WcbMaestro::ReplyKind::MOV));
        if (n > 0) WcbMaestro::decodeReply(WcbMaestro::ReplyKind::MOV, rb, (size_t)n, val);
        maeLatchRemoteRelay(slot, n);
        maestroReportQuery(slot, 1, 0, n, val);
      }
    } else if (verb.equalsIgnoreCase("ERR")) {          // ?MAE,ERR,<slot> — Get Errors (clears on read)
      if (c1 > 0) {
        uint8_t slot = (uint8_t)a.substring(c1 + 1).toInt();
        uint8_t rb[2]; uint16_t val = 0;
        int n = maestroLocalQuery(slot, 0xA1, nullptr, 0, rb, WcbMaestro::replyLen(WcbMaestro::ReplyKind::ERR));
        if (n > 0) WcbMaestro::decodeReply(WcbMaestro::ReplyKind::ERR, rb, (size_t)n, val);
        maeLatchRemoteRelay(slot, n);
        maestroReportQuery(slot, 2, 0, n, val);
      }
    } else if (c1 > 0 && c2 > c1) {                     // ?MAE,<slot>,<ch>,<pos> — set target
      maestroSetTarget((uint8_t)a.substring(0, c1).toInt(),
                       (uint8_t)a.substring(c1 + 1, c2).toInt(),
                       (uint16_t)a.substring(c2 + 1).toInt());
    }
    return true;
  }
  // ── Record/replay bench control (phase 1) + timeline editor transport (phase 2)
  //   ?REC,START/STOP/PLAY/SAVE/LOAD/LS/RM/RENAME/CLEAR/INFO   (case-insensitive)
  //   ?REC,EDITLOAD,<name>            — dump a clip as [CLIPDL:*] JSON lines
  //   ?REC,EDITBEGIN / EDITEV,<json> / EDITEND,<name> / EDITCANCEL — upload an edited clip
  if (line.length() >= 4 && line.substring(0, 4).equalsIgnoreCase("?REC")) {
    String arg = (line.length() > 5) ? line.substring(5) : "";   // after "?REC,"
    arg.trim();
    // Split "SUB[,name]" (comma or space) so SAVE/LOAD/RM/PLAY can take a clip name.
    String sub = arg, name = "";
    int sep = arg.indexOf(','); if (sep < 0) sep = arg.indexOf(' ');
    if (sep >= 0) { sub = arg.substring(0, sep); name = arg.substring(sep + 1); sub.trim(); name.trim(); }
    if      (sub.equalsIgnoreCase("START")) Serial.println(navirec::startRecord((uint8_t)FunctionSwState) ? "[REC] recording…" : "[REC] busy / no buffer");
    else if (sub.equalsIgnoreCase("STOP")) {                 // aborts recording OR an in-progress replay
      if (navirec::stop() == navirec::ST_REPLAYING) Serial.println("[REC] replay stopped");
      else                                          navirec::info(Serial);
    }
    else if (sub.equalsIgnoreCase("PLAY")) {
      if (name.length() && !navirec::loadClip(name.c_str())) { Serial.printf("[REC] clip '%s' not found\n", name.c_str()); return true; }
      if (navirec::startReplay())
        Serial.printf("[REC] replaying %lu events over %lums — ?REC,STOP to abort\n",
                      (unsigned long)navirec::eventCount(), (unsigned long)navirec::clipDurationMs());
      else Serial.println("[REC] busy / empty");
    }
    else if (sub.equalsIgnoreCase("SAVE")) {
      // No name → auto-name "rec_N", same as a blank-named Record trigger, so a
      // take saved from the Clips panel / CLI is never silently lost either.
      char autoName[16];
      const char* nm = name.c_str();
      if (!name.length()) { navirec::_autoClipName(autoName, sizeof(autoName)); nm = autoName; }
      Serial.println(navirec::saveClip(nm) ? (String("[REC] saved clip '") + nm + "'").c_str()
                                           : "[REC] save failed (see reason above)");
    }
    else if (sub.equalsIgnoreCase("LOAD"))  Serial.println(navirec::loadClip(name.c_str()) ? "[REC] loaded" : "[REC] load failed (not found / no FS)");
    else if (sub.equalsIgnoreCase("LS"))    {
      // Report the clips-partition storage first (short marker → survives the WCB
      // RTERM 160-byte wrap), then the per-clip list.
      if (g_clipsReady) Serial.printf("[CLIPFS]{\"total\":%u,\"used\":%u}\n",
                                      (unsigned)clipsFS.totalBytes(), (unsigned)clipsFS.usedBytes());
      Serial.println("[REC] clips:");
      navirec::listClips(Serial);
    }
    else if (sub.equalsIgnoreCase("RM"))    Serial.println(navirec::deleteClip(name.c_str()) ? "[REC] deleted" : "[REC] delete failed");
    else if (sub.equalsIgnoreCase("RENAME")) {
      int c2 = name.indexOf(',');
      if (c2 > 0) {
        bool ok = navirec::renameClip(name.substring(0, c2).c_str(), name.substring(c2 + 1).c_str());
        Serial.println(ok ? "[REC] renamed" : "[REC] rename failed (exists / not found)");
        // Machine marker so the config tool only re-points its trigger-action
        // references when the board ACTUALLY renamed the file.
        Serial.printf("[CLIPUL:RENAME,%s]\n", ok ? "OK" : "ERR");
      }
      else Serial.println("[REC] usage: ?REC,RENAME,<from>,<to>");
    }
    // ── Timeline editor transport (config-tool Phase 2) — see navicore_record.h
    //    "Phase 2: timeline editor transport" for the full protocol writeup.
    else if (sub.equalsIgnoreCase("EDITLOAD")) {
      // ?REC,EDITLOAD,<name>[,<from>[,<count>]]
      // The outer split at the top of this block cuts only at the FIRST comma, so
      // `name` is the whole remainder — without this second-level split
      // "?REC,EDITLOAD,clip,0,64" loads a clip literally named "clip,0,64", which
      // _clipPath sanitises to "clip064": the WRONG clip, silently. Same idiom as
      // RENAME and indexed EDITEV above. Safe because _clipPath keeps only
      // alnum/_/- , so a comma can never be part of a real clip's identity.
      String cname = name;
      uint32_t from = 0, want = 0xFFFFFFFFu;
      bool ranged = false, batch = false;
      { int c2 = name.indexOf(',');
        if (c2 > 0) {
          cname = name.substring(0, c2);
          String rest = name.substring(c2 + 1); rest.trim();
          int c3 = rest.indexOf(',');
          from   = (uint32_t)(c3 > 0 ? rest.substring(0, c3) : rest).toInt();
          if (c3 > 0) {
            // Optional 4th arg ",B" opts into BATCHED keyframe lines
            // ([CLIPDL:EVB]). Safe to append: the old parser read `want` with
            // toInt(), which stops at the comma and never saw the flag — so a new
            // tool talking to old firmware just gets the per-event form back, and
            // an old tool omits the flag and is unaffected. The tool handles both
            // shapes regardless, so neither side needs to know the other version.
            String cw = rest.substring(c3 + 1);
            int c4 = cw.indexOf(',');
            if (c4 >= 0) {
              String fl = cw.substring(c4 + 1); fl.trim();
              batch = fl.equalsIgnoreCase("B");
              cw = cw.substring(0, c4);
            }
            want = (uint32_t)cw.toInt();
          }
          ranged = true;
        } }

      // Reload only when this clip is not already resident. A ranged download
      // assembles across several requests and the board sits at ST_IDLE between
      // them, so residency must be RE-VERIFIED (a Record/Play trigger can replace
      // the buffer underneath) — but a blind reload per range re-reads
      // _count*140 B from LittleFS, ~420 KB for a 3000-event clip, stalling loop()
      // far longer than the streaming itself. Fail closed: any buffer mutator
      // clears _loadedName.
      if (strcmp(navirec::_loadedName, cname.c_str()) != 0) {
        if (!navirec::loadClip(cname.c_str())) {
          // Legacy byte-for-byte on the unranged path — an old tool matches
          // "[CLIPDL:ERR]" by exact prefix and slices a fixed offset.
          Serial.printf("[REC] clip '%s' not found\n", cname.c_str());
          return true;
        }
      }

      // Via-WCB (capture sink armed): every [CLIPDL:EV] line is an RTERM packet
      // and editStream runs in loop(), so a dense capture streamed whole would
      // stall SBUS/WCB servicing for tens of seconds. A RANGED request is the
      // answer to that — the caller asks for a bounded slice — so the size
      // refusal applies only to the legacy whole-clip form.
      const bool relayed = rcSerial.captureArmed();
      // BOUND THE SLICE ON THE BOARD, not in the caller. `ranged` is true as soon
      // as one extra comma is present, and `want` stays 0xFFFFFFFF when no count
      // is supplied — so "?REC,EDITLOAD,<name>,0" is a ranged request for the
      // ENTIRE clip. Skipping the size refusal for it (and the old message even
      // suggested "use a ranged request") handed a hand-typed remote-terminal
      // line the exact loop() stall the refusal exists to prevent: one ESP-NOW
      // packet per event plus delay(1) every 4, i.e. ~6 s of blocked SBUS on a
      // 24000-event clip. Protection must not depend on the client being polite.
      if (relayed) {
        const uint32_t MAX_RELAY_SLICE = 512;   // ~128 ms of pacing; the tool asks for 48
        if (want > MAX_RELAY_SLICE) want = MAX_RELAY_SLICE;
        if (!ranged && navirec::eventCount() > 3000) {
          Serial.printf("[CLIPDL:ERR]clip too large to edit over the WCB bridge (%lu events) — connect over USB\n",
                        (unsigned long)navirec::eventCount());
          return true;
        }
      }
      navirec::editStream(Serial, relayed, from, want, ranged, batch);
    }
    else if (sub.equalsIgnoreCase("EDITBEGIN")) {
      Serial.println(navirec::editBegin() ? "[CLIPUL:BEGIN,OK]" : "[CLIPUL:BEGIN,ERR,busy]");
    }
    else if (sub.equalsIgnoreCase("EDITEV")) {
      // `name` here is "<idx>,<event json>" — the SUB/name comma split above
      // only splits on the FIRST comma, so the JSON's own internal commas pass
      // through untouched. The index makes the write IDEMPOTENT (a timeout-
      // retry resend can't append a duplicate) and the ACK echoes it back so
      // the tool can correlate a late/stale ACK with the right event.
      int ci = name.indexOf(',');
      if (ci > 0 && navirec::editAddEvent((uint32_t)name.substring(0, ci).toInt(), name.substring(ci + 1).c_str()))
        Serial.printf("[CLIPUL:ACK,%s]\n", name.substring(0, ci).c_str());
      else
        Serial.println("[CLIPUL:NAK,bad event / bad index / not editing]");
    }
    else if (sub.equalsIgnoreCase("EDITEND")) {
      const char* err = navirec::editEnd(name.c_str());
      if (err) Serial.printf("[CLIPUL:END,ERR,%s]\n", err);
      else     Serial.println("[CLIPUL:END,OK]");
    }
    else if (sub.equalsIgnoreCase("EDITCANCEL")) { navirec::editCancel(); Serial.println("[CLIPUL:CANCEL,OK]"); }
    else if (sub.equalsIgnoreCase("CLEAR")) { navirec::clearClip(); Serial.println("[REC] cleared"); }
    else                                    navirec::info(Serial);   // bare "?REC" or "?REC,INFO"
    return true;
  }
  if (line.length() >= 1 && line[0] == '#') {
    // ── CLI commands ───────────────────────────────────────────────────────
    if (line.length() >= 3 && (line[1] == 'L' || line[1] == 'l')) {
      int fn = 0;
      if (line.length() >= 4)
        fn = (line[2]-'0')*10 + (line[3]-'0');
      else
        fn = line[2] - '0';
      switch (fn) {
        case 1:  Serial.println("NaviCore — WCB HW 3.2"); break;
        case 2:  ESP.restart(); break;
        case 9:  dumpSbusState(); break;
        case 10: sbusLiveDump = !sbusLiveDump;
                 Serial.printf("SBUS live dump %s\n", sbusLiveDump ? "ON (1Hz)" : "OFF"); break;
        case 11: {
          Serial.printf("WCB device ID: %d  quantity: %d\n",
                        rcConfig.wcbNetwork.deviceId, rcConfig.wcbNetwork.quantity);
          Serial.print("  Board status: ");
          { int q = rcConfig.wcbNetwork.quantity;
            int hi = rcTelemetry::wcbHighestKnown(q);
            for (int i = 1; i <= hi; i++)
              if (rcTelemetry::wcbBoardKnown(i, q))
                Serial.printf("WCB%d=%s%s ", i, wcb->isOnline(i) ? "UP" : "dn", (i > q) ? "*" : ""); }
          Serial.println(" (* = auto-discovered beyond quantity)");
          break;
        }
        case 12: Serial.printf("Mode=%d  matrixBtn=%d  matrixVal=%d\n",
                               FunctionSwState, pwmToButton(oldValueMatrix), oldValueMatrix); break;
        // #L13 — Raw SBUS frame hex dump.  Shows exactly what bytes the
        // RX is delivering, with channel-decoding offsets annotated so
        // we can see at a glance whether bytes 23-33 (CH17-24 in SBUS-24)
        // carry data or are zero on the wire.
        case 13: {
          const uint8_t* raw = sbusRx.rawFrameBytes();
          uint8_t        len = sbusRx.rawFrameLen();
          if (len == 0) {
            Serial.println("---- SBUS RAW ---- (no frame parsed yet)");
            break;
          }
          Serial.printf("---- SBUS RAW ---- (%u bytes, %s)\n",
                        len, len == 25 ? "SBUS-16" :
                             len == 36 ? "SBUS-24" : "unknown length");
          for (uint8_t i = 0; i < len; i++) {
            if (i % 8 == 0) Serial.printf("  [%2u] ", i);
            Serial.printf("%02X ", raw[i]);
            if (i % 8 == 7) Serial.println();
          }
          if (len % 8) Serial.println();
          Serial.println("  byte 0       = header (expect 0F)");
          if (len == 25) {
            Serial.println("  bytes 1-22   = CH1-16 data");
            Serial.println("  byte 23      = flags");
            Serial.println("  byte 24      = footer (expect 00)");
          } else if (len == 36) {
            Serial.println("  bytes 1-22   = CH1-16 data");
            Serial.println("  bytes 23-33  = CH17-24 data  ← check these");
            Serial.println("  byte 34      = flags");
            Serial.println("  byte 35      = footer (expect 00)");
          }
          break;
        }
        // ── HCR local-serial TX diagnostics ──────────────────────────────
        // These bypass rcConfig.hcrDest AND button mapping entirely. They
        // drive Serial3/Serial4 directly so a "nothing on the HCR" report
        // can be split into:  firmware-TX-path/wiring  vs  config/dispatch.
        //   #L20 → HCR on S3 (GPIO15 TX): SetEmotion(HAPPY,80) + raw marker
        //   #L21 → HCR on S4 (GPIO17 TX): same
        // If the HCR reacts to #L20 but not to a mapped button, the bug is
        // in config (hcrDest transport/target not saved) or button mapping,
        // NOT the serial wiring. If #L20 also does nothing, it's wiring
        // (TX/RX swap, ground, 3V3 vs 5V) or the EspSoftwareSerial port.
        case 20:
        case 21: {
          Stream*     h  = (fn == 20) ? s3 : s4;
          const char* pn = (fn == 20) ? "S3" : "S4";
          Serial.printf("[HCR TEST] -> %s : SetEmotion(HAPPY,80) via hcrFormatCommand + raw frame\n", pn);
          // Send the SetEmotion(HAPPY,80) payload we'd normally dispatch
          // through hcrFormatCommand, plus a second raw line in case a
          // formatter bug is the cause — both are the exact byte string a
          // working HCR expects, no library logic in between.
          h->print(hcrFormatCommand(2 /*SetEmotion*/, 0 /*HAPPY*/, 80));
          h->print("<OH80,QEH>\n");
          Serial.println("[HCR TEST] sent — watch the HCR; check TX wiring to HCR RX, common ground");
          break;
        }
        default:
          Serial.printf("Unknown #L code %d. Valid: 1,2,9,10,11,12,13,20,21\n", fn);
          break;
      }
    }
    return true;
  }
  return false;
}

// ── Aux-serial RX monitor ────────────────────────────────────────────────────
// NaviCore drives S3/S4/S5 write-only for peripherals; nothing ever consumed
// their RX. This drains each port and echoes complete lines to the USB console
// under the "Serial" debug chip (prefix "[DISPATCH] Serial RX <port>" so it
// groups with the outgoing Serial log). Line-buffered: flush on CR/LF, when the
// buffer fills, or after ~60 ms idle. Non-printable bytes shown as '.'. The
// per-line buffer is also the hook where a future "act on incoming serial"
// parser would live. Ports are drained every loop so their FIFOs never overflow;
// the dlog is a no-op (nothing sent) unless the Serial debug chip is on.
#define AUX_RX_BUF 128
// One assembled chunk off an aux port. `terminated` is true only for a real CR/LF
// line — a buffer-full or idle flush is a FRAGMENT, and so is the REMAINDER of a line
// that was already cut that way (see `cut` below). A fragment is shown on the terminal
// but never broadcast, because a command is a line (the same rule processIncomingSerial
// uses on a WCB). Runs on Core 1 (loop), so broadcasting from here is safe.
static void auxRxLine(int idx, const char* buf, bool terminated) {
  dlog(DBG_SERIAL, "[DISPATCH] Serial RX [%s]  %s\n", auxPortLabel(idx), buf);
  if (!terminated || !rcConfig.serialBcastIn[idx]) return;

  // ── Serial → mesh: broadcast in ───────────────────────────────────────────
  // This port is in the broadcast domain, so the line goes where it would go on a
  // WCB: out to the whole mesh, and out our OTHER opted-in ports (never back down
  // the one it came from). Unensured — this is a repeating console/device stream,
  // not a command worth spending a pending slot and retransmits on.
  int fwPort = idx + 3;
  // A device-owned port is skipped here for the same reason the fan-out skips it:
  // an HCR's status frames or an MP3 Trigger's replies are that driver's protocol,
  // not commands, and spraying them across the mesh would be noise at best.
  if (auxPortHasDevice(fwPort)) return;
  if (wcb && wcbReady) wcb->broadcast(buf, /*ensured=*/false);
  queueSerialBroadcastOut(buf, fwPort);
  dlog(DBG_SERIAL, "[DISPATCH] Serial RX [%s] → mesh broadcast\n", auxPortLabel(idx));
}
// `cut` remembers that THIS physical line was already flushed once as a fragment
// (buffer-full or idle split). Without it the remainder of an over-long line hits
// the CR/LF branch and is reported terminated=true, so auxRxLine broadcasts the
// tail to the whole mesh as if it were a complete command. The tail of a cut line
// is still just a fragment.
static void auxRxPollPort(Stream* p, int idx, char* buf, uint8_t& len, unsigned long& lastMs, bool& cut) {
  if (!p) return;
  while (p->available()) {
    int c = p->read();
    lastMs = millis();
    if (c == '\n' || c == '\r') {
      if (len) { buf[len] = 0; auxRxLine(idx, buf, !cut); len = 0; }
      cut = false;                                        // next line starts clean
    } else {
      if (len >= AUX_RX_BUF - 1) { buf[len] = 0; auxRxLine(idx, buf, false); len = 0; cut = true; }
      buf[len++] = (c >= 32 && c < 127) ? (char)c : '.';   // sanitize non-printable for the terminal
    }
  }
  if (len && (millis() - lastMs) > 60) { buf[len] = 0; auxRxLine(idx, buf, false); len = 0; cut = true; }
}
static void pollAuxSerialRx() {
  static char b3[AUX_RX_BUF], b4[AUX_RX_BUF], b5[AUX_RX_BUF];
  static uint8_t l3 = 0, l4 = 0, l5 = 0;
  static unsigned long m3 = 0, m4 = 0, m5 = 0;
  static bool c3 = false, c4 = false, c5 = false;
  auxRxPollPort(s3, 0, b3, l3, m3, c3);
  auxRxPollPort(s4, 1, b4, l4, m4, c4);
  auxRxPollPort(s5, 2, b5, l5, m5, c5);
}

// Execute ONE complete, already-framed input line — the shared entry point for
// every transport. handleSerialInput() reads bytes off Serial and does the framing;
// anything else that can produce a line (a WebSocket, a relayed CLI command) calls
// straight in here instead of duplicating this dispatch.
//
// Returns TRUE if the caller should yield to loop() before handling more input.
// That is not cosmetic: the '?' branch and a JSON parse failure both used to
// `return` out of handleSerialInput() mid-drain so heartbeats keep running between
// the host's ACK-paced OTA chunks. A caller that ignores this and keeps draining
// reintroduces the stall that early return was added to prevent.
//
// The caller owns the buffer. `line` is a const reference and is NOT cleared here —
// a SET_CONFIG payload can approach 98 KB and copying it per line would be real cost
// on a board whose config already lives in PSRAM for the same reason.
bool processInputLine(const String& line) {

  // ── "?..." CLI commands ───────────────────────────────────────────────
  // ?OTALOCAL,* / ?OTA,* (firmware OTA) and ?REC,* (record/replay) all route
  // through execCliLine (shared with the remote terminal). One command per
  // call (early return) so loop() keeps heartbeats alive between the host's
  // ACK-paced OTA chunks. Unrecognised "?" commands report back rather than
  // being silently dropped.
  if (line[0] == '?') {
    if (!execCliLine(line))
      Serial.printf("Unknown command: %s\n", line.c_str());
        return true;    // yield to loop() — see the header comment
  }

  if (line[0] == '{') {
    // ── JSON WebSerial protocol ──────────────────────────────────────────
    // The header doc has to use a Filter — without one, deserializing
    // a SET_CONFIG payload (tens of KB once mappings/knobs/maestros
    // are populated) into the small hdr buffer returns NoMemory and
    // breaks SET_CONFIG. The filter is a WHITELIST in ArduinoJson v6:
    // every field used by a non-SET_CONFIG handler must be listed
    // explicitly, otherwise it's stripped and hdr["x"] returns the
    // default. SET_CONFIG does its own un-filtered deserialize below.
    StaticJsonDocument<256> filter;
    filter["type"]   = true;
    filter["target"] = true;   // WCB_SEND
    filter["cmd"]    = true;   // WCB_SEND
    filter["on"]     = true;   // CALIB
    filter["flags"]  = true;   // SET_DEBUG_FLAGS
    filter["mode"]   = true;   // TRIGGER
    filter["btn"]    = true;   // TRIGGER
    filter["tap"]    = true;   // TRIGGER
    filter["id"]     = true;   // FORGET_PEER (without this the id is stripped → parsed as 0 → "out of range")
    filter["all"]    = true;   // FORGET_PEER
    filter["wcb"]    = true;   // GET_WCB_SEQ (stripped here → parsed as 0 → "wcb out of range")
    filter["key"]    = true;   // GET_WCB_SEQVAL (stripped → empty → "key required")
    DynamicJsonDocument hdr(256);
    DeserializationError perr = deserializeJson(
        hdr, line,
        DeserializationOption::Filter(filter));
    if (perr != DeserializationError::Ok) {
      // Include the received-buffer length so the host can spot truncation
      // (host-sent length vs. received length mismatch → USB RX overflow).
      Serial.printf("{\"type\":\"ERROR\",\"msg\":\"JSON parse failed (%s)\",\"rxLen\":%u}\n",
                    perr.c_str(), (unsigned)line.length());
          return true;    // yield to loop() — see the header comment
    }
    const char* type = hdr["type"] | "";

    if (strcmp(type,"PING")==0 || strcmp(type,"ping")==0) {
      // A fresh page connect pings first — clear any stale calibration
      // mute left behind by a previously crashed/closed calibration page.
      calibrationActive = false;
      // Report firmware version so the GUI can display it (footer +
      // Firmware tab "currently on board").
      Serial.print("{\"type\":\"PONG\",\"version\":\"");
      Serial.print(FW_VERSION);
      Serial.println("\"}");

    } else if (strcmp(type,"GET_CONFIG")==0) {
      // rcConfigToJSON() returns an ERROR envelope, NOT a config, when the document
      // overflowed — that guard exists precisely so a truncated config never reaches
      // the tool. Splicing it into "data" defeats it: the tool would apply an envelope
      // with no config fields, keep its built-in defaults, latch them as the baseline,
      // and the next Save would ship those defaults over the good on-board config.
      // Pass the error through at the TOP level so it can never read as a config.
      String cfg = rcConfigToJSON();
      if (cfg.startsWith("{\"type\":\"ERROR\"")) {
        Serial.println(cfg);
      } else {
        Serial.print("{\"type\":\"CONFIG\",\"data\":");
        Serial.print(cfg);
        Serial.println("}");
      }

    } else if (strcmp(type,"GET_CMDLIB")==0) {
      // Stream back the config tool's private command library stored on this
      // droid (opaque to the firmware). Empty library if nothing saved yet.
      // Carry size + signature so the tool can cache + skip re-pulls.
      String lib;
      if (!rcCmdlibLoadLFS(lib) || lib.length() == 0) lib = "{\"boards\":[],\"enums\":{}}";
      Serial.printf("{\"type\":\"CMDLIB\",\"size\":%u,\"hash\":%u,\"data\":",
                    (unsigned)lib.length(), (unsigned)rcCmdlibHash(lib));
      Serial.print(lib);
      Serial.println("}");

    } else if (strcmp(type,"GET_CMDLIB_META")==0) {
      // Cheap signature (size + hash) so a connect can skip re-pulling an
      // unchanged library. 0/0 when nothing is stored.
      String lib; unsigned sz = 0, h = 0;
      if (rcCmdlibLoadLFS(lib) && lib.length()) { sz = lib.length(); h = rcCmdlibHash(lib); }
      Serial.printf("{\"type\":\"CMDLIB_META\",\"size\":%u,\"hash\":%u}\n", sz, h);

    } else if (strcmp(type,"SET_CMDLIB")==0) {
      // Persist the library OPAQUELY. Pull the raw "data" value by substring
      // (it can be many KB — avoid a second big parse), but find its END by
      // matching brackets, NOT by taking the message's last '}'. `data` is not
      // guaranteed to be the last field — the config tool's sendJSON() stamps
      // its "sys":1 marker after spreading the caller's object — and the naive
      // lastIndexOf('}') swallowed the trailing `,"sys":1` into the stored
      // library, so /cmdlib.json wasn't valid JSON on its own and its
      // size/hash covered bytes that were not library content (which then
      // read as "different library" against a mesh-saved copy).
      bool ok = false; unsigned h = 0, sz = 0;
      int k = line.indexOf("\"data\":");
      if (k >= 0) {
        int s = k + 7, blen = (int)line.length();
        while (s < blen && isspace((unsigned char)line[s])) s++;
        int end = -1;
        if (s < blen && (line[s] == '{' || line[s] == '[')) {
          const char open  = line[s];
          const char close = (open == '{') ? '}' : ']';
          int depth = 0; bool inStr = false, esc = false;
          for (int i = s; i < blen; i++) {
            char c = line[i];
            if (esc)      { esc = false;  continue; }
            if (inStr)    { if (c == '\\') esc = true; else if (c == '"') inStr = false; continue; }
            if (c == '"') { inStr = true; continue; }
            if (c == open)  { depth++; continue; }
            if (c == close) { if (--depth == 0) { end = i + 1; break; } }
          }
        }
        if (end > s) {
          String lib = line.substring(s, end);
          lib.trim();
          if (lib.length() > 0) { ok = rcCmdlibSaveLFS(lib); if (ok) { h = rcCmdlibHash(lib); sz = lib.length(); } }
        }
      }
      Serial.printf("{\"type\":\"ACK\",\"of\":\"SET_CMDLIB\",\"ok\":%s,\"size\":%u,\"hash\":%u}\n",
                    ok ? "true" : "false", sz, h);

    } else if (strcmp(type,"SET_CONFIG")==0) {
      DynamicJsonDocument bigDoc(98304);
      if (deserializeJson(bigDoc, line) != DeserializationError::Ok) {
        // No saveId echoed here — the parse failed, so we genuinely don't know
        // which save this was. The tool treats a missing saveId as "old
        // firmware" and still rolls the baseline back, which is what we want.
        Serial.println("{\"type\":\"ACK\",\"of\":\"SET_CONFIG\",\"ok\":false,\"msg\":\"parse failed\"}");
      } else if (!bigDoc.containsKey("data")) {
        Serial.printf("{\"type\":\"ACK\",\"of\":\"SET_CONFIG\",\"ok\":false,\"msg\":\"missing data\",\"saveId\":%ld}\n",
                      (long)(bigDoc["saveId"] | 0));
      } else {
        // Echo the tool's per-save correlation id on every reply, exactly as the
        // Via-WCB path does (rc_telemetry.h). Without it the tool's stale-ACK gate
        // permanently takes its "undefined saveId = old firmware" fallback on USB,
        // so a late ACK from a timed-out save is attributed to a newer one.
        // bigDoc is parsed UN-filtered, so no filter-whitelist entry is needed.
        const long saveId = (long)(bigDoc["saveId"] | 0);
        bool ok = rcConfigFromJSON(bigDoc["data"].as<JsonObject>());
        if (ok) {
          bool saved = rcConfigSaveLFS();
          rcAdvertiseSerialLabels();   // a changed port label / HCR/MP3/WLED dest → re-advertise over WDP
          // Live re-apply of baud / SBUS-OUT / easing / auto-release. Shared with the
          // Via-WCB save path — see applyConfigSideEffects().
          if (!applyConfigSideEffects())
            Serial.println("{\"type\":\"INFO\",\"msg\":\"boardType changed — reboot to apply the new pin profile\"}");
          // rcConfigSaveLFS() (flash write) + applySerialBauds()
          // block loop() for 100+ ms, during which processSbus() can't
          // run.  If the operator was holding a matrix button across
          // that gap, the frozen debounce state could produce a phantom
          // edge when loop() resumes.  Reset the matrix state machine to
          // a clean "must see a confirmed neutral, then a fresh press"
          // condition so the save can't manufacture a button event.
          matrixArmed        = false;
          matrixCandidate    = 0;
          matrixCandCount    = 0;
          matrixNeutralCount = 0;
          if (saved) Serial.printf("{\"type\":\"ACK\",\"of\":\"SET_CONFIG\",\"ok\":true,\"saveId\":%ld}\n", saveId);
          else       Serial.printf("{\"type\":\"ACK\",\"of\":\"SET_CONFIG\",\"ok\":false,\"msg\":\"applied to RAM but could not be saved to flash (LittleFS write error)\",\"saveId\":%ld}\n", saveId);
        } else {
          Serial.printf("{\"type\":\"ACK\",\"of\":\"SET_CONFIG\",\"ok\":false,\"msg\":\"config apply failed\",\"saveId\":%ld}\n", saveId);
        }
      }

    } else if (strcmp(type,"START_MONITOR")==0) {
      wsMonitorActive   = true;
      wsMonitorLastSent = 0;
      Serial.println("{\"type\":\"ACK\",\"ok\":true}");

    } else if (strcmp(type,"STOP_MONITOR")==0) {
      wsMonitorActive = false;
      // Calibration always runs with the monitor on; stopping it is a
      // reliable "calibration is over" signal even if CALIB-off is lost.
      calibrationActive = false;
      Serial.println("{\"type\":\"ACK\",\"ok\":true}");

    } else if (strcmp(type,"CALIB")==0) {
      const bool calibOn = hdr["on"] | false;
      // A take must never straddle a calibration session. Dispatch and passthrough are
      // both muted while the wizard runs, so a recording left running would capture a
      // silent gap — and once calibrationActive is set the operator can no longer stop
      // it (the Record trigger is muted too), so the 60 s backstop becomes the only
      // exit and it SAVES, overwriting the take's named clip with the dead stretch.
      // Drop the capture instead: stopRecord() flushes and goes idle without saving.
      if (calibOn && !calibrationActive && navirec::_state == navirec::ST_RECORDING) {
        navirec::stopRecord();
        Serial.println("[REC] recording dropped — calibration started (not saved)");
      }
      calibrationActive = calibOn;
      Serial.printf("[CALIB] action dispatch %s\n",
                    calibrationActive ? "SUPPRESSED (calibrating)" : "resumed");
      Serial.println("{\"type\":\"ACK\",\"ok\":true}");

    } else if (strcmp(type,"RESET_DEFAULTS")==0) {
      rcConfigLoadDefaults();
      resetMaestroReleaseState();   // clear stale auto-release state so defaults take effect live
      Serial.println("{\"type\":\"ACK\",\"ok\":true}");

    } else if (strcmp(type,"TEST_ACTION")==0) {
      // Config tool's per-action Test button — fire one action live, no save.
      // hdr is a filtered parse (no nested "action"), so re-parse the raw
      // buffer into a small doc. We're in loop() (Core 1), so dispatch is safe.
      StaticJsonDocument<640> tdoc;
      if (deserializeJson(tdoc, line) != DeserializationError::Ok) {
        Serial.println("{\"type\":\"ACK\",\"of\":\"TEST_ACTION\",\"ok\":false,\"msg\":\"parse failed\"}");
      } else {
        bool ok = rcTestAction(tdoc["action"].as<JsonObject>());
        Serial.printf("{\"type\":\"ACK\",\"of\":\"TEST_ACTION\",\"ok\":%s}\n", ok ? "true" : "false");
      }

    } else if (strcmp(type,"REBOOT")==0) {
      // ACK first so the GUI hears the reply, then restart after a brief
      // delay so the USB TX buffer drains before the reset kills it.
      Serial.println("{\"type\":\"ACK\",\"ok\":true,\"msg\":\"rebooting\"}");
      Serial.flush();
      // Say goodbye before vanishing.
      //
      // A bare ESP.restart() drops the SoftAP without telling anyone, so an
      // associated client keeps talking to an AP that is no longer there and only
      // works it out by timing out. Measured with ping across a reboot: ELEVEN
      // seconds of "Request timed out" while the board itself was serving again
      // after 2.4 s. That gap was the client's, not ours, and no amount of
      // reconnect logic on the far end can shorten it.
      //
      // softAPdisconnect() deauthenticates the stations first, so the client learns
      // immediately and can re-associate the moment the AP returns. Costs one call
      // and a few ms on a path that is about to reboot anyway.
      naviota::otaFarewellAP();   // shared with both OTA restart paths — one
                                  // implementation, so a fix reaches every reboot
      delay(250);
      ESP.restart();

    } else if (strcmp(type,"TRIGGER")==0) {
      int mode = hdr["mode"] | 1;
      int btn  = hdr["btn"]  | 0;
      int tap  = hdr["tap"]  | 1;
      if (btn < 1 || btn > RC_NUM_THRESHOLDS || mode < 1 || mode > 3 || tap < 1 || tap > RC_NUM_TAP_TIERS) {
        Serial.println("{\"type\":\"ACK\",\"ok\":false,\"msg\":\"bad mode/btn/tap\"}");
      } else {
        Serial.printf("[TRIGGER] mode=%d btn=%d tap=%d\n", mode, btn, tap);
        rcDispatch(mode * 100 + btn, (uint8_t)tap);
        Serial.println("{\"type\":\"ACK\",\"ok\":true}");
      }

    } else if (strcmp(type,"WCB_SEND")==0) {
      int target     = hdr["target"] | 0;
      const char* cmd = hdr["cmd"]   | "";
      if (!wcb || !wcbReady) {
        Serial.println("{\"type\":\"ACK\",\"ok\":false,\"msg\":\"WCB not ready (init failed)\"}");
      } else {
        if (target == 0) {
          wcb->broadcast(cmd);
          Serial.println("{\"type\":\"ACK\",\"ok\":true}");
        } else if (target >= 1 && target <= WCB_MAX_BOARDS) {
          wcb->send((uint8_t)target, cmd);
          Serial.println("{\"type\":\"ACK\",\"ok\":true}");
        } else {
          Serial.printf("{\"type\":\"ACK\",\"ok\":false,\"msg\":\"target %d out of range (0=broadcast, 1-%d=unicast)\"}\n",
                        target, WCB_MAX_BOARDS);
        }
      }

    } else if (strcmp(type,"FORGET_PEER")==0) {
      // Drop a learned (auto-joined) peer from the ESP-NOW peer table + NVS.
      //   {"id":N}      forget learned WCB N
      //   {"all":true}  forget ALL learned peers
      // USB path is Core 1, so call the worker directly. The panel re-polls
      // GET_WCB_STATUS to refresh after this.
      bool all = hdr["all"] | false;
      int  id  = hdr["id"]  | 0;
      if (!wcb || !wcbReady) {
        Serial.println("{\"type\":\"ACK\",\"of\":\"FORGET_PEER\",\"ok\":false,\"msg\":\"WCB not ready\"}");
      } else if (all) {
        doForgetPeer(0);
        Serial.println("{\"type\":\"ACK\",\"of\":\"FORGET_PEER\",\"ok\":true,\"all\":true}");
      } else if (id >= 1 && id <= WCB_MAX_BOARDS) {
        doForgetPeer((uint8_t)id);
        Serial.printf("{\"type\":\"ACK\",\"of\":\"FORGET_PEER\",\"ok\":true,\"id\":%d}\n", id);
      } else {
        Serial.printf("{\"type\":\"ACK\",\"of\":\"FORGET_PEER\",\"ok\":false,\"msg\":\"id %d out of range (1-%d) or set all:true\"}\n",
                      id, WCB_MAX_BOARDS);
      }

    } else if (strcmp(type,"SET_DEBUG_FLAGS")==0) {
      // GUI debug chips drive this — bitmask of DBG_* categories to
      // enable. Default 0 silences every [DISPATCH] line.
      g_dbgFlags = (uint32_t)(hdr["flags"] | 0);
      Serial.printf("[DBG] flags=0x%02X\n", (unsigned)g_dbgFlags);
      Serial.println("{\"type\":\"ACK\",\"ok\":true}");

    } else if (strcmp(type,"GET_WCB_SEQ")==0) {
      // Stored-sequence key list for ONE board, for the config tool's command
      // library — so a "Run Sequence" action offers the sequences that board
      // actually holds instead of a free-text key box. NAMES ONLY: the library
      // can fetch a sequence body too, but one at a time and with no ceiling on
      // the set, which is the failure mode the WCB's own config pull already has.
      // The reply is ASYNC (a WCB_SEQ line when the board answers, or an ok:false
      // one on timeout) — this path is Core 1, so the pull is issued directly.
      const int seqB = hdr["wcb"] | 0;   // clamp: a wild value must reject, not wrap into a real board
      rcTelemetry::startSeqPull((seqB >= 1 && seqB <= 255) ? (uint8_t)seqB : 0, 0,
                                rcTelemetry::SEQ_PULL_NAMES);   // sender 0 = answer on USB

    } else if (strcmp(type,"GET_WCB_SEQVAL")==0) {
      // ONE stored sequence's contents, by key — what the command library shows
      // under a chosen sequence. Deliberately one key at a time: a single value
      // is bounded (~1800 chars) but the whole set is not, and pulling every body
      // is the failure mode the WCB's own config pull already has.
      // Reply is ASYNC (a WCB_SEQVAL line, or an ok:false one on timeout).
      const int seqVB = hdr["wcb"] | 0;
      rcTelemetry::startSeqPull((seqVB >= 1 && seqVB <= 255) ? (uint8_t)seqVB : 0, 0,
                                rcTelemetry::SEQ_PULL_VALUE, hdr["key"] | "");

    } else if (strcmp(type,"GET_WCB_STATUS")==0) {
      // Lightweight liveness poll for the GUI's sidebar WCB Status panel.
      // Reads wcb->isOnline(i) for the floor (1..quantity) PLUS any auto-
      // discovered boards above it (up to the highest known); known[] marks
      // which slots the tool should actually render.
      int q = rcConfig.wcbNetwork.quantity;
      if (q < 0) q = 0;
      if (q > WCB_MAX_BOARDS) q = WCB_MAX_BOARDS;
      // Self is by definition online — wcb->isOnline() tracks REMOTE
      // peer heartbeats via ETM and never returns true for our own
      // deviceId, so we force "1" for the local board.
      int selfId = rcConfig.wcbNetwork.deviceId;
      int hi = rcTelemetry::wcbHighestKnown(q);
      Serial.printf("{\"type\":\"WCB_STATUS\",\"quantity\":%d,\"self\":%d,\"online\":[",
                    q, selfId);
      for (int i = 1; i <= hi; i++) {
        const WCBNeighbor* nb = wcb ? wcb->getNeighbor(i) : nullptr;
        const bool client = nb ? nb->isClient : rcTelemetry::wcbIsClient(i);
        // "live now": heartbeat for a WCB, a fresh WDP advert for a CLIENT.
        bool up = (i == selfId) ? true
                : (client ? (nb != nullptr) : (wcb && wcb->isOnline(i)));
        Serial.printf("%s%s", (i > 1) ? "," : "", up ? "1" : "0");
        // Lazily learn each online board's friendly alias via the SHARED
        // bounded fallback: a board advertising its name over WDP is already
        // cached (onWcbNeighbor) and never queried; a nameless one is asked
        // only ALIAS_MAX_TRIES times per online session, not every poll. This
        // is the same cap the bridged status build uses — previously the
        // direct-USB poll was uncapped and pestered an un-aliased board (~3s).
        if (i != selfId) rcTelemetry::maybeQueryAlias(i, up, client);
      }
      Serial.print("],\"known\":[");
      for (int i = 1; i <= hi; i++)
        Serial.printf("%s%s", (i > 1) ? "," : "", rcTelemetry::wcbBoardKnown(i, q) ? "1" : "0");
      // clients[] — 1 = client device (mesh monitor / other controller), not a
      // WCB board. Live neighbor if present, else the cache (so a learned client
      // still tags after its advert ages out).
      Serial.print("],\"clients\":[");
      for (int i = 1; i <= hi; i++) {
        const WCBNeighbor* nb = wcb ? wcb->getNeighbor(i) : nullptr;
        bool client = nb ? nb->isClient : rcTelemetry::wcbIsClient(i);
        Serial.printf("%s%s", (i > 1) ? "," : "", client ? "1" : "0");
      }
      // temporary[] — 1 = this board advertised the WDP "temporary" flag (a mgmt
      // relay etc.). LIVE-neighbor-only, NO cache fallback (unlike clients[]): a temp
      // peer is never learned/persisted, so once its advert ages out getNeighbor()
      // returns null, this reads 0, and the board stops being "known" and drops off.
      // Lets the tool tag it "· temp" and hide the ✕ Forget button (nothing to forget).
      Serial.print("],\"temporary\":[");
      for (int i = 1; i <= hi; i++) {
        const WCBNeighbor* nb = wcb ? wcb->getNeighbor(i) : nullptr;
        Serial.printf("%s%s", (i > 1) ? "," : "", (nb && nb->temporary) ? "1" : "0");
      }
      // Friendly names (from the ?WHOAMI replies); "" until a board answers
      // (a board must have ?SPECIAL,ON to unicast its reply back to us).
      Serial.print("],\"aliases\":[");
      for (int i = 1; i <= hi; i++)
        Serial.printf("%s\"%s\"", (i > 1) ? "," : "",
                      (i == selfId) ? "" : rcTelemetry::wcbAlias(i));
      // Per-serial-port device labels each board advertises over WDP (the
      // WCB_Client library decodes them into WCBNeighbor.portLabels[5], filled
      // from a device's own @WDP1 announce or a user-set label). One 5-element
      // array (ports 1-5) per board; "" = unlabeled. USB path only — the
      // Via-WCB bridge's 252-byte frame can't carry this, so the tool falls
      // back to plain "Serial <n>" when bridged.
      Serial.print("],\"portLabels\":[");
      for (int i = 1; i <= hi; i++) {
        const WCBNeighbor* nb = wcb ? wcb->getNeighbor(i) : nullptr;
        Serial.printf("%s[", (i > 1) ? "," : "");
        for (int p = 0; p < 5; p++) {
          // Port labels are EXTERNAL input — a device's own @WDP1 announce, copied
          // verbatim into portLabels[] by WCB_Client with no filtering. Sanitize the
          // same JSON-hostile chars setWcbAlias() strips for aliases; otherwise a '"'
          // or '\' would break this hand-built JSON and drop the whole WCB_STATUS
          // line at the tool's JSON.parse (silently freezing the status panel).
          const char* lbl = (nb && !nb->isClient) ? nb->portLabels[p] : "";
          char safe[25]; size_t j = 0;
          for (int k = 0; k < 24 && lbl[k]; k++) {
            char c = lbl[k];
            if (c == '"' || c == '\\' || (unsigned char)c < 0x20) continue;   // JSON-hostile
            safe[j++] = c;
          }
          safe[j] = '\0';
          Serial.printf("%s\"%s\"", p ? "," : "", safe);
        }
        Serial.print("]");
      }
      // Per-board stored-sequence fingerprint (WDP SEQHASH, WCBNeighbor::seqHash)
      // — covers sequence NAMES and CONTENTS, so it moves on any save, rename,
      // edit or erase. The tool caches a board's key list (GET_WCB_SEQ) against
      // this and re-pulls only when it changes. USB path only, mirroring
      // portLabels above; the bridge gets it in the fragmented WCB_META instead.
      // 0 = not yet known — NOT "no sequences" (an empty inventory hashes non-zero).
      Serial.print("],\"seqHash\":[");
      for (int i = 1; i <= hi; i++) {
        const WCBNeighbor* nb = wcb ? wcb->getNeighbor(i) : nullptr;
        Serial.printf("%s%u", (i > 1) ? "," : "",
                      (unsigned)((nb && !nb->isClient) ? nb->seqHash : 0));
      }
      Serial.println("]}");

    } else if (strcmp(type,"RESET_MESH_STATS")==0) {
      // Deferred to loop() — see g_meshStatsResetPending. ACK now; the reset
      // itself lands within a loop pass, and the tool re-reads afterwards.
      g_meshStatsResetPending = true;
      Serial.println("{\"type\":\"ACK\",\"of\":\"RESET_MESH_STATS\",\"ok\":true}");

    } else if (strcmp(type,"GET_MESH_STATS")==0) {
      // ESP-NOW delivery counters for the tool's Mesh Stats panel.
      // OUT: WCB_Client's own per-peer/aggregate counters (what this board
      //      SENT and what came back). IN: g_meshRxFrom/g_meshRxCount, which
      //      the library does not track — see onWCBCommand.
      // All RAM-only: a reboot zeroes them, deliberately (session health, not
      // lifetime history), so there is nothing to load or persist here.
      //
      // Built by rcTelemetry::buildMeshStatsPage — the SAME builder the
      // bridged reply uses, so the two payload shapes cannot drift. USB has
      // no packet budget, so a full roster normally arrives in one line; but
      // the builder still pages if the roster plus its counters overrun the
      // buffer, and a page it had to cut carries NO "last":1 — which the
      // tool's merge refuses to promote, silently freezing the panel. So
      // drain the pages the way tick() does instead of assuming one. Each
      // continuation writes at least one row, so nextPeer always advances;
      // the page cap is a backstop, not an expected exit.
      //
      // No "sys":1 here: that marker exists so the WCB Wizard can mute tool
      // chatter when it shares the BRIDGE board's port, and a direct-USB
      // reply has no Wizard to mute it for — matching WCB_STATUS above.
      //
      // 20 boards x up to 71 B (the builder's own row buffer) + the aggregate
      // can exceed 900 once the counters reach 5-6 digits; static so a buffer
      // this size never lands on the loop task's stack.
      static char msbuf[900];
      int msNext = 0, msPage = 0, msStart = 1;
      do {
        rcTelemetry::buildMeshStatsPage(msbuf, sizeof(msbuf), sizeof(msbuf) - 1,
                                        msPage++, msStart, &msNext,
                                        /*includeSys=*/false);
        Serial.println(msbuf);
        msStart = msNext;
      } while (msNext && msPage <= WCB_MAX_BOARDS);

    } else {
      Serial.println("{\"type\":\"ERROR\",\"msg\":\"unknown type\"}");
    }

  } else if (line[0] == '#') {
    // ── CLI commands ─────────────────────────────────────────────────────
    // Dispatched through execCliLine (shared with the remote terminal).
    execCliLine(line);
  }
  return false;   // handled without needing a yield — caller may keep draining
}

void handleSerialInput() {
  // `> 0`, NOT truthiness. HWCDC::available() returns -1 (not 0) on a null RX
  // queue — unlike HardwareSerial, which returns 0. A bare truthiness test makes
  // -1 true, read() also returns -1, and (char)-1 is 0xFF, which is never '\n' or
  // '\r' — so this loop would spin forever appending 0xFF, silently. There is no
  // watchdog on the loop task to break out of it, so the board would hang with no
  // reset and no output at all.
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      serialInputBuf.trim();
      if (serialInputBuf.length() == 0) { serialInputBuf = ""; return; }
      // Framing is this function's job; the dispatch is processInputLine's. Pass the
      // buffer by reference and clear it AFTER — copying it would cost a ~98 KB
      // duplicate on a large SET_CONFIG.
      const bool yieldNow = processInputLine(serialInputBuf);
      serialInputBuf = "";
      if (yieldNow) return;
    } else {
      // Must be able to hold a full SET_CONFIG payload. A config with
      // double/triple-tap mappings is tens of KB; the old 8 KB cap silently
      // truncated large configs mid-JSON, so deserializeJson() (bigDoc, 98304
      // in the SET_CONFIG handler above) failed with "parse failed" and the
      // ENTIRE save was rejected — nothing persisted and reload reverted every
      // edit. Keep this in sync with that bigDoc capacity.
      if (serialInputBuf.length() < 98304) serialInputBuf += c;
    }
  }
}

// =============================================================================
//  SETUP
// =============================================================================
// ── Cold-boot auto-recovery (boot guard) ───────────────────────────────────
// The custom short-watchdog bootloader (RTC WDT 9000 → 3000 ms) auto-resets a
// board that stalls in the pre-app boot window — but IDF disables that RTC WDT
// right before setup() runs, so a stall INSIDE setup() (PSRAM alloc, WiFi/USB
// bring-up current spike on a cold rail) would otherwise sit dark until someone
// presses reset. This one-shot esp_timer fires if setup() hasn't finished
// within BOOT_GUARD_TIMEOUT_MS and restarts the board, so a cold boot
// auto-retries. The callback runs in the esp_timer task — independent of the
// loop task running setup() — so a hung setup() can't stop it. Cancelled at the
// end of a healthy setup(). The custom short-WDT bootloader and THIS guard are a
// matched pair: never run that bootloader on a board without this guard.
#define BOOT_GUARD_TIMEOUT_MS 15000
static esp_timer_handle_t _bootGuardTimer = nullptr;
static void _bootGuardFired(void*) { ESP.restart(); }

static void bootGuardArm() {
  esp_timer_create_args_t args = {};
  args.callback        = &_bootGuardFired;
  args.arg             = nullptr;
  args.dispatch_method = ESP_TIMER_TASK;
  args.name            = "bootguard";
  if (esp_timer_create(&args, &_bootGuardTimer) == ESP_OK) {
    esp_timer_start_once(_bootGuardTimer, (uint64_t)BOOT_GUARD_TIMEOUT_MS * 1000ULL);
  }
}

static void bootGuardDisarm() {
  if (_bootGuardTimer) {
    esp_timer_stop(_bootGuardTimer);
    esp_timer_delete(_bootGuardTimer);
    _bootGuardTimer = nullptr;
  }
}

// Report at boot which 2nd-stage bootloader is on the board (stock vs the custom
// short-WDT one). Reads the esp_bootloader_desc_t in IDF 5.2+ bootloader images —
// no flash dump. The custom bootloaders are identified by build timestamp; if
// either is rebuilt, add its new date here (read it from this very banner).
static void printBootloaderInfo() {
  static const char *CUSTOM_BOOT_DATES[] = {
    "Jun  8 2026 16:02:21",   // WCB_S3_custom_bootloader_16MB_wdt3s.bin
    "Jun 10 2026 14:36:20",   // WCB_S3_custom_bootloader_8MB_wdt3s.bin
  };
  esp_bootloader_desc_t desc;
  if (esp_ota_get_bootloader_description(NULL, &desc) == ESP_OK) {
    bool custom = false;
    for (size_t i = 0; i < sizeof(CUSTOM_BOOT_DATES) / sizeof(CUSTOM_BOOT_DATES[0]); i++)
      if (strncmp(desc.date_time, CUSTOM_BOOT_DATES[i], sizeof(desc.date_time)) == 0) { custom = true; break; }
    if (custom)
      Serial.printf("Bootloader: CUSTOM short-WDT (cold-boot auto-retry) — built %s\n", desc.date_time);
    else
      Serial.printf("Bootloader: stock (IDF %s, built %s)\n", desc.idf_ver, desc.date_time);
  } else {
    Serial.println("Bootloader: unknown (no description block)");
  }
}

// ── Boot telemetry ──────────────────────────────────────────────────────────
// Boot-attempt counter in RTC noinit RAM: survives watchdog/software/panic
// resets (and usually the reset button); garbage only after true power loss —
// the magic word detects that and restarts the count. After a "dark board"
// episode this tells you whether the chip had been reset-looping through the
// app (count climbing), brown-outing (RTC code 15), or never reached app code
// at all (count restarts at 1).
#define BOOT_MAGIC 0xB007C0DEUL
RTC_NOINIT_ATTR static uint32_t g_bootMagic;
RTC_NOINIT_ATTR static uint32_t g_bootAttempts;

static void printBootTelemetry() {
  esp_reset_reason_t r = esp_reset_reason();
  const char *name = "other";
  switch (r) {
    case ESP_RST_POWERON:  name = "Power-on / EN reset"; break;
    case ESP_RST_SW:       name = "Software restart (incl. boot-guard retry)"; break;
    case ESP_RST_PANIC:    name = "Crash (panic)"; break;
    case ESP_RST_INT_WDT:  name = "Interrupt watchdog"; break;
    case ESP_RST_TASK_WDT: name = "Task watchdog"; break;
    case ESP_RST_WDT:      name = "RTC watchdog (short-WDT bootloader fired)"; break;
    case ESP_RST_BROWNOUT: name = "BROWNOUT — supply rail sagged"; break;
    // The S3 has no bridge chip: the USB Serial/JTAG peripheral itself resets
    // the chip when the host drives the CDC control lines, which is how esptool
    // reboots one with nothing but a USB cable. A host merely OPENING the port
    // can do it — Chrome asserts DTR/RTS as part of open() and the order they
    // land in produces the reset edge — so this is the expected reason after
    // connecting the config tool, and is NOT a fault. See TROUBLESHOOTING.
    case ESP_RST_USB:      name = "USB peripheral (host toggled DTR/RTS — e.g. a tool opening the port)"; break;
    case ESP_RST_JTAG:     name = "JTAG"; break;
    default: break;
  }
  // Low-level per-core causes (rom/rtc.h). Named for the ones that actually turn
  // up, because a bare number sends you looking through the TRM mid-diagnosis.
  auto rtcName = [](int c) -> const char* {
    switch (c) {
      case 1:  return "power-on";
      case 3:  return "SW system";
      case 7:  return "MWDT0 (task/int watchdog)";  // TG0 main WDT — NOT the RTC-WDT
      case 9:  return "RTC-WDT system";
      case 15: return "brown-out";
      case 16: return "RTC-WDT (short-WDT bootloader fired — auto-retry)";
      case 21: return "USB UART chip reset";   // host opened/closed the CDC port
      case 22: return "USB JTAG chip reset";
      default: return "see rom/rtc.h";
    }
  };
  const int rc0 = (int)rtc_get_reset_reason(0), rc1 = (int)rtc_get_reset_reason(1);
  Serial.printf("Reset reason: %d - %s  (RTC codes core0=%d [%s] core1=%d [%s])\n",
                (int)r, name, rc0, rtcName(rc0), rc1, rtcName(rc1));
  if (g_bootMagic != BOOT_MAGIC) {          // true power loss → fresh count
    g_bootMagic = BOOT_MAGIC;
    g_bootAttempts = 0;
  }
  g_bootAttempts++;
  Serial.printf("Boot attempts since power applied: %lu%s\n",
                (unsigned long)g_bootAttempts,
                g_bootAttempts > 1 ? "   <-- board retried/reset before this boot" : "");
}

// Push what's attached to this NaviCore into the WDP advert, so the WCBs + the
// Wizard see it exactly as they see another WCB's. Two things ride along:
//
//   • Port labels — the three aux headers advertise under the MESH port numbers
//     that address them (WDP 1/2/3 = firmware S3/S4/S5 = the v2 silkscreen's
//     "Serial 1/2/3"), so a WCB printing "S2  HCR" names the same port an
//     operator would type in `;w20,;s2<cmd>`. The local Maestro takes WDP port 4:
//     visible in ?WDP,LIST, but not a port `;s<n>` can reach (it's a binary bus).
//     WDP port 5 is unused and explicitly cleared.
//   • Local Maestro IDs + bus baud — the same MAESTRO/MAESTRO_CFG TLVs a WCB
//     emits, so every board knows which Maestros live here.
//
// Called at boot (after setIdentity) and after every SET_CONFIG apply. WCB_Client
// dedupes unchanged values and re-broadcasts changed ones promptly.
void rcAdvertiseSerialLabels() {
  if (!wcb) return;
  for (int s = 0; s < RC_NUM_SLBL; s++)
    wcb->setPortLabel((uint8_t)rcWdpPortForLabel(s), rcSerialLabel(s));
  wcb->setPortLabel(5, "");                 // NaviCore has no 5th labelable port

  // Every LOCAL Maestro (type 1 = wired to Serial2), advertised by its POLOLU DEVICE
  // NUMBER — that is what the mesh means by a Maestro id. On a WCB the id in ;M<id>
  // goes straight onto the wire as the Pololu device byte, and it is a MULTICAST
  // address: every Maestro carrying that device number, on any port of any board,
  // runs the command. Several boards legitimately host the same device number, and
  // that is the point — so NaviCore joins that namespace by device, NOT by its own
  // slot index (which is a purely local handle).
  //
  // Deduped: two type-1 slots sharing a device number are, on NaviCore's single
  // Serial2 bus, necessarily the SAME physical Maestro — advertise it once. All local
  // Maestros share that one bus, hence the one baud.
  uint8_t  mIds[RC_NUM_MAESTROS];
  uint32_t mBauds[RC_NUM_MAESTROS];
  uint8_t  mCount = 0;
  for (int i = 0; i < RC_NUM_MAESTROS; i++) {
    if (rcConfig.maestros[i].type != 1) continue;
    const uint8_t dev = rcConfig.maestros[i].device;
    bool dup = false;
    for (uint8_t k = 0; k < mCount; k++) if (mIds[k] == dev) { dup = true; break; }
    if (dup) continue;
    mIds[mCount]   = dev;
    mBauds[mCount] = rcConfig.maestroBaud;
    mCount++;
  }
  wcb->setMaestroIds(mIds, mCount, mBauds);
}

void setup() {
  // OTA brick-loop guard — MUST be the very first thing. A freshly-OTA'd image
  // boots in PENDING_VERIFY because app-rollback is enabled in the Arduino esp32
  // sdkconfig. If anything later in boot (PSRAM/WiFi/WCB bring-up, a crash, or
  // the custom bootloader's short watchdog) resets the board before the image is
  // marked valid, the bootloader rolls it back / retry-loops it — the exact
  // "reboots forever after an OTA" symptom. Mark the running app valid NOW so
  // that can't happen. Harmless on a normal boot: returns an ignorable error
  // (ESP_ERR_OTA_ROLLBACK_INVALID_STATE) when the image is already valid.
  esp_ota_mark_app_valid_cancel_rollback();

  // Arm the boot guard FIRST so it covers all of setup() (PSRAM/WiFi/USB
  // bring-up). Disarmed at the very end once the board is confirmed healthy.
  bootGuardArm();

  // Hold the local Maestro command line (Serial2 TX) idle-HIGH from the very first
  // instant of boot. Until Serial2.begin() runs (~2 s from here — after the delay
  // below + config load) the pin would otherwise float, and a floating/noisy
  // command wire can feed the Maestro serial garbage it misreads as Set-Target /
  // script commands — servos twitch with NO SBUS connected. A UART line idles HIGH,
  // so driving it high = the Maestro sees "no data". MAESTRO_TX_PIN is 6 on both
  // board profiles, so this is correct before applyBoardProfile() runs.
  pinMode(MAESTRO_TX_PIN, OUTPUT);
  digitalWrite(MAESTRO_TX_PIN, HIGH);

  // Bump the USB-CDC RX buffer well above the 256-byte default. The SBUS OUT
  // byte-streaming tee blocks the main loop for ~2.75 ms per SBUS frame while
  // bit-banging SoftwareSerial; during that window the host can shove ~1-2 KB
  // into us at USB-CDC speed. A 4 KB buffer comfortably absorbs the worst case
  // (e.g. a 3-4 KB SET_CONFIG payload arriving in one shot) without dropping
  // bytes that would otherwise corrupt the JSON. Must be set BEFORE begin().
  // A relayed OTA raises the worst case again: the sender streams a window of 8
  // ?OTA,DATA lines of ~284 B (2272 B) back-to-back. That fits 4 KB only if very
  // little else is in flight, and an overflow here does not merely LOSE a line —
  // it drops a run of bytes from the MIDDLE of one, which still decodes as valid
  // base64 and corrupts the image silently (see the crc32 check in navicore_ota.h).
  // 8 KB keeps a full window plus a config push in flight simultaneously.
  Serial.setRxBufferSize(8192);
  // TX ring buffer — sized to hold an entire CONFIG response (rcConfigToJSON
  // can produce 2-4 KB depending on how populated the config is) in one
  // print() so the host has a full window to drain it before any byte gets
  // dropped.  Previously 1024 — too small; a fast Serial.println of the full
  // config overflowed mid-stream and the corrupt JSON appeared at the
  // browser as a truncated line (e.g. "matrixChannel" → "matrixCel" gap).
  // 8 KB is comfortable on the S3's 512 KB SRAM.  Must be set BEFORE begin().
  Serial.setTxBufferSize(8192);
  Serial.begin(115200);
  // ── ESP32-S3 USB-CDC TX-blocking guard (only when Serial = HWCDC) ──
  // If the board variant is built with ARDUINO_USB_CDC_ON_BOOT=1, Serial is
  // the native-USB HWCDC class.  Default tx timeout is ~100 ms which can
  // make the firmware appear frozen if no host ever attaches — but the
  // OTHER extreme (timeout = 0) drops bytes immediately when the host is
  // just briefly slow to drain, which mangles the CONFIG response.
  //
  // 50 ms is a deliberate middle ground:
  //   • host present + reading at any reasonable speed → buffer drains
  //     within microseconds, the 50 ms ceiling never gets hit, no drops
  //   • host absent / disappeared mid-write → writes give up after 50 ms
  //     instead of blocking the loop indefinitely
  //   • combined with the 8 KB tx buffer above, a full config print fits
  //     entirely in the buffer before any wait is needed
  //
  // When CDC-on-boot is DISABLED, Serial is HardwareSerial (UART0 through
  // a USB-to-serial bridge) and setTxTimeoutMs doesn't exist — the boot
  // latch in that case is the bridge chip's DTR/RTS autoreset circuit,
  // which is a hardware issue software can't directly suppress.
#if ARDUINO_USB_CDC_ON_BOOT
  Serial.setTxTimeoutMs(50);
#endif
  delay(1500);
  Serial.println("\n\n=== NaviCore ===");   // active board profile logged at boot by applyBoardProfile()
  printBootloaderInfo();
  printBootTelemetry();

  // Status LED
  statusLed.begin();
  setStatusLed(C_RED, 255);

  // SBUS IN, Serial2 (local Maestro), and Serial3/4 (aux SWSerial) are ALL
  // opened AFTER the config loads (below): their pins come from the selected
  // board profile (applyBoardProfile), which needs rcConfig.boardType, and the
  // Maestro/aux baud comes from rcConfig too. Nothing uses these ports before
  // setup() finishes, so deferring the open is safe.

  // RC Config lives in PSRAM — allocate it BEFORE any rcConfig.* access.
  // ps_calloc() pulls from the PSRAM heap (Tools → PSRAM must be "OPI PSRAM").
  // If PSRAM is unavailable the ~210 KB struct can't fit internal RAM, so halt
  // with a clear message instead of crashing cryptically on first access.
  g_rcConfig = (RcConfig*) ps_calloc(1, sizeof(RcConfig));
  if (!g_rcConfig) {
    Serial.println("\n[FATAL] PSRAM allocation for rcConfig failed.");
    Serial.println("        Set Arduino IDE: Tools -> PSRAM -> \"OPI PSRAM\", then reflash.");
    setStatusLed(C_RED, 255);
    while (true) { delay(1000); }
  }
  Serial.printf("[MEM] rcConfig (%u bytes) allocated in PSRAM, free PSRAM now %u\n",
                (unsigned)sizeof(RcConfig), (unsigned)ESP.getFreePsram());

  // RC Config: defaults then NVS overrides. Must run BEFORE constructing
  // WCB_Client so the network credentials come from rcConfig.wcbNetwork.
  rcConfigLoadDefaults();
  // Config persistence: LittleFS (/config.json) is primary; the legacy NVS store
  // is a one-time migration source. LittleFS removes the NVS 4000-byte-per-value
  // limit that silently dropped densely-mapped modes.
  rcConfigBeginLFS();
  // Clear any bulk-transfer staging file left by a reboot mid-push — its
  // pre-extended zeros would otherwise waste flash until the next transfer.
  // (bulkBegin also truncates it, so this only reclaims space early.)
  if (g_lfsReady) { LittleFS.remove(RC_CMDLIB_BULK_TMP); LittleFS.remove(RC_CMDLIB_SEND_TMP); }
  if (!rcConfigLoadLFS()) {
    if (g_lfsReady && LittleFS.exists(RC_CFG_PATH)) {
      // The file EXISTS but didn't load (parse error / transient low memory).
      // Do NOT migrate-and-save here — that would overwrite a good config with
      // defaults. Keep the file and run on defaults this boot; retry next boot.
      Serial.println("[CONFIG] /config.json present but unreadable — kept; running on defaults this boot");
    } else {
      // No file yet — fresh board, or first boot after the LittleFS upgrade.
      // Load the legacy NVS config and migrate it forward (once) so the next
      // boot reads LittleFS.
      rcConfigLoadNVS();
      if (g_lfsReady && rcConfigSaveLFS())
        Serial.println("[CONFIG] migrated existing config: NVS -> LittleFS");
    }
  }

  // Clip library filesystem — mount the dedicated 12 MB `clips` partition as a
  // SECOND LittleFS (own label + base path, so its format never touches config).
  // Absent on a board still running the old 4 MB partition table (pre-migration
  // full flash) — record/replay just runs in-RAM-only until then.
  g_clipsReady = clipsFS.begin(true, "/clips", 5, "clips");
  if (g_clipsReady) {
    navirec::setClipFS(&clipsFS);   // record/replay can now save/load named clips
    Serial.printf("[CLIPS] mounted: %u KB free of %u KB\n",
                  (unsigned)(clipsFS.totalBytes() - clipsFS.usedBytes()) / 1024,
                  (unsigned)clipsFS.totalBytes() / 1024);
  } else {
    Serial.println("[CLIPS] no `clips` partition — clips are in-RAM only (flash the 16 MB partition build)");
  }

  // Pin map is known now — load the selected board's profile into the pin
  // globals BEFORE opening any port (SBUS / Maestro / aux all read these pins).
  applyBoardProfile();

  // Bind the aux command ports to their backing objects for THIS board — their
  // type (hardware UART0 vs SoftwareSerial) is board-dependent (see the Stream*
  // declarations above). Done before applySerialBauds() opens them.
  if (sbusSharedUart) {            // shared SBUS frees UART0 → S3 = hardware UART0, S4 = SoftwareSerial
    s3 = &Serial0; s3IsHw = true;
    s4 = &swAux0;
    // Both boards now expose a third aux port (S5): NaviCore v2 on GPIO38/47, WCB 3.2 on
    // the freed Serial5 header GPIO9/10. swAux1 is the spare SoftwareSerial (s3 is hardware
    // UART0 in shared mode), so it backs S5 on both.
    s5 = &swAux1;
  } else {                         // dedicated SBUS-out on UART0 (fallback layout, not used by
    s3 = &swAux0; s3IsHw = false;  //   either current board): S3/S4 SoftwareSerial, no hardware aux.
    s4 = &swAux1;
    s5 = nullptr;
  }

  // SBUS IN (and, in shared mode, OUT) on Serial1/UART1 — brought up here, after
  // applyBoardProfile(), so it uses the selected board's SBUS pins + layout.
  if (sbusSharedUart) {
    // v2: SBUS IN + OUT share ONE full-duplex hardware UART (UART1 / Serial1):
    //   RX = SBUS_RX_PIN, TX = SBUS_OUT_PIN, 100k 8E2 INVERTED. The byte-tee in
    // SbusReader::read() writes each RX byte straight to TX; the 256-byte
    // non-blocking TX buffer keeps that write from stalling the RX-drain loop.
    // This is why UART0 is free here — it's the hardware S3 aux, not SBUS-out.
    Serial1.setTxBufferSize(256);                       // MUST precede begin()
    sbusRx.begin(&Serial1, SBUS_RX_PIN, /*txPin=*/SBUS_OUT_PIN);
    Serial.printf("[SBUS] IN+OUT share Serial1/UART1 — RX GPIO%d / TX GPIO%d, 100k 8E2 inverted. UART0 = hardware S3.\n",
                  SBUS_RX_PIN, SBUS_OUT_PIN);
  } else {
    // 3.2: SBUS IN on Serial1 (UART1), RX-only (txPin=-1). SBUS OUT is a DEDICATED
    // hardware UART (UART0 / Serial0), TX-only, brought up on demand by
    // applySbusOut() per rcConfig.sbusOutEnabled — so UART0 stays free when OUT is
    // off. UART0 is available because the debug console is on native USB CDC.
    sbusRx.begin(&Serial1, SBUS_RX_PIN, /*txPin=*/-1);
    Serial.printf("[SBUS] IN  on Serial1/UART1 RX (GPIO%d)\n", SBUS_RX_PIN);
  }

  // Open Serial2 (local Maestro) + Serial3/4 (aux SWSerial) at their
  // configured baud. Single source of truth: rcConfig (maestroBaud /
  // auxBaud[]). Same helper runs again after every SET_CONFIG save so a
  // baud change in the config tool applies live — no reboot needed.
  // Keep aux baud ≤ ~57600 (higher rates choke bit-banged SWSerial on
  // ESP32-S3). One port = one device = one baud.
  applySerialBauds(true);
  applySbusOut(/*initial=*/true);     // bring up SBUS OUT only if rcConfig.sbusOutEnabled

  // Initialize rc_telemetry's deferred-queue mutex BEFORE WCB_Client
  // brings the ESP-NOW receive callback online.  Otherwise the very
  // first inbound packet's handle() call would find _pendingMutex==null
  // and skip its critical section.  See rc_telemetry.h::init() for the
  // race details.
  rcTelemetry::init();

  // Record/replay — alloc the PSRAM clip buffer + the cross-core capture queue
  // BEFORE the WCB receive callback (onCommand) is registered, so a remote
  // ESP-NOW TRIGGER on Core 0 can't hit a null queue. 24000 events (RecEvent=136 B)
  // ≈ 3.1 MB PSRAM — sized for the 60 s REC_MAX_MS cap even with a few passthrough
  // servos moving continuously (~400 keyframes/s worst case). ~6.7 MB PSRAM was free.
  navirec::recBegin(24000, recCbDispatch, recCbEmitMaestro, recCbEmitHcrVol, recCbResetChan);

  // ── Optional SoftAP (rcConfig.wifiEnabled) ────────────────────────────────
  // ORDER IS LOAD-BEARING: this must run BEFORE wcb->begin(). WCB_Client checks
  // WiFi.getMode() at entry and, finding an AP already up, selects WIFI_AP_STA and
  // KEEPS it instead of forcing WIFI_STA and tearing the AP down (WCB_Client.cpp
  // ≈89-110). Raise the AP after begin() and the AP survives, but ESP-NOW has
  // already pinned the radio — the two then fight over the channel.
  //
  // CHANNEL: softAP()'s 3rd parameter defaults to 1. It MUST be passed
  // rcConfig.wcbNetwork.channel explicitly. Once an AP owns the radio WCB_Client
  // stops calling esp_wifi_set_channel and only WARNS on a mismatch (≈121-130), so
  // a defaulted channel 1 against a mesh on any other channel is a silent, total
  // blackout — every packet dropped, no fault LED, one line in a log nobody reads.
  //
  // PASSWORD: fail CLOSED. softAP() with an empty/short passphrase happily creates
  // an OPEN network, and this command surface has no per-command auth at all —
  // RESET_DEFAULTS and REBOOT dispatch on a bare "type" string — so an open AP is an
  // unauthenticated command channel to the whole mesh. Refuse and say why.
  if (rcConfig.wifiEnabled) {
    const size_t pwLen = strlen(rcConfig.wifiPassword);
    if (pwLen > 0 && pwLen < 8) {
      Serial.printf("[WIFI] REFUSED: password is %u character(s); WPA2 requires 8.\n", (unsigned)pwLen);
      Serial.println("[WIFI] Not starting an open AP — set a longer password and reboot.");
    } else if (pwLen == 0) {
      Serial.println("[WIFI] REFUSED: no AP password set. Refusing to start an OPEN access point —");
      Serial.println("[WIFI] this board accepts REBOOT / RESET_DEFAULTS / SET_CONFIG with no credential.");
    } else {
      // Empty SSID → derive one, so a droid always has a distinguishable name.
      char ssid[33];
      if (rcConfig.wifiSsid[0]) strlcpy(ssid, rcConfig.wifiSsid, sizeof(ssid));
      else snprintf(ssid, sizeof(ssid), "NaviCore-%u", (unsigned)rcConfig.wcbNetwork.deviceId);
      const int ch = rcConfig.wcbNetwork.channel;
      // max_connection 4 (the default): this is a config channel, not a hotspot.
      if (WiFi.softAP(ssid, rcConfig.wifiPassword, ch, /*hidden=*/0, /*max_conn=*/4)) {
        Serial.printf("[WIFI] SoftAP \"%s\" up on channel %d — %s\n",
                      ssid, ch, WiFi.softAPIP().toString().c_str());
        Serial.println("[WIFI] ESP-NOW will share this channel (WIFI_AP_STA).");
        // Only now, with an AP actually up and an IP to print. begin() is
        // self-contained: if it fails it says so and leaves everything inert, so a
        // WebSocket that will not start can never take the droid down with it.
        naviws::begin();
      } else {
        // Not fatal to the droid — the mesh and serial both still work.
        Serial.printf("[WIFI] SoftAP \"%s\" FAILED to start on channel %d.\n", ssid, ch);
      }
    }
  }

  // WCB Client — sets STA mode + custom MAC + inits ESP-NOW. With the SoftAP above
  // enabled it runs WIFI_AP_STA and shares that AP's channel; with it off (the
  // default) this is ESP-NOW only, exactly as before. Credentials come from NVS
  // (editable via the GUI's "WCB Network" sidebar); a reboot is required
  // for credential changes to take effect.
  wcb = new WCB_Client(rcConfig.wcbNetwork.macOct2,
                      rcConfig.wcbNetwork.macOct3,
                      rcConfig.wcbNetwork.password,
                      rcConfig.wcbNetwork.quantity,
                      rcConfig.wcbNetwork.deviceId);
  // Pin the ESP-NOW radio to the mesh channel every WCB is on BEFORE begin() — the
  // ESP32 has one radio, so a mismatched channel makes this RC silently unreachable.
  wcb->setMeshChannel(rcConfig.wcbNetwork.channel);
  // An EMPTY mesh password is a silent, symptomless blackout. The password is a
  // plain-text namespace field on every packet (wcb_packet_etm_t::structPassword)
  // and the receive path strncmp's it before anything else (WCB_Client.cpp ≈2036),
  // so a mismatch means nothing is heard and nothing is received. begin() does NOT
  // validate it — it only range-checks device_id and brings up WiFi/ESP-NOW — so
  // this returns true, no fault is latched, and the orange LED never lights. Name
  // the exact field, loudly, because nothing downstream will.
  if (rcConfig.wcbNetwork.password[0] == '\0') {
    Serial.println("┌───────────────────────────────────────────────────────────────┐");
    Serial.println("│  WCB MESH PASSWORD IS EMPTY — this board is deaf AND mute.     │");
    Serial.println("│                                                               │");
    Serial.println("│  Every ESP-NOW packet carries the network password, and a      │");
    Serial.println("│  receiver drops any packet whose password doesn't match. Until │");
    Serial.println("│  it is set, no WCB hears this board and this board hears none. │");
    Serial.println("│                                                               │");
    Serial.println("│  Fix: config tool → WCB Network → Password. It must match the  │");
    Serial.println("│  WCBs' own password. A reboot is required to take effect.      │");
    Serial.println("└───────────────────────────────────────────────────────────────┘");
  }
  if (!wcb->begin()) {
    Serial.println("[WCB] ERROR: wcb->begin() failed — check WCB Network settings in GUI");
    // Latch the fault for the LED arbiter: updateStatusLed() (loop) displays a
    // latched fault color STEADY and skips the SBUS indication entirely, so
    // this isn't silently overwritten on the first loop() pass. Steady orange
    // = WCB fault; FLASHING orange = no SBUS — distinguishable at a glance.
    g_ledFaultColor = C_ORANGE;
    setStatusLed(C_ORANGE, 200);   // show immediately for the rest of setup()
  } else {
    wcbReady = true;   // ESP-NOW is up — wcb-> calls are now safe
    // Create the remote-TRIGGER queue BEFORE registering onCommand so a TRIGGER
    // arriving the instant the callback is live has a live queue to land in
    // (queueRemoteTrigger no-ops on a null queue, so a miss is only dropped, but
    // ordering it first avoids losing the very first remote trigger).
    remoteTriggerQueue = xQueueCreate(8, sizeof(RemoteTrigger));
    remoteCliQueue     = xQueueCreate(3, sizeof(RemoteCliMsg));  // relayed CLI lines → drainRemoteCli()
    forgetPeerQueue    = xQueueCreate(4, sizeof(uint8_t));       // FORGET_PEER (Via-WCB) → drainForgetPeer()
    serialFwdQueue     = xQueueCreate(4, sizeof(SerialFwdMsg));  // mesh→serial writes → drainSerialFwd()
    maestroCmdQueue    = xQueueCreate(6, sizeof(MaestroCmdMsg)); // inbound ;M → drainMaestroCmd()
    // Same rule for the OTA packet queue: create it here (Core 1) BEFORE the raw-
    // packet hook goes live, so the very first OTA frame can't be lost to the
    // create/publish race. Registering onRawPacket first left a window where the
    // Core-0 hook lazily created its own queue and this line then overwrote the
    // handle — leaking that queue and anything already in it.
    naviota::otaPktQueue = xQueueCreate(12, sizeof(naviota::OtaPktSlot));
    wcb->onCommand(onWCBCommand);   // queues must be live BEFORE the callback that feeds them
    wcb->onRawPacket(naviota::otaRawPacketHook);   // OTA control/data structs (55/243 B) over the mesh
    // Bulk command-library push (config tool → mesh → LittleFS). These fire on the
    // LOOP task from wcb->update(), so the streamed flash writes are safe.
    wcb->onBulkBegin(rcTelemetry::bulkBegin);
    wcb->onBulkChunk(rcTelemetry::bulkChunk);
    wcb->onBulkComplete(rcTelemetry::bulkComplete);
    // Stored-sequence inventory replies (GET_WCB_SEQ → WCB_SEQ). Registering the
    // callback is also what ARMS interception of the reply packets in WCB_Client —
    // without it they fall through to the raw-packet hook. Fires on the LOOP task
    // (from wcb->update()), so building the reply String there is safe.
    wcb->onSequenceNames(rcTelemetry::seqNamesReply);
    wcb->onSequenceValue(rcTelemetry::seqValueReply);   // ONE sequence's contents (GET_WCB_SEQVAL)
    // WDP device-identity advertising (WCB_Client 1.7.0 "WDP-DA") — announce this
    // NaviCore on the mesh so every WCB auto-discovers it (it appears in ?WDP,LIST
    // / the config tool with its firmware + board, no manual labeling). The advert
    // goes out as a boot burst then re-broadcasts periodically from wcb->update()
    // (already called every loop()). Rides the ETM broadcast layer (WCB default).
    wcb->setIdentity("NaviCore", FW_VERSION,
                     rcConfig.boardType == 0 ? "NaviCore v2" : "WCB 3.2",
                     "rc sbus maestro hcr");
    rcAdvertiseSerialLabels();   // advertise what's on each NaviCore serial port (WDP PORTLABEL) — config override + auto-derive
    // Auto-join (default ON, set explicitly): discover + keep WCB peers live from
    // their WDP adverts, so wcb_quantity is only the pre-registered floor and the
    // fleet is reachable without it covering every board.
    wcb->setAutoJoin(true);
    // New-peer event: onNeighbor enqueues, drainPeerEvents() (loop) fires the
    // action + alert. Arm the grace window so the boot-time fleet discovery is
    // recorded silently instead of firing for every board that's already present.
    // Create the queue + arm the grace window BEFORE registering onNeighbor so an
    // advert that arrives immediately can't hit a null queue / unarmed grace.
    peerEventQueue   = xQueueCreate(8, sizeof(uint8_t));
    g_peerGraceUntil = millis() + PEER_GRACE_MS;
    wcb->onNeighbor(onWcbNeighbor);
    // Board online/offline transitions → one console line each. Registered after
    // begin() so the boot-time burst of first heartbeats is reported as ONLINE
    // rather than lost. See onWcbStatus for the two-task/Core-0 constraint.
    wcb->onStatusChange(onWcbStatus);
    // Arm the one-shot boot roll call — names any configured board still unheard.
    g_rollCallAt = millis() + ROLL_CALL_MS;
    Serial.printf("[WCB] Joined network as device ID %d (quantity=%d)\n",
                  rcConfig.wcbNetwork.deviceId, rcConfig.wcbNetwork.quantity);
  }

  // Construct the broadcast Maestro stream AFTER wcb exists. WCBStream's
  // constructor self-registers with the WCB_Client singleton; doing this here
  // (rather than at global scope) is what guarantees wcb->update() actually
  // drives the stream's flush so remote Maestro bytes go out over ESP-NOW.
  maestroBroadcast = new WCBStream(/*target_wcb=*/0, /*target_port=*/0);

  // Clear state
  memset(pendingActions, 0, sizeof(pendingActions));
  memset(switchPrevPos, -1, sizeof(switchPrevPos));
  memset(switchCandPos, -1, sizeof(switchCandPos));   // -1 = no settle candidate pending
  serialInputBuf.reserve(256);

  setStatusLed(C_BLUE, 10);
  Serial.print  ("[NaviCore] Firmware ");
  Serial.print  (FW_VERSION);
  Serial.println(" — setup complete.");
  Serial.println("  Connect config_tool/index.html via Web Serial for configuration.");
  Serial.println("  CLI: #L01=info  #L09=SBUS dump  #L10=live  #L11=WCB status  #L12=RC state  #L13=SBUS raw hex");
  Serial.println("  CLI: #L20=HCR S3 test  #L21=HCR S4 test  (direct, bypasses config+mapping)");
  Serial.println("  Send PING to test. Send GET_CONFIG to read mappings.");

  // setup() completed — cancel the boot guard so a healthy board never trips it.
  bootGuardDisarm();
}

// =============================================================================
//  LOOP
// =============================================================================
// ── Status LED: SBUS-aware running indicator ────────────────────────────────
// Once the board is up, the LED reflects SBUS reception:
//   • frames arriving        → steady BLUE  (ready / receiving SBUS)
//   • no frames (no signal)  → slow-flash ORANGE
// Non-blocking: toggles on a millis() timer so loop()/SBUS dispatch never stall,
// and only writes the NeoPixel on a state change or flash edge (not every loop).
// Boot/fault colours (RED, WCB-init-fail ORANGE) are set in setup(); this takes
// over on the first loop(), so once running, FLASHING orange == "no SBUS".
#define SBUS_LED_TIMEOUT_MS 500   // no SBUS frame for this long ⇒ "no signal"
#define SBUS_LED_FLASH_MS   600   // slow-flash half-period (ORANGE on, then off)
#define STATUS_LED_BRIGHT   12    // running-indicator brightness (0-255). Kept low —
                                  // the NeoPixel sits right on the board; bump if you
                                  // want it more visible across a room.
static void updateStatusLed() {
  unsigned long now = millis();
  static int8_t        mode       = -1;   // -1 unset, 0 steady-blue, 1 flashing-orange, 2 latched-fault
  static bool          flashOn    = false;
  static unsigned long lastToggle = 0;

  // Latched fault outranks the routine SBUS indication (see g_ledFaultColor).
  // Shown STEADY — distinct from the FLASHING no-SBUS pattern below.
  if (g_ledFaultColor) {
    if (mode != 2) { setStatusLed(g_ledFaultColor, STATUS_LED_BRIGHT); mode = 2; }
    return;
  }

  // Brief non-latching cyan pulse when a NEW WCB peer is detected (peerAlert).
  // Ranks below a latched fault, above the routine SBUS indication.
  if (g_peerFlashUntil) {
    if ((int32_t)(now - g_peerFlashUntil) < 0) {
      if (mode != 3) { setStatusLed(C_CYAN, STATUS_LED_BRIGHT); mode = 3; }
      return;
    }
    g_peerFlashUntil = 0; mode = -1;   // pulse ended — force a clean repaint of the SBUS state below
  }

  bool sbusAlive = (sbusLastFrameMs != 0) && (now - sbusLastFrameMs < SBUS_LED_TIMEOUT_MS);
  if (sbusAlive) {
    if (mode != 0) { setStatusLed(C_BLUE, STATUS_LED_BRIGHT); mode = 0; }   // receiving → steady blue
  } else if (mode != 1) {                                                   // just lost / never had SBUS
    mode = 1; flashOn = true; lastToggle = now; setStatusLed(C_ORANGE, STATUS_LED_BRIGHT);
  } else if (now - lastToggle >= SBUS_LED_FLASH_MS) {                       // slow flash
    lastToggle = now; flashOn = !flashOn;
    setStatusLed(flashOn ? C_ORANGE : C_OFF, flashOn ? STATUS_LED_BRIGHT : 0);
  }
}

// Run a CLI line relayed from the config tool (queued by onWCBCommand on Core
// 0), tee'ing its Serial output to the RTERM sink so each line ships back to
// the bridge and surfaces on the config-tool terminal. Runs in loop() context
// where flash I/O and long prints are safe.
void drainRemoteCli() {
  if (!remoteCliQueue) return;
  RemoteCliMsg m;
  if (xQueueReceive(remoteCliQueue, &m, 0) != pdTRUE) return;

  rtermSink.begin(wcb, m.relay);
  rcSerial.armCapture(&rtermSink);          // mirror this command's Serial output to the bridge
  g_rtermRelay = m.relay;                    // so an ASYNC reply (remote Maestro read) can re-arm the tee
  bool recognised = execCliLine(String(m.cmd));
  if (!recognised)
    Serial.printf("Unknown command: %s\n", m.cmd);
  g_rtermRelay = 0;
  rtermSink.finish();                        // flush any trailing partial line
  rcSerial.disarmCapture();
}

// ── Paced aux-port transmitter ───────────────────────────────────────────────
// One in-flight message per aux port, clocked out a few bytes per loop() pass.
//
// Writing a whole command in one go is NOT an option here: S4/S5 are bit-banged
// SoftwareSerial, whose write() blocks with interrupts disabled for the full frame
// time. A 200-char command at 9600 baud is ~208 ms of that — over twenty missed
// SBUS frames. So each pass hands a port only as many bytes as it can absorb
// without holding the CPU past AUX_TX_BUDGET_US, and the rest waits for the next
// pass. loop() runs far faster than any of these line rates, so a port still
// transmits at its full baud; it just never blocks long enough to be noticed.
//
// One slot per port (not a shared queue drain) so a slow port can't hold up a fast
// one mid-message. Messages for a port that's still busy stay in serialFwdQueue —
// see drainSerialFwd for the head-of-line note.
#define AUX_TX_BUDGET_US 500      // max blocking per port per loop() pass (~1/18 of an SBUS frame)
struct SerialFwdTx { char text[202]; uint8_t len; uint8_t sent; };   // len 0 = idle
static SerialFwdTx auxTx[3];

// How many bytes port `idx` (0=S3, 1=S4, 2=S5) may take this pass.
static int auxTxBudget(int idx) {
  // S3 on the shared-SBUS boards is a real UART — write() is a memcpy into the
  // driver's TX buffer and the hardware clocks it out in the background, so give
  // it whatever the buffer will hold and never block on a full one.
  if (idx == 0 && s3IsHw) {
    int room = ((HardwareSerial*)s3)->availableForWrite();
    return room > 0 ? room : 0;
  }
  // Bit-banged: bytes = budget_us * baud / (10 bits per byte * 1e6 us). Always at
  // least 1 so a slow port still makes progress instead of stalling forever.
  uint32_t baud = rcConfig.auxBaud[idx];
  if (baud == 0) return 1;
  int n = (int)(((uint64_t)AUX_TX_BUDGET_US * baud) / 10000000ULL);
  return n < 1 ? 1 : n;
}

// Clock out whatever each port has pending, within its budget. Called every loop().
static void auxTxPump() {
  for (int i = 0; i < 3; i++) {
    SerialFwdTx& t = auxTx[i];
    if (t.len == 0) continue;
    Stream* p = auxStreamFor(i + 3);
    if (!p) { t.len = 0; continue; }               // port vanished (board profile) — drop it
    int budget = auxTxBudget(i);
    while (budget-- > 0 && t.sent < t.len) p->write((uint8_t)t.text[t.sent++]);
    if (t.sent >= t.len) {
      t.text[t.len - 1] = '\0';   // drop the framing CR — logging it would yank the terminal cursor
      t.len = 0;
      dlog(DBG_SERIAL, "[DISPATCH] Serial TX [%s]  %s\n", auxPortLabel(i), t.text);
    }
  }
}

// Move queued mesh→serial writes into the per-port TX slots. Takes the head only
// when its port is free: a message whose port is still transmitting stays queued
// (and briefly blocks messages behind it — acceptable, since each one clears in
// milliseconds and ordering per port is preserved either way).
void drainSerialFwd() {
  if (!serialFwdQueue) return;
  SerialFwdMsg m;
  while (xQueuePeek(serialFwdQueue, &m, 0) == pdTRUE) {
    int idx = (int)m.fwPort - 3;
    if (idx < 0 || idx > 2) { xQueueReceive(serialFwdQueue, &m, 0); continue; }   // bad port — discard
    if (auxTx[idx].len != 0) break;                // port busy — leave it queued
    xQueueReceive(serialFwdQueue, &m, 0);
    size_t n = strlcpy(auxTx[idx].text, m.text, sizeof(auxTx[idx].text) - 1);
    if (n > sizeof(auxTx[idx].text) - 2) n = sizeof(auxTx[idx].text) - 2;
    auxTx[idx].text[n++]  = '\r';                  // same framing a WCB's writeSerialString gives
    auxTx[idx].text[n]    = '\0';
    auxTx[idx].len        = (uint8_t)n;
    auxTx[idx].sent       = 0;
  }
  auxTxPump();
}

// ── Inbound ";M" execution (Core 1) ──────────────────────────────────────────
// Drive ONE local Maestro slot from an already-parsed WcbCmd frame. Everything goes
// through the named wrappers rather than a raw maestroWrite, so the channel guard,
// the speed/accel caches, the record shadow and the idle-release re-arm all stay in
// play exactly as they do for a locally-triggered action.
//
// The frame's own device byte (frame[1]) is DELIBERATELY ignored: it carries the
// mesh id, which on NaviCore is the SLOT INDEX, while the wire needs the slot's
// Pololu address. maestroWrite() supplies that itself from the slot.
static void maeInboundActuate(uint8_t id, const uint8_t* frame, size_t n) {
  const uint16_t arg14 = (n >= 6) ? (uint16_t)(frame[4] | ((uint16_t)frame[5] << 7)) : 0;
  switch (frame[2]) {
    case WcbMaestro::CMD_SET_TARGET:  maestroSetTarget(id, frame[3], arg14);            break;
    case WcbMaestro::CMD_SET_SPEED:   maestroSetSpeed (id, frame[3], arg14);            break;
    case WcbMaestro::CMD_SET_ACCEL:   maestroSetAccel (id, frame[3], (uint8_t)arg14);   break;
    case WcbMaestro::CMD_GO_HOME:     maestroGoHome(id);                                break;
    case WcbMaestro::CMD_STOP_SCRIPT: maestroStopScript(id);                            break;
    // Straight to maestroRestartScript, NOT via executeMaestroCmd's "restartScript"
    // branch — that one first runs applyScriptEasing(), which can emit up to 64 extra
    // SetSpeed/SetAccel frames. A WCB's ;M35 puts exactly four bytes on the wire, and
    // so must ours; inheriting the easing burst would be a silent behavioral fork.
    case WcbMaestro::CMD_RESTART_SUB:   maestroRestartScript(id, frame[3]);             break;
    case WcbMaestro::CMD_RESTART_SUB_P: maestroSubParam(id, frame[3], arg14);           break;
    default: return;
  }
  dlog(DBG_MAESTRO, "[DISPATCH] Maestro slot %u (device %u) <- mesh  cmd 0x%02X\n",
       id, rcConfig.maestros[id - 1].device, frame[2]);
}

// Service an inbound get* and unicast the answer back to `replyTo`. The reply is the
// ";M!<var>=<value>" form a WCB's handleMaestroResult feeds into its RAM/IF state —
// NOT ":MQR", which is what a WCB sends to the CONTROLLER. Here the roles are
// reversed: NaviCore is the one answering a board. Variable names must match the
// WCB's maestroGetInfo() exactly (m<id>moving / m<id>err / m<id>pos<ch>), and <id>
// is echoed back as addressed, since the asker derived its variable name from it.
static void maeInboundQuery(uint8_t id, const uint8_t* frame, uint8_t replyTo) {
  WcbMaestro::ReplyKind kind; uint8_t len;
  if (!WcbMaestro::replyInfo(frame[2], kind, len)) return;
  // A query needs a SINGLE answer, so the 0/9 fan-out ids are illegal (the WCB rejects
  // them the same way, WCB_Maestro.cpp:386) and a device shared by several slots is
  // answered once, by the first that carries it — they are the same physical Maestro
  // on NaviCore's single bus, so any of them gives the same reading.
  if (id < 1) return;
  uint8_t slotId = 0;
  for (uint8_t s = 1; s <= RC_NUM_MAESTROS; s++)
    if (rcConfig.maestros[s - 1].type == 1 && rcConfig.maestros[s - 1].device == id) { slotId = s; break; }
  if (slotId == 0) {   // LOCAL only — no matching local device means nothing to read
    dlog(DBG_MAESTRO, "[DISPATCH] Maestro: inbound query for device %u matches no local slot\n", id);
    return;
  }

  const uint8_t ch  = (kind == WcbMaestro::ReplyKind::POS) ? frame[3] : 0;
  uint8_t       rb[2];
  const int     got = maestroLocalQuery(slotId, (uint8_t)(frame[2] | 0x80),
                                        (kind == WcbMaestro::ReplyKind::POS) ? &ch : nullptr,
                                        (kind == WcbMaestro::ReplyKind::POS) ? 1 : 0,
                                        rb, WcbMaestro::replyLen(kind));
  uint16_t val = 0;
  if (got < (int)WcbMaestro::replyLen(kind) ||
      !WcbMaestro::decodeReply(kind, rb, (size_t)got, val)) {
    dlog(DBG_MAESTRO, "[DISPATCH] Maestro %u <- mesh query: timeout — no reply sent\n", id);
    return;                                          // timeout → send nothing, as the WCB does
  }
  char res[40];
  if      (kind == WcbMaestro::ReplyKind::MOV) snprintf(res, sizeof(res), ";M!m%umoving=%u", id, val);
  else if (kind == WcbMaestro::ReplyKind::ERR) snprintf(res, sizeof(res), ";M!m%uerr=%u",    id, val);
  else                                         snprintf(res, sizeof(res), ";M!m%upos%u=%u",  id, ch, val);
  if (wcb && wcbReady && replyTo >= 1 && replyTo <= WCB_MAX_BOARDS) wcb->send(replyTo, res);
  dlog(DBG_MAESTRO, "[DISPATCH] Maestro %u <- mesh query -> WCB%u: %s\n", id, replyTo, res);
}

// Run one inbound ";M" line deferred from onWCBCommand (Core 0). All parsing is done
// by WcbMaestro::build(), the SAME function the WCB firmware uses, so the two can
// never drift on grammar, verb spelling, case-sensitivity or frame bytes.
void drainMaestroCmd() {
  if (!maestroCmdQueue) return;
  MaestroCmdMsg m;
  if (xQueueReceive(maestroCmdQueue, &m, 0) != pdTRUE) return;   // one per pass — a query can block 25 ms

  const char* p = m.text + 1;          // skip ';' → points at 'M', which build() tolerates
  uint8_t     replyTo = m.sender;
  uint8_t     frame[WcbMaestro::MAX_FRAME];
  size_t      n = 0;

  if (p[1] == '!') {                   // ;M!<var>=<value> — a WCB pushing a get RESULT at us.
    // That is WCB RAM/IF state; NaviCore has no variable namespace. Normally never
    // arrives (a WCB sends us :MQR instead), so just note it and drop.
    dlog(DBG_MAESTRO, "[DISPATCH] Maestro: ignoring inbound variable push  %s\n", m.text);
    return;
  }

  if (p[1] == 'G' || p[1] == 'g') {
    // ;MG<dev>,<replyTo>,<verb>[,<ch>] — a WCB forwarding a get to the board that
    // hosts the Maestro. Strip the replyTo field back out and hand the rest to the
    // library, exactly as the WCB's own handleMaestroGet reconstructs it.
    const char* s  = p + 2;
    const char* c1 = strchr(s,      ',');
    const char* c2 = c1 ? strchr(c1 + 1, ',') : nullptr;
    if (!c2) { dlog(DBG_MAESTRO, "[DISPATCH] Maestro: malformed ;MG  %s\n", m.text); return; }
    const int rt = atoi(c1 + 1);
    if (rt < 1 || rt > WCB_MAX_BOARDS) {   // guards an injected/garbled reply-to
      dlog(DBG_MAESTRO, "[DISPATCH] Maestro: ;MG bad replyTo %d\n", rt);
      return;
    }
    replyTo = (uint8_t)rt;
    char         body[sizeof(m.text)];
    const size_t devLen = (size_t)(c1 - s);
    if (devLen == 0 || devLen >= sizeof(body)) return;
    memcpy(body, s, devLen);
    const int bn = snprintf(body + devLen, sizeof(body) - devLen, ",%s", c2 + 1);
    if (bn <= 0 || (size_t)bn >= sizeof(body) - devLen) return;
    n = WcbMaestro::build(body, frame, sizeof(frame));
  } else {
    n = WcbMaestro::build(p, frame, sizeof(frame));
  }

  if (n == 0) {   // unknown verb / bad args — drop quietly, same as the WCB does
    dlog(DBG_MAESTRO, "[DISPATCH] Maestro: unparseable inbound  %s\n", m.text);
    return;
  }

  const uint8_t id = frame[1];         // the POLOLU DEVICE number parsed out of the text
  WcbMaestro::ReplyKind kind; uint8_t rlen;
  if (WcbMaestro::replyInfo(frame[2], kind, rlen)) { maeInboundQuery(id, frame, replyTo); return; }

  // Actuation. The inbound id is a Pololu DEVICE number and it is a MULTICAST address:
  // fire every local slot carrying it. More than one Maestro may legitimately share a
  // device number across the mesh — that is a supported architecture, not a collision.
  // Ids 0 and 9 are the reserved fan-out targets ("all" / "all local"); on NaviCore both
  // mean every local Maestro, since NaviCore never re-broadcasts.
  //
  // Everything is gated on type == 1: maestroWrite routes ANY non-1 type to the mesh
  // broadcast stream, so dispatching a remote slot here would push the frame back out
  // over ESP-NOW — duplicate actuation on every Kyber board. NaviCore only advertises
  // type-1 slots, so this gate is exactly the contract it published.
  //
  // Deduped by resolved device the way the WCB dedupes by physical destination
  // (WCB_Maestro.cpp:96-106): NaviCore has ONE Maestro bus, so two type-1 slots sharing
  // a device number are the same physical Maestro and must be written only once.
  const bool fanOut = (id == 0 || id == 9);
  uint8_t    fired[RC_NUM_MAESTROS];
  uint8_t    nFired = 0;
  for (uint8_t s = 1; s <= RC_NUM_MAESTROS; s++) {
    const RcMaestroSlot& slot = rcConfig.maestros[s - 1];
    if (slot.type != 1) continue;
    if (!fanOut && slot.device != id) continue;
    bool dup = false;
    for (uint8_t k = 0; k < nFired; k++) if (fired[k] == slot.device) { dup = true; break; }
    if (dup) continue;                       // same physical Maestro on the shared bus
    fired[nFired++] = slot.device;
    maeInboundActuate(s, frame, n);
  }
  if (nFired == 0)
    dlog(DBG_MAESTRO, "[DISPATCH] Maestro: inbound device %u matches no local slot\n", id);
}

// onNeighbor callback — runs on the WiFi/ESP-NOW task (Core 0). Do the MINIMUM:
// enqueue the WCB number; drainPeerEvents() (loop) does the rest. Only real WCBs
// count as "peers" here — client devices (other controllers advertising via
// setIdentity) are skipped.
void onWcbNeighbor(const WCBNeighbor& nb) {
  if (!nb.valid) return;
  if (nb.wcbNumber < 1 || nb.wcbNumber > WCB_MAX_BOARDS) return;
  // Adopt the node's LIVE advertised name for the status panel — for WCBs AND
  // client devices (a mesh monitor, another controller). The library replaces a
  // neighbor's facts wholesale each advert, so a RENAMED node updates its name
  // HERE the moment its next advert arrives (the ?WHOAMI cache alone is held once
  // set). This also makes ?WHOAMI self-limiting: buildWcbStatus only queries while
  // the alias is empty, so an advertising node is never queried. Same Core-0
  // context as the wcb_alias handler that also writes the cache.
  if (nb.name[0]) rcTelemetry::setWcbAlias(nb.wcbNumber, nb.name);
  // Cache client-vs-board so the status panel can still tag a learned client as a
  // client after its advert ages out (a client never heartbeats). Unconditional —
  // a client may advertise without a name.
  rcTelemetry::setWcbClient(nb.wcbNumber, nb.isClient);
  // The new-peer action/alert fires only for real WCB peers, not client devices.
  if (nb.isClient || !peerEventQueue) return;
  uint8_t id = nb.wcbNumber;
  xQueueSend(peerEventQueue, &id, 0);        // non-blocking; a dropped enqueue is caught on the next advert
}

// Drain queued new-peer detections (loop, Core 1): dedup by id, suppress the
// boot-time fleet-discovery burst (grace window), then fire the configured
// action tier + the passive alert. Fires at most once per board per session.
void drainPeerEvents() {
  if (!peerEventQueue) return;
  uint8_t id;
  while (xQueueReceive(peerEventQueue, &id, 0) == pdTRUE) {
    if (id < 1 || id > WCB_MAX_BOARDS) continue;
    const uint32_t bit = 1UL << (id - 1);
    if (g_peerSeenMask & bit) continue;                        // already handled this board this session
    g_peerSeenMask |= bit;
    if ((int32_t)(millis() - g_peerGraceUntil) < 0) continue;  // still in the boot grace window — record silently

    const WCBNeighbor* nb = (wcb && wcbReady) ? wcb->getNeighbor(id) : nullptr;
    const char* alias = (nb && nb->name[0]) ? nb->name : "";

    if (rcConfig.peerAlert) {                                  // passive alert = terminal line + LED pulse
      Serial.printf("[PEER] New WCB %u%s%s detected\n", id, alias[0] ? " " : "", alias);
      g_peerFlashUntil = millis() + 1500;                      // brief cyan LED pulse (see updateStatusLed)
    }
    const RcTier& tier = rcConfig.peerNewActions;              // configured action(s), reuse the normal dispatch
    for (int i = 0; i < tier.count; i++) rcExecuteAction(tier.a[i]);
  }
}

// ── Board ONLINE / OFFLINE transitions ──────────────────────────────────────
// The library tracks online/offline whether or not this is registered
// (isOnline()); registering it is what makes the TRANSITION visible. Without
// it a board that drops mid-show leaves no trace on this console at all.
//
// Runs on TWO different tasks: the ONLINE edge fires from the ESP-NOW receive
// callback (Core 0 — WCB_Client.cpp ≈2063, on the first heartbeat after
// silence), the OFFLINE edge from wcb->update() in loop() (Core 1 —
// _checkOfflineBoards, WCB_Client.cpp ≈1938). So the body must stay Core-0
// safe: one printf, no flash, no NVS, no droid hardware. Both edges are gated
// by the ETM heartbeat miss count, so this cannot flood the WiFi task.
//
// The name comes from rcTelemetry::wcbAlias() rather than wcb->getNeighbor():
// wcbAlias is written on Core 0 terminator-first and is invariantly terminated,
// so the Core-1 offline read can't tear onto a half-written name.
void onWcbStatus(uint8_t wcbID, bool online) {
  if (wcbID < 1 || wcbID > WCB_MAX_BOARDS) return;
  const char* alias = rcTelemetry::wcbAlias((int)wcbID);
  Serial.printf("[WCB] WCB%u%s%s %s\n", wcbID,
                alias[0] ? " · " : "", alias, online ? "ONLINE" : "OFFLINE");
}

// ── Boot roll call (one shot, ROLL_CALL_MS after begin) ─────────────────────
// Name every board in the configured floor we have STILL never heard from.
// Distinct from onWcbStatus: a board that comes up and later drops gets a
// transition line, but a board that was never there at all produces nothing.
// A roll call, not an alarm — one line is worth it whether the answer is
// "the dome is on the bench" or "check its power".
void checkBootRollCall() {
  if (!g_rollCallAt || (int32_t)(millis() - g_rollCallAt) < 0) return;
  g_rollCallAt = 0;                                  // one shot, however it ends
  if (!wcb || !wcbReady) return;
  const uint8_t self = rcConfig.wcbNetwork.deviceId;
  uint8_t total = 0, missing = 0;
  uint8_t floorMax = rcConfig.wcbNetwork.quantity;
  if (floorMax > WCB_MAX_BOARDS) floorMax = WCB_MAX_BOARDS;
  for (uint8_t id = 1; id <= floorMax; id++) {
    if (id == self) continue;                        // ourselves — never a peer
    total++;
    if (wcb->isOnline(id)) continue;
    const char* alias = rcTelemetry::wcbAlias((int)id);
    Serial.printf("[WCB] roll call: WCB%u%s%s never heard from\n", id,
                  alias[0] ? " · " : "", alias);
    missing++;
  }
  Serial.printf("[WCB] roll call: %u/%u board(s) online %us after join\n",
                (uint8_t)(total - missing), total, (unsigned)(ROLL_CALL_MS / 1000));
}

// Dispatch remote TRIGGERs deferred from the Core-0 onWCBCommand callback (loop,
// Core 1). Running rcDispatch here — the same core as a local matrix press —
// means the Maestro UART, HCR/MP3 aux-serial, speed/accel caches, tap-timing
// state machine and record-replay gate are all touched from ONE core, with no
// cross-core race. Args are re-validated defensively even though the enqueuer
// already checked them.
void drainRemoteTriggers() {
  if (!remoteTriggerQueue) return;
  RemoteTrigger t;
  while (xQueueReceive(remoteTriggerQueue, &t, 0) == pdTRUE) {
    if (t.mode < 1 || t.mode > 3) continue;
    if (t.btn  < 1 || t.btn  > RC_NUM_THRESHOLDS) continue;
    uint8_t tap = t.tap < 1 ? 1 : (t.tap > RC_NUM_TAP_TIERS ? RC_NUM_TAP_TIERS : t.tap);
    rcDispatch(t.mode * 100 + t.btn, tap);
  }
}

// Run any forget-peer requests deferred from the Core-0 Via-WCB path (loop/Core 1).
void drainForgetPeer() {
  if (!forgetPeerQueue) return;
  uint8_t id;
  while (xQueueReceive(forgetPeerQueue, &id, 0) == pdTRUE) doForgetPeer(id);
}

void loop() {
  // WCB — heartbeats, ACKs, WCBStream flushes
  if (wcb && wcbReady) wcb->update();

  // Firmware OTA over the WCB mesh — drain packets the receive callback queued
  // (esp_ota_* blocks, so it can't run in the WiFi task) and reap a stalled
  // session. Both are cheap no-ops when no OTA is in flight.
  naviota::drainOtaPackets();
  naviota::checkOtaTimeout();

  // Remote terminal — run any CLI line relayed from the config tool, with its
  // Serial output tee'd back to the bridge as RTERM packets. Cheap no-op when
  // nothing is pending.
  drainRemoteCli();

  // Same job for a WebSocket client on the optional SoftAP: the httpd handler runs
  // on Core 0 and only enqueues, so the command itself is executed HERE, where
  // flash, NVS and droid hardware are safe to touch. No-op unless wifiEnabled
  // brought the endpoint up.
  naviws::drain();

  // Surface any mesh-relayed Maestro read reply (:MQR) as a [MAE:] marker for the
  // config tool's Read-live readout — deferred here from the Core-0 consumer.
  maePumpRemoteEmits();

  // Run one inbound ";M" command a WCB routed to a Maestro we host (deferred from the
  // Core-0 receive callback). One per pass — a get* blocks up to 25 ms on the reply.
  drainMaestroCmd();

  // New WCB peer detected on the mesh → fire the configured action + passive
  // alert (deferred here from the Core-0 onNeighbor callback). Cheap no-op idle.
  drainPeerEvents();

  // One-shot boot roll call — name any configured board never heard from.
  // Self-disarms on its first run, so this is a single compare afterwards.
  checkBootRollCall();

  // Remote TRIGGER — dispatch any {"type":"TRIGGER"} relayed from the config tool
  // / mesh, deferred here from the Core-0 onWCBCommand callback so it shares the
  // local matrix press's single-core dispatch path (no cross-core Maestro/cache
  // race). Cheap no-op when nothing is pending.
  drainRemoteTriggers();

  // Forget-learned-peer — run any FORGET_PEER deferred from the Core-0 Via-WCB
  // path here on Core 1 (esp_now_del_peer + NVS write). Cheap no-op when idle.
  drainForgetPeer();

  // Per-action Test button (bridged) — fire any TEST_ACTION deferred from the
  // Core-0 Via-WCB path here on Core 1, sharing the same dispatch path. Cheap
  // no-op when nothing is pending. (Direct-USB TEST_ACTION fires inline above.)
  drainTestAction();

  // Record/replay — drain captured events into the PSRAM clip (Core-1 sole
  // writer) and advance any active replay. Both cheap no-ops when idle.
  navirec::pollControl();   // run any Record/Play trigger deferred from dispatch (Core-1 safe)
  navirec::drain();
  navirec::checkRecordBackstop();   // auto-stop+SAVE a capture that ran past REC_MAX_MS (never lose a long take)
  navirec::replayTick();
  if (navirec::takeReplayDone()) {
    Serial.println("[REC] ▶ playback complete");
    // The servos are ours again — pick up any easing a save deferred while the player
    // owned them (applyConfigSideEffects skips the re-apply during replay), and restore
    // the profile the player force-zeroed on every channel it touched. Cache-gated, so
    // this is a no-op when nothing changed.
    for (uint8_t mid = 1; mid <= RC_NUM_MAESTROS; mid++) reapplyMaestroEasing(mid);
  }

  // WCB-network telemetry bridge — periodic rc_hb (0.5 Hz) + rc_ch (5 Hz)
  // broadcasts so the config tool's "Via WCB" mode can discover and live-
  // monitor this RC.  Event-driven rc_trig / rc_mode are emitted from
  // rcDispatch() and the mode-decode block in processSbus(), respectively.
  rcTelemetry::tick();

  // SBUS
  processSbus();
  checkDeferredTap();

  // Re-send any owed easing repeats (cheap no-op when none are pending).
  easingRepeatTick();

  // Mesh-stats reset, deferred here from either transport (the library requires
  // loop(), not a receive callback). Cheap no-op when idle.
  if (g_meshStatsResetPending) {
    g_meshStatsResetPending = false;
    if (wcb && wcbReady) wcb->resetStats();      // the library's own send-side counters
    g_meshRxCount = 0;                           // ours: the library doesn't track RX
    memset(g_meshRxFrom, 0, sizeof(g_meshRxFrom));
    Serial.println("[WCB] mesh stats cleared");
  }

  // Status LED: steady BLUE while receiving SBUS, slow-flash ORANGE when not
  updateStatusLed();

  // Pending delayed actions
  checkPendingActions();

  // USB Serial WebSerial monitor stream
  sendPWMUpdate();

  // USB Serial input
  handleSerialInput();

  // Aux-serial RX monitor — drain S3/S4/S5, echo incoming lines under the "Serial"
  // debug chip, and broadcast them to the mesh on any port with bcastIn set.
  pollAuxSerialRx();

  // Mesh → serial: clock out whatever the bridge has queued for S3/S4/S5, a few
  // bytes per pass so a bit-banged port never blocks long enough to cost an SBUS
  // frame. Cheap no-op when nothing is pending.
  drainSerialFwd();

  // HCR audio fades (fn 12/15) on a LOCALLY-wired HCR: advance any in-flight ramp
  // and emit the next SetVolume step. Cheap no-op when nothing is fading. A remote
  // HCR fades on its own WCB, so this only runs on the local transport.
  if (rcConfig.hcrDest.transport == 0) {
    Stream* hp = hcrLocalSerial();
    if (hp) {
      g_hcrFade.tick(g_hcr, *hp);
    } else if (g_hcrFade.active(0) || g_hcrFade.active(1) || g_hcrFade.active(2)) {
      // Local transport but no resolvable serial port (e.g. an out-of-range dest) —
      // can't tick, so cancel any in-flight fade rather than freeze the ramp.
      g_hcrFade.cancel(0); g_hcrFade.cancel(1); g_hcrFade.cancel(2);
    }
  } else if (g_hcrFade.active(0) || g_hcrFade.active(1) || g_hcrFade.active(2)) {
    // HCR moved to a WCB while a local fade was in flight — it can't be ticked here,
    // so cancel it rather than leave the ramp frozen at a partial level.
    g_hcrFade.cancel(0); g_hcrFade.cancel(1); g_hcrFade.cancel(2);
  }

  // Auto-release idle passthrough servos (RcKnobOutput.releaseIdleMs) — de-energize a
  // resting servo so it stops buzzing/hunting; the next stick move re-energizes it.
  maestroIdleReleaseTick();

  // Trailing edge of the HCR volume throttle — send the last value a knob sweep
  // asked for, which the leading-edge gate held back.
  hcrVolFlushTick();

  // FPS counter
  trackSbusFps();

  // 1Hz SBUS live dump (#L10)
  if (sbusLiveDump && (millis() - sbusLiveDumpLastMs >= 1000)) {
    sbusLiveDumpLastMs = millis();
    dumpSbusState();
  }
}
