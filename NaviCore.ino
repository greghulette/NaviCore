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
};
TapState tapState;

// Last-seen switch positions for change detection
int switchPrevPos[RC_NUM_SWITCHES];

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
// read exactly `nReply` bytes — the caller knows which query it issued. Only a LOCAL
// slot (type 1, wired to Serial2) can answer synchronously; a Remote slot's reply
// would have to be relayed back by its WCB (Phase 2, not yet built). Runs in
// loop()/Core-1 (execCliLine) where Serial2 I/O is race-free; the ~25 ms deadline
// bounds the stall so a missing Maestro or an unwired RX line can't starve SBUS.
//   return: bytes read (== nReply on success, < nReply on timeout), or
//           -1 = slot not Local (no reply path yet), -2 = send failed (disabled/bad device#).
static int maestroLocalQuery(uint8_t id, uint8_t cmd_compact,
                             const uint8_t* payload, size_t plen,
                             uint8_t* reply, size_t nReply) {
  if (id < 1 || id > RC_NUM_MAESTROS) return -2;
  const uint8_t t = rcConfig.maestros[id - 1].type;
  if (t == 0) return -2;                                   // disabled slot
  if (t != 1) return -1;                                   // Remote (type 2) → no synchronous reply yet (Phase 2)
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
// return (bytes read, -1 remote, -2 disabled).
static void maestroReportQuery(uint8_t slot, uint8_t kind, uint8_t ch, int n, uint16_t val) {
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
struct MaeRemoteCache { uint16_t pos; uint8_t moving; uint16_t err; uint32_t ms; bool valid; };
static MaeRemoteCache g_maeRemote[RC_NUM_MAESTROS] = {};
static const uint32_t MAE_CACHE_FRESH_MS = 250;   // a busy-state older than this = "unknown" → the gate fails open

// Parse ":MQR,<id>,<chan>,<KIND>,<value>" (KIND = POS|MOV|ERR) into g_maeRemote.
// Runs on Core 0 (WiFi RX task) — parse + store only, never any I/O.
static void maeConsumeRemoteReply(const char* body) {
  int id = atoi(body);                                      // body = "<id>,<chan>,<KIND>,<value>"
  if (id < 1 || id > RC_NUM_MAESTROS) return;
  const char* p = strchr(body, ','); if (!p) return; p++;   // → chan
  p = strchr(p, ',');                if (!p) return; p++;   // → KIND
  const char* v = strchr(p, ',');    if (!v) return;        // v points at the comma before value
  long val = atol(v + 1);
  MaeRemoteCache& c = g_maeRemote[id - 1];
  if      (strncmp(p, "MOV", 3) == 0) c.moving = (uint8_t)(val != 0);
  else if (strncmp(p, "POS", 3) == 0) c.pos    = (uint16_t)val;
  else if (strncmp(p, "ERR", 3) == 0) c.err    = (uint16_t)val;
  c.ms = millis(); c.valid = true;
}

// Is Maestro `id` mid-movement (a servo still moving)? The skip-if-running gate uses this.
// LOCAL slot → query getMovingState synchronously (bounded ~25 ms read). REMOTE slot → read
// the WDP-relayed cache; if stale/missing, FAIL OPEN (return false = "not busy, go ahead").
// getMovingState is a PROXY for "sequence running" — it misses a paused / instant-move
// script, but it's the only native running-signal the Maestro offers.
static bool maestroSequenceBusy(uint8_t id) {
  if (id < 1 || id > RC_NUM_MAESTROS) return false;
  const RcMaestroSlot& slot = rcConfig.maestros[id - 1];
  if (slot.type == 1) {                                     // LOCAL — ask now (bounded raw Serial2 read)
    uint8_t rb[1];
    return maestroLocalQuery(id, 0x93, nullptr, 0, rb, 1) == 1 && rb[0] != 0;   // 0x93 = getMovingState
  }
  if (slot.type == 2) {                                     // REMOTE — read the relayed cache
    const MaeRemoteCache& c = g_maeRemote[id - 1];
    return c.valid && (millis() - c.ms) <= MAE_CACHE_FRESH_MS && c.moving != 0;
  }
  return false;                                             // disabled → never "busy"
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
static void maestroGoHome(uint8_t id)        { maestroWrite(id, 0xA2, nullptr, 0); navirec::shadowInvalidateSlot(id); maeReleaseArmSlot(id); }
static void maestroStopScript(uint8_t id)    { maestroWrite(id, 0xA4, nullptr, 0); navirec::shadowInvalidateSlot(id); maeSmoothInvalidateSlot(id); maeReleaseArmSlot(id); }
static void maestroRestartScript(uint8_t id, uint8_t sub) {
  maestroWrite(id, 0xA7, &sub, 1);
  maeSmoothInvalidateSlot(id);   // a device-side script may change speed/accel we can't see — re-apply on next stick move
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
      reapplyMaestroEasing(id);
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
      g_switchEasing[id - 1] = v; reapplyMaestroEasing(id);
    }
  }
  else if (strcmp(tok, "snappy") == 0) {          // LEGACY snappy toggle → setEasing (on/bare→Off full speed, off→Release)
    char* s = strtok(nullptr, ",");
    if (id >= 1 && id <= RC_NUM_MAESTROS) {
      int8_t v = EASE_OFF;                                                         // bare / "on" / "1" → full speed
      if (s && ((((s[0]=='o'||s[0]=='O') && (s[1]=='f'||s[1]=='F'))) || s[0]=='0')) v = EASE_RELEASED;  // "off"/"0" → local-only
      g_switchEasing[id - 1] = v; reapplyMaestroEasing(id);
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
    if (sN && sP) {
      uint16_t param = (uint16_t)atoi(sP); if (param > 16383) param = 16383;
      uint8_t p[3] = { (uint8_t)atoi(sN), (uint8_t)(param & 0x7F), (uint8_t)((param >> 7) & 0x7F) };
      maestroWrite(id, 0xA8, p, 3);          // 0xA8 & 0x7F = 0x28 → {0xAA,dev,0x28,sub,pl,ph} == WcbMaestro::buildSubParam
      maeSmoothInvalidateSlot(id);           // a device-side script may change speed/accel we can't see
    }
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
  // fn 7/10/13 are NOT in the WCB's numeric ";H,FN" switch (WCB_HCR.cpp
  // processHCRRuntimeCommand) — it only maps 2,3,4,5,6,8,9,11,14,16,17. Emit the
  // readable verbs the WCB DOES implement so HCR-over-WCB needs no firmware change
  // on the receiving board. All other fns use the numeric convention.
  switch (fn) {
    case 7:  return String(";H,MUSE,GAP,") + chan + "," + track;  // Muse(min,max)
    case 10: return String(";H,OVERRIDE,") + chan;                // OverrideEmotions(v)
    case 13: return String(";H,MUSE,") + track;                   // SetMuse(v)
    // SetVolume: a specific channel rides the numeric WCB FN-17 (SetVolume(ch,v));
    // chan 3 = ALL uses the WCB's ;H,VOL with the channel OMITTED (set V+A+B to the
    // value) in ONE message — the WCB's ;H,VOL handler treats a leading non-channel
    // field as the value and loops V/A/B (WCB_HCR.cpp, ch<0 → all). Works end-to-end.
    case 17: return (chan == 3) ? (String(";H,VOL,") + track)
                                : (String(";H,FN,17,") + chan + "," + track);
    // Volume Up/Down. chan 0 = ALL (V+A+B) in ONE message — the WCB's ;H,VOLUP/VOLDN
    // with the channel omitted loops V/A/B itself (WCB_HCR.cpp). chan 1/2/3 = a single
    // V/A/B channel → ;H,VOLUP,<V|A|B>[,step] (the WCB reads a named field-1 as the
    // channel; a numeric field-1 is the step). track = step (0 → the WCB's default 5).
    case 18:
    case 19: {
      String out = String(";H,") + (fn == 18 ? "VOLUP" : "VOLDN");
      if (chan >= 1 && chan <= 3) out += String(",") + (char)(chan == 1 ? 'V' : chan == 2 ? 'A' : 'B');
      if (track > 0) out += String(",") + track;
      return out;
    }
    default: return String(";H,FN,") + (int)fn + "," + chan + "," + track;
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
  if (n > (int)sizeof(buf)) n = sizeof(buf);
  if (Serial.availableForWrite() >= n) Serial.write((const uint8_t*)buf, (size_t)n);
}

// Dispatch an HCR action.
//
// The destination is GLOBAL — pulled from rcConfig.hcrDest rather than from
// the action itself. This lets every HCR action share one configured
// vocalizer wiring; the action only carries fn/chan/track.
static void executeHcrAction(const RcAction& a) {
  const RcHcrDest& dest = rcConfig.hcrDest;

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
    if (a.fn == 12) g_hcrFade.start(g_hcr, *hcrSerial, ch, 0, g_hcr.getVol(ch), sec, false, 0);   // FadeIn: 0 → current level
    else { const int cur = g_hcr.getVol(ch); g_hcrFade.start(g_hcr, *hcrSerial, ch, cur, 0, sec, true, cur); }  // FadeOut: cur → 0, StopWAV, restore
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
    bool ok = WcbWled::emit(*port, body, &Serial);   // build ;L verb → WLED JSON, newline-framed
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
        dlog(DBG_WCB, "[DISPATCH] WCB→%d  %s\n", boardId, a.cmd);
        wcb->send(boardId, a.cmd);
      }
      break;
    }
    case RA_WCB_BROADCAST:
      if (!wcb || !wcbReady) { dlog(DBG_WCB, "[DISPATCH] WCB broadcast skipped — WCB not ready\n"); break; }
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
    if (pendingActions[i].active && now >= pendingActions[i].fireAt) {
      pendingActions[i].active = false;
      // Fire the EFFECT directly (NOT rcExecuteAction) so the action's own
      // delayMs can't re-queue it forever. Honor a calibration that started
      // after it was scheduled.
      if (!calibrationActive) rcExecuteActionNow(pendingActions[i].action);
    }
  }
}

// =============================================================================
//  RC dispatch by buttonId + tap count
// =============================================================================
void rcDispatch(int buttonId, uint8_t tapCount) {
  int mode = buttonId / 100, btn = buttonId % 100;
  if (mode < 1 || mode > 3 || btn < 1 || btn > RC_NUM_THRESHOLDS) return;
  if (tapCount < 1 || tapCount > 3) return;

  // Broadcast a "this trigger fired" event over the WCB ESP-NOW network so
  // the config tool's "Via WCB" mode (and any listening Wizard) sees every
  // dispatch — local matrix press, Web-Serial JSON TRIGGER, or remote
  // ESP-NOW TRIGGER — uniformly.  Emitted BEFORE action execution so the
  // event arrives even if a synchronous action stalls.
  rcTelemetry::emitTrig(mode, btn, tapCount);

  const RcMapping& mapping = rcConfig.mappings[rcMapIndex(mode, btn)];
  if (mapping.exclusive) {
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
}

void checkDeferredTap() {
  if (!tapState.deferredPending) return;
  if (millis() >= tapState.deferredFireAt) {
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
  for (int i = 0; i < RC_NUM_SWITCHES; i++) {
    RcSwitch& sw = rcConfig.switches[i];
    if (sw.channel < 1 || sw.channel > 24) continue;
    int pos = readSwitchPos(sbusValues[sw.channel - 1], sw.positions);
    if (g_switchSeedPending) { switchPrevPos[i] = pos; continue; }   // seed only, don't fire
    if (pos == switchPrevPos[i]) continue;
    switchPrevPos[i] = pos;
    RcTier& tier = sw.t[pos];
    for (int ai = 0; ai < tier.count; ai++) rcExecuteAction(tier.a[ai]);
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
};
static HcrVolCache hcrVolCache;
static const unsigned long HCR_VOLUME_MIN_INTERVAL_MS = 80;  // ~12 Hz max per chan

static void dispatchHcrVolume(uint8_t audioChan, uint8_t vol) {
  if (audioChan > 3) {
    dlog(DBG_HCR, "[DISPATCH] HCR volume: audio channel %u out of range (0-3 = V/A/B/All) — "
         "check the knob's HCR output target; skipped\n", audioChan);
    return;
  }
  if (vol > 99) vol = 99;
  if ((int8_t)vol == hcrVolCache.lastVol[audioChan]) return;
  unsigned long now = millis();
  if (now - hcrVolCache.lastSent[audioChan] < HCR_VOLUME_MIN_INTERVAL_MS) return;
  hcrVolCache.lastVol[audioChan]  = vol;
  hcrVolCache.lastSent[audioChan] = now;
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

// Per-knob last-processed raw SBUS value. Knobs only re-dispatch when their
// channel moves past KNOB_CHANGE_DEADBAND counts — otherwise a stationary
// stick/knob would spam a Maestro/HCR command on every SBUS frame (~70+/s),
// and SBUS line jitter (±1-2 counts) would do the same. Matches the ≥5
// deadband the matrix/mode selectors already use. Sentinel 0xFFFF forces the
// first frame through so the initial position is always sent.
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

void processKnobs() {
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

    // Only dispatch when the source actually moved (or on the first frame).
    if (abs((int)raw - (int)lastKnobRaw[i]) < KNOB_CHANGE_DEADBAND) continue;
    lastKnobRaw[i] = raw;
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
  for (int i = 0; i < RC_NUM_KNOBS; i++)
    if (rcConfig.knobs[i].modeAware) lastKnobRaw[i] = 0xFFFF;
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
      if (matrixNeutralCount >= debFrames) matrixArmed = true;   // release confirmed
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
// does its (stack-heavy) ArduinoJson parsing — that extra 200 bytes was enough
// to overflow the WiFi-task stack and crash the board on every mesh command.
static void __attribute__((noinline)) queueRemoteCli(uint8_t relay, const char* command) {
  RemoteCliMsg m;
  m.relay = relay;
  strlcpy(m.cmd, command, sizeof(m.cmd));
  xQueueSend(remoteCliQueue, &m, 0);   // non-blocking; drop under load
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
  if (command && strncmp(command, ":MQR,", 5) == 0) { maeConsumeRemoteReply(command + 5); return; }

  // Unhandled (legacy/unknown) WCB command.  This runs in the ESP-NOW
  // receive callback on Core 0; a blocking Serial.printf here can stall the
  // WiFi task (if a USB host is attached but not draining) and interleave
  // with the main loop's Serial output on Core 1.  It's rare (almost all RC
  // traffic is JSON handled above), so gate it behind the same verbose flag
  // used for the fragment logging rather than printing unconditionally.
  if (rcTelemetry::RC_TELEM_VERBOSE)
    Serial.printf("[WCB RX] from WCB%d: %s\n", senderID, command);
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
  Serial.println(buf);
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

static void applySerialBauds(bool initial) {
  if (initial || rcConfig.maestroBaud != appliedMaestroBaud) {
    if (!initial) Serial2.end();
    Serial2.begin(rcConfig.maestroBaud, SERIAL_8N1, MAESTRO_RX_PIN, MAESTRO_TX_PIN);
    appliedMaestroBaud = rcConfig.maestroBaud;
    Serial.printf("[Serial2] Local Maestro %s @ %lu baud  TX=GPIO%d\n",
                  initial ? "open" : "re-open",
                  (unsigned long)rcConfig.maestroBaud, MAESTRO_TX_PIN);
  }
  if (initial || rcConfig.auxBaud[0] != appliedAuxBaud[0]) {
    if (!initial) auxEnd(s3, s3IsHw);
    auxBegin(s3, s3IsHw, rcConfig.auxBaud[0], S3_RX_PIN, S3_TX_PIN);   // hw UART0 (v2) or SoftwareSerial (3.2)
    appliedAuxBaud[0] = rcConfig.auxBaud[0];
    Serial.printf("[AUX] S3 %s @ %lu baud (%s)\n",
                  initial ? "open" : "re-open", (unsigned long)rcConfig.auxBaud[0], s3IsHw ? "hw UART0" : "sw");
  }
  if (initial || rcConfig.auxBaud[1] != appliedAuxBaud[1]) {
    if (!initial) auxEnd(s4, false);
    auxBegin(s4, false, rcConfig.auxBaud[1], S4_RX_PIN, S4_TX_PIN);
    appliedAuxBaud[1] = rcConfig.auxBaud[1];
    Serial.printf("[AUX] S4 %s @ %lu baud\n",
                  initial ? "open" : "re-open", (unsigned long)rcConfig.auxBaud[1]);
  }
  // S5 — both boards (v2 GPIO38/47, WCB 3.2 GPIO9/10). s5 is nullptr only in the unused
  // dedicated-SBUS fallback, so the guard still applies.
  if (s5 && (initial || rcConfig.auxBaud[2] != appliedAuxBaud[2])) {
    if (!initial) auxEnd(s5, false);
    auxBegin(s5, false, rcConfig.auxBaud[2], S5_RX_PIN, S5_TX_PIN);
    appliedAuxBaud[2] = rcConfig.auxBaud[2];
    Serial.printf("[AUX] S5 %s @ %lu baud\n",
                  initial ? "open" : "re-open", (unsigned long)rcConfig.auxBaud[2]);
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
  //    Queries are LOCAL-slot only (a Remote reply would need WCB relay — Phase 2).
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
        uint8_t rb[2]; int n = maestroLocalQuery(slot, 0x90, &ch, 1, rb, 2);
        maestroReportQuery(slot, 0, ch, n, (n == 2) ? (uint16_t)(rb[0] | (rb[1] << 8)) : 0);
      }
    } else if (verb.equalsIgnoreCase("MOVING")) {       // ?MAE,MOVING,<slot> — Get Moving State
      if (c1 > 0) {
        uint8_t slot = (uint8_t)a.substring(c1 + 1).toInt();
        uint8_t rb[1]; int n = maestroLocalQuery(slot, 0x93, nullptr, 0, rb, 1);
        maestroReportQuery(slot, 1, 0, n, (n == 1) ? rb[0] : 0);
      }
    } else if (verb.equalsIgnoreCase("ERR")) {          // ?MAE,ERR,<slot> — Get Errors (clears on read)
      if (c1 > 0) {
        uint8_t slot = (uint8_t)a.substring(c1 + 1).toInt();
        uint8_t rb[2]; int n = maestroLocalQuery(slot, 0xA1, nullptr, 0, rb, 2);
        maestroReportQuery(slot, 2, 0, n, (n == 2) ? (uint16_t)(rb[0] | (rb[1] << 8)) : 0);
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
      if (!navirec::loadClip(name.c_str())) { Serial.printf("[REC] clip '%s' not found\n", name.c_str()); return true; }
      // Via-WCB (capture sink armed): every [CLIPDL:EV] line is an RTERM packet
      // paced at 2 ms, and editStream runs in loop() — a dense capture would
      // stall SBUS/WCB servicing for tens of seconds. Refuse big clips on the
      // relayed path with a marker the config tool surfaces; USB streams at
      // full speed with only a light yield.
      const bool relayed = rcSerial.captureArmed();
      if (relayed && navirec::eventCount() > 3000) {
        Serial.printf("[CLIPDL:ERR]clip too large to edit over the WCB bridge (%lu events) — connect over USB\n",
                      (unsigned long)navirec::eventCount());
        return true;
      }
      navirec::editStream(Serial, relayed);
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
static void auxRxPollPort(Stream* p, int idx, char* buf, uint8_t& len, unsigned long& lastMs) {
  if (!p) return;
  while (p->available()) {
    int c = p->read();
    lastMs = millis();
    if (c == '\n' || c == '\r') {
      if (len) { buf[len] = 0; dlog(DBG_SERIAL, "[DISPATCH] Serial RX [%s]  %s\n", auxPortLabel(idx), buf); len = 0; }
    } else {
      if (len >= AUX_RX_BUF - 1) { buf[len] = 0; dlog(DBG_SERIAL, "[DISPATCH] Serial RX [%s]  %s\n", auxPortLabel(idx), buf); len = 0; }
      buf[len++] = (c >= 32 && c < 127) ? (char)c : '.';   // sanitize non-printable for the terminal
    }
  }
  if (len && (millis() - lastMs) > 60) { buf[len] = 0; dlog(DBG_SERIAL, "[DISPATCH] Serial RX [%s]  %s\n", auxPortLabel(idx), buf); len = 0; }
}
static void pollAuxSerialRx() {
  static char b3[AUX_RX_BUF], b4[AUX_RX_BUF], b5[AUX_RX_BUF];
  static uint8_t l3 = 0, l4 = 0, l5 = 0;
  static unsigned long m3 = 0, m4 = 0, m5 = 0;
  auxRxPollPort(s3, 0, b3, l3, m3);
  auxRxPollPort(s4, 1, b4, l4, m4);
  auxRxPollPort(s5, 2, b5, l5, m5);
}

void handleSerialInput() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      serialInputBuf.trim();
      if (serialInputBuf.length() == 0) { serialInputBuf = ""; return; }

      // ── "?..." CLI commands ───────────────────────────────────────────────
      // ?OTALOCAL,* / ?OTA,* (firmware OTA) and ?REC,* (record/replay) all route
      // through execCliLine (shared with the remote terminal). One command per
      // call (early return) so loop() keeps heartbeats alive between the host's
      // ACK-paced OTA chunks. Unrecognised "?" commands report back rather than
      // being silently dropped.
      if (serialInputBuf[0] == '?') {
        if (!execCliLine(serialInputBuf))
          Serial.printf("Unknown command: %s\n", serialInputBuf.c_str());
        serialInputBuf = ""; return;
      }

      if (serialInputBuf[0] == '{') {
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
        DynamicJsonDocument hdr(256);
        DeserializationError perr = deserializeJson(
            hdr, serialInputBuf,
            DeserializationOption::Filter(filter));
        if (perr != DeserializationError::Ok) {
          // Include the received-buffer length so the host can spot truncation
          // (host-sent length vs. received length mismatch → USB RX overflow).
          Serial.printf("{\"type\":\"ERROR\",\"msg\":\"JSON parse failed (%s)\",\"rxLen\":%u}\n",
                        perr.c_str(), (unsigned)serialInputBuf.length());
          serialInputBuf = ""; return;
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
          Serial.print("{\"type\":\"CONFIG\",\"data\":");
          Serial.print(rcConfigToJSON());
          Serial.println("}");

        } else if (strcmp(type,"SET_CONFIG")==0) {
          DynamicJsonDocument bigDoc(98304);
          if (deserializeJson(bigDoc, serialInputBuf) != DeserializationError::Ok) {
            Serial.println("{\"type\":\"ACK\",\"of\":\"SET_CONFIG\",\"ok\":false,\"msg\":\"parse failed\"}");
          } else if (!bigDoc.containsKey("data")) {
            Serial.println("{\"type\":\"ACK\",\"of\":\"SET_CONFIG\",\"ok\":false,\"msg\":\"missing data\"}");
          } else {
            bool ok = rcConfigFromJSON(bigDoc["data"].as<JsonObject>());
            if (ok) {
              bool saved = rcConfigSaveLFS();
              // Apply baud / SBUS-OUT changes live so HCR / MP3 / Maestro pick up
              // a new rate immediately — no reboot required. BUT the pin profile is
              // assigned only at boot (applyBoardProfile runs once in setup()), so if
              // this Save also changed boardType, re-opening ports now would use the
              // OLD board's pins; defer all pin-dependent apply to the reboot the GUI
              // already prompts for after a board change.
              if (rcConfig.boardType == appliedBoardType) {
                applySerialBauds(false);
                applySbusOut(false);     // apply a flipped SBUS-OUT toggle live
                // Re-apply Maestro easing so a changed smoothing profile / knob
                // assignment / profile value takes effect IMMEDIATELY (matching the
                // switch path) instead of waiting for a stick move — and resets any
                // channel the new profile leaves uncovered (the steady-state hot path
                // won't drive those down, so without this a re-assigned profile could
                // leave the OLD easing stuck on the servo). Cache-gated, so a save that
                // didn't touch easing is a no-op. See reapplyMaestroEasing().
                for (uint8_t mid = 1; mid <= RC_NUM_MAESTROS; mid++) reapplyMaestroEasing(mid);
                resetMaestroReleaseState();   // re-derive auto-release policy from the fresh config (no stale idle release)
              } else {
                Serial.println("{\"type\":\"INFO\",\"msg\":\"boardType changed — reboot to apply the new pin profile\"}");
              }
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
              if (saved) Serial.println("{\"type\":\"ACK\",\"of\":\"SET_CONFIG\",\"ok\":true}");
              else       Serial.println("{\"type\":\"ACK\",\"of\":\"SET_CONFIG\",\"ok\":false,\"msg\":\"applied to RAM but could not be saved to flash (LittleFS write error)\"}");
            } else {
              Serial.println("{\"type\":\"ACK\",\"of\":\"SET_CONFIG\",\"ok\":false,\"msg\":\"config apply failed\"}");
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
          calibrationActive = hdr["on"] | false;
          Serial.printf("[CALIB] action dispatch %s\n",
                        calibrationActive ? "SUPPRESSED (calibrating)" : "resumed");
          Serial.println("{\"type\":\"ACK\",\"ok\":true}");

        } else if (strcmp(type,"RESET_DEFAULTS")==0) {
          rcConfigLoadDefaults();
          resetMaestroReleaseState();   // clear stale auto-release state so defaults take effect live
          Serial.println("{\"type\":\"ACK\",\"ok\":true}");

        } else if (strcmp(type,"REBOOT")==0) {
          // ACK first so the GUI hears the reply, then restart after a brief
          // delay so the USB TX buffer drains before the reset kills it.
          Serial.println("{\"type\":\"ACK\",\"ok\":true,\"msg\":\"rebooting\"}");
          Serial.flush();
          delay(250);
          ESP.restart();

        } else if (strcmp(type,"TRIGGER")==0) {
          int mode = hdr["mode"] | 1;
          int btn  = hdr["btn"]  | 0;
          int tap  = hdr["tap"]  | 1;
          if (btn < 1 || btn > RC_NUM_THRESHOLDS || mode < 1 || mode > 3 || tap < 1 || tap > 3) {
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
            // Lazily learn each online board's friendly alias: ask once with
            // "?WHOAMI"; the {"type":"wcb_alias"} reply is cached by
            // rcTelemetry::handle(). Re-asks each poll only until cached. Skip
            // clients — they're named from their WDP advert, not ?WHOAMI.
            if (up && !client && i != selfId && wcb && wcbReady && !rcTelemetry::wcbAlias(i)[0])
              wcb->send((uint8_t)i, "?WHOAMI");
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
          Serial.println("]}");

        } else {
          Serial.println("{\"type\":\"ERROR\",\"msg\":\"unknown type\"}");
        }

      } else if (serialInputBuf[0] == '#') {
        // ── CLI commands ─────────────────────────────────────────────────────
        // Dispatched through execCliLine (shared with the remote terminal).
        execCliLine(serialInputBuf);
      }
      serialInputBuf = "";
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
    default: break;
  }
  // Low-level per-core causes (rom/rtc.h). Key S3 codes:
  //   1 = power-on   15 = RTC-WDT brown-out   16 = RTC-WDT system reset
  //   (16 = the short-WDT bootloader's 3 s watchdog fired — auto-retry)
  Serial.printf("Reset reason: %d - %s  (RTC codes core0=%d core1=%d)\n",
                (int)r, name, (int)rtc_get_reset_reason(0), (int)rtc_get_reset_reason(1));
  if (g_bootMagic != BOOT_MAGIC) {          // true power loss → fresh count
    g_bootMagic = BOOT_MAGIC;
    g_bootAttempts = 0;
  }
  g_bootAttempts++;
  Serial.printf("Boot attempts since power applied: %lu%s\n",
                (unsigned long)g_bootAttempts,
                g_bootAttempts > 1 ? "   <-- board retried/reset before this boot" : "");
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
  Serial.setRxBufferSize(4096);
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

  // WCB Client — sets STA mode + custom MAC + inits ESP-NOW.
  // No WiFi AP or web server — ESP-NOW only.  Credentials come from NVS
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
    wcb->onCommand(onWCBCommand);   // queues must be live BEFORE the callback that feeds them
    wcb->onRawPacket(naviota::otaRawPacketHook);   // OTA control/data structs (55/243 B) over the mesh
    // Create the OTA packet queue here (Core 1) instead of lazily inside the
    // Core-0 RX callback, so the very first OTA frame can't be lost to the
    // create/publish race. The lazy-create in enqueueOtaPacket() stays as a fallback.
    naviota::otaPktQueue = xQueueCreate(12, sizeof(naviota::OtaPktSlot));
    // WDP device-identity advertising (WCB_Client 1.7.0 "WDP-DA") — announce this
    // NaviCore on the mesh so every WCB auto-discovers it (it appears in ?WDP,LIST
    // / the config tool with its firmware + board, no manual labeling). The advert
    // goes out as a boot burst then re-broadcasts periodically from wcb->update()
    // (already called every loop()). Rides the ETM broadcast layer (WCB default).
    wcb->setIdentity("NaviCore", FW_VERSION,
                     rcConfig.boardType == 0 ? "NaviCore v2" : "WCB 3.2",
                     "rc sbus maestro hcr");
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
  bool recognised = execCliLine(String(m.cmd));
  if (!recognised)
    Serial.printf("Unknown command: %s\n", m.cmd);
  rtermSink.finish();                        // flush any trailing partial line
  rcSerial.disarmCapture();
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
    uint8_t tap = t.tap < 1 ? 1 : (t.tap > 3 ? 3 : t.tap);
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

  // New WCB peer detected on the mesh → fire the configured action + passive
  // alert (deferred here from the Core-0 onNeighbor callback). Cheap no-op idle.
  drainPeerEvents();

  // Remote TRIGGER — dispatch any {"type":"TRIGGER"} relayed from the config tool
  // / mesh, deferred here from the Core-0 onWCBCommand callback so it shares the
  // local matrix press's single-core dispatch path (no cross-core Maestro/cache
  // race). Cheap no-op when nothing is pending.
  drainRemoteTriggers();

  // Forget-learned-peer — run any FORGET_PEER deferred from the Core-0 Via-WCB
  // path here on Core 1 (esp_now_del_peer + NVS write). Cheap no-op when idle.
  drainForgetPeer();

  // Record/replay — drain captured events into the PSRAM clip (Core-1 sole
  // writer) and advance any active replay. Both cheap no-ops when idle.
  navirec::pollControl();   // run any Record/Play trigger deferred from dispatch (Core-1 safe)
  navirec::drain();
  navirec::checkRecordBackstop();   // auto-stop+SAVE a capture that ran past REC_MAX_MS (never lose a long take)
  navirec::replayTick();
  if (navirec::takeReplayDone()) Serial.println("[REC] ▶ playback complete");

  // WCB-network telemetry bridge — periodic rc_hb (0.5 Hz) + rc_ch (5 Hz)
  // broadcasts so the config tool's "Via WCB" mode can discover and live-
  // monitor this RC.  Event-driven rc_trig / rc_mode are emitted from
  // rcDispatch() and the mode-decode block in processSbus(), respectively.
  rcTelemetry::tick();

  // SBUS
  processSbus();
  checkDeferredTap();

  // Status LED: steady BLUE while receiving SBUS, slow-flash ORANGE when not
  updateStatusLed();

  // Pending delayed actions
  checkPendingActions();

  // USB Serial WebSerial monitor stream
  sendPWMUpdate();

  // USB Serial input
  handleSerialInput();

  // Aux-serial RX monitor — drain S3/S4/S5 and echo incoming lines under the
  // "Serial" debug chip (write-only ports otherwise; groundwork for acting on it).
  pollAuxSerialRx();

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

  // FPS counter
  trackSbusFps();

  // 1Hz SBUS live dump (#L10)
  if (sbusLiveDump && (millis() - sbusLiveDumpLastMs >= 1000)) {
    sbusLiveDumpLastMs = millis();
    dumpSbusState();
  }
}
