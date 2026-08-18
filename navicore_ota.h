#pragma once
// =============================================================================
//  navicore_ota.h — firmware OTA for NaviCore (ported from the WCB OTA core)
//
//  Brick-safe by construction: every write targets the INACTIVE OTA slot
//  (esp_ota_get_next_update_partition), the image is SHA-verified by
//  esp_ota_end BEFORE the boot pointer is switched, and any failure/timeout
//  calls esp_ota_abort (which never switches). A failed/interrupted transfer
//  always leaves the board on its current firmware. NaviCore builds with
//  PartitionScheme=min_spiffs, which already provides the ota_0/ota_1 + otadata
//  table this needs — no partition change required.
//
//  Two transports drive the SAME core (otaBegin/otaWrite/otaEnd):
//    * Direct USB   : ?OTALOCAL,* over the USB-CDC Serial (processOtaLocalCommand)
//    * ESP-NOW relay: ?OTA,* via a USB-tethered relay board (processOtaRelayCommand)
//      + target-side handlers fed by the WCB_Client raw-packet hook.
//
//  THE rule (esp_ota_* BLOCKS — esp_ota_begin erases ~1.2 MB): never run the
//  flash calls in the ESP-NOW receive (WiFi-task) callback. otaRawPacketHook()
//  only enqueues; drainOtaPackets() (loop context) runs the handlers. Same
//  pattern rc_telemetry already uses for deferred config saves.
//
//  Ported from Wireless_Communication_Board-WCB/Code/WCB/WCB_OTA.{h,cpp}.
//  NaviCore adaptations: USB-CDC Serial directly (no WCB_RemoteTerm redirect),
//  FW_VERSION for the version string, rcConfig.wcbNetwork.{deviceId,password}
//  for identity/auth, and wcb->sendRawPacket()/onRawPacket() instead of the
//  WCB's direct esp_now_send + size-router. The ESP-NOW wire format (struct
//  bytes + packet types) is IDENTICAL to the WCB so a WCB relay can update a
//  NaviCore and vice-versa.
// =============================================================================

#include <Arduino.h>
#include <esp_ota_ops.h>
#include <esp_partition.h>
#include <mbedtls/base64.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <WCB_Client.h>
#include "rc_config.h"     // rcConfig (= *g_rcConfig) — wcbNetwork.{deviceId,password}
#include "fw_version.h"    // FW_VERSION

// Provided by NaviCore.ino
extern WCB_Client* wcb;
extern bool        wcbReady;

namespace naviota {

// ── ESP-NOW relay OTA wire format — MUST match the WCB byte-for-byte ─────────
constexpr uint8_t  PACKET_TYPE_OTA_BEGIN = 20;   // relay -> target: start (size, chip family)
constexpr uint8_t  PACKET_TYPE_OTA_DATA  = 21;   // relay -> target: one firmware fragment
constexpr uint8_t  PACKET_TYPE_OTA_ACK   = 22;   // target -> relay: write cursor + status
constexpr uint8_t  PACKET_TYPE_OTA_END   = 23;   // relay -> target: finalize, verify, reboot
constexpr uint8_t  PACKET_TYPE_OTA_ABORT = 24;   // relay -> target: tear down
constexpr uint16_t OTA_ESPNOW_PAYLOAD    = 192;  // firmware bytes per OTA_DATA packet
constexpr uint8_t  OTA_ST_OK             = 0;
constexpr uint8_t  OTA_ST_ERR            = 1;

// Control packet — BEGIN / END / ABORT / ACK (packed, 55 B -> unique mesh size).
typedef struct __attribute__((packed)) {
  char     structPassword[40];
  uint8_t  packetType;
  uint8_t  targetWCB;
  uint8_t  sourceWCB;
  uint8_t  chipFamily;
  uint8_t  status;
  uint16_t sessionId;
  uint32_t imageSize;
  uint32_t ackedOffset;
} espnow_struct_ota_ctrl;

// Data packet — one firmware fragment (packed, 243 B -> unique mesh size).
typedef struct __attribute__((packed)) {
  char     structPassword[40];
  uint8_t  packetType;
  uint8_t  targetWCB;
  uint8_t  sourceWCB;
  uint16_t sessionId;
  uint16_t dataLen;
  uint32_t fragOffset;
  uint8_t  data[OTA_ESPNOW_PAYLOAD];
} espnow_struct_ota_data;

static_assert(sizeof(espnow_struct_ota_ctrl) == 55,  "ota_ctrl size changed — re-check mesh size uniqueness vs {43,204,226,230,249,252}");
static_assert(sizeof(espnow_struct_ota_data) == 243, "ota_data size changed — re-check mesh size uniqueness vs {43,204,226,230,249,252}");

// ── Single in-flight OTA session (one esp_ota handle) ───────────────────────
static struct {
  bool                   active;
  uint16_t               sessionId;
  uint32_t               imageSize;
  uint32_t               written;        // bytes committed == next expected offset
  uint8_t                chipFamily;
  esp_ota_handle_t       handle;
  const esp_partition_t *part;
  unsigned long          lastActivityMs;
  uint32_t               lastProgressPrint;
} ota = {};

constexpr unsigned long OTA_TIMEOUT_MS       = 30000UL;  // idle-session reaper
constexpr uint16_t      OTA_LOCAL_SESSION    = 1;        // fixed session id for the USB path
constexpr size_t        OTA_LOCAL_MAX_CHUNK  = 1024;     // max decoded bytes per ?OTALOCAL,DATA line
constexpr uint32_t      OTA_PROGRESS_STEP    = 65536UL;  // progress line every 64 KB

inline uint8_t otaLocalChipFamily() {
#if   defined(CONFIG_IDF_TARGET_ESP32S3)
  return 1;   // ESP32-S3 (NaviCore)
#elif defined(CONFIG_IDF_TARGET_ESP32)
  return 0;   // classic ESP32
#else
  return 0xFF;
#endif
}

inline uint32_t otaWrittenOffset() { return ota.active ? ota.written  : 0; }
inline uint16_t otaActiveSession() { return ota.active ? ota.sessionId : 0; }

// Free the handle + clear state WITHOUT switching the boot partition.
inline void otaTeardown() {
  if (ota.active && ota.handle) esp_ota_abort(ota.handle);   // safe on an in-progress handle
  ota.active = false; ota.handle = 0; ota.part = nullptr;
  ota.imageSize = 0; ota.written = 0; ota.sessionId = 0; ota.lastProgressPrint = 0;
}

inline void otaAbortSession(const char *reason) {
  if (ota.active) Serial.printf("[OTA] aborted: %s (current app intact)\n", reason ? reason : "");
  otaTeardown();
}

inline void checkOtaTimeout() {
  if (ota.active && (millis() - ota.lastActivityMs > OTA_TIMEOUT_MS))
    otaAbortSession("session timed out");
}

// ── Transport-agnostic core ─────────────────────────────────────────────────
inline bool otaBegin(uint16_t sessionId, uint32_t imageSize, uint8_t chipFamily) {
  if (ota.active) otaTeardown();   // supersede any stale session

  uint8_t mine = otaLocalChipFamily();
  if (chipFamily != mine) {
    Serial.printf("[OTA] BEGIN rejected: image chip family %u != this board %u (brick guard)\n", chipFamily, mine);
    return false;
  }
  const esp_partition_t *part = esp_ota_get_next_update_partition(NULL);
  if (!part) { Serial.println("[OTA] BEGIN rejected: no inactive OTA partition available"); return false; }
  if (imageSize == 0 || imageSize > part->size) {
    Serial.printf("[OTA] BEGIN rejected: image %u B exceeds partition '%s' (%u B)\n", imageSize, part->label, part->size);
    return false;
  }
  esp_err_t e = esp_ota_begin(part, imageSize, &ota.handle);   // ERASES the slot — BLOCKS
  if (e != ESP_OK) { Serial.printf("[OTA] esp_ota_begin failed: %s\n", esp_err_to_name(e)); ota.handle = 0; return false; }

  ota.active = true; ota.sessionId = sessionId; ota.imageSize = imageSize; ota.written = 0;
  ota.chipFamily = chipFamily; ota.part = part; ota.lastActivityMs = millis(); ota.lastProgressPrint = 0;
  Serial.printf("[OTA] BEGIN ok: session %u, %u B -> partition '%s' @0x%06x (%u B)\n",
                sessionId, imageSize, part->label, part->address, part->size);
  return true;
}

inline bool otaWrite(uint16_t sessionId, uint32_t offset, const uint8_t *data, uint16_t len) {
  if (!ota.active || sessionId != ota.sessionId) return false;
  if (offset != ota.written) return false;   // gap/dup -> caller rewinds to ota.written
  if (ota.written + len > ota.imageSize) {
    Serial.printf("[OTA] write overruns image (%u + %u > %u) — aborting\n", ota.written, len, ota.imageSize);
    otaTeardown(); return false;
  }
  esp_err_t e = esp_ota_write(ota.handle, data, len);
  if (e != ESP_OK) {
    Serial.printf("[OTA] esp_ota_write failed @%u: %s — aborting\n", offset, esp_err_to_name(e));
    otaTeardown(); return false;
  }
  ota.written += len; ota.lastActivityMs = millis();
  if (ota.written - ota.lastProgressPrint >= OTA_PROGRESS_STEP || ota.written == ota.imageSize) {
    ota.lastProgressPrint = ota.written;
    Serial.printf("[OTA] %u / %u B (%u%%)\n", ota.written, ota.imageSize, (uint32_t)((uint64_t)ota.written * 100 / ota.imageSize));
  }
  return true;
}

inline bool otaEnd(uint16_t sessionId) {
  if (!ota.active || sessionId != ota.sessionId) { Serial.println("[OTA] END: no matching active session"); return false; }
  if (ota.written != ota.imageSize) {
    Serial.printf("[OTA] END rejected: incomplete %u / %u B\n", ota.written, ota.imageSize);
    otaTeardown(); return false;
  }
  esp_err_t e = esp_ota_end(ota.handle);   // validates magic + SHA; consumes the handle
  ota.handle = 0;
  if (e != ESP_OK) {
    Serial.printf("[OTA] END verify FAILED: %s (image rejected, current app intact)\n", esp_err_to_name(e));
    ota.active = false; ota.part = nullptr; return false;
  }
  const esp_partition_t *part = ota.part;
  e = esp_ota_set_boot_partition(part);
  ota.active = false; ota.part = nullptr;
  if (e != ESP_OK) { Serial.printf("[OTA] END set_boot_partition failed: %s\n", esp_err_to_name(e)); return false; }
  Serial.printf("[OTA] END ok: verified %u B -> next boot '%s'\n", ota.imageSize, part->label);
  return true;   // caller reboots
}

inline void otaPrintStatus() {
  const esp_partition_t *run  = esp_ota_get_running_partition();
  const esp_partition_t *next = esp_ota_get_next_update_partition(NULL);
  Serial.println("---------- OTA Status ----------");
  Serial.printf("Chip:        %s (family %u)\n", ESP.getChipModel(), otaLocalChipFamily());
  Serial.printf("Firmware:    %s\n", FW_VERSION);
  if (run)  Serial.printf("Running:     '%s' @0x%06x (%u B)\n", run->label,  run->address,  run->size);
  if (next) Serial.printf("Next (OTA):  '%s' @0x%06x (%u B)\n", next->label, next->address, next->size);
  else      Serial.println("Next (OTA):  none — partition table has no spare OTA slot!");
  if (ota.active) Serial.printf("Session:     ACTIVE id=%u  %u / %u B\n", ota.sessionId, ota.written, ota.imageSize);
  else            Serial.println("Session:     idle");
  Serial.println("--------------------------------");
}

// ── Transport A: direct USB command driver (?OTALOCAL,*) ────────────────────
// Reusable scratch buffer for one decoded DATA chunk (avoids per-call malloc).
static uint8_t s_otaChunk[OTA_LOCAL_MAX_CHUNK];

inline void processOtaLocalCommand(const String &args) {
  int c1 = args.indexOf(',');
  String sub  = (c1 < 0) ? args : args.substring(0, c1);
  String rest = (c1 < 0) ? ""   : args.substring(c1 + 1);
  sub.trim(); sub.toUpperCase();

  if (sub == "STATUS" || sub.isEmpty()) { otaPrintStatus(); return; }

  if (sub == "BEGIN") {
    int p = rest.indexOf(',');
    if (p < 0) { Serial.println("[OTA] BEGIN usage: ?OTALOCAL,BEGIN,<imageSize>,<family 0|1>"); return; }
    uint32_t size   = (uint32_t) rest.substring(0, p).toInt();
    uint8_t  family = (uint8_t)  rest.substring(p + 1).toInt();
    // Ack RECEIPT before the (blocking, multi-second) inactive-slot erase inside
    // otaBegin(). Without this the host sees no response until the erase finishes,
    // and a cold full-slot erase could exceed its wait window — the classic
    // "fails first, works second" OTA timeout. The leading '\n' guards the marker
    // from gluing onto a partial telemetry line in the host's line-split reader;
    // flush() forces it onto the wire before the erase stalls the CPU.
    Serial.print("\n[OTA:BEGIN,START]\n");
    Serial.flush();
    bool ok = otaBegin(OTA_LOCAL_SESSION, size, family);
    Serial.printf("[OTA:BEGIN,%s,%u]\n", ok ? "OK" : "ERR", otaWrittenOffset());
    return;
  }

  if (sub == "DATA") {
    int p = rest.indexOf(',');
    if (p < 0) { Serial.println("[OTA] DATA usage: ?OTALOCAL,DATA,<offset>,<base64>"); return; }
    uint32_t offset = (uint32_t) rest.substring(0, p).toInt();
    String   b64    = rest.substring(p + 1); b64.trim();
    size_t outLen = 0;
    int rc = mbedtls_base64_decode(s_otaChunk, sizeof(s_otaChunk), &outLen,
                                   (const unsigned char *)b64.c_str(), b64.length());
    if (rc != 0) {
      Serial.printf("[OTA] DATA base64 error %d (chunk too big? max %u B decoded)\n", rc, (unsigned)sizeof(s_otaChunk));
      return;
    }
    if (!otaWrite(OTA_LOCAL_SESSION, offset, s_otaChunk, (uint16_t)outLen)) {
      Serial.printf("[OTA] DATA rejected at offset %u (write cursor at %u)\n", offset, otaWrittenOffset());
      Serial.printf("[OTA:NAK,%u]\n", otaWrittenOffset());   // machine-readable resync point
      return;
    }
    // Per-chunk ACK = flow control. The host waits for this before sending the
    // next chunk so the USB-CDC RX buffer can't overflow while a flash write is
    // in progress (USB has no other backpressure here).
    Serial.printf("[OTA:ACK,%u]\n", otaWrittenOffset());
    return;
  }

  if (sub == "END") {
    if (otaEnd(OTA_LOCAL_SESSION)) {
      Serial.println("[OTA:END,OK]");
      Serial.println("[OTA] rebooting into new firmware in 2s...");
      delay(2000);
      ESP.restart();
    } else {
      Serial.println("[OTA:END,ERR]");
    }
    return;
  }

  if (sub == "ABORT") { otaAbortSession("local abort command"); return; }

  Serial.printf("[OTA] unknown subcommand '%s' (use STATUS|BEGIN|DATA|END|ABORT)\n", sub.c_str());
}

// ── Transport B: ESP-NOW relay OTA ──────────────────────────────────────────
// Password + addressed-to-us gate shared by the target-side handlers.
inline bool otaPktAuth(const char *pw, uint8_t targetWCB) {
  return (strncmp(pw, rcConfig.wcbNetwork.password, sizeof(rcConfig.wcbNetwork.password) - 1) == 0) &&
         (targetWCB == rcConfig.wcbNetwork.deviceId);
}

// Build + unicast an OTA_ACK ctrl packet back to the relay (via WCB_Client).
inline void sendOtaAck(uint8_t relayWCB, uint16_t sessionId, uint8_t status, uint32_t ackedOffset) {
  if (!wcb || !wcbReady || relayWCB < 1 || relayWCB > WCB_MAX_BOARDS) return;
  espnow_struct_ota_ctrl ack; memset(&ack, 0, sizeof(ack));
  strncpy(ack.structPassword, rcConfig.wcbNetwork.password, sizeof(ack.structPassword) - 1);
  ack.packetType  = PACKET_TYPE_OTA_ACK;
  ack.targetWCB   = relayWCB;                       // addressed to the relay
  ack.sourceWCB   = rcConfig.wcbNetwork.deviceId;   // us (the target)
  ack.status      = status;
  ack.sessionId   = sessionId;
  ack.ackedOffset = ackedOffset;
  wcb->sendRawPacket(relayWCB, (const uint8_t *)&ack, sizeof(ack));
}

// ── Target side (reuses otaBegin/otaWrite/otaEnd) ───────────────────────────
inline void handleOtaBeginPacket(const uint8_t *raw) {
  espnow_struct_ota_ctrl pkt; memcpy(&pkt, raw, sizeof(pkt));
  pkt.structPassword[sizeof(pkt.structPassword) - 1] = '\0';
  if (!otaPktAuth(pkt.structPassword, pkt.targetWCB)) return;
  bool ok = otaBegin(pkt.sessionId, pkt.imageSize, pkt.chipFamily);
  sendOtaAck(pkt.sourceWCB, pkt.sessionId, ok ? OTA_ST_OK : OTA_ST_ERR, otaWrittenOffset());
}

inline void handleOtaDataPacket(const uint8_t *raw) {
  espnow_struct_ota_data pkt; memcpy(&pkt, raw, sizeof(pkt));
  pkt.structPassword[sizeof(pkt.structPassword) - 1] = '\0';
  if (!otaPktAuth(pkt.structPassword, pkt.targetWCB)) return;
  uint16_t len = pkt.dataLen > OTA_ESPNOW_PAYLOAD ? OTA_ESPNOW_PAYLOAD : pkt.dataLen;
  // Any in-session frame — even a dup or an ahead-of-cursor packet from the browser's
  // sliding window — proves the sender is still streaming, so keep the session alive.
  // otaWrite() refreshes lastActivityMs only on a SUCCESSFUL contiguous write, so
  // without this a window full of rejected-but-legitimate packets trips the 30 s idle
  // abort on a transfer that was never actually stalled. (Same keep-alive the WCB's
  // WCB_OTA.cpp carries; NaviCore's copy never got it, and the windowed sender made
  // non-contiguous arrivals routine rather than exceptional.)
  const bool inSession = (ota.active && pkt.sessionId == otaActiveSession());
  if (inSession) ota.lastActivityMs = millis();
  // Write (no-op on out-of-order/dup), then ALWAYS ack the current cursor so the
  // browser learns where we are — covers a lost DATA (cursor stalls -> resend
  // from cursor) and a lost ACK (re-acked on the resent DATA).
  otaWrite(pkt.sessionId, pkt.fragOffset, pkt.data, len);
  // Report the session's REAL state. If it has already been torn down (idle abort,
  // image overrun, flash write error) otaWrittenOffset() reads 0, and an OK+0 ACK is
  // indistinguishable from a stale duplicate to the browser: it ignores it, times out,
  // rewinds, resends, and is answered with OK+0 forever — a genuine infinite hang with
  // no error and a frozen progress bar. OTA_ST_ERR makes it fail loudly instead.
  // A frame for a DIFFERENT session is answered with its own id, which the browser
  // filters out, so this cannot disturb a live transfer.
  sendOtaAck(pkt.sourceWCB, pkt.sessionId, inSession ? OTA_ST_OK : OTA_ST_ERR,
             otaWrittenOffset());
}

inline void handleOtaEndPacket(const uint8_t *raw) {
  espnow_struct_ota_ctrl pkt; memcpy(&pkt, raw, sizeof(pkt));
  pkt.structPassword[sizeof(pkt.structPassword) - 1] = '\0';
  if (!otaPktAuth(pkt.structPassword, pkt.targetWCB)) return;
  bool ok = otaEnd(pkt.sessionId);
  // otaEnd() has already cleared ota.active on EVERY path it can return through (no
  // matching session, incomplete, SHA failure, set_boot failure, success), so
  // otaWrittenOffset() below is necessarily 0. That zero is the ONLY thing separating a
  // genuine END answer from a DATA cursor ACK still in flight — same struct, same status,
  // and a tail rewind routinely puts extra cursor ACKs on the air after the browser has
  // stopped reading them. Do not "improve" this to report ota.imageSize/ota.written, and
  // do not move the ota.active clear to after the ACK: either silently removes the
  // discriminator and a lost/failed END gets reported as a verified update. The WCB
  // firmware's WCB_OTA.cpp holds the same invariant, so both target types look alike.
  sendOtaAck(pkt.sourceWCB, pkt.sessionId, ok ? OTA_ST_OK : OTA_ST_ERR, otaWrittenOffset());
  if (ok) {
    Serial.println("[OTA] remote update verified — rebooting into new firmware...");
    // Re-send the success ACK a few times: a single dropped ESP-NOW frame here
    // would make the browser report the OTA FAILED even though the image is
    // verified and the boot slot already switched. Cheap redundancy before the
    // radio drops at restart. (Runs in loop()/Core 1 where OTA packets are drained.)
    for (int r = 0; r < 3; r++) { delay(80); sendOtaAck(pkt.sourceWCB, pkt.sessionId, OTA_ST_OK, otaWrittenOffset()); }
    delay(150);   // let the last ACK actually transmit before the radio drops
    ESP.restart();
  }
}

inline void handleOtaAbortPacket(const uint8_t *raw) {
  espnow_struct_ota_ctrl pkt; memcpy(&pkt, raw, sizeof(pkt));
  pkt.structPassword[sizeof(pkt.structPassword) - 1] = '\0';
  if (!otaPktAuth(pkt.structPassword, pkt.targetWCB)) return;
  otaAbortSession("remote abort");
  sendOtaAck(pkt.sourceWCB, pkt.sessionId, OTA_ST_OK, 0);
}

// ── Relay side ──────────────────────────────────────────────────────────────
// A relay (this board, USB-tethered) received the target's OTA_ACK — surface it
// to USB for the browser as "[OTA:ACK,<src>,<session>,<offset>,<status>]".
inline void handleOtaAckRelay(const uint8_t *raw) {
  espnow_struct_ota_ctrl pkt; memcpy(&pkt, raw, sizeof(pkt));
  pkt.structPassword[sizeof(pkt.structPassword) - 1] = '\0';
  if (strncmp(pkt.structPassword, rcConfig.wcbNetwork.password, sizeof(rcConfig.wcbNetwork.password) - 1) != 0) return;
  if (pkt.targetWCB != rcConfig.wcbNetwork.deviceId) return;   // ACK addressed to us (the relay)
  Serial.printf("[OTA:ACK,%u,%u,%lu,%u]\n", pkt.sourceWCB, pkt.sessionId, (unsigned long)pkt.ackedOffset, pkt.status);
}

// "?OTA,<SUB>,<target>,<session>,..." — build the OTA packet and unicast to target.
//   BEGIN,<target>,<session>,<size>,<family>
//   DATA,<target>,<session>,<offset>,<base64(<=192 B)>
//   END,<target>,<session>      ABORT,<target>,<session>
inline void processOtaRelayCommand(const String &args) {
  int c1 = args.indexOf(',');
  String sub  = (c1 < 0) ? args : args.substring(0, c1);
  String rest = (c1 < 0) ? ""   : args.substring(c1 + 1);
  sub.trim(); sub.toUpperCase();

  int c2 = rest.indexOf(',');
  uint8_t  target  = (uint8_t)((c2 < 0 ? rest : rest.substring(0, c2)).toInt());
  String   r2      = (c2 < 0) ? "" : rest.substring(c2 + 1);
  int c3 = r2.indexOf(',');
  uint16_t session = (uint16_t)((c3 < 0 ? r2 : r2.substring(0, c3)).toInt());
  String   r3      = (c3 < 0) ? "" : r2.substring(c3 + 1);

  if (!wcb || !wcbReady) { Serial.println("[OTA] relay: WCB not ready"); return; }
  if (target < 1 || target > WCB_MAX_BOARDS) { Serial.printf("[OTA] relay: invalid target %u\n", target); return; }

  if (sub == "BEGIN") {
    int p = r3.indexOf(',');
    uint32_t size   = (uint32_t)((p < 0 ? r3 : r3.substring(0, p)).toInt());
    uint8_t  family = (uint8_t) (p < 0 ? 0 : r3.substring(p + 1).toInt());
    espnow_struct_ota_ctrl pkt; memset(&pkt, 0, sizeof(pkt));
    strncpy(pkt.structPassword, rcConfig.wcbNetwork.password, sizeof(pkt.structPassword) - 1);
    pkt.packetType = PACKET_TYPE_OTA_BEGIN; pkt.targetWCB = target;
    pkt.sourceWCB  = rcConfig.wcbNetwork.deviceId; pkt.chipFamily = family;
    pkt.sessionId  = session; pkt.imageSize = size;
    wcb->sendRawPacket(target, (const uint8_t *)&pkt, sizeof(pkt));
    return;
  }

  if (sub == "DATA") {
    int p = r3.indexOf(',');
    if (p < 0) { Serial.println("[OTA] relay DATA: ?OTA,DATA,<t>,<s>,<offset>,<b64>"); return; }
    uint32_t offset = (uint32_t)r3.substring(0, p).toInt();
    String   b64    = r3.substring(p + 1); b64.trim();
    espnow_struct_ota_data pkt; memset(&pkt, 0, sizeof(pkt));
    strncpy(pkt.structPassword, rcConfig.wcbNetwork.password, sizeof(pkt.structPassword) - 1);
    pkt.packetType = PACKET_TYPE_OTA_DATA; pkt.targetWCB = target;
    pkt.sourceWCB  = rcConfig.wcbNetwork.deviceId; pkt.sessionId = session; pkt.fragOffset = offset;
    size_t outLen = 0;
    int rc = mbedtls_base64_decode(pkt.data, sizeof(pkt.data), &outLen, (const unsigned char *)b64.c_str(), b64.length());
    if (rc != 0) { Serial.printf("[OTA] relay DATA base64 error %d\n", rc); return; }
    pkt.dataLen = (uint16_t)outLen;
    wcb->sendRawPacket(target, (const uint8_t *)&pkt, sizeof(pkt));
    return;
  }

  if (sub == "END" || sub == "ABORT") {
    espnow_struct_ota_ctrl pkt; memset(&pkt, 0, sizeof(pkt));
    strncpy(pkt.structPassword, rcConfig.wcbNetwork.password, sizeof(pkt.structPassword) - 1);
    pkt.packetType = (sub == "END") ? PACKET_TYPE_OTA_END : PACKET_TYPE_OTA_ABORT;
    pkt.targetWCB  = target; pkt.sourceWCB = rcConfig.wcbNetwork.deviceId; pkt.sessionId = session;
    wcb->sendRawPacket(target, (const uint8_t *)&pkt, sizeof(pkt));
    return;
  }

  Serial.printf("[OTA] relay: unknown subcommand '%s'\n", sub.c_str());
}

// ── Deferred target-side processing — keep esp_ota flash writes OUT of the
//    WiFi receive callback (they block; running them there stalls ESP-NOW and
//    the session times out mid-stream). The raw hook enqueues; drainOtaPackets()
//    (loop context) runs the handlers. Mirrors rc_telemetry's deferred queue.
typedef struct { uint8_t buf[244]; uint16_t len; } OtaPktSlot;   // 244 >= 243 (ota_data)
static QueueHandle_t otaPktQueue = nullptr;

inline void enqueueOtaPacket(const uint8_t *raw, uint16_t len) {
  if (len == 0 || len > 244) return;
  // Plain guard, NOT a lazy create. setup() creates this queue on Core 1 before
  // onRawPacket is registered, so by the time this runs it exists. Creating it
  // here would race that: this runs on Core 0, and a queue made here can be
  // overwritten by the Core-1 create, leaking it and everything already in it.
  if (!otaPktQueue) return;
  OtaPktSlot slot; memcpy(slot.buf, raw, len); slot.len = len;
  xQueueSend(otaPktQueue, &slot, 0);   // drop if full — the browser resends from its cursor
}

inline void drainOtaPackets() {
  if (!otaPktQueue) return;
  OtaPktSlot slot;
  while (xQueueReceive(otaPktQueue, &slot, 0) == pdTRUE) {
    if (slot.len == sizeof(espnow_struct_ota_data)) {
      handleOtaDataPacket(slot.buf);
    } else if (slot.len == sizeof(espnow_struct_ota_ctrl)) {
      uint8_t pt = ((espnow_struct_ota_ctrl *)slot.buf)->packetType;
      if      (pt == PACKET_TYPE_OTA_BEGIN) handleOtaBeginPacket(slot.buf);
      else if (pt == PACKET_TYPE_OTA_END)   handleOtaEndPacket(slot.buf);
      else if (pt == PACKET_TYPE_OTA_ABORT) handleOtaAbortPacket(slot.buf);
    }
  }
}

// WCB_Client raw-packet hook — runs in the WiFi receive task. Route by size: the
// relay-side ACK is lightweight (a printf) and stays inline; target-side
// BEGIN/DATA/END/ABORT are DEFERRED to drainOtaPackets() in loop() because they
// do blocking flash writes. Register via wcb->onRawPacket() in setup().
inline void otaRawPacketHook(const uint8_t * /*mac*/, const uint8_t *data, int len) {
  if (len == (int)sizeof(espnow_struct_ota_ctrl)) {
    uint8_t pt = ((const espnow_struct_ota_ctrl *)data)->packetType;
    if (pt == PACKET_TYPE_OTA_ACK) handleOtaAckRelay(data);            // relay side, inline
    else                           enqueueOtaPacket(data, (uint16_t)len);  // target side, deferred
  } else if (len == (int)sizeof(espnow_struct_ota_data)) {
    enqueueOtaPacket(data, (uint16_t)len);                             // target side, deferred
  }
}

}  // namespace naviota
