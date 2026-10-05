#pragma once
#include <Arduino.h>
#include <WCB_Client.h>
#include "rc_config.h"

// ─────────────────────────────────────────────────────────────────────────────
//  NaviCore remote terminal (RTERM) — TARGET side only
// ─────────────────────────────────────────────────────────────────────────────
// Mirrors NaviCore's CLI output back over the WCB ESP-NOW bridge using the WCB
// firmware's RTERM wire format, byte-for-byte.  An UNMODIFIED bridge WCB
// already routes a 204-byte type-7 packet to its USB as
// "[TERM:<sourceWCB>]<text>\n", so the config-tool terminal can display
// NaviCore's CLI with NO change to the WCB firmware.
//
// NaviCore only ever SENDS these packets (it never relays or receives them),
// so none of the WCB's relay-side queue/drain/[TERM] code is needed here — only
// the wire struct and a Print sink that emits one packet per captured line.
namespace navirterm {

static const uint8_t PACKET_TYPE_REMOTE_TERM = 7;    // matches WCB WCB_RemoteTerm.h
static const uint8_t RTERM_TEXT_SIZE         = 160;  // max chars per line/packet

#pragma pack(push, 1)
struct espnow_struct_remote_term {
  char    structPassword[40];
  uint8_t packetType;   // = PACKET_TYPE_REMOTE_TERM
  uint8_t sourceWCB;    // board that produced the line (NaviCore's deviceId)
  uint8_t destWCB;      // relay (bridge) it is addressed to
  uint8_t textLen;      // <= RTERM_TEXT_SIZE
  char    text[RTERM_TEXT_SIZE];
};
#pragma pack(pop)
static_assert(sizeof(espnow_struct_remote_term) == 204,
              "espnow_struct_remote_term must be 204 B to match WCB RTERM");

// A Print sink that buffers bytes into one line and emits a 204-byte
// REMOTE_TERM packet per line (on '\n' or at RTERM_TEXT_SIZE) to the relay via
// WCB_Client::sendRawPacket().  Armed with the relay WCB id for the duration of
// one CLI command, then finish()ed to flush any trailing partial line.
class CaptureSink : public Print {
 public:
  void begin(WCB_Client* wcb, uint8_t relayWCB) {
    _wcb = wcb;
    _relay = relayWCB;
    _len = 0;
  }

  size_t write(uint8_t c) override {
    if (c == '\r') return 1;                 // strip CR; the relay re-adds one '\n'
    if (c == '\n') { flushLine(); return 1; }
    if (_len < RTERM_TEXT_SIZE) _line[_len++] = (char)c;
    if (_len >= RTERM_TEXT_SIZE) flushLine(); // hard-wrap over-long lines
    return 1;
  }
  size_t write(const uint8_t* buf, size_t n) override {
    for (size_t i = 0; i < n; i++) write(buf[i]);
    return n;
  }
  using Print::write;

  // Emit a trailing partial (non-newline-terminated) line, e.g. a prompt.
  void finish() { if (_len) flushLine(); }

 private:
  void flushLine() {
    if (_wcb && _relay) {
      espnow_struct_remote_term pkt;
      memset(&pkt, 0, sizeof(pkt));
      strncpy(pkt.structPassword, g_meshPasswordBoot,
              sizeof(pkt.structPassword) - 1);
      pkt.packetType = PACKET_TYPE_REMOTE_TERM;
      pkt.sourceWCB  = rcConfig.wcbNetwork.deviceId;
      pkt.destWCB    = _relay;
      pkt.textLen    = (_len > RTERM_TEXT_SIZE) ? RTERM_TEXT_SIZE : _len;
      memcpy(pkt.text, _line, pkt.textLen);
      // Adaptive pacing: USB-CDC writes aren't UART-rate-limited the way the
      // WCB's UART0 output is, so a multi-line dump can outrun the ESP-NOW TX
      // queue.  Retry briefly on backpressure (sendRawPacket returns false when
      // ESP-NOW won't accept the frame) instead of blindly delaying.
      for (int attempt = 0; attempt < 4; attempt++) {
        if (_wcb->sendRawPacket(_relay, (const uint8_t*)&pkt, sizeof(pkt))) break;
        delay(2);
      }
    }
    _len = 0;
  }

  WCB_Client* _wcb   = nullptr;
  uint8_t     _relay = 0;
  char        _line[RTERM_TEXT_SIZE];
  uint16_t    _len   = 0;
};

}  // namespace navirterm
