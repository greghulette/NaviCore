#pragma once
#include <Arduino.h>
#include <HWCDC.h>

// This tee wraps HWCDCSerial (native USB-CDC), which IS the core's `Serial`
// only on a USB-CDC-on-boot build. NaviCore always is (FQBN
// USBMode=hwcdc,CDCOnBoot=cdc — required for the OTG port), so fail loudly here
// rather than silently retarget every Serial.* to the wrong port on a non-CDC
// build.
#if !ARDUINO_USB_CDC_ON_BOOT
#error "rc_serial.h requires USB CDC on boot (build with USBMode=hwcdc, CDCOnBoot=cdc)."
#endif

// ─────────────────────────────────────────────────────────────────────────────
//  RcSerial — USB-CDC tee for the remote-terminal feature
// ─────────────────────────────────────────────────────────────────────────────
// NaviCore's CLI (#L diagnostics, ?OTALOCAL,*, ?OTA,*) prints to the global
// `Serial`.  To mirror that output back over the WCB ESP-NOW bridge — so the
// config tool's terminal can show NaviCore's CLI alongside the WCB's — we
// interpose a thin Stream that ALWAYS forwards to the real USB-CDC
// (HWCDCSerial) and ALSO copies writes to a capture sink while a remote
// command is running.  This mirrors the WCB firmware's `class WCBSerial` +
// `#define Serial` trick, adapted for the ESP32-S3 hardware USB-CDC.
//
// Capturing at the Serial layer means EVERY existing Serial.printf in the CLI
// (and in navicore_ota.h) is mirrored automatically — no Print& has to be
// threaded through the dozens of print sites.
//
// Include order matters: this header MUST come AFTER the Arduino core headers
// (so `Serial` is already the `#define Serial HWCDCSerial` from
// HardwareSerial.h) and BEFORE any project header that prints, so their
// Serial.* calls route through the tee.
class RcSerial : public Stream {
 public:
  // ── HWCDC-specific pass-throughs used by NaviCore setup() ────────────────
  void begin(unsigned long baud)   { HWCDCSerial.begin(baud); }
  void setRxBufferSize(size_t s)   { HWCDCSerial.setRxBufferSize(s); }
  void setTxBufferSize(size_t s)   { HWCDCSerial.setTxBufferSize(s); _txRing = s; if (!_lock) _lock = xSemaphoreCreateMutex(); }
  void setTxTimeoutMs(uint32_t ms) { HWCDCSerial.setTxTimeoutMs(ms); }

  // ── Stream interface → real USB ──────────────────────────────────────────
  int  available() override { return HWCDCSerial.available(); }
  int  read()      override { return HWCDCSerial.read(); }
  int  peek()      override { return HWCDCSerial.peek(); }
  // flush() must reach the CAPTURE too, not just USB.
  //
  // Callers use it to force bytes out before something blocks the CPU for seconds
  // — the OTA erase is the live example, and its comment names the failure it
  // prevents ("the classic 'fails first, works second' OTA timeout"). A capture
  // sink that buffers and relies on loop() to drain never gets that chance, because
  // loop() is exactly what is about to stall. So the marker sat unsent, the host
  // waited, and OTA timed out on every attempt over a non-USB transport.
  void flush() override {
    if (_capFlush) _capFlush();
    HWCDCSerial.flush();
  }
  // CRITICAL: must forward to HWCDC. Print's default returns 0, which would
  // make vlogf()'s `availableForWrite() >= n` guard ALWAYS false and silently
  // drop every hot-path log line.
  int availableForWrite() override { return HWCDCSerial.availableForWrite(); }

  // ── Print interface → USB pass-through ALWAYS, plus capture when armed ────
  // Capture is gated to the core that armed it so a Core-0 (WiFi/ESP-NOW)
  // print landing mid-command can't race the single-threaded sink buffer.
  size_t write(uint8_t c) override {
    Print* cap = _cap;
    if (cap && xPortGetCoreID() == _capCore) cap->write(c);
    return HWCDCSerial.write(c);
  }
  // A multi-byte write (each Serial.printf/println arrives as one) goes to USB whole or not at all. HWCDC writes what
  // fits its TX ring and gives up on the rest after the TX timeout (50 ms, setup()), so under a burst a line was cut
  // part-way and the next task's line followed straight on: '[RC] Unknown inboun[WCB RX] from WCB1: ...' with the rest
  // lost (HIL navicore.mesh_json_declined, full run 20261013-021405). Now writers take turns (a mutex), each waits up to
  // TX_WAIT_MS for room for the whole line, and a line that still does not fit is dropped whole and counted
  // (droppedLines, reported by loop()). A write larger than the ring, or with no host attached, passes straight through.
  size_t write(const uint8_t* buf, size_t n) override {
    Print* cap = _cap;
    if (cap && xPortGetCoreID() == _capCore) cap->write(buf, n);
    if (n <= 1 || n > _txRing || !(bool)HWCDCSerial || !_lock) return HWCDCSerial.write(buf, n);
    if (xSemaphoreTake(_lock, pdMS_TO_TICKS(TX_WAIT_MS)) != pdTRUE) { _dropped++; return 0; }
    const uint32_t t0 = millis();
    while ((size_t)HWCDCSerial.availableForWrite() < n && (uint32_t)(millis() - t0) < TX_WAIT_MS) vTaskDelay(1);
    size_t w = 0;
    if ((size_t)HWCDCSerial.availableForWrite() >= n) w = HWCDCSerial.write(buf, n);
    else _dropped++;
    xSemaphoreGive(_lock);
    return w;
  }
  // Lines dropped whole since the last call (and reset). loop() reports them.
  uint32_t takeDroppedLines() { uint32_t d = _dropped; _dropped = 0; return d; }
  using Print::write;
  operator bool() { return (bool)HWCDCSerial; }

  // ── Capture control (driven by the remote-CLI drain in loop()) ───────────
  // `flushFn` is optional and only meaningful for a sink that buffers (the
  // WebSocket one does; the RTERM one sends per line and passes nullptr).
  void armCapture(Print* sink, void (*flushFn)() = nullptr) {
    _capCore = xPortGetCoreID(); _cap = sink; _capFlush = flushFn;
  }
  void disarmCapture()         { _cap = nullptr; _capFlush = nullptr; }
  // True while a remote (WCB-relayed) command is running — lets long CLI output
  // (e.g. ?REC,EDITLOAD) pick RTERM pacing vs full-speed USB.
  bool captureArmed() const    { return _cap != nullptr; }


 private:
  Print* _cap      = nullptr;
  void (*_capFlush)() = nullptr;   // push a buffering sink before the CPU stalls
  int    _capCore = -1;
  static constexpr uint32_t TX_WAIT_MS = 50;   // as setup()'s setTxTimeoutMs
  SemaphoreHandle_t _lock   = nullptr;          // created by setTxBufferSize() in setup()
  size_t            _txRing = 0;                // the TX ring's size; 0 until set (writes pass through)
  volatile uint32_t _dropped = 0;
};

extern RcSerial rcSerial;

#undef Serial
#define Serial rcSerial
