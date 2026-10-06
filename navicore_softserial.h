#pragma once
// =============================================================================
//  navicore_softserial.h — S4/S5: EspSoftwareSerial RX + RMT (hardware-timed) TX
//
//  S4 and S5 have no UART (S3 takes UART0, SBUS UART1, the Maestro UART2). The
//  stock EspSoftwareSerial 8.1.0 did both directions in software, and its TX
//  bit-bangs every bit against the CPU clock with interrupts ENABLED (the
//  library's default, SoftwareSerial.cpp:92): an interrupt landing inside a byte
//  stretches the bit it lands in, so the receiver reads a wrong byte or loses one.
//  Measured on the WCB repo's HIL bench (ncwire.soft_tx_integrity, run
//  20261005-221308): 95-character lines under SBUS and mesh load, S4 23/100 exact
//  and S5 96/100. The library's cure, enableIntTx(false), masks the core's
//  interrupts for most of every byte, which blinds soft RX on both ports and
//  stalls everything else on Core 1 — the WCB tried it and backed out (WCB
//  CLAUDE.md rule 13). So NcSoftSerial keeps the library for RECEIVE and hands
//  TRANSMIT to an RMT channel: the peripheral clocks the start, data and stop
//  bits itself, so no interrupt can bend them and none is ever masked. This is
//  the WCB's WcbSoftSerial (WCB_SoftSerial.{h,cpp}), less its ;P pin-borrowing.
//
//  write() returns once the bytes are on the wire, as the bit-banged write did,
//  but it SLEEPS on the driver instead of spinning, so the loop task gives the
//  CPU up meanwhile. auxTxPump still hands a port only a few bytes a loop() pass
//  (its budget is time, and the time is the same).
//
//  Receive keeps two guards from the WCB (rule 13):
//    • The GPIO ISR service is installed at LEVEL 3 in setup(), before the first
//      begin() attaches an RX interrupt, so an edge pre-empts the level-1 UART,
//      RMT and USB interrupts. The library decodes bits from the time each edge's
//      interrupt starts, so a late edge is a wrong bit: HIL rx_monitor_bcast_in
//      saw '5' (0x35) on S5 read as '4', bit 0 lost to a late rising edge.
//    • available()/read()/peek() suspend the scheduler around the library's
//      rxBits(), which tests "ISR edge buffer empty" and only then reads micros()
//      (SoftwareSerial.cpp:483). A task switch between the two lets edges pile up
//      unseen while the check still passes, and the library then injects a faux
//      stop bit mid-byte (:486). ISRs keep running, so no edge is lost.
//
//  Channels: the ESP32-S3 has 4 TX-capable RMT channels. The status NeoPixel
//  takes one (Adafruit_NeoPixel, at its first show()) and S4/S5 one each. If a
//  channel cannot be had, or a port is begun with anything but 8N1 non-inverted,
//  begin() falls back to the library's bit-banged TX for that port and says so.
// =============================================================================
#include <Arduino.h>
#include <SoftwareSerial.h>
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "soc/soc_caps.h"

class NcSoftSerial : public SoftwareSerial {
 public:
  NcSoftSerial() : SoftwareSerial() {}

  // Same shape as the library's begin() auxBegin() calls; hides it (non-virtual), so
  // callers must hold an NcSoftSerial* — never a SoftwareSerial* — to reach this one.
  void begin(uint32_t baud, EspSoftwareSerial::Config config, int8_t rxPin, int8_t txPin,
             bool invert, int bufCapacity = 64) {
    stopRmt();
    _txPin = txPin;
    if (config == SWSERIAL_8N1 && !invert && txPin >= 0 && startRmt(baud, txPin)) {
      SoftwareSerial::begin(baud, config, rxPin, -1, invert, bufCapacity);   // RX only
    } else {
      if (txPin >= 0)
        Serial.printf("[AUX] TX GPIO%d: no RMT channel - TX falls back to bit-banging (interrupts can stretch its bits)\n",
                      txPin);
      SoftwareSerial::begin(baud, config, rxPin, txPin, invert, bufCapacity);
    }
  }

  void end() {
    SoftwareSerial::end();
    stopRmt();
  }

  using SoftwareSerial::write;
  size_t write(uint8_t byte) override { return write(&byte, 1); }

  size_t write(const uint8_t* buffer, size_t size) override {
    if (!_ch) return SoftwareSerial::write(buffer, size);   // bit-banged fallback
    if (size == 0) return 0;
    xSemaphoreTake(_lock, portMAX_DELAY);
    rmt_transmit_config_t tc = {};
    tc.loop_count      = 0;
    tc.flags.eot_level = 1;            // leave the line idle HIGH
    size_t sent = 0;
    while (sent < size) {
      const size_t n = (size - sent) < _chunk ? (size - sent) : _chunk;
      if (rmt_transmit(_ch, _enc, buffer + sent, n, &tc) != ESP_OK) break;
      // Wait for the wire, not just the queue: the encoder reads the buffer while it
      // transmits, and the caller may free or reuse it the moment we return (a
      // temporary String's payload). Bounded by n * 10 / baud.
      rmt_tx_wait_all_done(_ch, -1);
      sent += n;
    }
    xSemaphoreGive(_lock);
    return sent;
  }

  int availableForWrite() override {
    if (!_ch) return SoftwareSerial::availableForWrite();
    return 256;   // RMT TX never refuses: write() transmits and waits
  }

  // Receive: the library's rxBits() race - see the header comment. Loop task only (every
  // aux-port reader is); none of these block, so the scheduler is held for microseconds.
  using SoftwareSerial::read;   // keep read(buf, n) visible
  int available() override { vTaskSuspendAll(); const int n = SoftwareSerial::available(); xTaskResumeAll(); return n; }
  int read() override      { vTaskSuspendAll(); const int c = SoftwareSerial::read();      xTaskResumeAll(); return c; }
  int peek() override      { vTaskSuspendAll(); const int c = SoftwareSerial::peek();      xTaskResumeAll(); return c; }

  bool rmtTx() const { return _ch != nullptr; }   // false = bit-banged fallback (or not begun)

 private:
  static constexpr uint32_t RMT_HALF_MAX = 32767;   // rmt_symbol_word_t duration fields are 15 bits

  // 10 MHz keeps every bit edge within 0.1 us (0.6 % of a bit at 115200), and a whole
  // 10-bit frame fits one 15-bit duration down to 4800 baud. Below that, 1 MHz: a bit is
  // >= 833 ticks, and the longest frame (1200 baud, sanBaud's floor) splits across a few
  // halves in byteHalves().
  static uint32_t resolutionFor(uint32_t baud) { return baud >= 4800 ? 10000000UL : 1000000UL; }

  bool startRmt(uint32_t baud, int8_t txPin) {
    if (baud == 0) return false;
    const uint32_t res = resolutionFor(baud);
    rmt_tx_channel_config_t cc = {};
    cc.gpio_num          = (gpio_num_t)txPin;
    cc.clk_src           = RMT_CLK_SRC_DEFAULT;
    cc.resolution_hz     = res;
    cc.mem_block_symbols = SOC_RMT_MEM_WORDS_PER_CHANNEL;   // one block: 48 symbols on the S3
    cc.trans_queue_depth = 2;
    cc.flags.init_level  = 1;   // UART idle HIGH - but not driven until a first transaction: primeIdleHigh()
    if (rmt_new_tx_channel(&cc, &_ch) != ESP_OK) { _ch = nullptr; return false; }
    rmt_simple_encoder_config_t ec = {};
    ec.callback       = encodeCb;
    ec.arg            = this;
    ec.min_chunk_size = 8;      // byteHalves() never needs more than 7 symbols for one byte
    if (rmt_new_simple_encoder(&ec, &_enc) != ESP_OK || rmt_enable(_ch) != ESP_OK) {
      if (_enc) { rmt_del_encoder(_enc); _enc = nullptr; }
      rmt_del_channel(_ch);
      _ch = nullptr;
      return false;
    }
    // Bit boundaries measured from the start of the frame, rounded once each, so
    // rounding never accumulates across the ten bits.
    for (int k = 0; k <= 10; k++)
      _edge[k] = (uint32_t)(((uint64_t)k * res + baud / 2) / baud);
    // Bytes per transaction: few enough that the whole chunk is encoded into channel
    // memory up front, when rmt_transmit() starts it. The refill interrupt is then never
    // needed mid-frame - and it matters, because that ISR is not IRAM-safe: during a
    // flash write (every config save) it is held off, and a starved channel would put
    // stale symbols on the wire. A late "done" interrupt only stretches the idle-high
    // gap between chunks, which any UART receiver accepts. 2 symbols spare for the
    // driver's end marker.
    const int maxSymPerByte = (res == 10000000UL) ? 5 : 7;
    _chunk = (SOC_RMT_MEM_WORDS_PER_CHANNEL - 2) / maxSymPerByte;
    if (_chunk < 1) _chunk = 1;
    if (!_lock) _lock = xSemaphoreCreateMutex();
    primeIdleHigh();
    return true;
  }

  // On the ESP32-S3 a new TX channel does not drive init_level onto the pin until its
  // first transaction ends: the line sat LOW from begin() until the port's first write,
  // so that first line began out of frame and a receiver lost its first bytes (found on
  // the WCB's HIL bench, a WCB 3.2 S3/S4: "BOOT0S3ABCDEFGH" arrived as A8 EA 05
  // "ABCDEFGH"). One symbol that is high for two bit times ends with the line held at
  // eot_level - idle HIGH - from begin(). It has no start bit, so no receiver sees a
  // byte; where the line was already high nothing changes at all.
  void primeIdleHigh() {
    rmt_copy_encoder_config_t cfg = {};
    rmt_encoder_handle_t copy = nullptr;
    if (rmt_new_copy_encoder(&cfg, &copy) != ESP_OK) return;
    rmt_symbol_word_t idle = {};
    idle.duration0 = (uint16_t)_edge[1];
    idle.level0    = 1;
    idle.duration1 = (uint16_t)_edge[1];
    idle.level1    = 1;
    rmt_transmit_config_t tc = {};
    tc.loop_count      = 0;
    tc.flags.eot_level = 1;
    if (rmt_transmit(_ch, copy, &idle, sizeof(idle), &tc) == ESP_OK)
      rmt_tx_wait_all_done(_ch, 100);
    rmt_del_encoder(copy);
  }

  void stopRmt() {
    if (!_ch) return;
    if (_lock) xSemaphoreTake(_lock, portMAX_DELAY);   // never pull the channel from under a write
    rmt_disable(_ch);
    if (_enc) { rmt_del_encoder(_enc); _enc = nullptr; }
    rmt_del_channel(_ch);
    _ch = nullptr;
    if (_lock) xSemaphoreGive(_lock);
  }

  // One 8N1 frame as (duration, level) halves: start 0, data LSB first, stop 1. Runs of
  // equal bits merge; a run longer than a 15-bit duration splits. Always returns an EVEN
  // count (a symbol word holds two halves) by splitting the stop-bit run when needed - so
  // every byte starts on a symbol boundary and the callback can encode whole bytes.
  // Worst case 14 halves (7 symbols).
  int byteHalves(uint8_t b, uint16_t* dur, uint8_t* lvl) const {
    uint8_t bits[10];
    bits[0] = 0;
    for (int i = 0; i < 8; i++) bits[i + 1] = (b >> i) & 1;
    bits[9] = 1;
    int h = 0;
    uint8_t  level  = bits[0];
    uint32_t tStart = 0;
    for (int k = 1; k <= 10; k++) {
      if (k < 10 && bits[k] == level) continue;
      uint32_t d = _edge[k] - tStart;
      while (d > 0) {
        const uint32_t part = d > RMT_HALF_MAX ? RMT_HALF_MAX : d;
        dur[h] = (uint16_t)part;
        lvl[h] = level;
        h++;
        d -= part;
      }
      if (k < 10) { level = bits[k]; tStart = _edge[k]; }
    }
    if (h & 1) {                     // odd: split the stop-bit half (>= 8 ticks at any baud here)
      const uint16_t d = dur[h - 1];
      dur[h - 1] = d - d / 2;
      dur[h]     = d / 2;
      lvl[h]     = 1;
      h++;
    }
    return h;
  }

  // Encodes whole bytes into the free symbol space; 0 asks to be called again with more
  // room. symbolsWritten == 0 marks the start of a transaction. write() sizes each
  // transaction to fit the channel memory, so this completes in the first call, from
  // rmt_transmit() in the caller's task; the RMT ISR path (refill) is only a fallback and
  // must stay ISR-safe regardless.
  static size_t encodeCb(const void* data, size_t size, size_t symbolsWritten, size_t symbolsFree,
                         rmt_symbol_word_t* out, bool* done, void* arg) {
    NcSoftSerial* self = (NcSoftSerial*)arg;
    if (symbolsWritten == 0) self->_encPos = 0;
    const uint8_t* bytes = (const uint8_t*)data;
    size_t n = 0;
    uint16_t dur[16];
    uint8_t  lvl[16];
    while (self->_encPos < size) {
      const int h = self->byteHalves(bytes[self->_encPos], dur, lvl);
      if (n + (size_t)(h / 2) > symbolsFree) break;
      for (int i = 0; i < h; i += 2) {
        out[n].duration0 = dur[i];     out[n].level0 = lvl[i];
        out[n].duration1 = dur[i + 1]; out[n].level1 = lvl[i + 1];
        n++;
      }
      self->_encPos++;
    }
    if (self->_encPos >= size) *done = true;
    return n;
  }

  int8_t               _txPin  = -1;
  rmt_channel_handle_t _ch     = nullptr;
  rmt_encoder_handle_t _enc    = nullptr;
  SemaphoreHandle_t    _lock   = nullptr;   // one transaction in flight per port
  uint32_t             _edge[11] = {};      // tick offset of each bit boundary within a byte
  volatile size_t      _encPos = 0;         // next byte to encode (encoder callback)
  size_t               _chunk  = 8;         // bytes per RMT transaction - see startRmt()
};
