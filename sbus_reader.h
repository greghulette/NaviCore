// ─────────────────────────────────────────────────────────────────────────────
//  sbus_reader.h — Custom SBUS frame parser supporting BOTH standard 16-channel
//  AND FrSky's 24-channel "SBUS-24" extended frame format.
//
//  The two variants share:
//    • Header byte 0x0F
//    • Footer byte 0x00
//    • 11-bit channel encoding (same bit-packing for both)
//    • 100 kbaud, 8E2, inverted UART
//
//  They differ only in TOTAL FRAME LENGTH:
//    • SBUS-16:  25 bytes  = 0x0F + 22 data + 1 flags + 0x00
//    • SBUS-24:  36 bytes  = 0x0F + 33 data + 1 flags + 0x00
//
//  Reference: SBUSController.ino buildSbusFrame() / decodeSbusFrame()
//
//  Auto-detection method: timestamp-based gap detection.  After every byte
//  the inter-byte time is checked.  If the gap exceeds 1ms (frames are sent
//  every ~7-14ms with a much shorter inter-byte time within a frame), we
//  consider the buffer complete and parse based on its accumulated length.
// ─────────────────────────────────────────────────────────────────────────────
#pragma once

#include <Arduino.h>
#include <HardwareSerial.h>

class SbusReader {
public:
  static constexpr uint8_t SBUS_HEADER       = 0x0F;
  static constexpr uint8_t SBUS_FOOTER       = 0x00;
  static constexpr int     FRAME_LEN_16      = 25;
  static constexpr int     FRAME_LEN_24      = 36;
  static constexpr int     MAX_CHANNELS      = 24;
  static constexpr unsigned long INTER_FRAME_GAP_US = 1500;   // 1.5 ms — comfortably less than typical 3 ms gap, more than intra-frame byte spacing

  uint16_t channels[MAX_CHANNELS];
  uint8_t  detectedChCount = 0;   // 16 or 24 after first valid frame; 0 if none yet
  uint8_t  detectedFrameLen = 0;  // 25 or 36
  bool     failsafe = false;
  bool     lostFrame = false;
  unsigned long lastValidFrameMs = 0;

  // Most recently parsed raw frame, kept for diagnostic dumping (#L13).
  // lastFrameRawBuf_ is updated only on a successful parse, so it always
  // holds either a complete SBUS-16 (25 bytes) or SBUS-24 (36 bytes) image.
  // Length is in lastFrameRawLen_; bytes beyond that are unused.
  uint8_t  lastFrameRawBuf_[FRAME_LEN_24] = {0};
  uint8_t  lastFrameRawLen_ = 0;
  const uint8_t* rawFrameBytes() const { return lastFrameRawBuf_; }
  uint8_t  rawFrameLen()        const { return lastFrameRawLen_; }

  void begin(HardwareSerial* uart, int rxPin, int txPin) {
    uart_ = uart;
    // Standard SBUS: 100k baud, 8E2, inverted.  ESP32 HardwareSerial::begin
    // accepts the invert flag as the 6th argument.
    uart_->begin(100000, SERIAL_8E2, rxPin, txPin, true);
    bufIdx_ = 0;
    inFrame_ = false;
    lastByteUs_ = 0;
    for (int i = 0; i < MAX_CHANNELS; i++) channels[i] = 992;  // center
  }

  // ── SBUS OUT passthrough sink ────────────────────────────────────────────
  // Pass a Stream pointer (e.g. SoftwareSerial configured for 100k 8E2 inverted
  // TX) here to enable byte-streaming SBUS passthrough. Every byte read from
  // the SBUS RX UART will be written to this sink as it arrives — typical
  // added latency is ~110 µs (one byte time) end-to-end.
  //
  // No-op if sink is null. Call before/after begin(); takes effect immediately.
  void setPassthroughSink(Stream* sink) { sink_ = sink; }

  // Call frequently (e.g. each main loop iteration).
  // Returns true exactly once per successfully-parsed frame.
  bool read() {
    if (!uart_) return false;
    unsigned long nowUs = micros();
    bool gotFrame = false;

    while (uart_->available()) {
      uint8_t b = uart_->read();

      // Byte-streaming passthrough — tee the byte to the SBUS OUT sink BEFORE
      // any parsing work, so downstream sees it as close to real-time as the
      // sink allows. SoftwareSerial.write() is blocking at 100k baud (~110 µs
      // per byte) which naturally rate-limits us to the SBUS line speed.
      if (sink_) sink_->write(b);

      unsigned long sinceLastByte = nowUs - lastByteUs_;
      lastByteUs_ = nowUs;

      // Gap-based delimiter: if too much time elapsed since the previous byte,
      // the previous frame must have ended.  Try to parse it before consuming
      // the new byte as the start of a new frame. GATED on the buffer already
      // holding a STRUCTURALLY COMPLETE frame: nowUs is sampled once per read()
      // (line 76) and lastByteUs_ collapses to it after the first byte, so
      // sinceLastByte exceeds the gap only on the FIRST byte drained in a call —
      // frequently a mid-frame continuation byte. Flushing a PARTIAL there runs
      // tryParseAndReset on an incomplete buffer (variant==0), which resets
      // lockStreak_; under multi-ms loop timing that repeats every frame and the
      // stream NEVER locks (silent total loss of servo control). A partial just
      // keeps accumulating across read() calls instead.
      if (inFrame_ && sinceLastByte > INTER_FRAME_GAP_US && bufIsCompleteFrame()) {
        gotFrame = tryParseAndReset() || gotFrame;
      }

      // Frame must start with the SBUS header byte.
      if (!inFrame_) {
        // Prefix check for the eager 25-byte flush above. The byte immediately
        // after a REAL SBUS-16 frame is always the next frame's header; anything
        // else means we truncated a longer 24-ch frame at its 25-byte prefix.
        // That truncated parse RE-CONFIRMS the 16 lock, so without this the
        // reader wedges into 16 for as long as data byte 24 keeps landing on
        // 0x00 (i.e. ch17 < 256 and ch18 a multiple of 32, e.g. a switch parked
        // at the 172 endpoint): channels 17-24 are silently pinned to 992, and
        // frame[23] — ch17's low bits, not the flags byte — is decoded as flags,
        // which can assert a phantom failsafe and freeze ALL dispatch while the
        // status LED still reads "receiving". Drop the lock so the 24-variant
        // re-detects. Do NOT fold this into the gap-based flushes: a real gap
        // after 25 bytes IS a genuine 16-frame.
        if (pendingLen16Check_) {
          pendingLen16Check_ = false;
          if (b != SBUS_HEADER) {
            lockStreak_      = 0;
            lockVariant_     = 0;
            detectedFrameLen = 0;
            detectedChCount  = 0;
          }
        }
        if (b == SBUS_HEADER) {
          buf_[0] = b;
          bufIdx_ = 1;
          inFrame_ = true;
        }
        continue;
      }

      buf_[bufIdx_++] = b;

      // Eager length-based flush for a post-stall backlog. The gap check above
      // uses a single micros() sampled once per read() call, so two frames that
      // are BOTH sitting in the UART FIFO (drained back-to-back after a long
      // loop() stall) have no detectable inter-frame gap — and once bytes are
      // FIFO-buffered their original timing is gone, so only STRUCTURE can split
      // them. Once the stream is LOCKED we know the exact frame length, so flush
      // the instant a full frame's worth of bytes ends on a footer, splitting the
      // backlog on frame boundaries instead of overflowing (which would discard
      // both). Gated on a confirmed lock + the DETECTED length — but that gate is
      // NOT sufficient on its own: a 25-byte SBUS-16 frame is a byte-for-byte
      // PREFIX of a 36-byte SBUS-24 frame, so a 16-lock DOES misfire here on a
      // 24-ch stream whose data byte 24 happens to be 0x00. The prefix check
      // armed below is what catches that. Pre-lock/noisy streams fall through to
      // the unchanged gap-based path.
      if (lockStreak_ >= LOCK_FRAMES && detectedFrameLen &&
          bufIdx_ == detectedFrameLen && buf_[detectedFrameLen - 1] == SBUS_FOOTER) {
        // Arm the prefix check only for the ambiguous 25-byte flush: 25 is a
        // prefix of 36, 36 is a prefix of nothing.
        pendingLen16Check_ = (detectedFrameLen == FRAME_LEN_16);
        gotFrame = tryParseAndReset() || gotFrame;
        continue;
      }

      // Buffer-overflow guard — should never hit if framing works.
      if (bufIdx_ >= FRAME_LEN_24 + 4) {
        inFrame_ = false;
        bufIdx_ = 0;
        continue;
      }
    }

    // Also flush at end if a COMPLETE frame is sitting in the buffer and we
    // haven't seen more bytes within the gap window. Same complete-frame gate as
    // the mid-loop flush above — never flush a partial (it would reset the lock).
    if (inFrame_ && bufIsCompleteFrame() && (micros() - lastByteUs_) > INTER_FRAME_GAP_US) {
      gotFrame = tryParseAndReset() || gotFrame;
    }

    return gotFrame;
  }

private:
  HardwareSerial* uart_ = nullptr;
  Stream*  sink_ = nullptr;             // optional SBUS OUT passthrough sink
  uint8_t  buf_[FRAME_LEN_24 + 4];
  int      bufIdx_ = 0;
  bool     inFrame_ = false;
  unsigned long lastByteUs_ = 0;

  // True when buf_ currently holds a structurally complete frame (exact length +
  // header + footer) of either variant. The gap / post-loop flushes gate on this
  // so a partial buffer is never flushed (which would reset the lock streak); a
  // partial simply keeps accumulating across read() calls until it completes.
  bool bufIsCompleteFrame() const {
    return (bufIdx_ == FRAME_LEN_16 && buf_[0] == SBUS_HEADER && buf_[FRAME_LEN_16 - 1] == SBUS_FOOTER) ||
           (bufIdx_ == FRAME_LEN_24 && buf_[0] == SBUS_HEADER && buf_[FRAME_LEN_24 - 1] == SBUS_FOOTER);
  }

  // ── Frame-confirmation lock (noise rejection) ──────────────────────────────
  // Header + footer + exact length can be matched by RF noise / a half-booted
  // source. Require LOCK_FRAMES consecutive structurally-valid frames of the
  // SAME variant before we TRUST the stream — i.e. before we decode it, set
  // detectedChCount, or return true (which is what counts a frame for fps). A
  // real receiver locks in ~3 frames (~30-50 ms); noise that lands on the
  // 0x0F…0x00 pattern once almost never repeats, so it never locks. A
  // malformed frame breaks the streak. A genuine lost-frame/failsafe frame is
  // still structurally valid (correct length/header/footer, only flag bits
  // differ) so it does NOT break a lock once established.
  static constexpr uint8_t LOCK_FRAMES = 3;
  uint8_t  lockStreak_  = 0;   // consecutive same-variant valid frames seen
  int      lockVariant_ = 0;   // 16 or 24 — the variant currently being confirmed

  // Armed by the eager 25-byte flush in read(), consumed by the next byte read
  // outside a frame — the only thing that distinguishes a real SBUS-16 frame
  // from the 25-byte prefix of an SBUS-24 frame once the bytes are in the FIFO.
  bool     pendingLen16Check_ = false;

  // Returns true on a CONFIRMED frame (passes structure AND the lock streak),
  // resets framing state regardless.
  bool tryParseAndReset() {
    // 1) Structural check: exact length + header + footer. variant = 0 means
    //    malformed/partial/noise.
    int variant = 0;
    if (bufIdx_ == FRAME_LEN_16 && buf_[0] == SBUS_HEADER && buf_[FRAME_LEN_16 - 1] == SBUS_FOOTER)      variant = 16;
    else if (bufIdx_ == FRAME_LEN_24 && buf_[0] == SBUS_HEADER && buf_[FRAME_LEN_24 - 1] == SBUS_FOOTER) variant = 24;

    bool ok = false;
    if (variant == 0) {
      // Malformed / partial / noise — break the confirmation streak so a single
      // stray valid-looking frame amid garbage can't masquerade as a live signal.
      lockStreak_  = 0;
      lockVariant_ = 0;
    } else {
      // 2) Confirmation: require LOCK_FRAMES consecutive valid frames of the
      //    same variant before trusting the stream.
      if (variant == lockVariant_) { if (lockStreak_ < LOCK_FRAMES) lockStreak_++; }
      else                         { lockVariant_ = variant; lockStreak_ = 1; }

      if (lockStreak_ >= LOCK_FRAMES) {
        decodeFrame(buf_, variant);
        detectedChCount  = variant;
        detectedFrameLen = (variant == 16) ? FRAME_LEN_16 : FRAME_LEN_24;
        lastValidFrameMs = millis();
        memcpy(lastFrameRawBuf_, buf_, detectedFrameLen);
        lastFrameRawLen_ = detectedFrameLen;
        ok = true;
      }
      // else: structurally valid but not yet confirmed — do NOT decode, do NOT
      // set detectedChCount, do NOT count it (return false). Channels hold.
    }

    inFrame_ = false;
    bufIdx_ = 0;
    return ok;
  }

  // Bit-unpacking is identical for both frame lengths, just iterate
  // chcnt channels through the same 11-bit shift pattern.
  void decodeFrame(const uint8_t* frame, int chcnt) {
    for (int i = 0; i < chcnt; i++) {
      int b = i * 11;
      uint16_t raw = 0;
      raw  = (uint16_t)(frame[1 + b / 8])     >> (b % 8);
      raw |= (uint16_t)(frame[1 + b / 8 + 1]) << (8 - b % 8);
      if ((b % 8) > 5) {
        raw |= (uint16_t)(frame[1 + b / 8 + 2]) << (16 - b % 8);
      }
      channels[i] = raw & 0x07FF;   // 11-bit mask
    }
    // Zero out any channels above the detected count for safety
    for (int i = chcnt; i < MAX_CHANNELS; i++) channels[i] = 992;
    // Flags byte sits just before the footer
    uint8_t flags = frame[(chcnt == 16 ? FRAME_LEN_16 : FRAME_LEN_24) - 2];
    lostFrame = (flags & 0x04) != 0;
    failsafe  = (flags & 0x08) != 0;
  }
};
