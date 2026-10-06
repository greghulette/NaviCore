# NaviCore — Architecture

The system-level map: what runs where, on which core, over which wire. Read this
before changing firmware. Companion documents: [PROTOCOLS.md](PROTOCOLS.md) (wire
formats), [CONFIG_SCHEMA.md](CONFIG_SCHEMA.md) (the config object),
[CONFIG_TOOL.md](CONFIG_TOOL.md) (the browser GUI),
[BUILD_AND_RELEASE.md](BUILD_AND_RELEASE.md) (how a change ships).

---

## 1. The three pieces

ESP32-S3 firmware that decodes SBUS from an RC transmitter and dispatches the result to
hardware wired to the board or reachable over a **WCB (Wireless Communication Board) ESP-NOW
mesh**, plus the browser tool that configures it.

| Piece | Where it lives | Role |
|---|---|---|
| **Firmware** | [`NaviCore.ino`](../NaviCore.ino) + `*.h` at repo root | Reads SBUS, decodes controls, dispatches actions, hosts the config/telemetry protocols |
| **Config tool** | [`config_tool/index.html`](../config_tool/index.html) | Single-file browser GUI over Web Serial — mapping editor, live monitor, flasher, timeline editor |
| **Mesh** | `WCB_Client` library (external repo) | ESP-NOW transport shared with the WCB fleet; NaviCore is a first-class peer |

The droid's config is **data, not code**: everything a user maps lives in one
`RcConfig` struct persisted as `/config.json`. Adding a feature almost always means
extending that struct, its JSON serialiser, the tool's editor, and the dispatcher —
in that order. See [CONFIG_SCHEMA.md](CONFIG_SCHEMA.md).

---

## 2. Physical topology

```
   FrSky transmitter (X18 / X20 / X-Lite Twin)
            │ RF
            ▼
   RC receiver ──SBUS──► [ NaviCore ESP32-S3 ] ──SBUS OUT──► downstream device
                              │  │  │  │
        Serial2 (binary) ─────┘  │  │  └──── USB-CDC ──► config tool (Web Serial)
        local Pololu Maestro     │  │
                                 │  └──── S3 / S4 / S5 aux serial ──► HCR, MP3, WLED, …
                                 │
                                 └──── ESP-NOW (WCB mesh) ──► WCB boards, remote
                                        Maestros, other NaviCores, a tethered
                                        "bridge" WCB the config tool can dial through
```

NaviCore occupies **WCB special-peer slot 20** by convention, so the config tool can
always address "the RC" without asking which one.

---

## 3. Repo map

| Path | Responsibility |
|---|---|
| [`NaviCore.ino`](../NaviCore.ino) | Main sketch: setup/loop, SBUS decode, matrix/switch/knob state machines, action dispatch, all device executors (Maestro/HCR/MP3/WLED/Serial/WCB), CLI, USB JSON handler |
| [`rc_config.h`](../rc_config.h) | `RcConfig` struct + every sub-struct, factory defaults, JSON serialise/deserialise, LittleFS + legacy-NVS persistence, command-library file store |
| [`rc_telemetry.h`](../rc_telemetry.h) | The **Via-WCB bridge**: outbound telemetry, inbound command handling, fragment reassembly/send, bulk-transfer sink, WCB status/alias/meta |
| [`navicore_record.h`](../navicore_record.h) | Record/replay: capture queue, PSRAM clip buffer, clip files, replay interpolation, timeline-editor transport |
| [`navicore_ota.h`](../navicore_ota.h) | Firmware OTA over USB (`?OTALOCAL`) and over the mesh (`?OTA`) |
| [`navicore_rterm.h`](../navicore_rterm.h) | Remote terminal — ships captured CLI output back over the mesh as RTERM packets |
| [`navicore_hil.h`](../navicore_hil.h) | HIL test hooks — `DBG_WIRE` and the `#L90`–`#L93` fault verbs. Compiled **only** with `-DNAVICORE_HIL_HOOKS=1` ([BUILD_AND_RELEASE.md §3](BUILD_AND_RELEASE.md#3-verifying-a-firmware-change)); CI and releases never define it |
| [`rc_serial.h`](../rc_serial.h) | `RcSerial` USB-CDC tee — makes every existing `Serial.print` mirrorable to the remote terminal |
| [`sbus_reader.h`](../sbus_reader.h) | SBUS-16 / SBUS-24 parser with auto-detect + byte-tee passthrough |
| [`wcb_config.h`](../wcb_config.h) | Compile-time **factory defaults only** for mesh credentials (runtime values live in `RcConfig.wcbNetwork`) |
| [`fw_version.h`](../fw_version.h) | `FW_VERSION_BASE` (manual) + `FW_VERSION_DTG` (hook-stamped) |
| [`partitions.csv`](../partitions.csv) | Custom 16 MB table: OTA slots, config LittleFS, 12 MB `clips` LittleFS |
| [`config_tool/`](../config_tool/) | The GUI (`index.html`), flasher, cross-tab serial hub, vendored command library |
| [`tools/`](../tools/) | Build scripts, the DTG pre-commit hook, the Cloudflare relay worker source |
| [`.github/workflows/`](../.github/workflows/) | Firmware CI build + GitHub Pages deploy |
| [`firmware/`](../firmware/) | Committed `.bin` set the in-browser flasher serves |

`config_tool/index-Old.html` and `index1.html` are frozen snapshots — not loaded by
anything. Only `index.html` is live.

---

## 4. Hardware and board profiles

Target: **ESP32-S3-WROOM-1 N16R8** — 16 MB flash, 8 MB **OPI PSRAM** (mandatory).

One firmware image runs on two boards. `rcConfig.boardType` selects a profile at boot
in `applyBoardProfile()`; every pin is a runtime global, so all use sites are ordinary
`begin()` calls.

| Signal | NaviCore v2 PCB (`boardType 0`, default) | WCB HW 3.2 (`boardType 1`) |
|---|---|---|
| SBUS IN (UART1 RX) | GPIO4 | GPIO5 |
| SBUS OUT (UART1 TX) | GPIO5 | GPIO4 |
| Local Maestro (UART2) | TX 6 / RX 7 | TX 6 / RX 7 |
| Aux **S3** | TX 8 / RX 9 | TX 15 / RX 16 |
| Aux **S4** | TX 10 / RX 21 | TX 17 / RX 18 |
| Aux **S5** | TX 38 / RX 47 | TX 9 / RX 10 |
| Status NeoPixel | GPIO48 | GPIO48 |

**UART allocation.** Both profiles set `sbusSharedUart = true`: SBUS IN *and* OUT share
one full-duplex UART1 at 100 k 8E2 inverted, with a byte-tee re-emitting each received
byte. That frees UART0, which becomes the **hardware** aux port S3 (so S3 tolerates
bauds above 57600). S4 and S5 are `NcSoftSerial` (`navicore_softserial.h`): EspSoftwareSerial
receives (a GPIO interrupt per edge, so keep them ≤ 57600) and an **RMT channel transmits**, so
no interrupt can stretch a transmitted bit. The GPIO ISR service is installed at **level 3** in
`setup()`, ahead of the first `begin()`, so a receive edge pre-empts the level-1 UART, RMT and
USB interrupts; and `available()`/`read()`/`peek()` hold the scheduler around the library's
`rxBits()`, whose check-then-`micros()` race injects a false stop bit mid-byte. RMT channels:
the S3 has 4 for transmit, the status NeoPixel takes one and S4/S5 one each. A port begun
without one (or not 8N1) transmits bit-banged and prints `[AUX] TX GPIO<n>: no RMT channel`.
A `sbusSharedUart = false` fallback (SBUS OUT on its own UART0, S3 bit-banged) exists in
the code but no current board uses it.

**Silkscreen vs. firmware names.** The NaviCore v2 PCB labels its aux headers
*Serial 1 / 2 / 3*; the firmware calls the same ports **S3 / S4 / S5** (inherited from
WCB numbering). Any user-facing text must translate — and the **mesh-facing** names already
do: `;s<n>` routing and WDP PORTLABEL use **S1 / S2 / S3**, converted once at the edge by
`rcFwPortForMesh()` / `rcMeshPortForFw()` (rc_config.h). Everything past that edge is in
firmware numbering. See [ROADMAP.md §1](ROADMAP.md).

---

## 5. Memory and storage

| Region | Size | Contents |
|---|---|---|
| PSRAM heap | 8 MB | `RcConfig` (~210 KB, `ps_calloc` in `setup()`), record/replay clip buffer (24 000 events ≈ 3.1 MB) |
| Internal SRAM | 512 KB | Everything else; the fragment reassembly pool is static DRAM (~10 KB) |
| `nvs` @ `0x9000` | 20 KB | Legacy config store (migration source), WCB learned-peer table |
| `app0`/`app1` | 1.9 MB each | OTA slots |
| `spiffs` @ `0x3D0000` | 128 KB | Config LittleFS — `/config.json`, `/cmdlib.json`, staging temp files; `/config.json.hil` after a HIL hook build's `#L91` |
| `clips` @ `0x400000` | 12 MB | Second LittleFS (own label + instance) for record/replay clips |

**PSRAM is not optional.** Without `PSRAM=opi` in the FQBN, `ps_calloc` returns null and
`setup()` halts with a solid red LED and a printed diagnosis.

**Two filesystems, deliberately separate.** `LittleFS` (label `spiffs`) holds config;
`clipsFS` (label `clips`) holds clips. A first-boot format of the clips partition can
therefore never touch `/config.json`. The first six rows of `partitions.csv` are
byte-identical to stock `min_spiffs` so an upgrade does not relocate — and thus does not
reformat — the config filesystem.

---

## 6. Boot order (and why it is this order)

`setup()` in [`NaviCore.ino`](../NaviCore.ino). The sequence is load-bearing:

1. `esp_ota_mark_app_valid_cancel_rollback()` — **first statement**. A freshly-OTA'd
   image boots pending-verify; any later crash would roll it back into a reboot loop.
2. `bootGuardArm()` — one-shot `esp_timer` that restarts the board if `setup()` never
   completes (cold-boot auto-recovery). Disarmed on the last line.
3. Drive `MAESTRO_TX_PIN` high — a floating command line makes servos twitch before
   `Serial2.begin()` runs ~2 s later.
4. USB-CDC: 8 KB RX buffer, 8 KB TX buffer, 50 ms TX timeout — all **before**
   `Serial.begin()`. Then 1.5 s for a host to attach, and the boot banner: `=== NaviCore ===`,
   `App SHA256: <16 hex>` (which image this is — [PROTOCOLS.md §3](PROTOCOLS.md#other)), the
   bootloader line, the reset reason and the boot-attempt count.
   `loop()` also calls `kickUsbCdcTx()` every 20 ms, which flushes the USB-Serial/JTAG TX FIFO and re-arms
   its IN_EMPTY interrupt. The core (`HWCDC.cpp`, esp32 3.3.4) marks the host *disconnected* the moment one
   write makes no progress for the TX timeout, then only queues output, and the only thing that reliably
   reconnects it is the host sending a packet — so a reply could sit in the ring until the next command
   arrived (about 2 % of back-to-back commands on the HIL bench; 0 of 800 with the kick). IN_EMPTY only
   fires once the host drains the FIFO, so with no reader attached nothing changes and the no-host guard
   still holds.
5. `ps_calloc` the config, then `rcConfigLoadDefaults()` → `rcConfigBeginLFS()` →
   `rcConfigLoadLFS()`, falling back to a one-time NVS→LittleFS migration. A
   *present-but-unreadable* `/config.json` is kept and defaults run for that boot —
   never overwritten.
6. Mount `clipsFS`; register it with `navirec`.
7. `applyBoardProfile()` — pins are only known now. **Nothing may open a port before this.**
8. Bind `s3`/`s4`/`s5` to their backing objects, `sbusRx.begin()`, `applySerialBauds()`,
   `applySbusOut()`.
9. `rcTelemetry::init()` — creates the deferred-work mutex **before** any ESP-NOW
   callback can fire.
10. `navirec::recBegin()` — allocates the capture queue **before** the mesh callback that
    can feed it.
11. Construct `WCB_Client`, `setMeshChannel()`, banner if the mesh password is empty,
    `begin()`. On success: create every cross-core queue, *then* register `onCommand` /
    `onRawPacket` / bulk hooks / `onNeighbor` / `onStatusChange`; publish WDP identity and
    port labels; enable auto-join; arm the 8 s new-peer grace window and the 30 s boot
    roll call.
12. Construct the `WCBStream` broadcast channel — after `wcb` exists, so it self-registers
    and gets flushed by `wcb->update()`.

The recurring rule: **create the queue before registering the callback that writes to it.**

---

## 7. The main loop

`loop()` runs on **Core 1** and is a fixed sequence of cheap drains followed by the SBUS
path. Everything in it must stay non-blocking — SBUS arrives every ~9–14 ms and the
passthrough tee is in the same thread.

```
navihil::loopStall()           NAVICORE_HIL_HOOKS builds only: a stall #L90 asked for
kickUsbCdcTx()                 every 20 ms: release USB output the HWCDC core is holding (§6 step 4)
wcb->update()                  mesh heartbeats, ACKs, WCBStream flush
naviota::drainOtaPackets()     + checkOtaTimeout()
drainRemoteCli()               relayed CLI lines → execCliLine with output tee'd to RTERM
maePumpRemoteEmits()           mesh-relayed Maestro read replies → [MAE:] markers
drainMaestroCmd()              inbound ;M routed here by a WCB → local Maestro (one per pass)
drainPeerEvents()              new-peer action + LED alert
checkBootRollCall()            one shot at 30 s — name any configured board never heard
drainRemoteTriggers()          remote TRIGGER → rcDispatch on the right core
drainForgetPeer()              esp_now_del_peer + NVS write
drainTestAction()              bridged per-action Test button
navirec::pollControl/drain/checkRecordBackstop/replayTick
rcTelemetry::tick()            rc_hb 0.5 Hz, rc_ch at chRateHz, outbound fragment pump
processSbus()                  ← the real-time path
checkSbusGestureTimeout()      500 ms with no frame cancels a matrix gesture in flight (§9)
checkDeferredTap()
updateStatusLed()
checkPendingActions()          delayed actions
sendPWMUpdate()                PWM_UPDATE stream (50 ms) when monitoring
handleSerialInput()            one USB line per pass
pollAuxSerialRx()              drain S3/S4/S5 RX so their FIFOs never overflow
drainSerialFwd()               queued mesh→serial writes and serial actions, a few bytes per pass
HCR fade tick / maestroIdleReleaseTick() / trackSbusFps() / #L10 live dump
checkDeferredRestart()         a mesh REBOOT, once ACKed and the queues are quiet (last)
```

---

## 8. Concurrency model — the single most important invariant

Two cores touch this firmware:

- **Core 0** — the WiFi/ESP-NOW task. Runs `onWCBCommand`, `onNeighbor`,
  `otaRawPacketHook`, and everything `rcTelemetry::handle()` does synchronously.
  Small stack. Cannot safely do flash writes, `esp_ota_*`, NVS writes, or long serial I/O.
- **Core 1** — `loop()`. Owns Serial2/S3/S4/S5, the Maestro caches, tap timing, and the
  record/replay buffer.

**Anything arriving on Core 0 that needs to act on droid hardware is enqueued, never
executed inline.** So is anything a WebSocket client must *see*: `rcSerial` mirrors only the
loop core, so a line printed on Core 0 reaches USB alone. The queues:

| Queue | Producer (Core 0) | Consumer (Core 1) | Carries |
|---|---|---|---|
| `remoteTriggerQueue` | `onWCBCommand` → `rcTelemetry::handle` | `drainRemoteTriggers()` | `{mode, btn, tap}` |
| `remoteCliQueue` | `onWCBCommand` | `drainRemoteCli()` | relay id + CLI line |
| `serialFwdQueue` | `onWCBCommand` — targeted `;s<n>` + broadcast-out | `drainSerialFwd()` | `{fwPort, text[201]}` |
| `maestroCmdQueue` | `onWCBCommand` — inbound `;M` | `drainMaestroCmd()` | `{sender, text[48]}` |
| `forgetPeerQueue` | Via-WCB `FORGET_PEER` | `drainForgetPeer()` | board id (0 = all) |
| `peerEventQueue` | `onWcbNeighbor` | `drainPeerEvents()` | board id |
| `naviota::otaPktQueue` | `otaRawPacketHook` | `drainOtaPackets()` | OTA control/data structs — target side, and the ACKs this board relays |
| `navirec` capture queue | `rcExecuteActionNow` (Core 1 — hop kept as a safeguard) | `navirec::drain()` | `RecEvent` |
| `rcTelemetry` pending slots | `handle()` under `_pendingMutex` | `tick()` | deferred config saves, test actions |

The two serial queues are not conveniences: a write to S4/S5 returns only once its bytes are on
the wire (~1 ms a byte at 9600; `NcSoftSerial` sleeps on the RMT driver meanwhile), and a
Maestro `get*` blocks up to 25 ms waiting on the reply. Either one on the WiFi task stalls
ESP-NOW. Every soft-port writer is the loop task.

Enqueue helpers are marked `__attribute__((noinline))` so their locals do not inflate the
ESP-NOW callback's stack frame — a prior stack overflow was fixed exactly this way.

**Nothing on the Core-0 path may materialise a large temporary.** The rule is wider than
locals: `slot = BigStruct{}` is not elided on assignment, so it builds a full-size temporary
in the *caller's* frame. `FragSession` is ~3.3 KB, and clearing it that way put ~7 KB on the
WiFi-task stack once `handle()` and `_findOrAllocSession()` nested. Both sites now call
`rcTelemetry::_fragClear()`, which clears in place and is itself `noinline`; measured with
`-fstack-usage`, `handle()` is 368 B and `_findOrAllocSession()` 32 B. Never reintroduce
whole-struct assignment on this path.

`navirec` additionally uses `volatile` single-word flags (`_pendingCtl`, `_capturing`,
`_lastPos`) as lock-free edges; those are safe **only** because each is a single aligned
store with one logical writer.

The USB-CDC tee (`RcSerial`) gates its capture sink on `xPortGetCoreID() == _capCore`, so a
Core-0 print landing mid-command cannot corrupt the single-threaded RTERM line buffer.

**WebSocket output crosses the other way, Core 1 → the httpd task (Core 0).** The sink
(`naviws::WsSink`, loop task) moves its bytes onto NaviCore's own PSRAM queue (`wsTxHead`),
and one httpd work item at a time (`wsDrainWork`) sends them; the list and its flags are
shared under the spinlock `wsTxMux`, which nothing holds across a socket call or an
allocation. Never hand httpd one work item per frame: its control socket drops what it
cannot hold, without an error. Every socket write happens on the httpd task, through
`wsSendAll`, so the server's own control frames cannot interleave with ours — see
[PROTOCOLS.md](PROTOCOLS.md) on the WebSocket transport.

**`onWcbStatus` is the one callback that fires on *both* cores.** The ONLINE edge comes
from the ESP-NOW receive callback (Core 0, first heartbeat after silence); the OFFLINE edge
comes from `wcb->update()` inside `loop()` (Core 1, heartbeat-miss sweep). It therefore has
to satisfy the Core-0 rules regardless of which edge you are reasoning about — it does one
`printf` and nothing else. Its board name comes from `rcTelemetry::wcbAlias()`, which
writes the terminator first and so is safe to read from Core 1 while Core 0 rewrites it;
`wcb->getNeighbor()->name` carries no such guarantee.

**When adding any mesh-triggered feature: assume your handler runs on Core 0 and defer.**

---

## 9. Input pipeline

```
SBUS frame ─► SbusReader (auto-detect 25 B / 36 B) ─► sbusValues[24]
                     │
                     ├─► mode decode ....... FunctionSwState 1..3 (3-position switch)
                     ├─► matrix channel .... pwmToButton() → 3-state debounced edge
                     │                        machine → tap counter → rcDispatch()
                     ├─► processSwitches() . SA–SJ position change → RcTier
                     └─► processKnobs() .... continuous sources → Maestro passthrough
                                              or HCR volume
```

**Framing.** `SbusReader` trusts a stream only after `LOCK_FRAMES` (3) consecutive
structurally valid frames of one variant, and decodes whole frames only. Locked on SBUS-24 it
decodes a 36-byte buffer the moment it ends on `0x00`; a 25-byte buffer is decoded only when
the byte after it is a header or the line goes quiet. 25 bytes are a byte-for-byte prefix of
an SBUS-24 frame whose byte 24 is `0x00`, and decoding them eagerly handed `processSbus` a
misframed frame (flags read from CH17's low byte) on every return from SBUS-16 to SBUS-24.
A partial frame is dropped once the line has been quiet for `PARTIAL_DROP_US` (6 ms: longer
than a whole SBUS-24 frame, shorter than a frame period), and the drop breaks the lock, so a
window spanning a silence is never decoded — a cut frame or a lone header used to be filled by
the next frame's bytes and decoded as a frame nobody sent. Never drop a partial on the in-loop
gap: that gap is the loop period, not the line's, and dropping there kept the stream from ever
locking.

**Modes.** `FunctionSwState` (1/2/3) multiplies every button mapping: `RC_NUM_MAPPINGS =
108 = 3 modes × 36 slots`.

**Button slots.** 36 threshold bands on one matrix channel: slots 1–21 physical (21 is an
inert "Unassigned" sentinel drawn on the transmitter graphic), 22–36 user-defined logical
buttons. All decode identically through `pwmToButton()`; a `0/0` band is inert.

**Matrix debounce.** A press commits only after the decoded button holds in-band for
`matrixDebounceFrames` consecutive frames, and a re-arm only after NEUTRAL holds the same
number of frames — so a one-frame dip cannot split one press into a phantom double.
Runtime-tunable 1–4. Only a true sub-frame tap is unrecoverable (an SBUS-rate limit).

**Taps.** Each mapping has `RC_NUM_TAP_TIERS = 4` tiers (`t[0]` single, `t[1]` double, `t[2]`
triple, `t[3]` **long press**) and an `exclusive` flag: exclusive fires only the final tier
after the window closes; cumulative fires each tier as it is reached.

**Long press.** Holding a matrix button in-band for `holdMs` (default 750, configurable)
dispatches `t[3]` **at the threshold, while still held** — the release then fires nothing.
Three constraints make this work, and each is load-bearing:

- `holdMs` **must exceed `tapWindowMs`**, or the deferred tap dispatch fires first and the
  hold is unreachable. Both firmware and tool clamp a too-small value to `tapWindowMs + 250`.
- `checkDeferredTap()` **parks the tap dispatch while the button is down** (`holdActive`).
  Consequence: a press-and-hold now resolves on release (or at `holdMs`), not mid-hold.
- Tier 4 is **always dispatched exclusively**, regardless of the `exclusive` flag — it is a
  different gesture, not a 4th tap, so the cumulative rule must not fire t1+t2+t3 alongside it.

A hold only promotes on the **first** press of a gesture; holding the 2nd or 3rd tap leaves it
an ordinary double/triple. A 4-tap flurry still saturates at triple — tier 4 is reachable only
by holding. The hold is opened by the debounced press commit and closed by the debounced
NEUTRAL (`rcMatrixRelease()`), so a one-frame transient cannot cancel it, and sliding onto a
neighbouring band cannot fire the wrong button's long press (the threshold test requires
`decoded == holdBtn`). An SBUS failsafe frame, or `SBUS_GESTURE_TIMEOUT_MS` (500 ms, the
status LED's "no signal") with no frame at all, **cancels the whole gesture** — the hold and
any deferred tap — and re-arms the matrix (`rcMatrixResetGesture()`); the press must be made
again. Clearing only the hold left the deferred tap to fire during the failsafe, and with no
frame timeout a press held when the frames stopped resolved as a tap once they returned.

**Switch settle.** A switch position becomes a *candidate* on change and only dispatches
once it has held for `switchSettleMs` (default 80, `0` = fire immediately). A 3-position
switch swept end-to-end sits in the middle band for a frame or two, and without the settle
window that middle tier fired **in full** on the way past — sounds, scripts, easing. Only the
position the switch comes to rest in fires. Returning to the previous position mid-window
cancels the candidate.

**Switch easing is seeded, not just edge-driven.** `g_switchEasing[]` is otherwise only
written by an *executed* `setEasing` action, while `processSwitches()` seeds a switch's
position **without firing** (at boot and after every config apply). So the firmware booted
believing `EASE_RELEASED` regardless of where the switch physically sat, `resolveKnobEasing()`
returned `< 0`, and the hot path skipped easing until the pilot happened to flick the switch.
`seedSwitchEasingFromTier()` now adopts the easing the resting position selects, and the seed
ends with one `reapplyMaestroEasing()` per slot. **It must never execute the tier** — the seed
exists so a power-up cannot fire scripts or servo moves. Easing verbs only.

**Easing writes are retransmitted, because they cannot be verified.** The Pololu protocol has
**no readback for speed or acceleration** (`WcbMaestro`'s reply table is POS/MOV/ERR), and
`maestroWrite()` reports success on *queueing*: `HardwareSerial::write()` returns the byte
count unconditionally, and a remote slot only buffers into a `WCBStream` whose unacked
broadcast result is discarded. A lost easing write is therefore invisible *and* never retried —
the cache records it applied and the compare skips it forever. Each easing change schedules
`EASE_REPEATS` further sends `EASE_REPEAT_MS` apart (`easingRepeatTick()`), each invalidating
the cache first so the repeat is not a no-op against the entry it is meant to correct. It is a
bounded retry, not a heartbeat: an idle droid puts nothing extra on the mesh.

**Dispatch.** `rcDispatch(buttonId, tapCount)` → the tier's up-to-5 `RcAction`s →
`rcExecuteAction()` (schedules if `delayMs`, else `rcExecuteActionNow()`). Delays are
measured **from the trigger instant and run in parallel**, not cumulatively.

While `calibrationActive` is set (the tool's calibration wizard), all dispatch is muted **and
`processKnobs()` returns early**. Muting dispatch alone is not enough: passthrough is not a
dispatch, so without the second gate every passthrough servo tracked the operator's calibration
sweeps and drove its full mechanical travel. Any new path that moves hardware from SBUS input
needs its own `calibrationActive` check.

---

## 10. Action model and executors

An `RcAction` is `{type, target[6], cmd[96], delayMs, note[20], skipRunning, fn, chan, track}`.
`rcExecuteActionNow()` switches on `type`:

| `RcActionType` | Executor | Destination |
|---|---|---|
| `RA_WCB_UNICAST` (1) | `wcb->send(id, cmd)` | WCB board 1–20, ETM-acked |
| `RA_WCB_BROADCAST` (2) | `wcb->broadcast(cmd)` | whole mesh |
| `RA_MAESTRO_LOCAL` (3) | `executeMaestroCmd` → `maestroWrite` → Serial2 | wired Pololu bus |
| `RA_MAESTRO_REMOTE` (4) | discrete verbs unicast WCB-native; passthrough/replay streams raw via `WCBStream` | remote Maestro |
| `RA_SERIAL` (5) | `queueSerialAction()` → the paced `auxTxPump()` (`\r`-terminated) | aux port named in `target`; clocked out a few bytes per `loop()` pass like a mesh→serial forward, never written whole (an S4/S5 write returns only once its last bit is out). The `Serial TX` trace line prints when the line is out |
| `RA_HCR` (6) | `executeHcrAction` → `hcrFormatCommand` | port or WCB from **global** `hcrDest` |
| `RA_MP3` (7) | `executeMp3Action` → `;A,…` | **global** `mp3Dest` |
| `RA_RECORD` (8) / `RA_PLAY` (9) / `RA_STOP` (10) | `navirec` control (deferred to Core 1) | — never captured into a clip |
| `RA_SMOOTH_OVERRIDE` (11) | global passthrough smoothing latch | runtime only |
| `RA_WLED` (12) | `executeWledAction` → `;L<id>,<verb>` | per-id routing in `wledSlots`; a remote slot gets `;L<id>,<body>` rebuilt from the parse, so `L1,ON` (no `;`) works remotely as it does locally |
| `RA_DFPLAYER` (13) | `executeDfpAction` → `;D,…` | **global** `dfpDest` — local aux port or a WCB |

**Device writes never cut into a paced line.** Every device writer — an HCR action or fade
step, the MP3 Trigger, DFPlayer and WLED codecs, `#L20`/`#L21` — reaches S3/S4/S5 through
`auxDev()`, never the port itself. While `auxTxPump()` has a line in flight on that port (or
device bytes are already held behind one), the bytes go into the port's 512-byte hold buffer,
which the pump sends after the line's CR and before it takes the next queued line; an idle
port is written at once. A write that would overflow the buffer finishes the line in
`loop()` first (`auxTxFinish()`). Without this a device's bytes landed inside the line, and
both it and the line's reader got a broken command (HIL `ncwire.tx_interleave`).

HCR, MP3 and DFPlayer destinations are **global, not per-action** — an action carries only
`fn`/`chan`/`track`. Maestro slots 1–8 are logical: each slot stores `{type, device,
channels[]}` and a *Remote* slot is reached by mesh, disambiguated by the on-wire Pololu
device number.

Device byte-building for Maestro/MP3/WLED/HCR/DFPlayer lives in the shared **`WcbCmd`**
library, so NaviCore and the WCB firmware emit identical bytes from one source.

The MP3 Trigger and the DFPlayer Mini are **separate action types on purpose**, not one
audio device with a mode flag: their verb sets differ, and their volume scales are inverse
(MP3 Trigger `0` = loudest … `64` = inaudible; DFPlayer `0` = silent … `30` = loudest). A
droid can host both. See [DFPLAYER_DESIGN.md](DFPLAYER_DESIGN.md).

---

## 11. Transports

| Transport | Used for | Notes |
|---|---|---|
| **USB-CDC** (native, `HWCDC`) | Config tool "Direct USB", CLI, OTA-local | Wrapped by `RcSerial` tee. `#define Serial rcSerial` — include order in `NaviCore.ino` matters |
| **Serial2 / UART2** | Local Pololu Maestro | Binary Pololu protocol, baud from `rcConfig.maestroBaud` |
| **S3 / S4 / S5** | HCR, MP3, DFPlayer, WLED, raw serial actions | S3 = hardware UART0; S4/S5 `NcSoftSerial` (software receive, RMT transmit). One port = one device = one baud (a DFPlayer's is fixed at 9600) |
| **UART1** | SBUS IN + OUT | 100 k 8E2 inverted, shared, byte-teed |
| **ESP-NOW / WCB mesh** | Remote actions, config bridge, telemetry, OTA, RTERM, bulk transfer | 250 B MTU; **187 B effective payload cap** after the bridge's CRC suffix |

---

## 12. Status LED (GPIO48 NeoPixel)

| Appearance | Meaning |
|---|---|
| Solid red | Fatal — PSRAM allocation failed in `setup()` |
| Steady orange | Latched fault — `wcb->begin()` failed |
| Flashing orange | No SBUS frames arriving |
| Steady blue | Healthy, receiving SBUS |
| Brief pulse | New mesh peer detected (when `peerAlert` is on) |

---

## 13. Subsystems in one line each

- **`rcTelemetry`** — the bridge. Outbound `rc_hb`/`rc_ch`/`rc_trig`/`rc_mode`; inbound JSON
  reassembly and dispatch; WCB status/alias/port-label metadata; bulk-transfer sink.
  A saved config is applied identically on both transports: USB `SET_CONFIG` and the bridged
  `_applyReassembled()` both call **`applyConfigSideEffects()`** (in the .ino) for the live
  re-apply of baud, SBUS-OUT, Maestro easing and auto-release policy, and to forget any matrix
  gesture in progress (`rcMatrixResetGesture()`). **Any new post-save fixup
  belongs in that helper, not in one caller** — the two paths previously drifted, and a Save
  over the mesh silently left the board on its old settings until the next reboot.
- **`navirec`** — records dispatched actions plus synthesized servo/volume keyframes into a
  PSRAM clip, saves named clips to `clipsFS`, replays them with per-channel interpolation
  and ease-from-home anchoring, and streams clips to/from the tool's timeline editor.
- **`naviota`** — brick-safe OTA (always writes the inactive slot, SHA-verified before the
  boot pointer moves) over USB or the mesh, wire-compatible with WCB OTA.
- **`navirterm`** — one 204-byte packet per captured CLI line, in the WCB's own RTERM
  format, so an unmodified bridge WCB surfaces NaviCore's CLI on the tool's terminal.
- **WDP identity** — `setIdentity()` + `setPortLabel()` advertise NaviCore's name, firmware,
  board, and what is attached to each serial port, so WCBs and the Wizard discover it
  automatically.

---

## Revision log

Newest first. Add a row whenever a code change alters what this page describes — same commit
as the code. Page body stays present-tense; history lives here.

| Date | Commit | Change |
|---|---|---|
| 2026-10-06 | _(pending)_ | §10: device writes reach S3/S4/S5 through `auxDev()`, which holds them behind a paced line in flight, so they never land inside one (HIL `ncwire.tx_interleave`). |
| 2026-10-06 | _(pending)_ | §4 UART allocation, §8, §11: S4/S5 are `NcSoftSerial` — RMT transmit, EspSoftwareSerial receive with the GPIO ISR service at level 3 and the `rxBits()` race closed (WCB repo HIL `ncwire.soft_tx_integrity`, D-NC24; `ncwire.rx_monitor_bcast_in`). |
| 2026-10-05 | `bb0dda6` | §8: WebSocket output is a Core 1 → httpd-task hand-off through NaviCore's own PSRAM queue under `wsTxMux`, one `wsDrainWork` at a time. |
| 2026-10-04 | `aa6ae0a` | §7 lists `checkDeferredRestart()`, last in `loop()`: a mesh `REBOOT` is ACKed from `rcTelemetry::tick()` and restarts once the inbound queues are quiet (HIL `ncboot.mesh_reboot`). |
| 2026-10-04 | `4833716` | §7, §10: a serial action goes through the paced aux transmitter (`queueSerialAction()` → `auxTxPump()`) instead of one blocking whole-line write (HIL `ncdev.serial_action_paced`). |
| 2026-10-04 | `42c8a61` | §10: a WLED action to a remote slot forwards `;L<id>,<body>` rebuilt from the parse instead of the text as written (HIL `ncdev.wled_forward_normalised`). |
| 2026-10-04 | `90c58bc` | §9 "Framing": a partial frame is dropped after 6 ms of real line silence and breaks the lock, so a cut frame can no longer join the next frame's bytes into a decoded phantom (HIL `sbus.truncated_frame_no_phantom`). |
| 2026-10-04 | `5a648ad` | §9 "Framing": locked on SBUS-16 the reader decodes a 25-byte buffer only on the next header or a silence, never eagerly (HIL `sbus.sbus24_return_no_prefix_decode`); the post-stall eager flush is SBUS-24 only. |
| 2026-10-04 | `5b894b9` | §7 lists `checkSbusGestureTimeout()`. A failsafe frame, or 500 ms with no SBUS frame (`checkSbusGestureTimeout()`), cancels any matrix gesture in flight — the deferred tap as well as the hold (HIL `sbus.failsafe_deferred_tap`, `sbus.frame_stop_held_press`). |
| 2026-10-04 | `6cdaa8a` | §13: `applyConfigSideEffects()` also forgets a parked tap or hold and re-arms the matrix (`rcMatrixResetGesture()`), so every config apply on either transport, `RESET_DEFAULTS` included, drops a gesture in progress (HIL `sbus.reconfig_parked_tap_cleared`). |
| 2026-09-28 | `703a0e7` | §3 lists `navicore_hil.h`, the HIL hook header compiled only with `-DNAVICORE_HIL_HOOKS=1`; §5 the `/config.json.hil` copy its `#L91` leaves; §7's loop order gains the hook build's `#L90` stall and `kickUsbCdcTx()`, which already ran first. |
| 2026-09-28 | `1e15601` | §6 step 4: the boot banner now carries `App SHA256: <16 hex>`, the running image's identity (PROTOCOLS.md §3), and the USB RX buffer is 8 KB (`Serial.setRxBufferSize(8192)`; this page said 4 KB). |
| 2026-09-22 | _(pending)_ | `kickUsbCdcTx()` in `loop()`: flushes the USB-Serial/JTAG TX FIFO and re-arms IN_EMPTY every 20 ms, so output the HWCDC core stopped sending after a brief host stall (its `connected` flag only comes back on host input) is delivered without waiting for the next command. Found by the WCB HIL bench: ~2 % of back-to-back commands lost their reply; 0 of 800 after. |
| 2026-09-10 | _(uncommitted)_ | §8: the OTA queue also carries the ACKs this board relays, and the Core-0 rule now covers output a WebSocket client must see — `rcSerial` mirrors only the loop core, which is why relay OTA over WiFi never received an ACK. See PROTOCOLS.md's row of the same date. |
| 2026-08-25 | _(uncommitted)_ | **Switch settle window** (`switchSettleMs`, default 80): a position must rest before its tier fires, so a 3-position switch swept end-to-end no longer fires the middle tier on the way past. **Switch easing is now seeded** from the resting position at boot/apply (`seedSwitchEasingFromTier`) — previously `g_switchEasing` was only ever set by an executed action, so a power-up believed `EASE_RELEASED` wherever the switch physically sat and easing silently did nothing until the pilot flicked it. **Easing writes are retransmitted** (bounded burst) because the Pololu protocol has no speed/accel readback and `maestroWrite()` reports success on queueing, making a lost write invisible and never retried. |
| 2026-08-24 | `083207c` | **Long press added as tap tier 4.** `RcMapping::t[]` is now `RC_NUM_TAP_TIERS = 4`; holding a matrix button for the new `holdMs` config field (default 750) dispatches `t[3]` at the threshold while still held. Three constraints documented in §Taps: `holdMs` must exceed `tapWindowMs`, `checkDeferredTap()` parks the tap dispatch while the button is down (so press-and-hold now resolves on release, not mid-hold), and tier 4 always dispatches exclusively regardless of the `exclusive` flag. |
| 2026-08-18 | _(uncommitted)_ | The three global audio destinations (`hcrDest`/`mp3Dest`/`dfpDest`) gained a **disabled** state (`transport` 2, JSON `"off"`) and now default to it. `RA_HCR`/`RA_MP3`/`RA_DFPLAYER` are no-ops while their device is disabled — the gate is in the executor, not at the port. |
| 2026-08-18 | _(uncommitted)_ | §7/§8 completed: the loop sequence and the Core-0→Core-1 queue table gained `drainMaestroCmd()` / `serialFwdQueue` and `drainSerialFwd()` / `maestroCmdQueue`, with the reason they must be deferred (bit-banged S4/S5 writes block with interrupts off; a Maestro `get*` blocks 25 ms). §8's `navirec` capture row now reads **Core 1** — remote TRIGGERs are deferred through `drainRemoteTriggers()`, so the queue hop is a safeguard rather than a cross-core requirement. §4: the mesh-facing **S1–S3** renumber is shipped, stated as current instead of planned. |
| 2026-08-18 | _(uncommitted)_ | Core-0 stack: both `FragSession` slot-clear sites now call `rcTelemetry::_fragClear()` instead of assigning `FragSession{}`, which was materialising a ~3.3 KB temporary per site and ~7 KB nested on the ESP-NOW callback (`handle()` 3632→368 B, `_findOrAllocSession()` 3328→32 B, `-fstack-usage`). Recorded the wider rule in §8. `processKnobs()` now returns early while `calibrationActive` (§9) — dispatch muting alone let passthrough servos track the wizard's full-range sweeps. Post-save live re-apply factored into `applyConfigSideEffects()` and called from BOTH the USB and Via-WCB save paths (§13); the mesh path previously skipped it entirely. |
| 2026-08-12 | _(uncommitted)_ | `rcTelemetry::tick()` gained the 30 s mesh-stats `;V` push and the deferred bridged `MESH_STATS` reply (`_pendingMeshStatsSender`, same Core-0-defer discipline as `WCB_STATUS`). Inbound COMMANDs are now counted in `onWCBCommand` (`g_meshRxCount`/`g_meshRxFrom`) because `WCB_Client`'s own statistics are outbound-only. |
| 2026-08-12 | _(uncommitted)_ | `onStatusChange`/`onWcbStatus` + the 30 s boot roll call added to the setup order and the loop sequence. Recorded the concurrency rule that `onWcbStatus` is the **one callback firing on both cores** (ONLINE from the RX task, OFFLINE from `update()`), and why its name must come from `rcTelemetry::wcbAlias()` rather than `getNeighbor()->name`. |
| 2026-08-05 | _(uncommitted)_ | `RA_DFPLAYER` (13) added to the executor table + `dfpDest`; noted why it is a separate type from `RA_MP3` (inverse volume scales, different verb sets). |
| 2026-08-04 | _(uncommitted)_ | Initial version. |
