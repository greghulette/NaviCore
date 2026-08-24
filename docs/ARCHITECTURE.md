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
bauds above 57600); S4 and S5 are bit-banged `SoftwareSerial` and should stay ≤ 57600.
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
| `spiffs` @ `0x3D0000` | 128 KB | Config LittleFS — `/config.json`, `/cmdlib.json`, staging temp files |
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
4. USB-CDC: 4 KB RX buffer, 8 KB TX buffer, 50 ms TX timeout — all **before**
   `Serial.begin()`.
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
checkDeferredTap()
updateStatusLed()
checkPendingActions()          delayed actions
sendPWMUpdate()                PWM_UPDATE stream (50 ms) when monitoring
handleSerialInput()            one USB line per pass
pollAuxSerialRx()              drain S3/S4/S5 RX so their FIFOs never overflow
drainSerialFwd()               queued mesh→serial writes, a few bytes per pass
HCR fade tick / maestroIdleReleaseTick() / trackSbusFps() / #L10 live dump
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
executed inline.** The queues:

| Queue | Producer (Core 0) | Consumer (Core 1) | Carries |
|---|---|---|---|
| `remoteTriggerQueue` | `onWCBCommand` → `rcTelemetry::handle` | `drainRemoteTriggers()` | `{mode, btn, tap}` |
| `remoteCliQueue` | `onWCBCommand` | `drainRemoteCli()` | relay id + CLI line |
| `serialFwdQueue` | `onWCBCommand` — targeted `;s<n>` + broadcast-out | `drainSerialFwd()` | `{fwPort, text[201]}` |
| `maestroCmdQueue` | `onWCBCommand` — inbound `;M` | `drainMaestroCmd()` | `{sender, text[48]}` |
| `forgetPeerQueue` | Via-WCB `FORGET_PEER` | `drainForgetPeer()` | board id (0 = all) |
| `peerEventQueue` | `onWcbNeighbor` | `drainPeerEvents()` | board id |
| `naviota::otaPktQueue` | `otaRawPacketHook` | `drainOtaPackets()` | OTA control/data structs |
| `navirec` capture queue | `rcExecuteActionNow` (Core 1 — hop kept as a safeguard) | `navirec::drain()` | `RecEvent` |
| `rcTelemetry` pending slots | `handle()` under `_pendingMutex` | `tick()` | deferred config saves, test actions |

The two serial queues are not conveniences: S4/S5 are bit-banged `SoftwareSerial`, so a write blocks
with interrupts off for the whole frame time, and a Maestro `get*` blocks up to 25 ms waiting
on the reply. Either one on the WiFi task stalls ESP-NOW and jitters the SBUS path.

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
`decoded == holdBtn`). An SBUS failsafe clears the hold — otherwise `holdActive` would park
the tap dispatch forever and the button would go dead after recovery.

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
| `RA_SERIAL` (5) | `writeS3/S4/S5` (`\r`-terminated) | aux port named in `target` |
| `RA_HCR` (6) | `executeHcrAction` → `hcrFormatCommand` | port or WCB from **global** `hcrDest` |
| `RA_MP3` (7) | `executeMp3Action` → `;A,…` | **global** `mp3Dest` |
| `RA_RECORD` (8) / `RA_PLAY` (9) / `RA_STOP` (10) | `navirec` control (deferred to Core 1) | — never captured into a clip |
| `RA_SMOOTH_OVERRIDE` (11) | global passthrough smoothing latch | runtime only |
| `RA_WLED` (12) | `executeWledAction` → `;L<id>,<verb>` | per-id routing in `wledSlots` |
| `RA_DFPLAYER` (13) | `executeDfpAction` → `;D,…` | **global** `dfpDest` — local aux port or a WCB |

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
| **S3 / S4 / S5** | HCR, MP3, DFPlayer, WLED, raw serial actions | S3 = hardware UART0; S4/S5 SoftwareSerial. One port = one device = one baud (a DFPlayer's is fixed at 9600) |
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
  re-apply of baud, SBUS-OUT, Maestro easing and auto-release policy. **Any new post-save fixup
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
| 2026-08-24 | `083207c` | **Long press added as tap tier 4.** `RcMapping::t[]` is now `RC_NUM_TAP_TIERS = 4`; holding a matrix button for the new `holdMs` config field (default 750) dispatches `t[3]` at the threshold while still held. Three constraints documented in §Taps: `holdMs` must exceed `tapWindowMs`, `checkDeferredTap()` parks the tap dispatch while the button is down (so press-and-hold now resolves on release, not mid-hold), and tier 4 always dispatches exclusively regardless of the `exclusive` flag. |
| 2026-08-18 | _(uncommitted)_ | The three global audio destinations (`hcrDest`/`mp3Dest`/`dfpDest`) gained a **disabled** state (`transport` 2, JSON `"off"`) and now default to it. `RA_HCR`/`RA_MP3`/`RA_DFPLAYER` are no-ops while their device is disabled — the gate is in the executor, not at the port. |
| 2026-08-18 | _(uncommitted)_ | §7/§8 completed: the loop sequence and the Core-0→Core-1 queue table gained `drainMaestroCmd()` / `serialFwdQueue` and `drainSerialFwd()` / `maestroCmdQueue`, with the reason they must be deferred (bit-banged S4/S5 writes block with interrupts off; a Maestro `get*` blocks 25 ms). §8's `navirec` capture row now reads **Core 1** — remote TRIGGERs are deferred through `drainRemoteTriggers()`, so the queue hop is a safeguard rather than a cross-core requirement. §4: the mesh-facing **S1–S3** renumber is shipped, stated as current instead of planned. |
| 2026-08-18 | _(uncommitted)_ | Core-0 stack: both `FragSession` slot-clear sites now call `rcTelemetry::_fragClear()` instead of assigning `FragSession{}`, which was materialising a ~3.3 KB temporary per site and ~7 KB nested on the ESP-NOW callback (`handle()` 3632→368 B, `_findOrAllocSession()` 3328→32 B, `-fstack-usage`). Recorded the wider rule in §8. `processKnobs()` now returns early while `calibrationActive` (§9) — dispatch muting alone let passthrough servos track the wizard's full-range sweeps. Post-save live re-apply factored into `applyConfigSideEffects()` and called from BOTH the USB and Via-WCB save paths (§13); the mesh path previously skipped it entirely. |
| 2026-08-12 | _(uncommitted)_ | `rcTelemetry::tick()` gained the 30 s mesh-stats `;V` push and the deferred bridged `MESH_STATS` reply (`_pendingMeshStatsSender`, same Core-0-defer discipline as `WCB_STATUS`). Inbound COMMANDs are now counted in `onWCBCommand` (`g_meshRxCount`/`g_meshRxFrom`) because `WCB_Client`'s own statistics are outbound-only. |
| 2026-08-12 | _(uncommitted)_ | `onStatusChange`/`onWcbStatus` + the 30 s boot roll call added to the setup order and the loop sequence. Recorded the concurrency rule that `onWcbStatus` is the **one callback firing on both cores** (ONLINE from the RX task, OFFLINE from `update()`), and why its name must come from `rcTelemetry::wcbAlias()` rather than `getNeighbor()->name`. |
| 2026-08-05 | _(uncommitted)_ | `RA_DFPLAYER` (13) added to the executor table + `dfpDest`; noted why it is a separate type from `RA_MP3` (inverse volume scales, different verb sets). |
| 2026-08-04 | _(uncommitted)_ | Initial version. |
