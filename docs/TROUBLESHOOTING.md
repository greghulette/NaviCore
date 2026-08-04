# NaviCore — Symptom Index

Keyed by what you actually observe. Each row names the cause and where to look. Confirm in
the code before acting — this page is a shortlist of known causes, not a diagnosis.

---

## Boot and LED

| Symptom | Cause | Where |
|---|---|---|
| Solid red LED, `[FATAL] PSRAM allocation for rcConfig failed` | Built without `PSRAM=opi`. The ~210 KB `RcConfig` cannot fit internal RAM, so `setup()` halts deliberately | `setup()` |
| Steady orange LED | Latched fault: `wcb->begin()` failed. Check mesh credentials | `setup()`, `g_ledFaultColor` |
| Flashing orange LED | No SBUS frames arriving. Distinguishable from the WCB fault by flashing vs. steady | `updateStatusLed()` |
| Reboots forever after an OTA | The new image was never marked valid, so the bootloader rolls it back. `esp_ota_mark_app_valid_cancel_rollback()` is the **first statement** of `setup()` for this reason — anything that crashes ahead of it reintroduces the loop | `setup()` |
| Reboots ~a few seconds into boot, repeatedly | `setup()` never completed and the boot guard fired. It is armed first and disarmed last | `bootGuardArm()` / `bootGuardDisarm()` |
| Crash-loops ~2 s after boot | Heap starvation — classically from raising `FRAG_MAX_PARTS` (384 does this; 192 is stable). The config load needs ~96 KB | `rc_telemetry.h` |
| Servos twitch at power-on with no SBUS connected | Maestro TX floating before `Serial2.begin()` runs (~2 s in). The pin is driven HIGH at the top of `setup()` to prevent it — check that still happens | `setup()` |
| Config lost unexpectedly | `⚠ Full Wipe & Flash` erases NVS and OTA data. A routine `⬆ Update Firmware` and any serial app-flash preserve `/config.json` (LittleFS, not NVS) | [BUILD_AND_RELEASE.md](BUILD_AND_RELEASE.md) |
| `[CONFIG] /config.json present but unreadable` | Parse failure or transient low memory. The file is **kept** and defaults run for that boot — deliberately, so a good config is never overwritten. Retries next boot | `setup()` |
| `[CLIPS] no clips partition` | Board still on the 4 MB table. Record/replay runs in-RAM only until a full flash with the 16 MB layout | `setup()`, `partitions.csv` |

## USB / Direct-connect

| Symptom | Cause | Where |
|---|---|---|
| `{"type":"ERROR","msg":"JSON parse failed",...,"rxLen":N}` | Line truncated in transit — compare `rxLen` against what the host sent. The board sets a 4 KB RX buffer and the tool chunks writes at 512 B with 4 ms pacing | `handleSerialInput()`, `sendLine()` |
| `CONFIG` reply arrives truncated / mangled mid-string | USB-CDC TX overflow. The 8 KB TX buffer and 50 ms TX timeout exist for exactly this | `setup()` |
| A new JSON field reads as its default on the board | Not added to the ArduinoJson **filter whitelist**. The header parse is a whitelist — unlisted fields are silently stripped | `handleSerialInput()` |
| Board appears frozen with no host attached | TX timeout tuning. 0 drops bytes when the host is briefly slow; ~100 ms stalls the loop. 50 ms is the deliberate middle | `setup()` |

## Mesh / Via WCB

| Symptom | Cause | Where |
|---|---|---|
| Command silently does nothing over the bridge | Payload exceeded **187 bytes** → the bridge's `\|CRC` suffix truncated → dropped as "Missing CRC". Measure escaped UTF-8 bytes | [PROTOCOLS.md §1](PROTOCOLS.md#1-transport-overview) |
| Save over WCB fails with no ACK | Fragment loss. Background polls interleaving the stream is the usual cause — `_bridgeUploadInFlight` quiesces them. Pacing below 150 ms overflows the WiFi TX queue mid-burst | `sendJSON()` |
| RC resets (task watchdog) during a bridged save | The whole `mappings` branch shipped instead of a per-button sub-diff — ~40 fragments | `_diffMappings()` |
| Reassembled config is garbage / shows `U+FFFD` | A multi-byte character split across fragments. Slices must break on codepoint boundaries | `_utf8Chunks()`, `_startFragSend()` |
| Two tools fight, fragments interleave | Sessions key on `(sid, senderID)` — both tools start at `sid = 1` | `_findOrAllocSession()` |
| Board silently unreachable on the mesh | Mesh channel mismatch. One radio, so `wcbNetwork.channel` must match the fleet. Set before `begin()`; **needs a reboot** | `setup()` |
| WCB Status shows fewer boards than expected | The reply is shrunk to fit one ESP-NOW packet — aliases dropped, then relay name, then the roster trimmed. `quantity` is still reported, so the tail renders as placeholders | `buildWcbStatus()` |
| Remote Maestro read shows `(no response)` | The WCB-side read verb has not shipped yet | [ROADMAP.md §2](ROADMAP.md#2-wcb-native-maestro--partially-shipped) |
| `skipRunning` action fires when it should have been gated | The gate **fails open** by design — no busy reply within `maeGateMs` (default 250) means fire anyway | `maestroVerbBusy()` |
| `[CLIPDL:ERR] clip too large to edit over the WCB bridge` | Over 3000 events on the relayed path. Connect over USB | `execCliLine()` |
| Bridge WCB reboots when a second tab connects | DTR toggling. The shared hub never asserts DTR and does not hand off on visibility, for this reason | `serial-hub.js` |

## Inputs and dispatch

| Symptom | Cause | Where |
|---|---|---|
| One press registers as a double | A one-frame NEUTRAL dip split the press. Raise `matrixDebounceFrames` (1–4) | matrix edge machine |
| Fast re-press of the same button is dropped | A true sub-frame tap (press+release inside one ~9–14 ms frame) is unrecoverable — an SBUS-rate limit, not logic | `processSbus()` |
| A button fires once at power-on | `matrixArmed` starts **false** specifically so a button already in-band at boot cannot fire | matrix edge machine |
| Nothing dispatches at all | `calibrationActive` is set. A crashed/closed calibration page can leave it — a fresh `PING` or `STOP_MONITOR` clears it | `handleSerialInput()` |
| A configured band never triggers | A `0/0` threshold is the inert "Unassigned" sentinel and never matches | `pwmToButton()` |
| Two delayed actions fire together instead of in sequence | Delays are measured **from the trigger** and run in parallel, not cumulatively | [MAESTRO_ACTIONS.md](MAESTRO_ACTIONS.md) |
| A servo moves at the wrong speed long after the action that set it | Maestro speed/accel are **sticky limits** on the channel. Reset to 0 to clear | [MAESTRO_ACTIONS.md](MAESTRO_ACTIONS.md) |
| Servo buzzes or hunts at rest | Set `releaseIdleMs` to de-energize after idle. **A released servo has no holding torque** — not for a load-bearing bearing | `maestroIdleReleaseTick()` |
| Panels sit half-open at stick rest | Use `midClosed` on the output: centre maps to `posMin`, only the upper half of travel sweeps | `sbusToRangeMidClosed()` |
| Remote trigger behaves differently from a local press | It must be dispatching on Core 1. Remote `TRIGGER` is queued through `remoteTriggerQueue`, not run inline on Core 0 | `drainRemoteTriggers()` |

## Serial peripherals

| Symptom | Cause | Where |
|---|---|---|
| HCR does nothing from a mapped button | Run `#L20` (S3) or `#L21` (S4) — they bypass config *and* mapping. Reacts → the fault is config/mapping. Silent → wiring, ground, or the port | `execCliLine()` |
| Garbled output above ~57600 baud | S4/S5 are bit-banged `SoftwareSerial`. Only S3 (hardware UART0) is reliable at higher rates | `applySerialBauds()` |
| WLED unreliable | It wants 115200, so it needs S3 | `RcWledSlot` |
| A live port blips on an unrelated save | Only ports whose baud actually changed are re-opened — verify the guard still holds | `applySerialBauds()` |

## Build and tooling

| Symptom | Cause | Where |
|---|---|---|
| `'class WCB_Client' has no member named 'setPortLabel'` | Sketchbook `WCB_Client` is 1.10.0; `main` needs ≥ 1.11.0 | [BUILD_AND_RELEASE.md §3](BUILD_AND_RELEASE.md#3-verifying-a-firmware-change) |
| `#error "rc_serial.h requires USB CDC on boot"` | FQBN missing `USBMode=hwcdc,CDCOnBoot=cdc` | `rc_serial.h` |
| Flasher installs an old build | Stale bins accumulated and the alphabetical `.find()` locked onto the oldest. CI stages deletions with `git add -A firmware/` | `build-firmware.yml` |
| Config tool loads but nothing responds to clicks | JavaScript syntax error — there is no build step to catch it. Run `node C:\Users\ghulette\tools\jscheck.js config_tool/index.html` | [CONFIG_TOOL.md](CONFIG_TOOL.md) |
| Firmware CI did not run | The paths filter excludes `fw_version.h` (the hook stamps it every commit). A bare version bump needs `workflow_dispatch` | `build-firmware.yml` |
| A library fix did not reach the flashed board | CI clones `greghulette/WCBClient`; the `Arduino-Code` copy is local-bench only | [BUILD_AND_RELEASE.md §2](BUILD_AND_RELEASE.md#2-dependencies) |
| Two tabs cannot share a port | They are not same-origin. `BroadcastChannel` and Web Locks are origin-scoped | `serial-hub.js` |

## Config tool state

| Symptom | Cause | Where |
|---|---|---|
| A save pushed defaults and cut the bridge | Saved before the first `CONFIG` arrived — `_configLoaded` guards this | `saveConfigToBoard()` |
| Every save right after a load ships the whole config | Key-order instability in the diff. `_stringifyStable()` sorts keys before comparing | `_diffConfigBranches()` |
| Phantom "unsaved changes" prompt | Render-time padding ran after the baseline snapshot. `_hwSetupBaseline` (on open) and `_postFlashReload` (after flashing) exist for this | save guard rails |
| Deleting a button mapping does not take on the board | An absent key means "leave untouched" firmware-side; a deleted button must ship as `{}` | `_diffMappings()` |

---

## When the cause is not here

1. Reproduce with the relevant `SET_DEBUG_FLAGS` category on — the chips gate firmware output,
   not just the display.
2. `#L11` for mesh state, `#L12` for mode/button decode, `#L09`/`#L13` for SBUS.
3. Check whether the path crosses cores. A hang or corruption under mesh traffic is usually
   work running inline on Core 0 that should have been queued —
   [ARCHITECTURE.md §8](ARCHITECTURE.md#8-concurrency-model--the-single-most-important-invariant).
4. Read the source comment nearest the code. Most non-obvious choices here record the failure
   that caused them.

---

## Revision log

Newest first. Add a row whenever a code change alters what this page describes — same commit
as the code. Page body stays present-tense; history lives here.

| Date | Commit | Change |
|---|---|---|
| 2026-08-04 | _(uncommitted)_ | Initial version. |
