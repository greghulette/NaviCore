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
| Board reboots when the config tool connects (incl. every page refresh) | **Expected on a NaviCore v2, not a fault.** `Reset reason: 11 - USB peripheral`, RTC code 21 = `USB UART chip reset`. The S3 has no bridge chip: the USB Serial/JTAG peripheral itself resets the chip from the host CDC control lines — the mechanism esptool uses. It fires on port **OPEN** (the boot log appears as the port opens; the first PING follows `settleMs` later), and Chrome asserts DTR/RTS as part of `open()` with no Web Serial way to suppress it. The `pagehide` teardown does **not** address it — that only cleans the close edge. The only real lever is the `USBMode` FQBN field (`hwcdc` → USB Serial/JTAG hardware reset; `default`/TinyUSB implements reset in software instead) — a load-bearing build change needing hardware validation, not a config tweak. **Consequence: every RAM-only counter resets on connect, including the mesh stats** | `printBootTelemetry()` |
| Board appears frozen with no host attached | TX timeout tuning. 0 drops bytes when the host is briefly slow; ~100 ms stalls the loop. 50 ms is the deliberate middle | `setup()` |
| Remote Maestro passthrough choppy while the live monitor is open | The monitor's per-frame USB write can back-pressure and stall `loop()`, which paces the passthrough broadcast. Now best-effort — drops the frame if the tx buffer is full. **First** rule out servo power: servos off USB power sag under load (that was the real cause once) | `sendPWMUpdate()` |
| Tool says "Connected" but nothing arrives — panel frozen, triggered requests get no reply, passthrough still works | The USB device re-enumerated (board reboot / power cycle / cable) and the `SerialPort` handle is dead. The `[monitor]` line shows `resubscribe FAILED: The device has been lost.` with `lastRx` frozen. **Two independent detectors, because either alone misses cases.** The read loop treats `NetworkError`/`InvalidStateError` (or 40 consecutive failures) as fatal and breaks — but a `read()` on a device that vanished can park forever and never reject, so that path may never fire at all. A **write** rejects instantly in that state, so `sendLine()` applies the same classification to write failures. Either detector runs `handleLinkLost()` → teardown + auto-reconnect, and logs `[link] serial link lost` to the terminal | `startReading()`, `sendLine()`, `handleLinkLost()` |
| SBUS panel stale — **read the `[monitor]` line in the terminal first** | It self-reports: `lastRx` still moving = the monitor subscription lapsed (re-subscribe should recover it); `lastRx` frozen too = nothing is arriving at all, so suspect the link or a shared-port hub whose leader died. `port=shared` means another tab owns the port | `_monitorDiag()`, `_sbusStaleTick()` |
| SBUS panel reads "No data — link idle" but the passthrough still follows the stick | The live-**monitor** stream stopped, not SBUS. A board reboot resets `wsMonitorActive`, so the frozen tool never gets a fresh `START_MONITOR`. `_sbusStaleTick()` auto-re-subscribes when the panel goes stale — **including in shared-port mode**, where the hub owns the port and the tool's own `port` is null (gating on `port` alone left a second tab stuck forever). Over Via WCB the 10 s keep-alive PING renews the `rc_ch` subscription; if that timer is dead the stream stops after 15 s. Otherwise disconnect + reconnect | `sendPWMUpdate()`, `_sbusStaleTick()`, `_wcbKeepaliveTimer` |

## Mesh / Via WCB

| Symptom | Cause | Where |
|---|---|---|
| Command silently does nothing over the bridge | Payload exceeded **187 bytes** → the bridge's `\|CRC` suffix truncated → dropped as "Missing CRC". Measure escaped UTF-8 bytes | [PROTOCOLS.md §1](PROTOCOLS.md#1-transport-overview) |
| Save over WCB fails with no ACK | Fragment loss. Background polls interleaving the stream is the usual cause — `_bridgeUploadInFlight` quiesces them. Otherwise suspect pacing: `FRAG_PACE_FLOOR_MS` (100 ms) is a tuned value, and 50 ms is a known hard failure — "[SEND CB] MAC-layer FAILED" in the RC's serial log is the signature. Raise the floor before looking elsewhere | `sendJSON()` |
| RC resets (task watchdog) during a bridged save | The whole `mappings` branch shipped instead of a per-button sub-diff — ~40 fragments | `_diffMappings()` |
| Reassembled config is garbage / shows `U+FFFD` | A multi-byte character split across fragments. Slices must break on codepoint boundaries | `_utf8Chunks()`, `_startFragSend()` |
| Two tools fight, fragments interleave | Sessions key on `(sid, senderID)` — both tools start at `sid = 1` | `_findOrAllocSession()` |
| Board silently unreachable on the mesh | Mesh channel mismatch. One radio, so `wcbNetwork.channel` must match the fleet. Set before `begin()`; **needs a reboot**. Valid range is **1–11**, not 1–13: `WCB_Client::setMeshChannel()` rejects >11 and returns *without setting*, so a stored 12/13 leaves the radio on `WCB_MESH_CHANNEL` while every readout claims 12/13. Out-of-range values now fall back to 1 (= `WCB_MESH_CHANNEL`, the channel the radio was actually on) rather than clamping to 11, which would move a working board off its fleet | `setup()` |
| WiFi enabled but no network appears | Check the boot log — the bring-up is loud on every path. `[WIFI] REFUSED:` means the password is empty or under 8 chars and the AP was **not** started (fail closed — it will never fall back to an open network). `[WIFI] SoftAP ... FAILED` means `softAP()` itself returned false. **No `[WIFI]` line at all** means `wifiEnabled` was false at boot: it is read once in `setup()`, so a Save without a reboot changes nothing. Confirm what the board actually holds with `GET_CONFIG`, not what the tool shows | `setup()` |
| Nothing is sent and nothing arrives, but there is **no** fault LED | Mesh password empty or not matching the fleet. It is a plain-text namespace field on every packet, strncmp'd on receive — but `begin()` does not validate it, so it returns true and latches no fault. An empty one prints a boxed banner at boot; a *wrong* one is indistinguishable from a dead mesh | `setup()` |
| Half a Via-WCB action works and the rest silently does nothing | A `^`-chain sent **unicast** containing an *implicitly-routed* verb — `;M` `;L` `;H` `;A` `;D` `;C`/`;SEQ` — whose device lives on a board other than the target. A mesh-arrived command is not re-routed onward, so that part is dropped with no error. Explicit `;w<n>` routing is **not** capped, so a chain of `;w`/`;s` parts is fine. Broadcast, or split into one action per part | [PROTOCOLS.md §1](PROTOCOLS.md#1-transport-overview) |
| A board was absent the whole session and nothing said so | The boot roll call names every configured board never heard from, once, 30 s after join. A board that comes up and *later* drops produces an `ONLINE`/`OFFLINE` transition line instead | `checkBootRollCall()`, `onWcbStatus()` |
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
| Passthrough servo jumps when you flip the **global mode** switch | A knob bound to its OWN switch via `modeSwitchOverride` was re-armed by the global-mode change and re-dispatched to the current stick position. Now only true global-mode followers (`modeSwitchOverride < 0`) re-arm. If it still moves, it's `releaseIdleMs` going limp then snapping back on the next dispatch | `resetModeAwareKnobs()` |
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
| 2026-08-28 | _(uncommitted)_ | The SoftAP farewell now covers **all three** restart paths, not just the `REBOOT` command: local OTA (`?OTALOCAL,END`), relay OTA, and `REBOOT` all call `naviota::otaFarewellAP()`. This is the one that matters in practice — config changes needing a restart are rare, firmware updates are the common case, and every one ends in a restart. A bare `ESP.restart()` drops the AP silently, so the client keeps talking to an AP that is gone and only times out: ~11 s of dead air while the board is serving again at 2.4 s. |
| 2026-08-28 | _(uncommitted)_ | `REBOOT` now deauthenticates SoftAP clients before restarting (`WiFi.softAPdisconnect(false)`, gated on `wifiEnabled`). A bare `ESP.restart()` drops the AP without telling anyone, so an associated client keeps talking to an AP that is gone and only discovers it by timing out — measured with ping across a reboot: **11 s of "Request timed out" while the board was serving again after 2.4 s**. That gap belongs to the client and no reconnect logic on the far end can shorten it. `false` keeps the radio up for ESP-NOW until the restart. |
| 2026-08-28 | _(uncommitted)_ | Added a row for "WiFi enabled but no network appears", keyed on the `[WIFI]` boot lines — REFUSED (short/empty password, fails closed), FAILED (`softAP()` returned false), or no line at all (flag was false at boot; it is read once in `setup()`, so a Save without a reboot does nothing). |
|---|---|---|
| 2026-08-27 | _(uncommitted)_ | Mesh-channel row: recorded that the valid range is 1–11, that `setMeshChannel()` silently ignores 12/13 (leaving the radio on `WCB_MESH_CHANNEL` while every readout claims otherwise), and that out-of-range now falls back to 1 rather than 11. Bridged-save row: pointed at `FRAG_PACE_FLOOR_MS` as the thing to raise first, and named "[SEND CB] MAC-layer FAILED" as the signature of pacing set too low (50 ms is a known hard failure). |
| 2026-08-19 | _(uncommitted)_ | Second half of the "connected but dead" fix: breaking the read loop on a fatal `NetworkError` was not enough, because a `read()` parked on a device that has vanished can simply never settle — no rejection, no teardown. Observed for 1.8 h with the panel frozen while the monitor self-heal re-sent `START_MONITOR` every 3 s and merely logged each `The device has been lost.` rejection. **A write is the only liveness proof once the read side goes quiet** — `sendLine()` now applies the same fatal classification to write failures and calls `handleLinkLost()`, which also announces itself in the terminal. `startReading()` is bound to the port object it was started for, so a zombie read that settles after a reconnect cannot grab the new session's stream or tear it down. |
| 2026-08-19 | _(uncommitted)_ | Root cause of the long-standing "tool connected but dead" report, found by the new `[monitor]` diagnostic: `startReading()`'s catch treated a fatal `NetworkError` ("The device has been lost.") as a recoverable read glitch and retried every 50 ms forever, so the loop never exited and `handleLinkLost()` never ran — UI stayed "Connected" with a frozen panel for as long as 16.7 h. Now breaks on `NetworkError`/`InvalidStateError`, plus a 40-consecutive-failure backstop; `readFailStreak` resets on any successful chunk. |
| 2026-08-18 | _(uncommitted)_ | The live-monitor watchdog now reports itself. Whenever the SBUS panel is stale, `_monitorDiag()` prints one throttled line (max 1 per 10 s) to the terminal naming transport, port owner, hub role, `isMonitoring`, **time since ANY inbound line and its type**, and what the re-subscribe attempt did. `lastRx` is the discriminator: still climbing traffic = the monitor subscription lapsed; `lastRx` climbing with it = the link or the shared-port hub. The re-subscribe is no longer gated on `isMonitoring` — a partial disconnect can leave that flag false while the port is live, which silently disabled the only recovery path. |
| 2026-08-18 | _(uncommitted)_ | `_sbusStaleTick()`'s monitor self-heal was gated on `port`, so it never fired in shared-port mode — the hub owns the port there and the tab's own `port` is null, leaving the live panel at "No data — link idle" indefinitely while the droid was fine. Now `port || sharedActive`, matching the Via-WCB keep-alive, which already carried that fix for the same reason. |
| 2026-08-13 | _(uncommitted)_ | Confirmed the reboot-on-connect: `ESP_RST_USB` (11) / RTC 21, fired by the USB Serial/JTAG peripheral on port OPEN. Corrected the row — the `pagehide` teardown does not address it, and the only lever is the `USBMode` FQBN field. `printBootTelemetry()` now names reasons 11/12 and the common RTC codes instead of printing "other". |
| 2026-08-13 | _(uncommitted)_ | Added the reboot-on-page-refresh row, pointing at the existing `Reset reason:` boot line as the thing that distinguishes an EN reset from a watchdog stall. |
| 2026-08-12 | _(uncommitted)_ | Added Mesh rows for the three Sabé-review adoptions: empty/mismatched mesh password (no fault LED — `begin()` does not validate it), `^`-chain unicast dropping an *implicitly-routed* part (`;M` `;L` `;H` `;A` `;D` `;C`/`;SEQ`) whose device is hosted off-target — explicit `;w<n>` routing is not capped — and the boot roll call naming boards never heard from. |
| 2026-08-12 | _(uncommitted)_ | Added rows: choppy remote passthrough while live-monitoring (`sendPWMUpdate` loop-stall, now guarded) + servo-power caveat; "SBUS panel No data but passthrough works" (stale monitor after reboot, tool auto-re-subscribes); passthrough servo jumps on global-mode flip (`resetModeAwareKnobs` no longer re-arms override-switch knobs). |
| 2026-08-04 | _(uncommitted)_ | Initial version. |
