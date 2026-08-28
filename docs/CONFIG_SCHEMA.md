# NaviCore — Configuration Model

`RcConfig` is the whole user-facing product surface. Almost every feature request lands
here first. This document is the reference for the struct, its JSON form, how it persists,
and the checklist for extending it without breaking the tool.

Defined in [`rc_config.h`](../rc_config.h). Related: [PROTOCOLS.md](PROTOCOLS.md) ·
[CONFIG_TOOL.md](CONFIG_TOOL.md)

---

## 1. Where the config lives at runtime

One global, allocated from PSRAM in `setup()`:

```cpp
g_rcConfig = (RcConfig*) ps_calloc(1, sizeof(RcConfig));   // ~210 KB
#define rcConfig (*g_rcConfig)
```

It is roughly 210 KB — far too large for internal SRAM, which is why `PSRAM=opi` is a hard
build requirement and why a failed allocation halts the board with a solid red LED rather
than crashing later.

Size discipline matters: `RcAction::cmd[96]` is multiplied by
`RC_NUM_MAPPINGS × RC_NUM_TAP_TIERS × RC_ACTIONS_PER_TIER = 108 × 4 × 5 = 2160` action slots.
Widening a field in `RcAction` costs kilobytes per byte.

---

## 2. Capacity constants

| Constant | Value | Meaning |
|---|---|---|
| `RC_NUM_PHYSICAL` | 21 | Matrix slots drawn on the transmitter graphic (21 = inert "Unassigned" sentinel) |
| `RC_NUM_LOGICAL` | 15 | User-defined logical buttons (slots 22–36) |
| `RC_NUM_THRESHOLDS` | 36 | Total matrix-channel bands |
| `RC_NUM_MAPPINGS` | 108 | 3 modes × 36 slots |
| `RC_ACTIONS_PER_TIER` | 5 | Actions per tap tier |
| `RC_NUM_TAP_TIERS` | 4 | Tap tiers per mapping — `t1`/`t2`/`t3` taps, `t4` = long press (`RC_TAP_LONG`) |
| `RC_NUM_SWITCHES` | 10 | SA–SJ |
| `RC_NUM_KNOBS` | 11 | S1, S2, LS, RS, S3, J1–J4, J5, J6 |
| `RC_KNOB_MAX_OUTPUTS` | 10 | Passthrough/volume outputs per knob |
| `RC_NUM_MAESTROS` | 8 | Logical Maestro slots |
| `RC_MAESTRO_CHANNELS` | 32 | Pololu max channels |
| `RC_NUM_WLED` | 4 | WLED routing slots (ids 1–9 addressable) |
| `RC_NUM_SMOOTH_PROFILES` | 6 | Smoothing profiles |
| `RC_MAX_WCB_PROFILES` | 6 | Saved WCB-credential profiles (Dev/In-droid/…) |

---

## 3. Top-level `RcConfig`

| Field | Type | Purpose |
|---|---|---|
| `txModel` | `uint8_t` (`RcTxModel`) | Transmitter model — drives the GUI's SVG, default channels, and which controls appear |
| `threeAxisGimbals` | `bool` | X20 only: show/hide J5/J6 twist axes and stick-click buttons |
| `sbusOutEnabled` | `bool` | Enable the SBUS passthrough tee (applies live) |
| `wifiEnabled` | `bool` | Raise a SoftAP + comms server for the desktop app. **Defaults false, and false means the bring-up code never runs** — not "runs idle". Read at boot only: changing it needs a Save *and* a reboot. NVS key `wifien`. Deserialised with `\| false` so a config or backup predating the field can never enable the radio. The AP must be raised on `wcbNetwork.channel` — one radio, so any other channel takes the droid off its own mesh. **No visible UI**: the toggle lives in the cloud-backup modal, itself behind the 4×-wordmark gesture ([CONFIG_TOOL.md](CONFIG_TOOL.md)) |
| `wifiSsid` | `char[33]` | SoftAP SSID. 32 bytes is the 802.11 max, +1 NUL. **Empty = derive `NaviCore-<deviceId>` at bring-up**, so a droid always has a distinguishable name without the user inventing one |
| `wifiPassword` | `char[64]` | SoftAP WPA2 passphrase, 8–63 chars +1 NUL. **Empty or <8 must refuse to raise the AP — fail closed, never fall back to an open network.** `WiFi.softAP()` will happily create an open AP on an empty password, and this command surface has no per-command auth (`RESET_DEFAULTS`/`REBOOT` dispatch on a bare `type`), so an open AP is an unauthenticated command channel to the whole mesh. **Never reuse `wcbNetwork.password`** — that one rides in cleartext in every ESP-NOW packet the droid emits and is public by construction. NVS key `wifi` (a JSON blob, kept separate from `wcb` so mesh and AP credentials cannot be confused) |
| `boardType` | `uint8_t` | 0 = NaviCore v2 PCB, 1 = WCB HW 3.2 — selects the pin profile |
| `tapWindowMs` | `int` | Multi-tap detection window |
| `holdMs` | `int` | Long-press (tier `t4`) threshold, default 750. **Must exceed `tapWindowMs`** — the tap dispatch is deferred by `tapWindowMs` and would fire first. Both sides clamp a too-small value to `tapWindowMs + 250`, and 5000 ms is the ceiling |
| `switchSettleMs` | `uint16_t` | A switch position must hold this long before its tier fires, default 80, clamped 0–1000. **0 = fire immediately (pre-settle behaviour).** Without it, a 3-position switch swept end-to-end fires the *middle* tier in full on the way past |
| `chRateHz` | `uint8_t` | `rc_ch` broadcast rate, 1–20 (default 5). High rates flood the mesh |
| `matrixChannel` | `int` | SBUS channel carrying the button matrix |
| `matrixDebounceFrames` | `int` | 1–4 consecutive in-band frames to commit a press/re-arm |
| `thresholds[36]` | `RcThreshold` | `{id, label[24], minPwm, maxPwm}` — a `0/0` band is inert |
| `mappings[108]` | `RcMapping` | `{exclusive, RcTier t[4]}` — indexed `(mode-1)*36 + (slot-1)`. `t[3]` is the long press and is **always dispatched exclusively**, whatever `exclusive` says |
| `switches[10]` | `RcSwitch` | `{channel, positions, RcTier t[3]}` (down/mid/up) |
| `knobs[11]` | `RcKnob` | See below |
| `funcBindings` | `RcFuncBindings` | `{modeSwitch}` — which switch selects the global mode |
| `hcrDest` | `RcHcrDest` | **Global** HCR destination |
| `maestros[8]` | `RcMaestroSlot` | `{type, device, channels[32]}`. Per-channel `{ch, name, min, max}` endpoints are in **quarter-µs** (the Pololu wire unit). They are not cosmetic: they set the timeline's vertical scale, **clamp every timeline edit** (`_tlClampUs`), and are pushed onto passthrough knob outputs by `_applyMaestroRangesToKnobs` — so a wrong endpoint is also what a stick maps to. Editable by hand in the tool (Maestro tab → ✎) as well as by importing a Control Center settings file |
| `wcbNetwork` | `RcWcbNetwork` | ACTIVE mesh credentials (runtime; `wcb_config.h` only seeds defaults) |
| `wcbProfiles[≤6]` + `wcbProfileCount` | `RcWcbProfile` | Saved mesh identities (`{name[24], macOct2, macOct3, password[40], quantity, deviceId, channel}`) the config tool switches between. **Inert storage** — firmware never acts on them; loading one just writes `wcbNetwork` (tool-side, Direct USB) |
| `mp3Dest` | `RcMp3Dest` | **Global** MP3 Trigger destination |
| `dfpDest` | `RcDfpDest` | **Global** DFPlayer Mini destination — `{transport, target}`, defaults local `S3` |
| `wledSlots[4]` | `RcWledSlot` | Per-id WLED routing |
| `auxBaud[3]` | `uint32_t` | `[0]`=S3, `[1]`=S4, `[2]`=S5 |
| `maestroBaud` | `uint32_t` | Serial2 line rate |
| `serialLabels[4][25]` | `char` | WDP port-label **overrides**, indexed by FIRMWARE port (`RC_SLBL_S3`/`S4`/`S5`/`MAESTRO`); `""` = auto-derive |
| `serialBcastOut[3]` / `serialBcastIn[3]` | `bool` | Per-aux-port mesh bridging, indexed like `auxBaud` (`[0]`=S3, `[1]`=S4, `[2]`=S5). JSON key `serialBcast`, keyed `"S3"/"S4"/"S5"` with `{out,in}`. Both default **off** — a port only joins the broadcast domain when asked; targeted `;s<n>` writes need neither flag. See [ROADMAP.md §1](ROADMAP.md) |
| `maeGateMs` | `uint16_t` | Remote Maestro busy-gate validity (default 250; fails **open**) |
| `smoothProfiles[6]` | `RcSmoothProfile` | ~4.6 KB of per-mode/per-channel speed+accel |
| `peerNewActions` | `RcTier` | Up to 5 actions fired when a new mesh peer appears |
| `peerAlert` | `bool` | Also flash the LED and print a terminal line |
| `modeReport` | `RcModeReport` | `{enabled, wcb, tmpl[48], cmds[3][48]}` — optional: send the mode-select position to one WCB on every change and every 60 s. `{mode}` in `tmpl` → the position; a non-empty `cmds[mode-1]` overrides it |
| `statsReport` | `RcStatsReport` | `{enabled, wcb}` — `enabled` states that this droid uses mesh stats (drives the tool's default view); `wcb` is an **optional** collector, 0 = collect/display only. See below |

**The counters are never gated by this.** `WCB_Client` accumulates from `begin()` and `g_meshRxCount` counts from boot, so a tool that starts reading mid-session still sees everything since the last reset. `enabled` is a statement of intent (show me these), not a switch on collection; `wcb` = 0 means collect and display without shipping anywhere.

They can be zeroed **without a reboot** by `{"type":"RESET_MESH_STATS"}` (Mesh Stats → ⌫ Clear), which resets the library's send-side counters plus NaviCore's own `g_meshRxCount`/`g_meshRxFrom`. **The reset is deferred to `loop()` on both transports** — `WCB_Client::resetStats()` takes the pending-table lock and races the RX task's `ackd` increment, so the library forbids calling it from a receive callback, which is exactly where a bridged request arrives.

**`statsReport` — `?` and not `;` is load-bearing.** The report is one command:

```
?STATS,RPT,<from>,<sent>,<ackd>,<retries>,<failed>,<unguaranteed>,<bcast>,<recv>
```

`executeCommand()` routes a `?` command to `processLocalCommand()` and returns
(`WCB.ino` ≈3964), so a report is handled **locally on the receiving board and can never
fall through to `processBroadcastCommand()`** — it is never written to that board's serial
ports and never re-broadcast to the mesh. A `;` verb would have to be excluded from those
paths by hand; `?` is that exclusion by construction.

`<from>` travels in the payload because `processLocalCommand()` takes no `sourceID`
(`WCB.ino` ≈3965) — a `?` handler cannot tell who sent it.

The receiver stores it in `reportedStats[]`, **RAM-only**, surfaced under `?STATS` in a
*Reported by Other Nodes* section and cleared by `?STATS,RESET` or a reboot. A report with
fewer than 8 fields is dropped whole rather than stored partially, so a truncated packet
cannot leave a row that looks like data with zeros in the missing columns.

This carries **only this board's own numbers** — every other node reports its own the same
way. It is never a fleet roll-up. **Requires WCB firmware with `?STATS,RPT`.**

### `RcAction` — the atom

```cpp
struct RcAction {
  RcActionType type;      // see ARCHITECTURE.md §10
  char     target[6];     // WCB id "1".."20" | Maestro slot "1".."8" | port "S3".."S5"
  char     cmd[96];       // command string, or Maestro verb "setTarget,ch,pos"
  uint16_t delayMs;       // measured from the TRIGGER, parallel — not cumulative
  char     note[20];      // GUI caption
  bool     skipRunning;   // remote Maestro: skip if a sequence is already running
  uint8_t  fn;            // HCR function / MP3 function
  int8_t   chan;          // HCR emotion or audio channel
  int16_t  track;         // track #, volume, level, seconds…
};
```

Two encodings in `chan` deliberately differ and must not be "harmonised":
HCR **fn 18/19** (volume up/down) use `0 = ALL, 1 = V, 2 = A, 3 = B`, while
**fn 14/16/17** (audio) use `0 = V, 3 = ALL`. Legacy actions depend on it.

### `RcKnob` and `RcKnobOutput`

A knob/slider/joystick axis is a *source* with up to 10 *outputs*:

| `RcKnob` field | Purpose |
|---|---|
| `channel` | SBUS 1–24, 0 = disabled |
| `function` | `KF_NONE` / `KF_MAESTRO_PASSTHROUGH` / `KF_HCR_VOLUME` |
| `reverse` | Invert around centre |
| `modeAware` | Opt-in: the **outputs** follow the mode switch (source stays global) |
| `modeSwitchOverride` | `-1` = global mode switch, else switch index 0–7 |
| `smoothProfile` | `-1` = none, else index into `smoothProfiles[]` |

| `RcKnobOutput` field | Purpose |
|---|---|
| `target` | Maestro slot 1–8, or HCR audio channel 0=V/1=A/2=B |
| `maestroCh` | Servo channel 0–31 |
| `posMin` / `posMax` | Value at SBUS min/max (¼ µs, or 0–99 volume) |
| `smoothSpeed` / `smoothAccel` | Hardware ramping; 0 = leave the channel's own limits alone |
| `midClosed` | Centre = `posMin`; only the upper half of travel sweeps (keeps panels shut at rest) |
| `releaseIdleMs` | De-energize after this long with no movement; 0 = never. **A released servo has no holding torque** |

### Destinations are global, not per-action

`hcrDest` (`transport` 0 = local `S3`/`S4`/`S5`, 1 = WCB with `wcbPort`), `mp3Dest` and
`dfpDest` (both 0 = local serial, 1 = WCB unicast) are configured once. HCR/MP3/DFPlayer
actions carry only `fn`/`chan`/`track`. WLED is different — routing is **per id** in
`wledSlots[]`.

**`transport` 2 = DISABLED**, serialised as `"off"`. All three devices default to it, and
**every one of them starts disabled on a fresh config**. A board cannot know which of HCR /
MP3 Trigger / DFPlayer is actually wired, and defaulting a device onto a real port means its
actions fire blind at whatever else shares that wire. The executor refuses the action outright
(`executeHcrAction` / `executeMp3Action` / `executeDfpAction` return early, as does
`dispatchHcrVolume`) — the check is at the executor, not the port, so a stale `target` left
over from an earlier setup cannot leak output.

The stored `port`/`target` is preserved while disabled, so re-enabling a device puts it back
where it was rather than on a default. An existing stored config is unaffected: it carries an
explicit `transport`, so upgrading does not silently switch a working device off.

All three live in the config tool's single **Audio** tab, as the first entry in each device's
**Via:** dropdown.

**The two audio players' volume scales are inverse.** MP3 Trigger: `0` = loudest …
`64` = inaudible. DFPlayer: `0` = silent … `30` = loudest. Nothing converts between them —
each action type carries its own device's native value, and copying a number from one to the
other is always wrong. See [DFPLAYER_DESIGN.md](DFPLAYER_DESIGN.md).

### Maestro slots are logical

Actions reference slot **1–8**. The slot stores `type` (0 disabled / 1 local Serial2 /
2 remote), the Pololu `device` number (0–127, always Pololu protocol so the device filter
disambiguates a shared bus), and imported per-channel names/endpoints the firmware never
acts on — they exist so the tool's labels travel with the config.

### Serial port labels

`serialLabels[4]` is indexed by **firmware** port — `RC_SLBL_S3`/`S4`/`S5`/`MAESTRO` — so the
stored config stays independent of the mesh S1/S2/S3 numbering, and its JSON keys are the same
strings `auxBaud` uses: `"S3"/"S4"/"S5"/"maestro"`. There is no SBUS slot. An empty string
falls back to `rcSerialLabelAuto()`, which derives a label from
what the config routes there (`Maestro`, `HCR`, `MP3`, `DFPlayer`, `WLED`). That one
function is also what `auxPortHasDevice()` reads, so a device added there is automatically
excluded from the serial broadcast fan-out — there is no second list. `rcSerialLabel()` applies
override-then-auto, and `rcAdvertiseSerialLabels()` pushes the four through
`rcWdpPortForLabel()`: S3/S4/S5 advertise as **WDP 1/2/3**, the local Maestro as **WDP 4**
(labelled but deliberately not reachable by `;s<n>` — it is a binary bus), then WDP 5 is
explicitly cleared because NaviCore has no fifth labelable port.

**An unrecognised key is worse than ignored.** `rcConfigFromJSON()` memsets all four slots
before matching keys, so a `serialLabels` object written the old way (WDP numbers `"1"`–`"5"`)
does not just fail to apply — it clears every stored override, and every port falls back to
its auto-derived default.

---

## 4. JSON serialisation

| Direction | Function | Notes |
|---|---|---|
| struct → JSON | `rcConfigToJSON()` | 64 KB document — sized for 6 smoothing profiles |
| JSON → struct | `rcConfigFromJSON(JsonObject)` / `(String)` | Tolerant of missing keys; absent field = leave default |

Serialisation is **sparse** where it pays: empty serial-label overrides, all-zero Maestro
channel metadata, and unused slots are omitted. That matters because the bridged upload
path caps at ~15 KB (see [PROTOCOLS.md §4](PROTOCOLS.md#4-the-via-wcb-bridge)).

`SET_CONFIG` re-applies live, without a reboot:
`applySerialBauds()` (only ports whose baud actually changed re-open, so an unrelated save
does not blip a live port), `applySbusOut()`, and a board-profile change check. Mesh
credentials are the exception — `WCB_Client` is constructed once, so those need a reboot.

---

## 5. Persistence

| Store | Path | Role |
|---|---|---|
| LittleFS (`spiffs`) | `/config.json` | **Primary** |
| LittleFS | `/config.json.tmp` | Atomic-write staging |
| LittleFS | `/cmdlib.json` | Command library (opaque to firmware) |
| LittleFS | `/cmdlib.json.tmp`, `/cmdlib.json.bulk.tmp`, `/cmdlib.send.tmp` | Staging; the bulk/send temps are removed at boot |
| NVS namespace `rcfg` | keys split under the 4000 B/value limit | **Legacy** — read once to migrate forward |

Load order in `setup()`: try LittleFS → if the file exists but will not parse, **keep it**
and run on defaults for this boot (never overwrite a possibly-good config) → if no file
exists, read NVS and save it forward once.

The command library is stored **verbatim**, pulled out of `SET_CMDLIB` by substring rather
than re-serialised, and identified by size + FNV-1a hash so the tool can skip re-pulling an
unchanged library.

---

## 6. Cross-file invariants

These constants exist in two or more places and **must be changed together**. This is the
highest-risk category of edit in the repo.

| Firmware | Config tool | Value |
|---|---|---|
| `RC_NUM_TAP_TIERS` / `RC_TAP_LONG` | `NUM_TAP_TIERS` / `TAP_LONG` / `TIER_KEYS` | 4 (`t1..t4`, 4 = long press) |
| `RC_KNOB_MAX_OUTPUTS` | `KNOB_MAX_OUTPUTS` | 10 |
| `RC_NUM_MAESTROS` | `NUM_MAESTRO_SLOTS` | 8 |
| `RC_NUM_WLED` | `NUM_WLED_SLOTS` | 4 |
| `RC_MAX_WCB_PROFILES` | `WCB_MAX_PROFILES` | 6 |
| `RC_KNOB_LABELS[]` | knob source table | `S1,S2,LS,RS,S3,J1..J4,J5,J6` |
| `RC_SWITCH_LABELS[]` | switch table | `SA..SJ` |
| `RcTxModel` enum | per-model metadata tables | model ids |
| `RcActionType` enum | action-type dropdown + `_normActionType` | 0–13 |
| `RcDfpFn` enum | `dfpFnLabels` + `DFP_FN_USES_CHAN`/`_TRACK` + bounds | 1–18 |
| `dfpFormatCommand()` arg ranges | `dfpTrackBounds` / `dfpChanBounds` | must equal `DfPlayerCodec::handle()`'s |
| `DBG_DFP` | `DEBUG_CATEGORIES` `dfp` bit | `1 << 6` |
| `rcTelemetry::FRAG_CHUNK_BYTES` | `FRAG_CHUNK_BYTES` | 80 |
| `rcTelemetry::FRAG_MAX_PARTS` | `FRAG_MAX_PARTS` | 192 (upload) |
| `rcTelemetry::FRAG_SEND_MAX_PARTS` | `FRAG_MAX_PARTS_RECV` | 512 (download) |
| `MAX_ENV_BYTES` | `FRAG_MAX_ENV_BYTES` | 187 |
| `FRAG_PACING_MS` (download only) | — | 150 ms |
| — | `FRAG_PACE_FLOOR_MS` (upload only) | 100 ms |
| `FRAG_TIMEOUT_MS` | `FRAG_TIMEOUT_MS` | 5000 ms |
| `WCB_BULK_CHUNK_RAW` (library) | `BULK_CHUNK_RAW` | 96 |
| `WCB_BULK_MAX_CHUNKS` (library) | `BULK_MAX_CHUNKS` | 512 |
| `WCB_DEVICE_ID` | `RC_WCB_DEVICE_ID` | 20 |
| OTA/RTERM struct sizes | — | must stay unique mesh-wide |

Firmware-internal pairs that also travel together: `SBUS_CENTER 992` / `US_CENTER 1500`
(the tool's SBUS↔PWM display conversion), and the `partitions.csv` `clips` offset versus
the flasher's expectations.

---

## 7. Adding a config field — checklist

1. **Struct** — add the field to `rc_config.h`. Watch the multiplier if it is inside
   `RcAction`.
2. **Defaults** — set it in `rcConfigLoadDefaults()`.
3. **Serialise** — add to `rcConfigToJSON()`; keep it sparse if it is usually empty.
4. **Deserialise** — add to `rcConfigFromJSON()`; guard with `containsKey` so an older
   config still loads and the field keeps its default.
5. **Apply** — if it changes hardware behaviour, extend the `SET_CONFIG` apply path
   (`applySerialBauds` / `applySbusOut` / profile check) so it takes effect without a reboot,
   or document that it needs one.
6. **Dispatch** — use it wherever the behaviour lives.
7. **Tool** — add the editor UI, wire it in `applyConfig()` (load) and the save-collection
   path (see [CONFIG_TOOL.md](CONFIG_TOOL.md)), and register it in the diff baseline so
   Save actually sends it.
8. **Size check** — if the config grew materially, re-check that a populated config still
   fits the 192-fragment bridged upload; otherwise the field is Direct-USB-only.
9. **Syntax-check the tool** — `node C:\Users\ghulette\tools\jscheck.js config_tool/index.html`.
10. **Wiki** — a user-visible option needs a matching wiki edit
    (`C:\Users\ghulette\Documents\GitHub\NaviCore.wiki`, branch `master`).

---

## Revision log

Newest first. Add a row whenever a code change alters what this page describes — same commit
as the code. Page body stays present-tense; history lives here.

| Date | Commit | Change |
| 2026-08-28 | _(uncommitted)_ | Added `wifiSsid` (char[33]) and `wifiPassword` (char[64]) alongside `wifiEnabled` — the flag alone could not actually raise an AP. Empty SSID derives `NaviCore-<deviceId>`. Empty or under-8-char password **refuses to raise the AP**: `WiFi.softAP()` creates an OPEN network on an empty password, and with no per-command auth on this surface that is an unauthenticated command channel to the whole mesh, so it fails closed rather than falling back. Kept in their own NVS key (`wifi`) rather than folded into `wcb`, because AP credentials and mesh credentials must never be confused — `wcbNetwork.password` is public by construction, riding in cleartext in every ESP-NOW packet. The tool warns inline on a short password rather than letting the user save, reboot, and find no AP with only a serial line to explain it. |
| 2026-08-28 | _(uncommitted)_ | Added top-level `wifiEnabled` (bool, **default false**) — the opt-in for a SoftAP + comms server so the forthcoming desktop app can reach the droid without a cable. Modelled site-for-site on `sbusOutEnabled`: struct, defaults, serialize, deserialize, and the NVS `putBool`/`getBool` pair (key `wifien`). Two properties are load-bearing rather than stylistic: the `\| false` on deserialise means a config or backup written before the field existed can never enable the radio, and false is intended to make the bring-up code **unreachable**, not merely idle. Read at boot only, so a change needs a Save *and* a reboot. The AP, when it exists, must be raised on `wcbNetwork.channel` — the ESP32 has one radio, and any other channel takes the droid off its own mesh. No visible UI by request; the toggle is parked in the cloud-backup modal behind the existing 4x-wordmark gesture. |
|---|---|---|
| 2026-08-27 | _(uncommitted)_ | Split the `FRAG_PACING_MS` invariant row — the firmware constant (150 ms) now governs the **download** direction only, and the tool’s upload pacing is its own `FRAG_PACE_FLOOR_MS` (100 ms). They are deliberately no longer a pair. Mesh `wcbNetwork.channel` valid range corrected to **1–11** (was documented and clamped as 1–13); `WCB_Client::setMeshChannel()` rejects >11 and returns without setting, so out-of-range now falls back to 1 = `WCB_MESH_CHANNEL` — the channel the radio was actually on — rather than clamping to 11, which would move a working board off its fleet. |
| 2026-08-26 | _(uncommitted)_ | Corrected the mesh-counter note: they are no longer "never reset in-session". `RESET_MESH_STATS` (Mesh Stats → ⌫ Clear) zeroes the library counters plus `g_meshRxCount`/`g_meshRxFrom`, deferred to `loop()`. |
| 2026-08-26 | _(uncommitted)_ | Documented that `maestros[].channels` endpoints are in **quarter-µs** and what they actually govern — timeline scale, the timeline edit clamp, and passthrough knob endpoints. They are now editable by hand in the tool (Maestro tab → ✎); previously the only way to set them was importing a Control Center settings file, which left no recourse when a channel's real travel differed from that file. |
| 2026-08-25 | _(uncommitted)_ | Added `switchSettleMs` (default 80, clamped 0–1000, 0 = fire immediately) — the rest period a switch position must hold before its tier dispatches. |
| 2026-08-24 | `083207c` | Added `holdMs` (long-press threshold, default 750) and a `RC_NUM_TAP_TIERS`/`NUM_TAP_TIERS` invariants row. Button mappings now carry a 4th tier `t4` (long press); `t4` is omitted from the JSON when empty, same as the other tiers, so it costs nothing until used. |
| 2026-08-18 | _(uncommitted)_ | `hcrDest`/`mp3Dest`/`dfpDest` gained `transport` **2 = disabled** (JSON `"off"`), and all three now default to it — a fresh config has every audio device switched off until the user sets it up. Executors refuse a disabled device's actions; the stored port/target survives so re-enabling restores it. Round-trips through JSON and CSV. |
| 2026-08-18 | _(uncommitted)_ | §3 gained `serialBcastOut[3]` / `serialBcastIn[3]` (JSON `serialBcast`) — the per-aux-port mesh bridging flags were shipped but undocumented here. Corrected `serialLabels`: **4** slots indexed by firmware port (`RC_SLBL_S3/S4/S5/MAESTRO`, JSON keys `"S3"/"S4"/"S5"/"maestro"`), not 5 indexed by WDP port with an SBUS slot — the page still described the pre-migration layout. Recorded that `rcConfigFromJSON()` memsets the slots before matching, so an old-style (`"1"`–`"5"`) object clears every override rather than misapplying it. |
| 2026-08-13 | _(uncommitted)_ | `statsReport.wcb` documented as **optional** (0 = collect/display, ship nothing) and `enabled` clarified as a statement of intent — the counters run from boot regardless and are never reset in-session, so nothing here gates collection. |
| 2026-08-12 | _(uncommitted)_ | Added `statsReport` (`{enabled, wcb}`) — the optional 30 s `;V` push of this board's ESP-NOW delivery counters to one WCB, with the one-variable-per-counter and `;V`-not-`;VP` constraints. Also documented `modeReport`, which was in the firmware but missing from this table. |
| 2026-08-11 | _(uncommitted)_ | Added `wcbProfiles[≤6]` + `wcbProfileCount` (`RcWcbProfile`) — saved WCB mesh identities the config tool switches between, now stored in the config (was browser localStorage) so they travel with the droid + backups. New capacity constant `RC_MAX_WCB_PROFILES` (6) and its cross-file pair `WCB_MAX_PROFILES`. |
| 2026-08-05 | _(uncommitted)_ | Added `dfpDest` (`RcDfpDest`), recorded the inverse MP3-Trigger/DFPlayer volume scales, and added four new rows to the cross-file invariants table (`RcActionType` now 0–13, `RcDfpFn`, the duplicated arg ranges, `DBG_DFP`). |
| 2026-08-04 | _(uncommitted)_ | Initial version. |
