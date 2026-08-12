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
`RC_NUM_MAPPINGS × 3 tiers × RC_ACTIONS_PER_TIER = 1620` action slots. Widening a field in
`RcAction` costs kilobytes per byte.

---

## 2. Capacity constants

| Constant | Value | Meaning |
|---|---|---|
| `RC_NUM_PHYSICAL` | 21 | Matrix slots drawn on the transmitter graphic (21 = inert "Unassigned" sentinel) |
| `RC_NUM_LOGICAL` | 15 | User-defined logical buttons (slots 22–36) |
| `RC_NUM_THRESHOLDS` | 36 | Total matrix-channel bands |
| `RC_NUM_MAPPINGS` | 108 | 3 modes × 36 slots |
| `RC_ACTIONS_PER_TIER` | 5 | Actions per tap tier |
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
| `boardType` | `uint8_t` | 0 = NaviCore v2 PCB, 1 = WCB HW 3.2 — selects the pin profile |
| `tapWindowMs` | `int` | Multi-tap detection window |
| `chRateHz` | `uint8_t` | `rc_ch` broadcast rate, 1–20 (default 5). High rates flood the mesh |
| `matrixChannel` | `int` | SBUS channel carrying the button matrix |
| `matrixDebounceFrames` | `int` | 1–4 consecutive in-band frames to commit a press/re-arm |
| `thresholds[36]` | `RcThreshold` | `{id, label[24], minPwm, maxPwm}` — a `0/0` band is inert |
| `mappings[108]` | `RcMapping` | `{exclusive, RcTier t[3]}` — indexed `(mode-1)*36 + (slot-1)` |
| `switches[10]` | `RcSwitch` | `{channel, positions, RcTier t[3]}` (down/mid/up) |
| `knobs[11]` | `RcKnob` | See below |
| `funcBindings` | `RcFuncBindings` | `{modeSwitch}` — which switch selects the global mode |
| `hcrDest` | `RcHcrDest` | **Global** HCR destination |
| `maestros[8]` | `RcMaestroSlot` | `{type, device, channels[32]}` |
| `wcbNetwork` | `RcWcbNetwork` | ACTIVE mesh credentials (runtime; `wcb_config.h` only seeds defaults) |
| `wcbProfiles[≤6]` + `wcbProfileCount` | `RcWcbProfile` | Saved mesh identities (`{name[24], macOct2, macOct3, password[40], quantity, deviceId, channel}`) the config tool switches between. **Inert storage** — firmware never acts on them; loading one just writes `wcbNetwork` (tool-side, Direct USB) |
| `mp3Dest` | `RcMp3Dest` | **Global** MP3 Trigger destination |
| `dfpDest` | `RcDfpDest` | **Global** DFPlayer Mini destination — `{transport, target}`, defaults local `S3` |
| `wledSlots[4]` | `RcWledSlot` | Per-id WLED routing |
| `auxBaud[3]` | `uint32_t` | `[0]`=S3, `[1]`=S4, `[2]`=S5 |
| `maestroBaud` | `uint32_t` | Serial2 line rate |
| `serialLabels[5][25]` | `char` | WDP port-label **overrides**; `""` = auto-derive |
| `maeGateMs` | `uint16_t` | Remote Maestro busy-gate validity (default 250; fails **open**) |
| `smoothProfiles[6]` | `RcSmoothProfile` | ~4.6 KB of per-mode/per-channel speed+accel |
| `peerNewActions` | `RcTier` | Up to 5 actions fired when a new mesh peer appears |
| `peerAlert` | `bool` | Also flash the LED and print a terminal line |
| `modeReport` | `RcModeReport` | `{enabled, wcb, tmpl[48], cmds[3][48]}` — optional: send the mode-select position to one WCB on every change and every 60 s. `{mode}` in `tmpl` → the position; a non-empty `cmds[mode-1]` overrides it |
| `statsReport` | `RcStatsReport` | `{enabled, wcb}` — optional: push **this board's** ESP-NOW delivery counters to one WCB every 30 s as one chained `;V` command. See below |

**`statsReport` — the `;V` shape is load-bearing.** The report is sent as
`;V,STATS_SENT,<n>^;V,STATS_ACK,<n>^…` — **one variable per counter, never one variable
holding a tuple.** A WCB variable is a single `int32_t` and the `;V` parser reads exactly one
value field (`WCB_Variables.cpp` ≈23, ≈226-251), so `;V,STATS,<sent>,<ackd>,…` would set
`STATS` to `<sent>` and silently discard every later field.

Plain `;V` and **never `;VP`**: a plain `;V` leaves a *new* variable **volatile** on the
receiving WCB (`WCB_Variables.cpp` ≈256-261) — RAM-only, gone on that board's reboot, no
flash write per report. `;VP` would persist to NVS on every 30 s report, which is flash wear,
not a preference. The seven names (`STATS_SENT`, `STATS_ACK`, `STATS_RETRY`, `STATS_FAIL`,
`STATS_NOSLOT`, `STATS_BCAST`, `STATS_RECV`) are each ≤ `WCB_VAR_NAME_MAX` (15) and the whole
chain is ≤ 177 B worst case, inside the one-packet budget.

This carries **only this board's own numbers** — every other board reports its own the same
way. It is never a fleet roll-up.

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

All three live in the config tool's single **Audio** tab. `dfpDest` defaults to *local `S3`*
where `mp3Dest` defaults to *WCB 2* — a DFPlayer is usually soldered to the controller's own
aux header, an MP3 Trigger usually is not.

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

`serialLabels[5]` maps to WDP ports 1–5: `[0]` SBUS, `[1]` local Maestro, `[2]`–`[4]` =
S3/S4/S5. An empty string falls back to `rcSerialLabelAuto()`, which derives a label from
what the config routes there (`Maestro`, `HCR`, `MP3`, `DFPlayer`, `WLED`). That one
function is also what `auxPortHasDevice()` reads, so a device added there is automatically
excluded from the serial broadcast fan-out — there is no second list. `rcSerialLabel()` applies
override-then-auto and `rcAdvertiseSerialLabels()` pushes all five to
`WCB_Client::setPortLabel()`.

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
| `FRAG_PACING_MS` | fragment `await` delay | 150 ms |
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
|---|---|---|
| 2026-08-12 | _(uncommitted)_ | Added `statsReport` (`{enabled, wcb}`) — the optional 30 s `;V` push of this board's ESP-NOW delivery counters to one WCB, with the one-variable-per-counter and `;V`-not-`;VP` constraints. Also documented `modeReport`, which was in the firmware but missing from this table. |
| 2026-08-11 | _(uncommitted)_ | Added `wcbProfiles[≤6]` + `wcbProfileCount` (`RcWcbProfile`) — saved WCB mesh identities the config tool switches between, now stored in the config (was browser localStorage) so they travel with the droid + backups. New capacity constant `RC_MAX_WCB_PROFILES` (6) and its cross-file pair `WCB_MAX_PROFILES`. |
| 2026-08-05 | _(uncommitted)_ | Added `dfpDest` (`RcDfpDest`), recorded the inverse MP3-Trigger/DFPlayer volume scales, and added four new rows to the cross-file invariants table (`RcActionType` now 0–13, `RcDfpFn`, the duplicated arg ranges, `DBG_DFP`). |
| 2026-08-04 | _(uncommitted)_ | Initial version. |
