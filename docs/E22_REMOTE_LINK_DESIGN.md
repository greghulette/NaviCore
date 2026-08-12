# E22 Handheld Remote Link (NaviHiltCore) — Design Note

Status: **DESIGN — nothing built yet.** Last updated 2026-08-11.

Goal: let one NaviCore firmware image also run the **NaviHiltCore** board, where two EBYTE
E22-900M22S (SX1262) 915 MHz transceivers listen for a pair of handheld remotes. The remotes
become an **alternate input source** alongside SBUS — same mappings, same actions, same mesh,
same config tool, same release. No fork, no second firmware repo, no separate update cycle.

The **handheld remotes run their own custom firmware** and are out of scope for this document
except for §7, which defines the wire contract both ends must implement.

Board hardware: [NaviHiltCore/DESIGN.md](../../KiCad-Files-Public/NaviHiltCore/DESIGN.md) and
[R2_HH_Controllers/DESIGN.md](../../KiCad-Files-Public/R2_HH_Controllers/DESIGN.md).

---

## 1. Why this is cheap: two seams that already exist

The whole feasibility argument rests on these. Both are verified in the current code.

### 1.1 `sbusValues[24]` is the universal input seam

Every downstream consumer reads that one array, and there is **exactly one writer**.

| Consumer | Site |
|---|---|
| mode decode (`readBoundSwitchSbus`) | [NaviCore.ino:502-506](../NaviCore.ino#L502-L506) ≈ |
| matrix buttons | [NaviCore.ino:2267](../NaviCore.ino#L2267) ≈ |
| switches (`processSwitches`) | [NaviCore.ino:1975](../NaviCore.ino#L1975) ≈ |
| knobs / Maestro passthrough (`processKnobs`) | [NaviCore.ino:2095](../NaviCore.ino#L2095) ≈ |
| `rc_ch` telemetry | [NaviCore.ino:2544](../NaviCore.ino#L2544) ≈ |
| `#L09` SBUS dump | [NaviCore.ino:2337](../NaviCore.ino#L2337) ≈ |
| **writer** | [NaviCore.ino:2219](../NaviCore.ino#L2219) ≈ — one line, inside `processSbus()` |

Anything that writes a plausible 24-channel frame into `sbusValues[]` inherits the **entire
existing feature set for free**: 108 mappings (3 modes × 36 slots), tap tiers, matrix
debounce, switches, knobs, every action executor, record/replay, OTA, the config tool's live
monitor, and the calibration wizard. That is the difference between a feature and a fork.

### 1.2 `boardType` board profiles already exist

`applyBoardProfile()` at [NaviCore.ino:2583](../NaviCore.ino#L2583) ≈ already runs one image
on two pin maps via runtime pin globals ([NaviCore.ino:140-149](../NaviCore.ino#L140-L149) ≈).
Adding a third profile is the established, low-risk path — the mechanism, the `SET_CONFIG`
live-change detection ([NaviCore.ino:3128](../NaviCore.ino#L3128) ≈), and the
"reboot to apply" message all exist.

---

## 2. Hardware: NaviHiltCore is boardType 0 minus S5

Comparing the board's verified pin map against `applyBoardProfile()`'s NaviCore v2 branch:

| Signal | boardType 0 (NaviCore v2) | NaviHiltCore | |
|---|---|---|---|
| SBUS IN | GPIO4 | GPIO4 | same |
| SBUS OUT | GPIO5 | GPIO5 | same |
| Maestro (UART2) | TX6 / RX7 | TX6 / RX7 | same |
| Aux S3 (silk "Serial 1") | TX8 / RX9 | TX8 / RX9 | same |
| Aux S4 (silk "Serial 2") | TX10 / RX21 | TX10 / RX21 | same |
| Aux S5 (silk "Serial 3") | TX38 / RX47 | **absent** | 38 = IC2 TXEN, 47 = shared E22 RST |

The board drops Serial3 and PWM4 outright — their connectors J5/J9 are removed from the
schematic, not merely unpopulated, because two bidirectional E22s plus PWM did not fit the
~14 clean GPIOs the N16R8's octal PSRAM leaves behind.

### 2.1 Radio pins

Shared SPI bus and one shared reset for both modules:

| Signal | Pin |
|---|---|
| MOSI | 11 |
| SCK | 12 |
| MISO | 13 |
| RST (both modules) | 47 |

| Per-module | IC1 (remote A) | IC2 (remote B) |
|---|---|---|
| NSS | 14 | 38 |
| BUSY | 15 | 39 |
| DIO1 | 16 | 40 |
| RXEN | 17 | 41 |
| TXEN | 18 | 42 |

Both modules are bidirectional, one dedicated per remote. Per-module reset is done over SPI,
since the hardware reset line is shared.

**None of these pins is touched by the firmware today** — a grep of `NaviCore.ino` finds zero
`SPI.` uses and zero `ledc*` uses. The radios claim only pins that are currently free on this
board, plus S5's two.

### 2.2 Two radios, two bands, no conflict

915 MHz LoRa (external SPI) and 2.4 GHz ESP-NOW (internal) are independent radios. **The hilt
is a full WCB mesh peer and a LoRa base station simultaneously.** That gives the topology
remotes → LoRa → hilt → ESP-NOW → whole droid with no new bridging concept: the hilt is just
NaviCore, which is already special-peer slot 20 on the mesh.

---

## 3. Board profile: `boardType 2`

Add `BOARD_NAVIHILTCORE = 2` to the `BoardType` enum at
[NaviCore.ino:138](../NaviCore.ino#L138) ≈ and a branch in `applyBoardProfile()` that is the
NaviCore v2 branch with two changes:

- `S5_TX_PIN` / `S5_RX_PIN` left unset and **`s5 = nullptr`** at bind time. The
  dedicated-SBUS fallback path already models a null `s5`
  ([NaviCore.ino:218-220](../NaviCore.ino#L218-L220) ≈), so this is a supported state rather
  than a new one — but every `writeS5` / `pollAuxSerialRx` site needs confirming against it.
- Radio pins claimed and `navie22::begin()` gated on this profile.

`auxPortLabel()` ([NaviCore.ino:235](../NaviCore.ino#L235) ≈) currently branches on
`boardType == 0` for silkscreen naming and needs a third case. NaviHiltCore silkscreens
*Serial 1 / Serial 2* for what the firmware calls S3 / S4, matching the v2 convention.

**Only one SoftwareSerial port instead of two.** This matters beyond connector count — see §8.

---

## 4. Input-source abstraction

### 4.1 The one structural refactor

`processSbus()` ([NaviCore.ino:2211](../NaviCore.ino#L2211) ≈) currently does two jobs:
read the UART, then decode. Split it:

- **`sbusIngest()`** — `sbusRx.read()`, stamp `sbusLastFrameMs` / FPS counters, copy
  `sbusRx.channels[]` into `sbusValues[]`, set `sbusFailsafe`.
- **`sbusDecode()`** — everything from the failsafe gate down: mode, matrix, switches, knobs.

Then either source can feed the decoder. This refactor is correct on its own merits and
should land first, separately, so it can be verified against a plain NaviCore before any
radio code exists.

### 4.2 New config field

`rcConfig.inputSource`, `uint8_t`:

| Value | Meaning |
|---|---|
| 0 | SBUS only (**default** — every existing board keeps today's behaviour) |
| 1 | E22 remotes only |
| 2 | Both — SBUS preferred while live, E22 fills in |

Ignored unless `boardType == 2`; a board with no radios cannot honour 1 or 2.

### 4.3 Loop placement

`navie22::tick()` goes in `loop()` **immediately before** the SBUS path, so a frame decoded
this pass is acted on this pass:

```
… existing drains …
rcTelemetry::tick()
navie22::tick()        ← new: drain DIO1 flags, decode, write sbusValues[], stamp liveness
sbusIngest()           ← was the first half of processSbus()
sbusDecode()           ← was the second half
checkDeferredTap()
…
```

**Arbitration when `inputSource == 2`** is a decision to make deliberately, not emergently.
The proposal is *SBUS wins while live*: if `now - sbusLastFrameMs < SBUS_LED_TIMEOUT_MS`
(500 ms, [NaviCore.ino:3817](../NaviCore.ino#L3817) ≈), E22 frames are decoded for liveness
and telemetry but do **not** write `sbusValues[]`. A last-writer-wins scheme is tempting and
should be avoided — two sources interleaving into one channel array produces control glitches
that are near-impossible to diagnose from the config tool's live monitor.

---

## 5. Channel mapping: remote controls → `sbusValues[24]`

Each remote carries a thumbstick (X, Y, press), 8 buttons, a trim pot, and 3 NeoPixels.
Two remotes: **16 buttons + 2 stick presses + 4 analog axes + 2 pots.**

### 5.1 Buttons go through the matrix decoder

`pwmToButton()` ([NaviCore.ino:509](../NaviCore.ino#L509) ≈) decodes **one analog channel**
into up to `RC_NUM_THRESHOLDS` = 36 bands ([rc_config.h:542](../rc_config.h#L542) ≈) — slots
1–21 physical, 22–36 user-defined logical. A `0/0` band is an inert sentinel.

So the E22 decoder **synthesizes a matrix-channel value** in the band of whichever button is
currently down, and the SBUS centre value (992) otherwise. Consequences:

- 18 discrete inputs against 21 physical slots — fits, with 22–36 free for chords.
- The debounce state machine, tap tiers, and `exclusive`/cumulative semantics all apply
  unchanged. A LoRa link is *lossier* than SBUS, so `matrixDebounceFrames` (1–4) interacts
  with packet rate rather than frame rate and will need retuning on the bench.
- **One button at a time** per matrix channel, by construction. Simultaneous presses must
  either be resolved to a priority winner or mapped to a logical chord slot. Two matrix
  channels (one per hand) is the obvious alternative but `rcConfig.matrixChannel` is a single
  scalar today — see §11.

### 5.2 Analog inputs go through knobs

Sticks and pots map straight onto knob source channels, scaled to the SBUS range the rest of
the firmware assumes (`sbusToRange` maps 172–1811,
[NaviCore.ino:522-533](../NaviCore.ino#L522-L533) ≈; `readSwitchPos` thresholds at 582/1401,
[NaviCore.ino:495-500](../NaviCore.ino#L495-L500) ≈). The decoder must emit values on that
scale, not raw 12-bit ADC counts, or every threshold in the config becomes meaningless.

Budget: 1 matrix + 4 stick axes + 2 pots + 2 stick presses ≈ 8 of 24 channels. Ample room for
mode selection and future controls.

---

## 6. Link liveness and failsafe — the hard part

**This is the highest-risk area of the whole design.** Not the radio, not the CPU budget.

`sbusFailsafe` is a load-bearing safety gate with a long justifying comment at
[NaviCore.ino:2221-2242](../NaviCore.ino#L2221-L2242) ≈: on a failsafe frame the firmware
freezes dispatch entirely and resets the matrix debounce, so a transmitter power-off cannot
drive servos to parked positions or trip switch thresholds. It deliberately gates on
`failsafe` only, **not** `lostFrame`.

**An SBUS receiver hands you an explicit failsafe bit. A LoRa link gives you nothing but
silence.** The equivalent must be synthesized:

- Per-remote `lastGoodPacketMs`, with a timeout that has to be chosen against the actual
  packet rate (§7). It will be far longer than SBUS's 500 ms LED timeout.
- On timeout, set the same freeze the SBUS path uses — reuse `sbusFailsafe`, do not invent a
  parallel gate, so there is exactly one place where "inputs are not trustworthy" is decided.
- **Decide what one-remote-down means.** Left hand silent, right hand live is a state SBUS
  never produces. Freezing everything on either remote's loss is the safe default and the
  recommendation; anything else needs an explicit argument.

### 6.1 Battery is part of failsafe here, not a nicety

The remotes run on a Li-ion cell with firmware thresholds already specified in
[R2_HH_Controllers/DESIGN.md §6](../../KiCad-Files-Public/R2_HH_Controllers/DESIGN.md):
warn near 3.4 V, stop transmitting near 3.2 V, hardware lockout at 3.0 V. That document also
records, in its own words, that **there is no battery field in the E22 link payload yet.**

Without one, the hilt's only warning that a remote is about to die is the remote going quiet —
which is indistinguishable from walking out of range. **Put battery state in the link payload
from the first revision** (§7), and surface it in `rc_hb` telemetry so the config tool can
show it.

### 6.2 Status LED

`updateStatusLed()` ([NaviCore.ino:3845](../NaviCore.ino#L3845) ≈) keys "healthy blue" vs
"flashing orange" off `sbusLastFrameMs`. If the E22 path stamps that same variable, the LED
works unchanged. A distinct colour for "one remote of two" is worth considering once the
one-remote-down semantics of §6 are settled.

---

## 7. Link protocol — the contract, and the prior art

The E22 driver is new, but **the link protocol has direct prior art in this ecosystem** that
should be mined before designing from scratch.

### 7.1 What exists, and what does not carry over

`Arduino-Code/Droid_Remote` and `Arduino-Code/Droid_Gateway` are a working 915 MHz remote
system — but on **TTGO LoRa32 (SX1276) hardware via the Sandeep Mistry `LoRa` library**
(`LoRa.begin(915E6)`, `setSignalBandwidth(500E3)`,
[Droid_Remote.ino:1397-1404](../../Arduino-Code/Droid_Remote/Droid_Remote.ino#L1397-L1404) ≈).

**That library does not drive an SX1262-based E22.** The driver is new work — RadioLib's
SX126x class is the obvious candidate. The *protocol design*, however, transfers directly.

### 7.2 Lessons worth carrying forward

- **Binary structs with compile-time size guards.** `Droid_Remote.ino` uses `static_assert` on
  every wire struct with the note *"If a struct edit trips one, fix BOTH ends together"*
  ([Droid_Remote.ino:684-688](../../Arduino-Code/Droid_Remote/Droid_Remote.ino#L684-L688) ≈).
  Keep this. It is the only mechanism that catches wire-format drift between two separately
  compiled firmwares.
- **A passcode field** (`struct_LoraPasscode`) doubles as pairing and packet validation.
- **Fields right-sized and ordered big→small to de-pad**, with readable values reconstructed
  at the far end (bitmask→bools, centivolts→float). Worth copying.

### 7.3 One lesson worth *changing*

The old system **dispatches packet types by `packetSize`**, requiring every struct to have a
mutually unique size. That is fragile — a field edit that happens to preserve total size
routes silently to the wrong handler. **Use an explicit type byte.** It costs one byte and
removes a whole class of failure. (The old code half-acknowledges this by also carrying
`magic` bytes on some structs — 0xA5, 0xC6, 0xD1 — so the type tag partly exists already.)

### 7.4 The concurrency lesson, already learned the hard way

[Droid_Gateway.ino:405-411](../../Arduino-Code/Droid_Gateway/Droid_Gateway.ino#L405-L411) ≈
records a real failure worth quoting in full, because it is exactly the trap this design walks
toward:

> the old code called `sendACK()` (which does a full `LoRa.endPacket` taking ~250ms) directly
> inside the `onReceive` callback path. During a config burst, this starved
> `tickCfgChunkQueue` and caused the chunk queue to overflow. Now `onReceive` just sets
> `pendingLoRaAck=true` and the main loop fires `sendACK` AFTER `tickCfgChunkQueue` has had a
> chance to drain.

**A LoRa transmit is a ~hundreds-of-milliseconds blocking operation.** It can never happen
inline in a receive callback, and on NaviCore it can never happen inline in `loop()` either
(§8). Any hilt→remote traffic — ACKs, telemetry, battery requests, NeoPixel/rumble feedback —
must be queued and paced.

### 7.5 Minimum payload for revision 1

Remote → hilt, at the control packet rate:

| Field | Notes |
|---|---|
| type byte | explicit, per §7.3 |
| passcode / pairing id | validates the packet and identifies which remote |
| sequence number | detects loss and reordering; feeds a link-quality figure |
| stick X, Y | scaled to SBUS range at the *remote*, or raw with scaling at the hilt — decide |
| button bitmask | 8 buttons + stick press in one `uint16_t` |
| trim pot | |
| **battery centivolts** | §6.1 — not optional |

Hilt → remote is lower rate and can be a separate struct.

---

## 8. Concurrency and timing

### 8.1 `loop()` is a real-time path

`loop()` runs on Core 1 and must stay non-blocking: SBUS arrives every 9–14 ms and the
passthrough byte-tee is in the same thread ([ARCHITECTURE.md §7](ARCHITECTURE.md)). Rules for
the radio code:

- **DIO1 interrupt sets a `volatile` flag; `loop()` drains it.** Same shape as `navirec`'s
  lock-free `_pendingCtl` edge ([ARCHITECTURE.md §8](ARCHITECTURE.md)). Never poll-and-block.
- **Never block on BUSY inside `loop()`.** A `standby → config → TX → wait-BUSY` round trip is
  a multi-hundred-millisecond stall (§7.4). Transmits are queued and drained across passes.
- SPI *receive* of a small payload at 8–10 MHz is ~50–100 µs, which is affordable per pass.
  The cost is transmit, not receive.

### 8.2 SPI alongside SoftwareSerial

This is the firmware's **first SPI peripheral**. S4/S5 bit-bang with interrupts masked per
byte, so a SoftwareSerial write can delay a DIO1 interrupt and a long SPI transaction can
stretch a bit-banged byte.

Mitigating: **NaviHiltCore has no S5** (its pins are the radios), so the board runs one
SoftwareSerial port instead of two. That is a genuine reduction in exposure, and it falls out
of the pin budget rather than being designed in — worth recording so it is not "fixed" later
by reassigning S5 to spare pins.

Spare pins on the board are **GPIO44 (U0RXD) and GPIO3**, both carrying `no_connect` flags.

### 8.3 Latency budget — set expectations now

Both old sketches run BW 500 kHz. Even at aggressive settings a LoRa round trip is **tens of
milliseconds**, against SBUS's 9–14 ms, and slower spreading factors are far worse.

- **Button taps will not notice.** The tap-tier windows are already hundreds of ms.
- **Knob and Maestro passthrough will notice.** `processKnobs` drives servos continuously from
  stick position; added latency is directly felt as sloppy control.

This is physics, not firmware. Record the chosen SF/BW/CR and the measured packet rate here
once benched, because every debounce and deadband tuning decision depends on it.

---

## 9. Config schema changes

Follow the checklist in [CONFIG_SCHEMA.md §7](CONFIG_SCHEMA.md#7-adding-a-config-field--checklist).
Sites for `inputSource`, mirroring how `boardType` is already plumbed:

| Step | Site |
|---|---|
| Struct field | [rc_config.h:606](../rc_config.h#L606) ≈ (next to `boardType`) |
| Default (`= 0`) | [rc_config.h:746](../rc_config.h#L746) ≈ |
| `rcConfigToJSON()` | [rc_config.h:1149](../rc_config.h#L1149) ≈ |
| `rcConfigFromJSON()` (guard with `containsKey`) | [rc_config.h:1403](../rc_config.h#L1403) ≈ |
| Legacy NVS put/get | [rc_config.h:2048](../rc_config.h#L2048) ≈ / [rc_config.h:2232](../rc_config.h#L2232) ≈ |
| Tool: editor UI + `applyConfig()` + save-collection + diff baseline | [CONFIG_TOOL.md](CONFIG_TOOL.md) |

**The ArduinoJson filter whitelist at [NaviCore.ino:3040-3050](../NaviCore.ino#L3040-L3050) ≈
does not need a new entry for this field.** That filter whitelists *header* fields of
non-`SET_CONFIG` commands; `SET_CONFIG` does its own un-filtered deserialize of the config
payload, as the comment above the filter states. The whitelist rule applies only if this work
adds a new top-level *command* type with new header fields (a pairing command would).

`boardType` and `inputSource` are both **reboot-to-apply** — pins are assigned once in
`setup()`, and `SET_CONFIG` already detects a live `boardType` change and says so
([NaviCore.ino:3124-3141](../NaviCore.ino#L3124-L3141) ≈). Extend that check to cover
`inputSource`.

### 9.1 Cross-file invariants

If the link introduces paired constants (packet rate, timeout, remote count, band table),
add them to the table in
[CONFIG_SCHEMA.md §6](CONFIG_SCHEMA.md#6-cross-file-invariants). Note that the pairing here is
**three-way** — firmware, config tool, *and the remote firmware*, which is a separately
compiled binary that no compiler checks against this one. That is what §7.2's `static_assert`
guards are for.

---

## 10. Overhead budget

Measured, not estimated:

| Resource | Capacity | Used today | Headroom |
|---|---|---|---|
| App partition | 1,966,080 B (`0x1e0000`, `partitions.csv`) | 1,105,888 B (`firmware/NaviCore_v0.2.0_101629QAUG26_ESP32S3.bin`) | **~860 KB (56% used)** |
| PSRAM | 8 MB | `RcConfig` ~210 KB + 3.1 MB clip buffer | ample |
| Internal SRAM | 512 KB | fragment pool ~10 KB static + everything else | radio state is a few hundred bytes ×2 |

RadioLib's SX126x driver is roughly 30–60 KB of flash. **Neither flash nor RAM is a
constraint.** The cost of this feature is CPU time in `loop()` and design attention on §6 —
budget the effort there.

---

## 11. Out of scope

Things this design explicitly does **not** cover, recorded so they are not assumed included:

- **The handheld remote firmware.** Separate custom firmware, per the project decision. Note
  for the record that it *could not* run this image regardless: the remotes are
  ESP32-S3-MINI-1-N8 with **no PSRAM**, and `setup()` halts on a solid red LED when
  `ps_calloc` returns null ([ARCHITECTURE.md §5](ARCHITECTURE.md)).
- **NaviHiltCore's PWM outputs** (J2/J3/J4 on GPIO 43/2/1). The firmware has no PWM output
  capability at all — zero `ledc*` calls in `NaviCore.ino`. Supporting them means a new
  `RA_PWM` action type, config, and tool editor. Separate feature, separate branch.
  The board's own note that **PWM1 sits on GPIO43 = U0TXD** stands: on this profile UART0
  backs aux S3 remapped to GPIO8/9, so 43 is free — but anything printing to a default-pin
  UART0 `Serial` would drive PWM1.
- **Per-hand matrix channels.** `rcConfig.matrixChannel` is a single scalar; two matrix
  channels is a config-schema change with tool implications (§5.1).
- **Synthesizing SBUS OUT from E22 input.** The board has an SBUS OUT connector (J1) and the
  byte-tee only re-emits *received* SBUS bytes — so with `inputSource == 1` nothing leaves
  that connector. Generating real SBUS frames from remote input would let the hilt drive
  downstream SBUS devices. Deferred, but the connector exists and someone will ask.

---

## 12. Suggested build order

Each step is independently verifiable, which matters because the only check is the compiler
plus bench behaviour.

1. **Split `processSbus()` into `sbusIngest()` + `sbusDecode()`.** No behaviour change.
   Verify on an existing NaviCore board before any radio work.
2. **Add `boardType 2` + `inputSource`** with no radio code. Board boots, S5 is null, SBUS
   works. Confirms the profile and the config plumbing in isolation.
3. **`navi_e22.h` with a stub decoder** driven by a CLI command that injects a synthetic
   frame into `sbusValues[]`. Proves the seam end-to-end — mappings, taps, telemetry, the
   config tool's live monitor — with **no radio involved at all.** This step is where most of
   the integration risk actually dies.
4. **RadioLib SX126x bring-up**, one radio, receive only, DIO1 → flag → drain.
5. **Second radio**, shared bus, per-module SPI reset.
6. **Link protocol + failsafe (§6).** The real design work.
7. **Hilt → remote path** (battery request, feedback), queued and paced per §7.4.

Module shape matches the existing single-header namespaces (`navirec`, `naviota`,
`navirterm`):

```cpp
namespace navie22 {
  bool begin();                      // no-op unless boardType==2 && inputSource!=0
  void tick();                       // drain DIO1 flags → decode → sbusValues[] → liveness
  bool linkAlive(uint8_t remote);    // per-remote, feeds the §6 failsafe
}
```

---

## 13. Open questions

Unresolved decisions, each of which changes the implementation:

1. **One-remote-down semantics** (§6). Freeze everything, or degrade? Recommendation: freeze.
2. **SF / BW / CR**, and therefore the packet rate and the failsafe timeout (§8.3). Needs a
   bench measurement, not a guess.
3. **Simultaneous button presses** (§5.1) — priority winner, chord slots, or per-hand matrix
   channels?
4. **Where scaling happens** — remote emits SBUS-range values, or hilt scales raw ADC?
   Scaling at the remote keeps the hilt simple; scaling at the hilt keeps calibration in the
   config where the tool can edit it. Leaning toward the hilt.
5. **Pairing / binding UX.** The old system used a static `LoraPasscode`. A config-tool
   pairing flow is nicer but adds a new command type (and *that* would need a filter
   whitelist entry, §9).
6. **Does `inputSource == 2` earn its complexity?** A hilt with both a receiver and remotes
   live is a real scenario, but arbitration is a bug farm. Possibly ship 0 and 1 first.
7. **Where does the remote firmware live?** A `remote/` folder in this repo would version and
   ship it alongside the firmware it must stay wire-compatible with — which is the stated
   motivation for the single-codebase approach in the first place.

---

## Revision log

Newest first. Add a row whenever a code change alters what this page describes — same commit
as the code. Page body stays present-tense; history lives here.

| Date | Commit | Change |
|---|---|---|
| 2026-08-11 | _(uncommitted)_ | Initial version. Feasibility established against the current code: the `sbusValues[24]` single-writer seam, the existing `boardType` profile mechanism, and the measured ~860 KB flash headroom. Records that NaviHiltCore's pin map equals boardType 0 minus S5, that the E22 driver is new work but `Droid_Remote`/`Droid_Gateway` are protocol prior art (including the ~250 ms inline-TX starvation failure), and that link failsafe (§6) — not CPU or memory — is the real risk. |
