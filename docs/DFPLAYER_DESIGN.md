# DFPlayer Mini — Design Note

Adding the **DFRobot DFPlayer Mini** (and its YX5300 / MH2024K clones) as an audio device
alongside the SparkFun MP3 Trigger, reachable either on one of NaviCore's own serial ports or
on a WCB across the mesh.

Companion pages: [ARCHITECTURE.md](ARCHITECTURE.md) (where executors live),
[PROTOCOLS.md](PROTOCOLS.md) (the `;` verb space), [CONFIG_SCHEMA.md](CONFIG_SCHEMA.md) (the
config object), [CONFIG_TOOL.md](CONFIG_TOOL.md) (the GUI).

**Status: shipped**, in every repo §8 lists. Section 9 is the piece-by-piece record.

---

## 1. Why a second audio device

The MP3 Trigger is discontinued and increasingly hard to source; the DFPlayer Mini is £2, in
every parts bin, and already the default sound board in most new droid builds. It is not a
drop-in replacement — a different serial protocol, a different volume scale, and a folder
addressing model the MP3 Trigger does not have — so it earns its own device rather than a
compatibility shim behind `;A`.

---

## 2. Decisions locked

These were settled before implementation. Each was a real fork.

| # | Decision | Rejected alternative | Why |
|---|---|---|---|
| 1 | **Its own action type and destination** — `RA_DFPLAYER` + `RcConfig::dfpDest`, parallel to `RA_MP3` + `mp3Dest` | One "audio player" destination with a device-type selector, reusing `RA_MP3` | The two devices' verb sets and volume scales genuinely differ. A shared type would need per-device bounds on every field and would silently reinterpret existing `VOL` values if the device selector flipped. Separate types also let a droid run both at once. |
| 2 | **Both transports from day one** — wired to NaviCore's `S3`/`S4`/`S5`, *or* hosted on a WCB and reached over the mesh | Local serial only, WCB support as a follow-up | Matches how HCR and the MP3 Trigger already work; a builder should not have to care which board the speaker amp ended up next to. Costs a WCB firmware change and a WDP capability bit. |
| 3 | **Full native verb set**, not MP3-Trigger parity | The eight parity verbs only | Folder+track addressing (`/01/002.mp3`) is how droid sound banks are actually organised, and pause/resume/loop/EQ are free once the frame builder exists. |
| 4 | **New verb letter `;D`** | Overloading `;A` with a device discriminator | `;D` is unused across the WCB dispatch ([`WCB.ino` `processCommandCharcter`](../../Wireless_Communication_Board-WCB/Code/WCB/WCB.ino)) — taken letters are `S W C M P A H L V`. A distinct letter keeps the two devices independently routable. |
| 5 | **Volume stays on each device's native scale** — DFPlayer `0`(silent)–`30`(loudest) | A canonical scale converted per device | Lossless, and the number a builder types matches every DFPlayer datasheet and forum post they will read. The scales are inverse, so no value is ever ambiguous about which device it belongs to. |
| 6 | **Config tool collapses HCR + MP3 Trigger + DFPlayer into one "Audio" tab** | A fourth per-device tab | The tab strip was already 12 wide. Audio destinations are one decision made once, not three. |

---

## 3. Wire format

Every DFPlayer command is exactly ten bytes:

```
 7E   FF   06   CMD  ACK  PARAM_HI  PARAM_LO  CK_HI  CK_LO   EF
 │    │    │    │    │    └──────┬─────────┘  └───┬──────┘   │
 start ver  len  op   0=no ack   16-bit param   checksum     end
```

`checksum = -(sum of bytes[1..6])` as a `uint16`, big-endian — the two's complement of
Version..ParamLo. Verified against `DFRobotDFPlayerMini::calculateCheckSum()`.

Every verb below is pinned by a byte-exact vector in `WcbCmd`'s `examples/GoldenVectors` —
20 frames plus 8 rejection cases. That example is the contract; if a change to the codec
makes it fail, the change is what's wrong.

**ACK is always 0.** The reply frame is never awaited. The DFRobot library's own `sendStack()`
calls `delay(10)` after each ACK-off frame and blocks outright with ACK on; neither survives
on a board whose `loop()` re-emits SBUS every ~9 ms. This is why the library is *not* a
dependency — [`WcbDfPlayer.cpp`](../../WcbCmd/src/WcbDfPlayer.cpp) builds the frames directly,
matching how every other `WcbCmd` module works.

**Pacing is free.** At the DFPlayer's fixed 9600 baud a 10-byte frame takes ~10.4 ms to shift
out, so two commands issued back-to-back in one `loop()` pass are spaced by the UART itself.
No software delay is needed or wanted.

### Verbs

`;D,<CMD>` — parsed by `DfPlayerCodec::handle()`. Bracketed `[,ONFIN,key]` is the optional
play-finished callback, same grammar as `;A`.

| Verb | Opcode | Param | Notes |
|---|---|---|---|
| `PLAY,<n>[,ONFIN,key]` | `0x03` | 1–2999 | global track index across the card |
| `FOLDER,<f>,<t>[,ONFIN,key]` | `0x0F` | f 1–99, t 1–255 | the `/01/002.mp3` layout — hi byte folder, lo byte track |
| `MP3FOLDER,<n>[,ONFIN,key]` | `0x12` | 1–9999 | the reserved `/MP3` folder |
| `STOP` | `0x16` | — | also clears any pending `ONFIN` |
| `NEXT` / `PREV` | `0x01` / `0x02` | — | |
| `PAUSE` / `RESUME` | `0x0E` / `0x0D` | — | |
| `VOL,<n>` | `0x06` | 0–30 | **0 = silent, 30 = loudest** |
| `VOLUP` / `VOLDN` | `0x06` | — | absolute frame, shadow ±2 (see below) |
| `LOOP,<n>` | `0x08` | 1–2999 | repeat one track forever |
| `LOOPALL,<0\|1>` | `0x11` | 0/1 | |
| `LOOPFOLDER,<f>` | `0x17` | 1–99 | |
| `RANDOM` | `0x18` | — | |
| `EQ,<0-5>` | `0x07` | 0–5 | Normal · Pop · Rock · Jazz · Classic · Bass |
| `DEVICE,<n>` | `0x09` | 1–5 | 1 USB · 2 SD · 3 AUX · 4 sleep · 5 flash |
| `RESET` | `0x0C` | — | also clears any pending `ONFIN` |
| `STATUS` | `0x42` | — | query, diagnostic only |

An unknown verb or an out-of-range argument emits **nothing** and returns `false`.

### Inbound frames

`DfPlayerCodec::poll()` resyncs on `0x7E` and reads whole 10-byte frames, so an unsolicited
init or finish frame cannot swallow the next good one.

| Opcode | Meaning | Action |
|---|---|---|
| `0x3D` / `0x3C` | track finished (SD / USB) | fire `onFinished(pendingKey)` |
| `0x40` | module error, param = code | fire `onError()`, clear pending |
| `0x3F` | init complete, param = storage bitmask | log only |

---

## 4. Two deliberate differences from `Mp3Codec`

Both look like oversights and are not:

1. **Volume is not re-sent before every play.** `Mp3Codec` emits `'v'<vol>` ahead of each
   `'t'<track>` because the MP3 Trigger needs it. The DFPlayer keeps its volume across
   tracks, so a redundant frame would only add 10 ms of UART to pace behind.
2. **`VOLUP`/`VOLDN` emit an absolute `0x06` frame**, not the device's own relative
   `0x04`/`0x05`. The codec's volume shadow is what the host persists and what `volume()`
   reports; a relative step would drift it out of sync with the device. Step is ±2 on the
   0–30 scale (`Mp3Codec` uses ±5 on 0–64 — the same ~8%).

---

## 5. Data model

```c
RA_DFPLAYER = 13                       // rc_config.h RcActionType

struct RcDfpDest {                     // GLOBAL — every RA_DFPLAYER action shares it
  uint8_t transport;                   //  0 = local serial, 1 = WCB unicast
  char    target[6];                   //  "S3"/"S4"/"S5"   |  WCB id "1".."20"
};
```

An `RcAction` carries only the function and its arguments — never a destination, matching
`RA_HCR` and `RA_MP3`:

| Field | Use |
|---|---|
| `fn` | `RcDfpFn` (1–18) |
| `chan` | folder (1–99) for `FOLDER` / `LOOPFOLDER`; EQ mode; device id; loop-all flag |
| `track` | track number, `/MP3` index, or volume — `int16_t`, so 2999 and 9999 both fit |

`dfpFormatCommand(fn, chan, track)` in `NaviCore.ino` is the **single producer** of the
`;D,…` string for both transports, mirroring `mp3FormatCommand()`. The local path feeds that
same string to `g_dfp.handle()`; the remote path unicasts it verbatim to a WCB. One command
string, two transports, byte-identical device output — the whole reason `WcbCmd` exists.

Its range checks deliberately duplicate `DfPlayerCodec::handle()`'s. Both must agree: an
action valid locally has to be valid remotely, so a garbage folder/track/volume is never
forwarded to a WCB that would only reject it.

Two things a DFPlayer needs and does **not** get its own code for, because existing
mechanisms already cover them:

- **Port ownership.** `auxPortHasDevice()` delegates to `rcSerialLabelAuto()`, so adding
  `"DFPlayer"` there is what stops the broadcast fan-out and the serial→mesh path from
  fighting the DFPlayer driver for the wire. No second list to keep in sync.
- **Record/replay.** `navirec::captureAction()` captures every action type except the
  record/play/stop meta-triggers, so DFPlayer actions record and replay for free.

**Not persisted to NVS, deliberately.** `rcConfigSaveNVS()` exists but is never called —
LittleFS `/config.json` is the live store and NVS is a one-time migration source for configs
written by firmware that predates all of this. A `dfp` key there would be dead weight.

---

## 6. Routing

| `dfpDest.transport` | Path |
|---|---|
| `0` local | `dfpFormatCommand()` → `g_dfp.begin(port)` → `g_dfp.handle()` → S3/S4/S5 bytes |
| `1` WCB | `dfpFormatCommand()` → `wcb->send(id, ";D,…")` → that WCB's `processDFPCommand()` |

On the WCB side `;D` routes through the existing `routeStoredOrCap()` helper with a new
`WDP_CAP_DFPLAYER` bit, so a `;D` arriving at a board that does not host a DFPlayer is
forwarded to the board that does — identical to how `;A` and `;H` already behave.

`WDP_CAP_DFPLAYER = 0x0100` — the next free bit after `WDP_CAP_MAESTRO_LOC = 0x0080`
([`WCB_WDP.h`](../../Wireless_Communication_Board-WCB/Code/WCB/WCB_WDP.h)). The bit is
mirrored in `WCB_Client.h` and the Wizard's `_WDP_CAP_BITS` table.

**NaviCore can never be elected the `;D` host.** Worth stating because the WCB-native Maestro
work hit the neighbouring version of this and lost time to it. Two independent reasons:
`wdpCapOwner()` skips any neighbour with `isClient` set, and `WCB_Client` only ever *reads*
`capFlags` off an advert — it has no path to publish its own. So a `;D` on a WCB with no local
DFPlayer elects another **WCB**, or runs locally and prints "not configured". It cannot be
forwarded to NaviCore, which has no inbound `;D` handler and would silently drop it.

---

## 7. Traps

| Trap | Consequence | Mitigation |
|---|---|---|
| **Boot silence** — a DFPlayer needs ~1.5–3 s after power-on before it accepts commands | A sound action fired at boot is silently lost | The codec has no clock and does not gate. Documented for the user; a boot-time sound should carry a `delayMs` |
| **Volume scale inversion** vs the MP3 Trigger | `VOL,25` is near-silent on one and near-max on the other | Separate action types; the tool labels the field `Volume (0=silent, 30=loud)` |
| **Clone modules** vary in opcode support | `RANDOM`, `EQ`, `LOOPFOLDER` may no-op on some clones | Fire-and-forget means a no-op is harmless; `STATUS` exists for bench diagnosis |
| **`S4`/`S5` are bit-banged `SoftwareSerial`** | — | 9600 is well inside their ≤57600 limit; any aux port works |
| **A port hosting a DFPlayer must not be drained as raw serial** | Response frames would be consumed by the wrong reader | `auxPortHasDevice()` must return true for a DFPlayer port, as it already does for HCR/MP3/WLED |

---

## 8. Repos in play

Build order matters — `WcbCmd` ships first, because both firmwares compile it.

| Order | Repo | Change |
|---|---|---|
| 1 | **`WcbCmd`** | `WcbDfPlayer.h/.cpp` (`DfPlayerCodec`), `WcbCmd.h` include, version → 0.8.0. **Push to master before either firmware builds** — CI clones it |
| 2 | **`NaviCore`** | `RA_DFPLAYER`, `RcDfpFn`, `RcDfpDest`, JSON + NVS persistence, `dfpFormatCommand`, `executeDfpAction`, dispatch, debug category, port labels |
| 3 | **`NaviCore` (tool)** | Audio tab merge (HCR + MP3 + DFPlayer), DFPlayer action editor, `wcb-dfplayer` command-library board, CSV export/import |
| 4 | **`Wireless_Communication_Board-WCB`** | `WCB_DFP.cpp/.h`, `?DFP,S<port>` config, `;D` dispatch, `WDP_CAP_DFPLAYER`, NVS + backup, Wizard capability label |
| 5 | **`WCBClient`** | Mirror `WDP_CAP_DFPLAYER` in the capability bitmap comment block |
| 6 | wikis | NaviCore + WCB wikis, present tense, no changelog |

---

## 9. Implementation status

| Piece | State |
|---|---|
| `WcbCmd` `DfPlayerCodec` | **done** — `WcbDfPlayer.h/.cpp`, wired into `WcbCmd.h`, version 0.8.0, mirrored into the sketchbook |
| NaviCore config model (`RA_DFPLAYER`, `RcDfpFn`, `RcDfpDest`, JSON) | **done** — `rc_config.h` |
| NaviCore dispatch (`dfpFormatCommand`, `executeDfpAction`, `DBG_DFP` = bit 6) | **done** — `NaviCore.ino` |
| Config tool — Audio tab (HCR + MP3 + DFPlayer merged) | **done** |
| Config tool — DFPlayer editor, CSV, `wcb-dfplayer` cmdlib board, debug chip | **done** |
| WCB firmware `WCB_DFP` + `?DFP` + `;D` + `?HELP,DFP` | **done** |
| `WDP_CAP_DFPLAYER` in WCB firmware, `WCBClient`, Wizard | **done** |
| Compile verification | **done** — NaviCore 6% flash, WCB-S3 63% flash; tool passes `jscheck` |
| Wikis — NaviCore (`Actions-Reference`, `WLED-and-HCR-Audio`, `Config-Tool-Guide`, `Configuration-Schema`, `Setup-Guide`, `Troubleshooting`, `Action-Editor…`) | **done** |
| Wikis — WCB (new `DFPlayer-Mini` page, `Command-Reference`, `Home`, `_Sidebar`) | **done** |
| `WcbCmd` golden vectors for `;D` | **done** — 20 byte-exact frames + 8 rejection cases in `examples/GoldenVectors`, compiles clean |

**Everything is pushed.** All six repos in §8 are on their masters — `WcbCmd` 0.8.0 first (the
prerequisite both firmwares' CI clones), then `NaviCore` firmware + tool,
`Wireless_Communication_Board-WCB`, `WCBClient`, the `Arduino-Code` sketchbook mirrors, and
both wikis. §8's order is the rule for the *next* change that touches `WcbCmd`, not
outstanding work.

**Verification note.** Both firmwares were compiled against the **repo** `WcbCmd` via
`--library c:/Users/ghulette/Documents/GitHub/WcbCmd`. Without that flag the sketchbook copy
shadows it and neither build would see `DfPlayerCodec` at all. The sketchbook copy has since
been refreshed, so a plain local compile works too — and CI clones `greghulette/WcbCmd`, whose
master carries 0.8.0. That gate is permanent: **nothing ships until that repo's master has the
push**, so any future `;D` change goes there first.

---

## Revision log

Newest first. Add a row whenever a code change alters what this page describes — same commit
as the code. Page body stays present-tense; history lives here.

| Date | Commit | Change |
|---|---|---|
| 2026-08-18 | _(uncommitted)_ | Status corrected to **shipped** — the header and the §9 tail still read "in progress" / "Nothing is pushed" after every repo had landed, which reads as a build-blocking `WcbCmd` prerequisite that was satisfied long ago. §8's push order restated as the rule for the next change rather than outstanding work. |
| 2026-08-05 | _(uncommitted)_ | Feature built across all five repos: `WcbCmd` `DfPlayerCodec` (0.8.0), NaviCore `RA_DFPLAYER` + `dfpDest` + `;D` dispatch, config-tool Audio tab and DFPlayer editor, WCB `WCB_DFP` + `?DFP` + `;D` routing + `WDP_CAP_DFPLAYER`, capability mirrors, both wikis. Page written alongside — decisions locked, wire format and verb table, data model, routing, traps, repo build order. |
