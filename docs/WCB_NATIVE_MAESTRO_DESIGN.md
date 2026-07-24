# WCB-Native Maestro (Pololu servo) — Design Note

Status: **DESIGN — starting point for the WCB-firmware session.** Not yet built.
Last updated 2026-07-22.

Goal: make the raw **Pololu servo protocol** (setTarget / goHome / setSpeed / …) a
**first-class native verb on the WCB**, the same way HCR / MP3 / WLED already are —
instead of the WCB being a dumb byte-pipe for Maestro commands that NaviCore
pre-builds. This "expands the WCBs' capability" (they can drive a Maestro on their
own, from a clean verb) and deletes NaviCore's special-case raw-byte path.

> Companion context: the `;M` **sequence-trigger** verb is *already* native on the
> WCB (WCB_Maestro triggers stored Maestro scripts). This note is about the **other**
> half — direct servo control (the Pololu channel/position/speed protocol) — which
> today is NOT native. When both exist, the WCB fully owns Maestro.

---

## 1. How remote Maestro works TODAY (the thing we're changing)

NaviCore is the brain; the WCB is a byte pipe.

- A NaviCore **configured Maestro** is a slot `config.maestros[0..7]`, shape
  `{ type, device, channels }`:
  - `type` = 0 Disabled / 1 **Local** / 2 **Remote (broadcast)**
  - `device` = Pololu protocol device # 0–127 (set on the physical Maestro in
    Maestro Control Center; this is what disambiguates which Maestro obeys)
  - `channels[]` = `{ ch, name, min, max }` (¼µs), imported from Pololu XML
  - **No WCB id / no serial-port # is stored** — a "Remote" Maestro is reached by
    *broadcast*, and the on-wire Pololu `device#` picks the right one.
- Dispatch: `maestroWrite(id, …)` in [NaviCore.ino:519](../NaviCore.ino#L519):
  `Stream* dest = (slot.type == 1) ? &Serial2 : maestroBroadcast;`
  - **Local** → bytes go out **Serial2** (GPIO6, the wired Maestro bus).
  - **Remote** → bytes go out **`maestroBroadcast`** ([NaviCore.ino:198](../NaviCore.ino#L198)),
    a single shared `WCBStream` over ESP-NOW. **Every** WCB forwards the raw bytes
    to *its* configured Maestro port; the Maestro hardware itself ignores commands
    not addressed to its `device#`.
- The Pololu bytes are built by the **shared translator** `<WcbCmd.h>`
  ([NaviCore.ino:50](../NaviCore.ino#L50)) — e.g. `WcbMaestro::buildSubroutineFrame(...)`
  at [NaviCore.ino:540](../NaviCore.ino#L540). **This translator is already a shared
  library** — that is the key enabler below.

Config-tool side (already done, 2026-07-22): the command library now has **one**
Pololu-servo board, `nc-maestro` = **"Maestro (Pololu servo)"** (kind `maestro`,
`local:'maestro'`). Authoring from it writes a firmware action
`{ type:'maestro', target: <slot 1-8>, cmd:'setTarget,0,6000' }`; the destination
picker is `maestroIdOptions()` (the configured slots, labeled
`Maestro N — Local/Remote · dev D`). The old raw-`;W<wcb>;S<port>` board
(`maestro-native`) was collapsed into it via `NC_CMDLIB_SUPERSEDE` — **no raw WCB
port numbers anywhere in the Maestro UX.**

---

## 2. Proposed change

> **STATUS 2026-07-23 — DISCRETE writes: BUILT (commit `dea2220`).** The WCB side
> (WcbMaestro verb parser) and NaviCore's discrete-write emit both shipped. NaviCore
> auto-routes at dispatch: a **discrete** command (via `executeMaestroCmd`) to a
> **Remote** slot is sent WCB-native; **passthrough / scrub / replay-keyframe streams
> stay RAW** (a `g_maeDiscrete` flag scoped to `executeMaestroCmd` is the discriminator
> — see `maestroWrite`). Addressing is **unicast-by-WDP, not broadcast** (§3b was
> revised — see below). Remaining: the WCB-method **reads** (Phase 2, §10) and a
> `stopScript`/`setEasing` WcbCmd verb (today `stopScript` falls back to the raw path).

Move the **byte-building from NaviCore to the WCB**, so the mesh carries a **verb**,
not raw Pololu bytes:

- **WCB (new):** parse a native Maestro verb (`setTarget,ch,pos`, `goHome`,
  `setSpeed,ch,v`, `setAccel,ch,v`, `setSpeedAccel,ch,s,a`, `setEasing,profile`,
  `restartScript,n`, `stopScript`), build the Pololu frame with **the same
  `<WcbCmd.h>` / `WcbMaestro` builder NaviCore uses**, and write it to the WCB's
  already-configured Maestro serial port. No new WCB config — the WCB already knows
  its Maestro port (it forwards raw bytes there today).
- **NaviCore (change):** for a **Remote** slot, emit the **verb frame** over the
  existing `maestroBroadcast` stream instead of pre-built raw bytes. **Local** slots
  (type 1 → Serial2) stay exactly as they are — NaviCore keeps talking Pololu to a
  directly-wired Maestro.

The verbs are identical to what `nc-maestro` already emits (`cmd` is
`setTarget,0,6000` etc.), so the NaviCore command model doesn't change — only the
*wire format between NaviCore and the WCB* does.

---

## 3. Key decisions (get these right up front)

### 3a. Keep the model-ownership split clean
- **NaviCore = author.** Maestro Locations owns names, channel min/max, XML import,
  and the `device#`. This is the single source of truth.
- **WCB = executor.** It needs only the incoming verb + `device#` + a verb parser.
  **Do NOT** start storing channel names/limits on the WCB — that forks the config.

### 3b. Addressing — DECIDED: unicast-by-WDP (Option B, as built)
> **Revised 2026-07-23.** We went with **Option B (unicast)**, NOT the broadcast in
> Option A — but with **no schema change**, because WDP already carries the mapping:
> the WCB advertises its local Maestro device#s (`WDP_TLV_MAESTRO 0x06`), NaviCore
> decodes them into `WCBNeighbor.maestroIds[]`, and `wcbHostingMaestro(dev)` resolves
> `device# → hosting WCB` from live neighbors at dispatch. So it self-heals if a
> Maestro moves WCBs, and stays location-independent (you still address by device#, the
> firmware finds the WCB). If no WCB advertises the device# yet (WDP not converged),
> it falls back to the raw broadcast so the command still lands. The original Options
> A/B text is kept below for context.

- **Option A (drop-in, NOT chosen):** NaviCore broadcasts the verb frame tagged
  with the target `device#`. Every WCB builds the Pololu bytes (which embed that
  `device#`) and writes to its Maestro port; only the addressed Maestro obeys —
  **exactly today's semantics**, just with the byte-building moved to the WCB. This
  preserves the nice **location-independence**: you address a Maestro by `device#`
  and don't care which WCB it's plugged into. Lowest risk, no schema change.
- **Option B (future optimization):** unicast the verb to a specific WCB. Less mesh
  chatter, but requires the slot to record **which WCB** each Maestro is on — a new
  `{ wcb, port }` field on `config.maestros[]` (the WLED slot `{id,port,wcb,configured}`
  is the precedent). Defer until Option A is working; add as an optional target.

### 3c. Backward compatibility
- Keep the raw-byte passthrough path alive behind a flag during migration, so a WCB
  running old firmware still works while the new verb path is rolled out.
- Version the verb frame (a type/opcode byte) so the WCB can tell verb frames from
  legacy raw bytes on the shared stream.

---

## 4. Why it's worth it
- **Consistency** — Maestro joins HCR / MP3 / WLED as a native WCB verb; the library
  routes it like `wcb-hcr`. Removes NaviCore's special-case.
- **Autonomy** — a WCB can drive its Maestro from local triggers without NaviCore in
  the loop (resilience: the dome keeps working if the controller drops out).
- **Cleaner record/replay** — NaviCore records the readable verb (`setTarget,0,6000`)
  instead of an opaque byte stream. See [RECORD_REPLAY_DESIGN.md](RECORD_REPLAY_DESIGN.md).
- **Less NaviCore work** — no Pololu byte construction on the controller for remotes.
- **Cheap to build** — the encoder (`<WcbCmd.h>`/`WcbMaestro`) is already shared, so
  the WCB reuses it; this is "wire it up," not "write a Pololu stack."

## 5. Full-circle: the library board comes back (as a legit one)
Once the WCB speaks Maestro natively, re-introduce a native **`wcb-maestro-servo`**
board in the DroidNet/NaviCore command library — a real `wcb-verb` / `device-native`
board routed **by WCB** (like `wcb-hcr`), NOT the raw-`;W;S`-port hack we just
deleted. It can coexist with `nc-maestro` (configured-slot addressing) — or, if
Option A stays, `nc-maestro` simply starts emitting the verb and no new board is
needed. Decide once the WCB side exists.

---

## 6. Implementation checklist

**WCB firmware** (`greghulette/Wireless_Communication_Board-WCB`, `Code/WCB`, branch OTA):
- [ ] Link/confirm `<WcbCmd.h>` is available to the WCB (it's the shared translator).
- [ ] Add a Maestro **verb parser** → `WcbMaestro::build…` → write to the configured
      Maestro serial port. Mirror how WCB_HCR / WCB_MP3 / WCB_WLED handle their verbs.
- [ ] Define the verb frame on the mesh (opcode + device# + verb payload); keep it
      distinguishable from legacy raw bytes.
- [ ] Sits alongside the existing `;M` sequence-trigger (WCB_Maestro) so the WCB owns
      both stored-sequence and direct-servo control.

**NaviCore firmware** (this repo):
- [ ] `maestroWrite()` remote branch ([NaviCore.ino:519-553](../NaviCore.ino#L519)):
      emit the verb frame over `maestroBroadcast` instead of pre-built raw bytes.
      Leave the **Local/Serial2** branch untouched.
- [ ] Feature-flag the old raw path for migration.

**WCB_Client library** (`greghulette/WCBClient`, master; **mirror to**
`Arduino-Code/libraries/WCB_Client/`): if the verb framing/helpers belong client-side,
add them here and keep both copies byte-identical (as with the temporary-peer work).

**Config tool** (already Pololu-servo-clean): no change needed for Option A. For
Option B, add `{ wcb, port }` to the Maestro slot schema + Maestro Locations UI + CSV
export/import.

---

## 7. File / entry-point map
- Remote dispatch + shared translator: [NaviCore.ino:519](../NaviCore.ino#L519)
  (`maestroWrite`), [:198](../NaviCore.ino#L198) (`maestroBroadcast`),
  [:50](../NaviCore.ino#L50) (`#include <WcbCmd.h>`), [:540](../NaviCore.ino#L540)
  (`WcbMaestro::buildSubroutineFrame`).
- Maestro slot schema `config.maestros[]` `{type,device,channels}` + picker
  `maestroIdOptions()`: `config_tool/index.html` (Maestro Locations panel + composer).
- `nc-maestro` board (the verb source) + `NC_CMDLIB_SUPERSEDE`: `config_tool/index.html`.
- Related specs: [MAESTRO_ACTIONS.md](MAESTRO_ACTIONS.md),
  [RECORD_REPLAY_DESIGN.md](RECORD_REPLAY_DESIGN.md).

## 8. Repos in play
- NaviCore (this repo) — controller firmware + config tool.
- `greghulette/Wireless_Communication_Board-WCB` (`Code/WCB`, OTA) — WCB firmware (the new verb parser).
- `greghulette/WCBClient` (master) + vendored `Arduino-Code/libraries/WCB_Client/` — keep in sync.
- `<WcbCmd.h>` — shared device-command translators (Maestro/MP3/WLED/HCR); the encoder to reuse.

## 9. Open questions
- Verb frame format on the mesh (opcode/versioning) — reuse an existing WCB verb
  envelope if there is one.
- Option A vs B addressing (see §3b) — start with A.
- Whether `nc-maestro` keeps slot-addressing or a new `wcb-maestro-servo` board is
  added (see §5) — decide after the WCB side lands.

---

## 10. Maestro query readback (2-way) — separate but related

The shared `WcbMaestro` also builds the **query** verbs `getPosition` / `getMovingState`
/ `getErrors`, which make the Maestro *reply* with bytes. The library builds the REQUEST;
**reading** the reply bytes off serial is still the firmware's job — but **WcbCmd ≥ 0.7.0
now ships the shared decode/format helpers** so the reply text can't drift from NaviCore's
parser:
- `WcbMaestro::replyInfo(cmd, kind, len)` — map a get* frame's command byte (`out[2]`:
  `0x10`/`0x13`/`0x21`) to its reply `ReplyKind` (POS/MOV/ERR) and byte count (2/1/2).
- `WcbMaestro::decodeReply(kind, bytes, len, value)` — raw LOW/HIGH bytes → `uint16_t`.
- `WcbMaestro::formatReply(out, cap, id, chan, kind, bytes, len)` — emit the exact
  `:MQR,<id>,<chan>,<KIND>,<value>` line below (`REPLY_TEXT_MAX`-byte buffer).

**Reply wire format (both hosts):** `getPosition` → 2 bytes **LOW then HIGH**, value
`(hi<<8)|lo` in ¼µs. `getMovingState` → 1 byte (0/1). `getErrors` → 2 bytes LOW/HIGH,
16-bit bitmask, **and reading CLEARS the Maestro's error register**. Replies are
**anonymous** — no address, no channel, no sequence; `getPosition` and `getErrors` are
both 2 bytes and indistinguishable except by knowing which query you sent. → **one
query outstanding at a time.**

### Phase 1 — LOCAL readback: DONE (commit `fe8ecf4`)
NaviCore reads a local Maestro's reply off Serial2 (GPIO7) synchronously in
`loop()`/Core-1: `maestroLocalQuery()` drains stale RX, sends one request via
`maestroWrite()`, reads N bytes with a ~25 ms deadline. Surfaced via `?MAE,GET/MOVING/ERR`
CLI → `[MAE:<slot>]{…}` marker → config-tool Maestro Locations inline readout
(`_maeReadFeed`). Local slots only.

### Phase 2 — REMOTE readback: NaviCore side DONE; WCB relay is the remaining TODO
The mechanism exists but the addressing does not:
- **WCB already pumps Maestro RX back:** `forwardMaestroDataToRemoteKyber()` (WCB.ino
  ~3861) drains the Maestro UART and re-broadcasts the bytes via `sendESPNowRaw()`
  (WCB.ino ~2223) as an ESP-NOW frame with `structTargetID = "98"` (`WCB_TARGET_KYBER`),
  a 2-byte LE length in `structCommand[0..1]`, raw bytes at `structCommand+2`. But it's
  a **verbatim, untagged byte pump** — no query id, no channel, no source Maestro id.
- **WCB_Client has NO reply message type** — only COMMAND/ACK/HEARTBEAT/WDP. Inbound
  hooks are `onCommand(cb)` (252-B ETM text, CRC-checked) and `onRawPacket(cb)` (any
  non-252-B packet, fires BEFORE password/addressing gates → must re-validate). Both run
  in the WiFi RX task → must queue and defer to `loop()` (copy the MgmtRelay pattern).
- **Recommended Phase-2 transport:** have the WCB **correlate the reply to its request on
  the WCB side** (it knows which query/port/Maestro it just forwarded) and relay it as a
  normal **text COMMAND** unicast to NaviCore (device id 20) carrying the context the raw
  bytes lack — `:MQR,<maestroId>,<chan>,<KIND>,<value>`, built with
  `WcbMaestro::formatReply()` (WcbCmd 0.7.0) rather than by hand, since that is the exact
  text `maeConsumeRemoteReply()` parses. Rides the existing CRC/ETM/ACK'd path, needs no
  new packed struct, lands in `onCommand`. NaviCore queues
  it in the callback and matches it to the outstanding request in `loop()`. Ensure
  `enableSpecialPeer(20)` + the relaying WCB has `?SPECIAL,ON,20`.
- **NaviCore side is now implemented (this commit):** a REMOTE `?MAE,GET/MOVING/ERR`
  (and the skip-if-running gate) broadcasts the Pololu query frame via `maestroWrite()`;
  the relayed `:MQR,<id>,<chan>,<KIND>,<value>` is parsed by `maeConsumeRemoteReply()`
  (Core 0) into `g_maeRemote[]` and flagged, then `maePumpRemoteEmits()` (Core 1, loop)
  surfaces it as the existing `[MAE:<slot>]` marker — so the Maestro Locations "Read live"
  controls are now ungated for Remote slots and render with no further UI work (a reply
  relayed as `[TERM:20][MAE:…]` still unwraps). The skip-if-running gate warms
  `g_maeRemote[]` with a fresh `getMovingState` when the cache is stale and fails open until
  the reply lands. **All that remains is the WCB relay above** — read the reply bytes,
  `WcbMaestro::formatReply()`, unicast to id 20; until it ships, remote reads show
  `(no response)`.
- **Gotcha:** `getErrors` clears on read, so a dropped/duplicated remote relay silently
  loses or double-consumes error state — correlate + treat as best-effort async.
