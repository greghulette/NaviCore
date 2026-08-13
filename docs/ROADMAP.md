# NaviCore — Roadmap and Open Work

Queued and in-progress work, with the decisions already made and the questions still open.
Start here when picking up a feature rather than re-deriving the design.

Design notes with their own documents: [RECORD_REPLAY_DESIGN.md](RECORD_REPLAY_DESIGN.md) ·
[WCB_NATIVE_MAESTRO_DESIGN.md](WCB_NATIVE_MAESTRO_DESIGN.md) ·
[MAESTRO_ACTIONS.md](MAESTRO_ACTIONS.md)

---

## 1. Serial ports as full mesh citizens — **queued, design settled**

**Goal.** NaviCore's serial ports behave like a WCB's on the mesh: reachable from the mesh,
and able to forward mesh traffic outward.

**Decisions locked:**

1. **Mesh port numbering is S1 / S2 / S3**, mapping to firmware **S3 / S4 / S5** — matching
   the NaviCore v2 silkscreen ("Serial 1/2/3"). This needs a translation layer.
2. Build **both directions**.
3. Primary ask — **Broadcast Out**: a mesh broadcast NaviCore receives is forwarded out each
   serial port that has broadcast-out enabled.
4. Also wanted — **targeted writes**: a WCB sends `;W20;S<n>,<cmd>` and NaviCore writes
   `<cmd>` to that port, exactly like WCB↔WCB.

**Already shipped, needs renumbering.** WDP port labels: `WCB_Client` 1.11.0 `setPortLabel()`
+ the PORTLABEL TLV; `rcConfig.serialLabels` + `rcSerialLabel()` / `rcSerialLabelAuto()` +
`rcAdvertiseSerialLabels()`; the tool's "Serial Ports" tab (per-port Baud + Label table).
These currently advertise WDP port 2 = Maestro and 3/4/5 = S3/S4/S5 — **renumber to
S1/S2/S3 = firmware S3/S4/S5**.

**Open questions to resolve during the build:**

- What a slot-20 client's `WCBCommandCallback(senderID, command)` actually receives after the
  bridge relays `;W20;S<n>,cmd` — does the relay strip `;W20`? Is the `;S<port>` routing
  still in the text? Decide between parsing routing from the command text on NaviCore, or
  adding a `WCB_Client` hook exposing was-broadcast + target-port (the callback exposes
  **neither** today).
- The exact WCB `;W<id>;S<port>` wire format — trace `processWCBMessage` and
  `getSerialStream(port).write()` in the WCB firmware. The raw binary Maestro/Kyber path is
  separate (`structCommand[0]` = target port).
- Where the local Maestro sits in S1–S3 numbering. It is a binary bus and not text-`S`
  addressable — likely labelled but non-addressable.
- Real-time safety: serial read/forward in `loop()` must not disturb the ~111 fps SBUS path.

**Build order.** Protocol trace → S1–S3 renumber (WDP labels + tool table) → per-port
`broadcastOut` / `broadcastIn` config → firmware forwarding (mesh→serial broadcast, then
targeted `;S<n>`→serial) → optional serial→mesh → tool toggle columns.

`pollAuxSerialRx()` in [`NaviCore.ino`](../NaviCore.ino) is the existing hook where an
"act on incoming serial" parser belongs.

---

## 2. WCB-native Maestro — **partially shipped**

Full design in [WCB_NATIVE_MAESTRO_DESIGN.md](WCB_NATIVE_MAESTRO_DESIGN.md).

Goal: make the raw Pololu servo protocol a first-class **verb** on the WCB, the way HCR /
MP3 / WLED already are, so the mesh carries verbs instead of pre-built Pololu bytes.

**Shipped.** Discrete writes. The WCB parses the native Maestro verb and builds frames with
the same shared `WcbCmd` / `WcbMaestro` builder; NaviCore auto-routes at dispatch — a
discrete command (through `executeMaestroCmd`) to a **Remote** slot goes WCB-native
**unicast-by-WDP**, while passthrough / scrub / replay-keyframe streams stay **raw** (the
`g_maeDiscrete` flag scoped to `executeMaestroCmd` is the discriminator, read in
`maestroWrite`).

**Remaining.** WCB-method **reads** (design §10), and `stopScript` / `setEasing` as `WcbCmd`
verbs — `stopScript` currently falls back to the raw path.

This is a **WCB-firmware-side** effort; NaviCore only needs the emit side kept in step.

---

## 3. Record / replay — phases 1 and 2 shipped

[RECORD_REPLAY_DESIGN.md](RECORD_REPLAY_DESIGN.md) is the full spec and remains the
reference for the data model, the servo-anti-snap decision, and the concurrency analysis
that dictated the capture-queue design.

Working today: capture, named clips on the 12 MB `clips` partition, replay with per-channel
interpolation and ease-from-home anchoring, loop mode, the 60 s `REC_MAX_MS` backstop with
auto-save, and the full timeline editor with live servo preview and Maestro-script export.

Constraints worth remembering: the clip buffer is 24 000 events (~3.1 MB PSRAM);
`?REC,EDITLOAD` over the mesh refuses clips above 3000 events (use USB).

---

## 4. Mesh dedup on WCB boards — **separate WCB session**

NaviCore and `WCB_Client` (≥ 1.10.0) already de-duplicate inbound COMMANDs by sequence
number using a per-sender `{haveSeq, high, mask32}` window in `WCBBoardStatus`. The
equivalent needs porting into the **WCB boards' own** ETM_Droid firmware
(`_cmdSeqDup` → `ETM_BoardStatus`). Tracking note: one droid board (Leia) has the
`WCB_Client` dedup after a reflash; HP_Controller is still pending.

---

## 5. Known deliberate non-fixes

Do not "fix" these — each was decided:

| Thing | Why it stays |
|---|---|
| Cloud backup keyed on the WCB password **alone** | A per-install discriminator would break restore-into-a-blank-tool. The caveat is surfaced in the modal instead |
| HCR `chan` encoding differs between fn 18/19 and fn 14/16/17 | Legacy actions carry `chan=0` meaning ALL; harmonising would silently repoint existing configs |
| Dedup window not reset on sender reboot | Essentially unreachable (needs ~5000 post-reboot commands) and fixing it costs a `WCB_Client` reflash |
| `FRAG_MAX_PARTS` capped at 192 | 384 crash-loops the board. A larger bridged payload needs a heap-allocated buffer, not a bigger static array |
| Cloud backup has no visible button | Intentionally hidden: click the "NaviCore" wordmark 4× |
| The `sbusSharedUart = false` code path | Kept as a fallback if shared SBUS ever proves unreliable on a board |
| No NaviCore port of the WCB's `?ETM,CHAR` network test | Decided 2026-08-13. **There is no knob to act on the result**: the test's whole output is a recommended ACK timeout, and `ETM_RETRY_INTERVAL_MS` is a compile-time `#define` in `WCB_Client` with no setter — NaviCore would run a 30–60 s test to produce a number it cannot apply, while the WCBs *can* apply it fleet-wide with `?ETM,TIMEOUT`. Also: phase 3 deliberately floods the mesh, and NaviCore's `loop()` carries `processSbus()` every 9–14 ms; phase 3 needs peer orchestration NaviCore has no way to drive; and the library has no RTT instrumentation to measure with. If fleet characterization is ever wanted from this tool, **relay `?MGMT,ETM,CHAR,<n>` to the bridge WCB** — the existing, proven, orchestrated test, no firmware work. A NaviCore-native version would be a small per-peer link probe (RTT min/avg/max, no flooding), not this |

---

## Revision log

Newest first. Add a row whenever a code change alters what this page describes — same commit
as the code. Page body stays present-tense; history lives here.

| Date | Commit | Change |
|---|---|---|
| 2026-08-13 | _(uncommitted)_ | Recorded the decision NOT to port the WCB `?ETM,CHAR` network test to NaviCore, with the reasoning — chiefly that `ETM_RETRY_INTERVAL_MS` has no setter, so the recommended timeout it produces cannot be applied. Relaying `?MGMT,ETM,CHAR` is the cheap path if it is ever wanted. |
| 2026-08-04 | _(uncommitted)_ | Initial version. |
