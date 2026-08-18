# NaviCore — Roadmap and Open Work

Queued and in-progress work, with the decisions already made and the questions still open.
Start here when picking up a feature rather than re-deriving the design.

Design notes with their own documents: [RECORD_REPLAY_DESIGN.md](RECORD_REPLAY_DESIGN.md) ·
[WCB_NATIVE_MAESTRO_DESIGN.md](WCB_NATIVE_MAESTRO_DESIGN.md) ·
[MAESTRO_ACTIONS.md](MAESTRO_ACTIONS.md)

---

## 1. Serial ports as full mesh citizens — **shipped**

NaviCore's serial ports are mesh citizens in both directions, the same way a WCB's are. The
design comment above `struct SerialFwdMsg` in [`NaviCore.ino`](../NaviCore.ino) is the
authority; this is the summary.

**Mesh numbering is S1 / S2 / S3 → firmware S3 / S4 / S5**, matching the NaviCore v2
silkscreen. `rcFwPortForMesh()` / `rcMeshPortForFw()` in [`rc_config.h`](../rc_config.h) are
the whole translation layer — every mesh-facing path converts once at the edge and works in
firmware numbering after that. WDP PORTLABEL advertises the same numbers
(`rcWdpPortForLabel()`), with the **Maestro on WDP port 4**: visible in `?WDP,LIST` and the
Wizard, but deliberately not a port `;s<n>` can reach, because it is a binary bus and not
text-addressable.

**Targeted writes are always accepted — no config needed.** A WCB sending `;w20,;s2<cmd>`
delivers exactly `";s2<cmd>"` to `onWCBCommand`: the sending board strips the `;w20,` and the
rest arrives verbatim. The payload is written raw + CR with no comma stripping, byte-identical
to what the same command would put on a WCB's own port.

**Broadcast is per-port opt-in**, `rcConfig.serialBcastOut[]` / `serialBcastIn[]` — both
default off, so a port joins the broadcast domain only when enabled in the tool's **Serial
Ports** tab:

- **Out** — an unprefixed mesh command goes out every `serialBcastOut` port
  (`queueSerialBroadcastOut()`). It is never re-broadcast to the mesh; the out-path is
  mesh→serial only, so there is no loop to break.
- **In** — a **terminated** line from a `serialBcastIn` port is broadcast to the mesh and out
  the other opted-in ports (`auxRxLine()`). A fragment from a buffer-full or idle flush is
  shown on the terminal but never broadcast — a command is a line, the same rule a WCB's
  `processIncomingSerial` uses. Device-owned ports are skipped: an HCR's status frames are
  that driver's protocol, not commands.

**Every write is deferred to Core 1 through `serialFwdQueue`.** `onWCBCommand` runs on the
Core-0 WiFi task and S4/S5 are bit-banged SoftwareSerial, where a write blocks with interrupts
off for the whole frame time (~1 ms per 10 chars at 9600) — on the WiFi task that stalls
ESP-NOW, and on either core it jitters the ~111 fps SBUS path. `drainSerialFwd()` in `loop()`
does the writing. **Never write an aux port directly from a mesh callback.**

`auxRxLine()` in [`NaviCore.ino`](../NaviCore.ino) is where an "act on incoming serial" parser
belongs — it already has the assembled line and the terminated/fragment distinction.

**Remaining.** Nothing queued. WCB-method reads for the local Maestro are tracked in §2.

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
| Cloud backup keyed on user-supplied credentials **only**, with no per-install discriminator | A per-install discriminator would break restore-into-a-blank-tool. A **username + password** pair you choose (independent of the WCB password) is the entire input to both the slot address and the AES-GCM key — `_cfgSecret(user, pw)` feeds `_cfgSlotBase()` and `_cfgKey()` and nothing device- or browser-specific joins it, which is exactly what lets a blank tool on a new machine find and decrypt the ring. See [CONFIG_TOOL.md §9](CONFIG_TOOL.md#9-two-extras-worth-knowing). The caveat is surfaced in the modal instead |
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
| 2026-08-18 | _(uncommitted)_ | §1 rewritten as **shipped** — the mesh↔serial bridge (S1-S3 → firmware S3/S4/S5 numbering, always-on targeted `;s<n>` writes, per-port broadcast in/out, the `serialFwdQueue` Core-1 hop) is built in firmware and tool; the answered open questions are carried forward as constraints. §5: corrected the cloud-backup row — backups are keyed on an independent **username + password** pair, not the WCB password; the no-per-install-discriminator rationale still stands. |
| 2026-08-13 | _(uncommitted)_ | Recorded the decision NOT to port the WCB `?ETM,CHAR` network test to NaviCore, with the reasoning — chiefly that `ETM_RETRY_INTERVAL_MS` has no setter, so the recommended timeout it produces cannot be applied. Relaying `?MGMT,ETM,CHAR` is the cheap path if it is ever wanted. |
| 2026-08-04 | _(uncommitted)_ | Initial version. |
