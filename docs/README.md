# NaviCore — Design and Troubleshooting

**Working documents for designing changes and diagnosing failures.** Not a tutorial and not
onboarding material — they assume you know the domain and want the constraints, the wire
contracts, and the reasons a thing is shaped the way it is.

Teaching material for droid builders lives in the
[wiki](https://github.com/greghulette/NaviCore/wiki), which is written in present tense for
someone using NaviCore today. These pages are the other audience: us, mid-change.

---

## Designing a change

| Document | Read it when |
|---|---|
| **[ARCHITECTURE.md](ARCHITECTURE.md)** | Always first. Hardware, module map, boot order, the main loop, and the dual-core concurrency rules that constrain every firmware change |
| **[PROTOCOLS.md](PROTOCOLS.md)** | Touching anything that crosses a wire — USB JSON, the CLI, the WCB mesh bridge, fragmentation, bulk transfer, OTA, RTERM |
| **[CONFIG_SCHEMA.md](CONFIG_SCHEMA.md)** | Adding or changing a user-configurable setting. Includes the cross-file invariants table and an add-a-field checklist |
| **[CONFIG_TOOL.md](CONFIG_TOOL.md)** | Editing `config_tool/index.html` — connect paths, save flow, function map, command library, timeline editor |
| **[BUILD_AND_RELEASE.md](BUILD_AND_RELEASE.md)** | Building, verifying, versioning, flashing, and the library-source rule that decides whether a change actually ships |
| **[ROADMAP.md](ROADMAP.md)** | Picking up queued work — decisions already made, questions still open, and the list of deliberate non-fixes |

## Diagnosing a failure

| Document | Read it when |
|---|---|
| **[TROUBLESHOOTING.md](TROUBLESHOOTING.md)** | Start from the symptom. LED states, silent mesh drops, save failures, phantom presses, build errors — each row names the cause and where to look |

## Feature design notes

| Document | Subject |
|---|---|
| [RECORD_REPLAY_DESIGN.md](RECORD_REPLAY_DESIGN.md) | Record / replay data model, storage, concurrency, timeline editor |
| [WCB_NATIVE_MAESTRO_DESIGN.md](WCB_NATIVE_MAESTRO_DESIGN.md) | Making Pololu servo control a native WCB verb |
| [DFPLAYER_DESIGN.md](DFPLAYER_DESIGN.md) | DFPlayer Mini as a second audio device — `;D` verb set, wire frames, routing |
| [MAESTRO_ACTIONS.md](MAESTRO_ACTIONS.md) | Maestro speed/accel limits, unit maths, delay semantics |
| [E22_REMOTE_LINK_DESIGN.md](E22_REMOTE_LINK_DESIGN.md) | NaviHiltCore (`boardType 2`) — E22 LoRa handheld remotes as an alternate input source alongside SBUS |

## Cross-repo findings

Reviews of other `WCB_Client` consumers on the same mesh. Decision input, not settled design
— each item carries a cost estimate and a recommendation. Items that have since been adopted
are marked **shipped** inline in the page; the rest are still open questions.

| Document | Subject |
|---|---|
| [SABE_FINDINGS_2026-08-11.md](SABE_FINDINGS_2026-08-11.md) | Sabé (Todd Word's droid, device 20). The `^`-chain one-hop cap, six adoption candidates, two corrections to how the ensured-send table and `?ETM,CHKSM` default actually behave. Shipped: the authoring-time chain warning, board ONLINE/OFFLINE logging, the boot roll call, the empty-password banner, both corrections. Open: checksum escape hatch, inbound rate limit, host-side tests |

---

## Three rules that cover most mistakes

1. **Mesh callbacks run on Core 0.** Anything touching flash, NVS, or droid hardware must be
   queued to `loop()` on Core 1.
2. **A mesh payload is capped at 187 bytes** after the bridge's CRC suffix. Larger means
   fragmentation or bulk transfer.
3. **Firmware and tool constants come in pairs.** Changing one without the other breaks
   silently — see [CONFIG_SCHEMA.md §6](CONFIG_SCHEMA.md#6-cross-file-invariants).

---

## Verification

There are no automated tests. What exists:

```bash
# Firmware — the compiler is the check
arduino-cli compile --fqbn "esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=cdc,PartitionScheme=custom,FlashSize=16M,PSRAM=opi" NaviCore.ino

# Config tool — no build step, so a syntax slip breaks all event wiring
node C:\Users\ghulette\tools\jscheck.js config_tool/index.html
```

A local firmware compile currently fails on a stale sketchbook `WCB_Client` —
see [BUILD_AND_RELEASE.md §3](BUILD_AND_RELEASE.md#3-verifying-a-firmware-change) for the
state and the fix. Until then, **CI is the authoritative compile**.

---

## How to use these pages

**Orient here, then verify in the code.** These documents are a map written at a point in
time. The firmware and the config tool are the territory, and they move. Use a page to find
*where* something lives and *why* it is shaped that way — then open the file and confirm the
detail before you rely on it.

- **Where a doc and the code disagree, the code is right and the doc is a bug.** Fix the doc
  as part of the same piece of work.
- Do not quote a constant, field name, or message shape out of a doc without checking it.
  Cite `file:line` when you report a finding.
- Line numbers drift and are marked `≈`; grep for the function or symbol name instead.
- Source comments outrank these pages. This codebase documents the reasoning behind
  non-obvious choices, usually after a real failure — treat those comments as constraints.

## Keeping them current

**A change to the code updates the affected page in the same commit.** These pages are only
worth reading if a cold session can trust them, and they earn that by never lagging behind.

Update a page when you change: a protocol or message shape, a config field, a capacity
constant, one of the paired firmware/tool invariants, a board profile or pin map, a build
flag, CI behaviour, a concurrency rule, or anything recorded as a deliberate non-fix.

Each page carries a **Revision log** at the bottom. Add a dated row for every change, with
the commit hash once it exists — that is where "what changed when" is answered. Page bodies
stay present-tense and describe how things work *now*; history belongs in the log, not
narrated inline. The user-facing wiki keeps no changelog at all.

---

## Revision log

Newest first. Add a row whenever a code change alters what this page describes — same commit
as the code. Page body stays present-tense; history lives here.

| Date | Commit | Change |
|---|---|---|
| 2026-08-11 | _(uncommitted)_ | Indexed [E22_REMOTE_LINK_DESIGN.md](E22_REMOTE_LINK_DESIGN.md) under Feature design notes. |
| 2026-08-12 | _(uncommitted)_ | Cross-repo findings blurb updated — items from the Sabé review have now been adopted, so the section no longer claims nothing has been acted on. |
| 2026-08-11 | _(uncommitted)_ | Added the Cross-repo findings section and indexed [SABE_FINDINGS_2026-08-11.md](SABE_FINDINGS_2026-08-11.md). |
| 2026-08-05 | _(uncommitted)_ | Indexed [DFPLAYER_DESIGN.md](DFPLAYER_DESIGN.md) under Feature design notes. |
| 2026-08-04 | _(uncommitted)_ | Initial version. |
