# NaviCore — Working Notes

Astromech animation controller: ESP32-S3 firmware that reads SBUS from an RC transmitter and
dispatches the pilot's inputs to servos, sound, and lights — locally over serial or across a
WCB ESP-NOW mesh — plus a single-file browser config tool that talks to it over Web Serial.

**Design and troubleshooting documentation is in [`docs/`](docs/README.md). Read the page for
the area you are changing before editing.** Those pages are working documents for making
changes and diagnosing failures — not tutorials. The user-facing teaching material is the
[wiki](https://github.com/greghulette/NaviCore/wiki), a separate repo with a different
audience and different rules.

| Working on | Read |
|---|---|
| Diagnosing an observed failure | [docs/TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md) |
| Firmware, anything structural | [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) |
| Anything that crosses a wire | [docs/PROTOCOLS.md](docs/PROTOCOLS.md) |
| A user-configurable setting | [docs/CONFIG_SCHEMA.md](docs/CONFIG_SCHEMA.md) |
| `config_tool/index.html` | [docs/CONFIG_TOOL.md](docs/CONFIG_TOOL.md) |
| Building / shipping / flashing | [docs/BUILD_AND_RELEASE.md](docs/BUILD_AND_RELEASE.md) |
| Picking up queued work | [docs/ROADMAP.md](docs/ROADMAP.md) |

---

## This repo is worked on from more than one machine

**`git fetch` and check ALL BRANCHES before making any change.** The same repo is worked on
from a Windows box and a Mac, so the local clone may be behind even when nothing here has
changed.

**Checking only `origin/main` is not enough, and this has already gone wrong.** Work pushed
from the Mac landed on a branch called `macos-first-run`; `git status -sb` and
`git log HEAD..origin/main` both reported "nothing incoming" for two days while eleven
commits sat on the remote. The report was true and useless. Look at every branch:

```bash
git fetch --all --prune
git branch -r                                  # EVERY remote branch, not just main
git log --oneline --all --not main             # anything anywhere that main lacks
git status -sb                                 # then the usual "behind N" check
```

**Commit to `main`. Do not create feature branches in this repo.** The default Claude Code
guidance is to branch when on the default branch, and that is what produced the split above
— one session branched, another committed straight to `main`, and neither was wrong on its
own. This repo wants a single line of history. If you think a branch is genuinely warranted,
say so and get agreement first.

Pull before you start, not when the push is rejected. If a push IS rejected, rebase onto
what arrived and re-read anything you were about to edit — do not force.

## The code is the source of truth

**Read the docs to orient, then confirm against the code before you act.** These pages are a
map written at a point in time; the firmware and the tool are the territory. If a doc and the
code disagree, **the code is right and the doc is a bug** — fix the doc as part of your work.

In practice:

- Never assert a constant, field name, message shape, or behaviour from a doc alone. Open the
  file and check. Cite `file:line` in what you report back.
- Line numbers in the docs drift — they are marked `≈`. Function and symbol names are the
  reliable handle; grep for those.
- The **comments in the source outrank the docs too.** They record why a non-obvious choice
  was made, often after a real failure. Treat them as constraints.
- If verifying is expensive and you proceed on a doc's word anyway, say so explicitly rather
  than presenting it as checked.

## Keep the docs current — same commit as the code

**Any change that alters something `docs/` describes updates `docs/` in the same commit.** Not
"later", not a follow-up issue. The docs are only worth reading if a cold session can trust
them, and they earn that by never lagging the code.

What counts: protocol/message shapes, config fields, capacity constants, the invariant pairs,
board profiles or pin maps, build flags, CI behaviour, concurrency rules, and anything listed
as a deliberate non-fix. Plus **any fix whose *cause* is a trap that could be walked into
again** — the Core-0 callback rule and the 187-byte cap are both one-line constraints that
came out of real failures, and the constraint is the part worth writing down, not the diff.

What doesn't: a fix that restores behaviour `docs/` already describes correctly, a refactor
with no observable change, a perf tweak nothing can see, typos, or test-only changes. Logging
those buries the rows that matter. When unsure, document it.

Every doc in `docs/` ends with a **Revision log** — add a dated row there for each change,
with the commit hash once it exists. The body of a doc stays present-tense (describe how it
works *now*, no "this used to…" narration inline); the revision log is where history lives.
The wiki is present-tense too, and carries **no changelog at all** — different audience.

---

## Rules that are easy to break

1. **Mesh callbacks run on Core 0** (`onWCBCommand`, `onNeighbor`, raw-packet hooks, and
   everything `rcTelemetry::handle()` does inline). Anything touching flash, NVS, or droid
   hardware must be queued to `loop()` on Core 1. Create the queue *before* registering the
   callback that feeds it.
2. **A mesh payload is capped at 187 bytes** — ESP-NOW's 250 minus the bridge's
   `"|CRC%08X"` suffix in a 200-byte buffer. Over that, the packet is silently dropped.
   Measure escaped UTF-8 *bytes*, not string length.
3. **Firmware and tool constants come in pairs.** Changing one without the other fails
   silently — the table is in [docs/CONFIG_SCHEMA.md §6](docs/CONFIG_SCHEMA.md#6-cross-file-invariants).
4. **New top-level JSON fields must be added to the ArduinoJson filter whitelist** in
   `handleSerialInput()`, or they are stripped and read as defaults.
5. **A `WCB_Client` change only ships once pushed to `greghulette/WCBClient` master.** CI
   clones that repo; the `Arduino-Code/libraries/WCB_Client` copy is local-bench only.

## Verifying

```bash
# Firmware — the compiler is the only check (no unit tests)
arduino-cli compile --fqbn "esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=cdc,PartitionScheme=custom,FlashSize=16M,PSRAM=opi" NaviCore.ino

# Config tool — no build step, so a syntax slip silently breaks all event wiring
node C:\Users\ghulette\tools\jscheck.js config_tool/index.html
```

Every FQBN field is load-bearing (`PSRAM=opi` especially — without it the board halts on a
red LED). A local compile of current `main` fails on a stale sketchbook `WCB_Client`; see
[docs/BUILD_AND_RELEASE.md §3](docs/BUILD_AND_RELEASE.md#3-verifying-a-firmware-change).
Until it is refreshed, **CI is the authoritative compile**.

## Conventions

- **Comments explain *why*.** This codebase documents the reasoning behind non-obvious
  choices, especially where a naive change would reintroduce a fixed bug. Match that density
  — and when you find such a comment, treat it as a constraint, not decoration.
- **Pushing is outward-facing.** It triggers CI, republishes the public config tool, and can
  change what users flash. Confirm before pushing unless told to proceed.
- The **wiki is a separate repo** (`C:\Users\ghulette\Documents\GitHub\NaviCore.wiki`, branch
  `master`). A user-visible change likely needs a matching wiki edit.
- The pre-commit hook stamps the firmware DTG and the tool's footer on every commit —
  leave `FW_VERSION_DTG` alone.
