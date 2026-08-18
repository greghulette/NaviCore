# Maestro Actions — Speed / Acceleration limits & script-completion feedback

Reference for using the Maestro action commands (buttons, switches, knobs, timeline
actions) — specifically the **speed** and **acceleration** limits — and a note on
what it would take to detect when a Maestro **script/move finishes**.

The Maestro action commands available in the config tool's action editor are:

| Command (tool label)        | Wire form              | Notes |
|-----------------------------|------------------------|-------|
| Set servo position          | `setTarget,ch,pos`     | `pos` in ¼µs (tool shows µs, ×4 on save) |
| Send all servos home        | `goHome`               | |
| Stop the script             | `stopScript`           | |
| Run a script                | `restartScript,sub`    | `sub` = subroutine # (0–127) |
| Set a speed limit           | `setSpeed,ch,value`    | value 0–16383, 0 = unlimited |
| Set an acceleration limit   | `setAccel,ch,value`    | value 0–255, 0 = unlimited |

Firmware maps these to the real Pololu Maestro serial commands
(`executeMaestroCmd()` / `maestroWrite()` in `NaviCore.ino`): 0x84 Set Target,
0x87 Set Speed, 0x89 Set Acceleration, 0xA2 Go Home, 0xA4 Stop Script,
0xA7 Restart Script at Subroutine.

---

## Speed & Acceleration limits

Both are **limits** applied to a channel — they don't move a servo themselves,
they shape the motion of the next **Set servo position** (or a script's moves on
that channel):

- **Speed limit** — caps how fast the servo slews. `0` = unlimited (snaps as fast
  as it physically can); higher = faster. Units are quarter-µs per 10 ms, so:

  ```
  value × 0.25 µs ÷ 10 ms  =  µs / second
  ```

  | value | speed      | full 1000→2000 µs swing |
  |-------|------------|--------------------------|
  | 10    | 250 µs/s   | ~4.0 s (very slow)       |
  | 30    | 750 µs/s   | ~1.3 s (graceful)        |
  | 100   | 2500 µs/s  | ~0.4 s (brisk)           |
  | 0     | unlimited  | instant snap             |

- **Acceleration limit** — caps how quickly it ramps up to / down from that speed
  (the ease-in / ease-out). `0`–`255`; `0` = no ramp (instant), lower = gentler.
  ~5–20 gives a nice ease.

### They are STICKY
Once set on a channel, a speed/accel limit **stays until you change it** — it
affects *every* subsequent move on that channel, including a joystick passthrough
driving the same servo. To clear a limit, set it back to `0`
(Set a speed limit → 0, Set an acceleration limit → 0).

### How action delays schedule (important)
Every action in a tap tier is dispatched at the **moment the trigger fires**
(`for each action: rcExecuteAction()` in `NaviCore.ino`). Each action's
**Delay(ms)** field schedules it for `trigger_time + delay` — so **delays are
measured from the button press and run in PARALLEL, not cumulatively**. Two
actions with Delay `3000` both fire 3 s after the press.

### Two patterns

**A — Set-before (recommended).** Have every script/move button set the speed/accel
it wants *up front*, so it never inherits the previous button's settings and no
timing is involved:

- *Slow* button: Set speed → 20, Set accel → 5, Set servo position (or Run a script).
- *Normal* button: Set speed → 0, Set accel → 0, Run a script.  ← explicitly resets.

**B — Reset-after (only if you specifically need it).** Set limits, run the move/
script, then a *delayed* reset. Because NaviCore can't detect when a script ends
(see below), the reset is a fixed timer — pad it a bit longer than the script:

| # | Command                  | Value   | Delay(ms)          |
|---|--------------------------|---------|--------------------|
| 1 | Set a speed limit        | ch → 20 | 0                  |
| 2 | Set an acceleration limit| ch → 5  | 0                  |
| 3 | Run a script             | sub N   | 0                  |
| 4 | Set a speed limit        | ch → 0  | 3200 (≈ script len)|
| 5 | Set an acceleration limit| ch → 0  | 3200               |

> Caveat: it's a fixed timer with no feedback. If the script runs longer than the
> delay, the reset fires mid-move and speeds it up. Pad the delay.

---

## Detecting when a script / move finishes — NOT implemented

**Status: the 2-way read path ships; completion *inference* is deferred (2026-07-02).**
The set-before pattern covers the practical need, so nothing polls for "the script is
done". Captured here for future reference.

### Current state — the reads work, the inference doesn't
`maestroLocalQuery()` in `NaviCore.ino` is the **one** Maestro read path, and it covers
both slot types:

- **Local slot (`Serial2`).** Drains stale RX, sends exactly one Pololu query, then reads
  exactly the expected reply bytes under a **25 ms deadline** — a *blocking* read, run on
  Core 1 from `execCliLine` where Serial2 I/O is race-free. The deadline is deliberate: it
  bounds the stall so a missing Maestro or an unwired RX line can't starve SBUS.
- **Remote (broadcast) slot.** Sends the shared `;M<dev>,<verb>` **text** verb over the mesh
  (not raw Pololu bytes); the hosting WCB reads its Maestro and unicasts
  `:MQR,<id>,<chan>,<KIND>,<value>` home. `maeConsumeRemoteReply()` parses it on Core 0
  (parse + store only, never I/O) and `maePumpRemoteEmits()` surfaces it from `loop()`.

Either path lands as a `[MAE:<slot>]{…}` marker (`maestroReportQuery()`) that the config
tool's Maestro panel reads back, and is reachable from the CLI as
`?MAE,GET / MOVING / ERR`. `maestroSequenceBusy()` uses the same 0x93 read as the
skip-if-running gate.

**The hardware half is still on you (local slots only):** the Maestro's serial-**out** (TX)
must be jumpered to NaviCore **GPIO7** (`MAESTRO_RX_PIN`, begun live as `Serial2` RX). No
jumper, no reply — the query just times out. Remote slots need no NaviCore jumper; the
hosting WCB does the reading.

### The catch: no "is the script done?" command
The Maestro's **serial** protocol does not expose script run-state (that's only
visible over USB to Control Center). The read commands available are:

- **Get Position** (0x90) — a channel's current pulse width (¼µs).
- **Get Moving State** (0x93) — 1 byte: are any servos still slewing?
  **Mini Maestro 12/18/24 only — not the Micro 6.**
- **Get Errors** (0xA1) — error flags.

So completion would be inferred one of two ways:

1. **Get Moving State** — poll until it returns "stopped." Works when the motion is
   smoothed by speed/accel (servos visibly slewing); useless for instant jumps
   (reads "stopped" immediately). Mini Maestro only.
2. **Sentinel channel (robust, any Maestro incl. Micro 6).** Add a line at the end
   of the Maestro script that parks an *unused* channel at a marker value
   (e.g. `9000 11 servo`). NaviCore polls **Get Position** on channel 11; reading
   the marker = script finished. Unambiguous.

### What it would take
The transport is done; what's missing is the layer above it — a poller that repeats the
read on a timer until the sentinel channel reads its marker (or Get Moving State reads
"stopped"), then releases the follow-up actions.

- **Hardware:** the Maestro TX → NaviCore GPIO7 jumper above, on any local slot you want
  to poll.
- **Firmware:** drive it from `loop()` through `maestroLocalQuery()` / the `:MQR` cache.
  **Do not add a second `Serial2` reader** — it would race `maestroLocalQuery()`'s
  stale-RX drain and corrupt the existing `?MAE,GET` and skip-if-running replies. Keep any
  new read on the same bounded-blocking pattern; the 25 ms deadline is what protects SBUS.

### Limits
- **Remote Maestros answer asynchronously.** A *Remote (broadcast)* slot does reply — the
  hosting WCB relays `:MQR` — but a mesh round-trip later, so a poller can't read it
  inline the way a local slot allows. `maestroSequenceBusy()` handles that by caching the
  last reply and **failing open** once it's older than `maeGateMs` (default 250 ms).
- **Get Moving State is a proxy, not a "script done" flag** — it misses a paused or
  instant-move script, which is why the sentinel channel is the robust option.
- **Daisy-chains** need the read-back topology wired correctly (each device
  addressed by number).

### Bonus if ever implemented
Reading **actual servo positions** and **error flags** already works (`?MAE,GET` /
`?MAE,ERR`), but nothing consumes it beyond the config tool's readout and the busy gate.
Feeding it into **record/replay** would make that more accurate — it currently records a
"shadow" of last-*commanded* positions, not measured ones. Cleanest first version of
completion detection: **sentinel channel + Get Position poll**.

---

## Revision log

Newest first. Add a row whenever a code change alters what this page describes — same commit
as the code. Page body stays present-tense; history lives here.

| Date | Commit | Change |
|---|---|---|
| 2026-08-18 | _(uncommitted)_ | Corrected the "Current state" section: it claimed the local Maestro bus is write-only and that a Remote slot has no return path. Both reads are implemented — `maestroLocalQuery()` does a bounded 25 ms blocking read off Serial2 for Get Position / Get Moving State / Get Errors, and a Remote slot gets an asynchronous `:MQR` reply relayed home over the mesh. |
