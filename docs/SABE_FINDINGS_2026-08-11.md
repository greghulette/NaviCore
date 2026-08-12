# Findings from reviewing Sabé — 2026-08-11

**What this is.** A read-only pass over Todd Word's Sabé firmware (`Sabe-main`, ~9.2 kLOC
Arduino/C++), which is another `WCB_Client` consumer on the same mesh, looking for things
NaviCore should adopt or worry about. Everything below was checked against the actual
sources in `NaviCore`, `WCBClient`, and `Wireless_Communication_Board-WCB` — not taken from
Sabé's own notes. Where Sabé's docs and our code disagreed, our code won and the
disagreement is recorded.

**Status.** The review itself was read-only. Acting on it since, §1 shipped as option 1
(authoring-time warning + docs), §2.1/2.2/2.5 shipped, and both §3 corrections are fixed.
§2.3, §2.4 and §2.6 are still open — each carries a cost estimate and a "why it matters
here" so the call can be made without re-reading Sabé. Adopted items are marked inline and
describe what NaviCore does **now**.

Sabé is device 20, the out-of-band special/controller slot. The WCB firmware's controller
auto-adopt vocabulary is `{ "NaviCore", "Sab\xC3\xA9", "Sabe" }` (`WCB_WDP.cpp` ≈391,
`wdpControllerType()`), so from the fleet's point of view Sabé and NaviCore are the same
*kind* of citizen and are subject to the same routing rules. That is why its findings
transfer.

---

## 1. The one that matters: the one-hop cap vs `^`-chains

**The claim as first written** — "a mesh-arrived command is executed locally and is *never*
re-forwarded, so a `^`-chain unicast runs only that board's local parts and every part
hosted elsewhere is silently dropped" — **is too broad.** The cap is real but narrower, and
the difference decides whether a given chain works.

**Capped — the receiving board would have to *decide* where the device lives:**

| Site | Symbol | Verbs | Effect when `lastReceivedViaESPNOW` is set |
|---|---|---|---|
| `WCB.ino` ≈5382 (called ≈5413/5416/5419) | `routeStoredOrCap()` | `;A` MP3, `;D` DFPlayer, `;H` HCR | runs local, never forwarded |
| `WCB_Maestro.cpp` ≈143 | `sendMaestroCommand()` | `;M` | remote Maestro forward skipped |
| `WCB_WLED.cpp` ≈151 | WLED runtime dispatch | `;L` | remote proxy forward skipped |
| `WCB.ino` ≈5672 | `recallStoredCommand()` | `;C` / `;SEQ` | does not fan out |
| `WCB.ino` ≈2145 | `sendESPNowMessage()` | any | re-**broadcast** suppressed (`target == 0` only) |

**Not capped — explicit addressing:** `processCommandCharcter()` dispatches `;w` with no
gate at all (≈5404). A `;w` naming the receiving board runs locally (≈5586,
`enqueueCommand(payload, 0)`); a `;w` naming another board re-forwards by **unicast**
(≈5604), which `sendESPNowMessage()` permits because its cap covers broadcast only. `;s<n>`
(local serial write) and `;P` (local PWM) never route in the first place.

So the real consequence for a controller: **a unicast `^`-chain loses a part only when that
part is an implicitly-routed verb whose device is hosted on a board other than the target.**
`;w3;s4:PP100^;w3;s4:PL5` is delivered correctly whichever board it is aimed at.
`;M316^<CA1022>` aimed at WCB1 loses the Maestro trigger when that Maestro lives on WCB3 —
which is exactly Sabé's case, and why the original claim looked general from one data point.

The comments call it "the one-hop cap — no loops, no duplicate fires." It is deliberate and
correct anti-storm behaviour, and the loop it prevents is the *broadcast* one.

**Sabé found this the hard way on their first ground day (2026-08-09):** `;M316^<CA1022>`
unicast to WCB1 played the sound and silently dropped the WCB3 Maestro trigger. Their fix is
to split the chain at the source and route each part independently —
`Mesh_sendCommand()` / `routePart()` in `SabeMesh.cpp` ≈139-246.

### Where NaviCore is exposed

`RA_WCB_UNICAST` sends the action's command string verbatim to one board
(`NaviCore.ino` ≈1729, `rcExecuteActionNow()`). The command is free text authored in the
config tool, and the firmware neither validates nor splits it — the config tool warns at
authoring time instead (see the options below).

So an action authored as `;M316^;L2,ON` targeted at WCB1 loses the `;L2,ON` **if WLED 2 is
wired to a board other than WCB1** — and the symptom is "half my action works", which reads
as a WLED problem, not a routing one. If both devices are on WCB1 the chain is correct.

`RA_WCB_BROADCAST` (`NaviCore.ino` ≈1737) is **not** exposed the same way: every board
receives the broadcast and runs its own local parts, so a mixed chain resolves correctly.
The hazard is unicast-only.

`executeWledAction()` (`NaviCore.ino` ≈1682) forwards `a.cmd` to `w.remoteWCB`. That's a
single `;L` verb by construction, so it is fine today, but it inherits the same rule if
chains ever reach it.

### The open question, answered

**Nothing authors `^`-chains today.** The strongest evidence is the field data, not the
catalog: five real config exports spanning 2026-07-27 → 2026-08-12 contain **zero `^`
characters** anywhere in the file.

The catalog agrees. Checked across every source the tool can load — the 21 DroidNet board
files, `cmdlib/navicore/navicore.json`, and the private
`Leia_Projector/navicore-command-library/leia.json` — no template emits a chain and no
encoder joins parts with `^`. The tool's own 67 `^` occurrences are all regex anchors or
negated character classes.

Two qualifiers on that sweep:

- **`;T` is chain-shaped by design.** `wcb.timer` is `;T{ms},{command}` with a free-text
  `command` param, and `;T` is not a container — `command_timer.cpp` ≈97 splits the whole
  line on `^` *first*, then treats `;T` tokens as group boundaries. "Do A, then 500 ms later
  do B" is `;A,PLAY,1^;T500,;M3,goHome`. So the library can produce a chain; it just has no
  template that does so on its own.
- **The one `^` in the catalog is inert, and backwards.** `wcb-native.json` ≈880 puts
  `[^^]+` on the `?SEQ,SAVE` value. `pattern` is never read anywhere in the tool
  (`_ncCommandPattern()` builds its decode regex from `width`/`enum`/`type` only), so it
  enforces nothing — and the firmware deliberately *preserves* `^` inside a `?SEQ,SAVE`
  value (`WCB.ino` ≈1914-1946: bare `^` and `^;` belong to the sequence, only `^?` ends it).
  A catalog bug with no current effect.

Neither changes the conclusion. A chained Via-WCB unicast can only come from a human typing
free text into the command
box. That is what option 1 covers, and it is what shipped.

### Options, cheapest first

1. **Config-tool validation only** — warn at authoring time, where the user can actually fix
   it. No firmware change, no wire change. **Shipped** — `refreshChainWarn()` in
   `_appendCommandView()`, re-evaluated on both the command text and the "Send to"
   destination. It fires only when a part starts with an implicitly-routed verb
   (`IMPLICIT_ROUTED` = `;A` `;D` `;H` `;M` `;L` `;C`/`;SEQ`), so an explicit-routing chain
   like `;w3;s4:PP100^;w3;s4:PL5` stays quiet.
   See [CONFIG_TOOL.md §9](CONFIG_TOOL.md#9-two-extras-worth-knowing).
2. **Firmware warning** — `dlog(DBG_WCB, …)` when a `RA_WCB_UNICAST` command contains `^`.
   Cheap, but only visible to someone already watching the console. *Not adopted* — the
   authoring-time warning reaches the user who can fix it; this one only reaches someone
   already debugging.
3. **Split and route, Sabé-style** — split on `^` and route each part. *Deferred, and the
   case for it is weaker than it first looked.* Splitting at the source would mean
   duplicating the fleet's routing table in the controller to decide where each part goes —
   and the mesh already has a correct answer for that: address the part explicitly with
   `;w<n>`, which is not capped. The cap itself is right and should not be worked around; it
   is what stops a re-broadcast loop and duplicate fires. Only worth revisiting if chained
   Via-WCB actions become common, which the evidence says they are not.

**A note on the cap itself.** Nothing above is a criticism of the one-hop rule. It is the
correct design — without it a mesh-arrived broadcast would be re-broadcast by every board
that heard it, and every capability-routed verb would fire once per hop. The cost is that
*implicit* routing is one-hop-only, and the fix for that is explicit `;w<n>` addressing, not
a weaker cap.

---

## 2. Adoption candidates

Ranked by value per line of code. All are small.

| # | Gap | Sabé's version | Est. | Status |
|---|---|---|---|---|
| 2.1 | Board ONLINE/OFFLINE logging | `onMeshStatusChange()`, `SabeMesh.cpp` ≈352 | ~10 lines | **shipped** |
| 2.2 | Boot roll call for boards never heard | `Mesh_update()`, `SabeMesh.cpp` ≈92-104 | ~15 lines | **shipped** |
| 2.3 | No `setChecksum()` escape hatch | config field + `SabeMesh.cpp` ≈55 | ~15 lines + config field | open |
| 2.4 | No inbound mesh rate limit | `SabeMesh.cpp` ≈317-347 | ~12 lines | open |
| 2.5 | Loud banner when mesh credentials are unset | `Mesh_begin()`, `SabeMesh.cpp` ≈37-44 | ~8 lines | **shipped** |
| 2.6 | No host-side tests | `test/run-tests.sh` + `test/shims/` | half a day | open |

### 2.1 — Status callback — shipped

`onWcbStatus()` prints one line per transition: `[WCB] WCB3 · dome ONLINE`. Registered with
`wcb->onStatusChange()` right after `onNeighbor`.

The trap worth knowing: **this callback fires on two different cores.** The ONLINE edge
comes from the ESP-NOW receive callback (`WCB_Client.cpp` ≈2063, first heartbeat after
silence); the OFFLINE edge comes from `_checkOfflineBoards()` under `wcb->update()`
(≈1938), which runs on Core 1. So the body has to obey the Core-0 rules either way — it
does one `printf` and nothing else, and both edges are gated by the ETM miss count so it
cannot flood the WiFi task. The board name comes from `rcTelemetry::wcbAlias()`, which
writes its terminator first and is therefore safe to read from Core 1 mid-rewrite;
`wcb->getNeighbor()->name` carries no such guarantee.

### 2.2 — Boot roll call — shipped

`checkBootRollCall()`, one shot at `ROLL_CALL_MS` (30 s) after join, armed in `setup()` and
polled from `loop()`. For every board in `1..quantity` except ourselves, `isOnline(id)`
false prints a named line, then a `n/total board(s) online` summary.

Kept as a **roll call, not an alarm** — Sabé's framing, and right: "WCB3 absent" is worth
one line whether the answer is "the dome is on the bench" or "check its power". Distinct
from 2.1: a board that comes up and later drops gets a transition line; this catches the one
that was never there at all, which otherwise leaves no evidence on this console.

### 2.3 — Checksum has no escape hatch

**We never call `setChecksum()` anywhere** — zero hits across `NaviCore.ino`, the headers,
and `config_tool/`. That is *correct today*: the library defaults to `true`
(`WCB_Client.h` ≈784) and the firmware defaults to `true` (`WCB.ino` ≈257,
`etmChecksumEnabled`).

The exposure is that the Wizard can write `?ETM,CHKSM,OFF` fleet-wide. If a builder does
that, NaviCore goes **100 % silently deaf** with no setting that can fix it short of a
firmware rebuild. Sabé carries it as a config field (`meshChecksumOn`).

Also worth knowing regardless: checksum on drops the usable payload from 200 to ~188 chars.
Our own docs use 187 as the cap, which is the conservative number and stays correct.

### 2.4 — Inbound rate limit with a stop-exempt carve-out

We have no inbound throttle, and `onWCBCommand` fans into five queues
(`remoteTriggerQueue`, `remoteCliQueue`, `forgetPeerQueue`, `serialFwdQueue`,
`maestroCmdQueue`, created `NaviCore.ino` ≈3741-3745). Sabé enforces a 50 ms floor
(~20/s) on inbound commands, with an explicit exemption so `&ESTOP` / `&DISABLE` always land.

The design point worth stealing even if we skip the throttle: **stopping is never gated on
the things that gate starting** — not by rate limits, not by arming state, not by config
locks. Sabé applies that carve-out identically on the console, the web bridge, the physical
buttons, and the mesh.

Whether this matters for us depends on whether any of those five queues can be driven into
a self-amplifying loop. Worth a look during the same session.

### 2.5 — Loud credential banner — shipped

Worse than the review first read it. The orange-LED fault path
(`NaviCore.ino`, `g_ledFaultColor`) only fires when `wcb->begin()` **fails** — and an empty
password does not fail `begin()`. `begin()` range-checks `device_id` and brings up
WiFi/ESP-NOW; it never looks at the password (`WCB_Client.cpp` ≈51-100). The password is a
plain-text namespace field on every packet (`wcb_packet_etm_t::structPassword`) that the
receive path `strncmp`s before anything else (≈2036).

So an empty password meant `begin()` returned true, no fault latched, the LED stayed
normal, and the board was **completely deaf and mute with no indication at all**. There is
also no console command to set it — the only writer is the config tool's WCB Network panel
(via `SET_CONFIG`), so the banner names that path rather than a `#` command. It prints
before `begin()`, boxed, and says both what is broken and where to fix it.

### 2.6 — Host-side tests

The most interesting structural thing in Sabé. `test/run-tests.sh` compiles the **real,
untouched** firmware `.cpp` files against fake Arduino headers in `test/shims/`
(`Arduino.h`, `Preferences.h`, `esp_task_wdt.h`, `freertos/`) with
`-fsanitize=address,undefined`, and runs assertions on a laptop in ~2 s. Five suites: safety
core, control/failsafe ladder, button edges, config import, config migration.

NaviCore has no equivalent. The natural candidates are all pure logic with no hardware
dependency:

- `rc_config.h` config migration (same append-only discipline we already follow)
- the `;L` id/verb parser in `executeWledAction()`
- `mp3FormatCommand()` / the `;D` DFPlayer verb builder — both are single-producer functions
  whose bounds must match `WcbCmd`'s, which is exactly the kind of pairing that drifts
- the mesh payload length check against the 187-byte cap

This is the item with the highest ceiling and the highest cost. It is also the only one that
would keep paying after it's written.

---

## 3. Corrections to carry forward

Two places where checking the source changed the answer. Both are worth knowing in NaviCore
regardless of what we adopt.

**3.1 — The ensured-send table degrades better than documented.** Sabé's audit says the 10
in-flight ensured slots (`WCB_PENDING_MAX`, `WCB_Client.h` ≈83) evict oldest-first. They
don't. `_findFreePending()` (`WCB_Client.cpp` ≈1971) reclaims **only** a best-effort slot or
an ensured slot that has already completed; a still-outstanding ensured delivery is never
dropped. If every slot is outstanding, `_sendPacket()` transmits the command best-effort once
and **returns `false`** (`WCB_Client.cpp` ≈1840-1847) so the caller knows it did not get
ensured semantics.

So the real contract is: *ensured sends never silently lose a guaranteed command; they
degrade to best-effort and tell you.* Anywhere we ignore `send()`'s return value on a burst,
that `false` is the signal we're discarding — and `rcExecuteActionNow()` ignores it on both
`RA_WCB_UNICAST` and `RA_WCB_BROADCAST`. That is acceptable for animation traffic (a lost
pose or sound is recoverable, and the next trigger supersedes it), but it is the wrong
default for anything that must land exactly once.
**Recorded** in [PROTOCOLS.md §1](PROTOCOLS.md#1-transport-overview).

**3.2 — `?ETM,CHKSM` default was documented backwards in our own wiki — fixed.** `ETM.md`
≈223 read `?ETM,CHKSM,OFF // Disable checksum (default)`. Both the firmware
(`WCB.ino` ≈257, and `WCB_Storage.cpp` ≈2142 restores it with a `true` default) and the
client library (`WCB_Client.h` ≈784) default to **ON**. Since a CHKSM mismatch is a silent
total blackout in both directions, that was the single most expensive line in the wiki to
have wrong. (`ETM.md` ≈50 and ≈226 both correctly warned that the boards must match — only
the `(default)` annotation was wrong.)

A sweep for the same error found a **second** instance the original review missed:
`Configuration-Guide.md` ≈538, identical `(default)` on the OFF line, plus a
"checksum verification (optional)" heading. Both are corrected, along with `ETM.md`'s
"Step 5: Optionally enable checksum" and its "When to Enable Checksum" section, which all
implied checksum was opt-in. `Command-Reference.md` ≈345 and `Third-Party-Integration.md`
≈553 were already correct.

---

## 4. Ecosystem items — ours to fix, not NaviCore's

Recorded here so they aren't lost; neither blocks anything.

- ~~**`ETM.md` ≈223 default is wrong**~~ — **fixed**, along with `Configuration-Guide.md`
  ≈538. See §3.2. WCB wiki, branch `master` — needs pushing from that repo.
- **The Wizard's Controller selector offers NaviCore | Kyber | None**
  (`Wizard/index.html` ≈108-118) though the firmware auto-adopts on hearing `Sabé` too
  (`WCB_WDP.cpp` ≈391). A Sabé build configured through the Wizard has to pick "NaviCore" to
  land on the right ruleset. Cosmetic, but it is a real builder-facing gap now that Sabé is
  running in a droid.
- Sabé's own audit notes that `&` is a legal alternate chain delimiter (`?DELIM,&`) and their
  `&SABE,…` telemetry prefix collides on any board so configured. Not our problem, but if we
  ever prefix telemetry, pick a character that isn't a configurable delimiter.

---

## 5. Deliberately not adopting

Things Sabé does that don't transfer, so the next session doesn't re-evaluate them:

- **WDP by polling rather than `onNeighbor()`** (`SabeWdp.cpp`). Sabé rebuilds its neighbour
  table once a second from `getNeighbor()` and lets the library own liveness. We already use
  `onNeighbor()` with a deferred queue and a boot grace window (`NaviCore.ino` ≈3775-3777),
  which is strictly better — event-driven, and the grace window suppresses the boot-time
  discovery storm. Their own header notes their copy can tear mid-row; ours can't.
- **Their `;W` parser.** `routePart()` (`SabeMesh.cpp` ≈160-165) accepts `;W<1-9><rest>`
  only — single digit, no comma. `;W20,<cmd>` (their *own* address) mis-parses as WCB2 with
  payload `0,<cmd>`, and the canonical comma form `;W2,;S4…` leaves a stray leading comma.
  Known to them, dormant on Todd's droid. Do not copy this parser; if we ever need `;W`
  emission, accept 1-2 digits and an optional comma.
- **Their `;M<digit> → WCB<digit>` mapping.** A topology assumption, not a protocol rule —
  Maestro IDs are decoupled from board numbers and resolved by the fleet's `?MAESTRO` routing
  table. We already have the right answer via WDP `maestroIds[]`.
- **Everything safety-state.** The e-stop latch, the 250 ms stale-input hold, the 300 ms
  watchdog, the arming chord, the no-flash-writes-while-armed rule. NaviCore is an animation
  controller and does not drive a 200 lb machine. Not applicable, and adopting the vocabulary
  without the mechanism would be worse than not having it.

---

## 6. Two things worth reading for their own sake

Not adoption candidates — just good, and both are ~30 lines.

- **`SabeVictron.cpp` ≈49-64 — repairing a lossy serial copy from the mesh copy.** A bit flip
  at 9600 baud turned `"soc":94.3` into `14.3` and the droid reported a critical pack. The
  ESP-NOW copy of the same broadcast was intact, so the RX callback deposits it in a
  one-slot mailbox and `loop()` parses whichever is good. Plus a plausibility filter: a SoC
  jump over 10 % is held as a candidate and only accepted if the *next* reply agrees, on the
  reasoning that a real step confirms itself one poll later and a bit flip never repeats
  identically.
- **`SabeDroidNet.cpp` ≈43-53 — `sanitize()`.** Because a WCB fans every mesh broadcast out
  all of its serial ports, echoing an unrecognised command back verbatim turns
  "I didn't understand `<SH1>`" into an actual `<SH1>` arriving at the vocalizer. Their NAK
  path strips `< > ; ^` for exactly that reason. Any NaviCore path that echoes untrusted
  text onto the mesh has the same problem.

---

## 7. What was not checked

- Sabé's C6 co-processor, web UI, bridge protocol, XBee parsing, and button/action layers —
  out of scope, nothing mesh-facing.
- Nothing was compiled. No files in any repo were modified by this review.
- Sabé pins `WCB_Client` **1.9.10**; our working copy is **1.12.0**. Behavioural claims about
  the library above are checked against 1.12.0, which is what NaviCore builds against.

---

## Revision log

Newest first. Add a row whenever a code change alters what this page describes — same commit
as the code. Page body stays present-tense; history lives here.

| Date | Commit | Change |
|---|---|---|
| 2026-08-12 | _(uncommitted)_ | **Corrected §1.** The one-hop cap is narrower than first written: it gates *implicit* routing only (`;A`/`;D`/`;H` via `routeStoredOrCap`, `;M`, `;L`, `;C`/`;SEQ`, and any re-broadcast). Explicit `;w<n>` is **not** capped — self-target runs local (`WCB.ino` ≈5586), remote re-forwards by unicast (≈5604), and `sendESPNowMessage` caps `target == 0` only (≈2145). So `;w3;s4:PP100^;w3;s4:PL5` is delivered correctly. Narrowed the config-tool warning to the implicitly-routed verbs, and downgraded option 3 — the cap is correct and `;w` is the supported answer. Also recorded the `;T` chain-shape qualifier and that catalog `pattern` fields are inert. |
| 2026-08-12 | _(uncommitted)_ | §1 open question answered (no catalog emits `^`-chains) and option 1 shipped; §2.1/2.2/2.5 adopted; §3.1 recorded in PROTOCOLS.md; §3.2 fixed in the WCB wiki, incl. a second instance in `Configuration-Guide.md`. |
| 2026-08-11 | _(uncommitted)_ | Initial version. Read-only review of Sabé `Sabe-main` against NaviCore, WCBClient 1.12.0, and WCB firmware 6.2.0. |
