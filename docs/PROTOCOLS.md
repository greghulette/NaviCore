# NaviCore — Wire Protocols

Every contract between the firmware and something outside it. If you change anything on
this page, you are changing an interface with a counterpart that must change with it —
check the **Counterpart** column before editing.

Related: [ARCHITECTURE.md](ARCHITECTURE.md) · [CONFIG_SCHEMA.md](CONFIG_SCHEMA.md) ·
[CONFIG_TOOL.md](CONFIG_TOOL.md)

---

## 1. Transport overview

| Path | Physical | Framing | Size limit |
|---|---|---|---|
| **Direct USB** | USB-CDC 115200 | newline-delimited UTF-8 | none (chunked at 512 B by the tool, 4 KB RX buffer on the board) |
| **Via WCB** | Web Serial → bridge WCB → ESP-NOW → NaviCore | `;w20,<payload>` lines to the bridge | **187 bytes per payload** — larger must be fragmented |
| **Shared port** | Same as either, but the port is owned by another browser tab | BroadcastChannel proxy | as above |
| **WebSocket** | client → NaviCore's own SoftAP → `ws://<softAPIP>/ws` | one text frame per line | 98 KB per frame — the dispatcher's own ceiling, not a transport one |

**The WebSocket path is optional and off by default** (`wifiEnabled`, see
[CONFIG_SCHEMA.md](CONFIG_SCHEMA.md)). It carries the **same newline-delimited JSON** as
Direct USB, because it feeds the same `processInputLine()`; there is no second command
surface to keep in step. Unlike Via WCB it has no 187-byte cap and no fragmentation — it is
a direct link, so a large `SET_CONFIG` crosses in one frame.

**The 187-byte number.** ESP-NOW carries 250 B. `WCB_Client::_sendPacket()` copies the
command into a 200-byte `structCommand[]` and appends `"|CRC%08X"` (12 B). A payload over
187 B truncates the CRC and the receiver silently drops the packet as "Missing CRC". Both
sides hard-code this: `rcTelemetry::MAX_ENV_BYTES` and the tool's `FRAG_MAX_ENV_BYTES`.

**Ensured delivery degrades, it does not evict.** `wcb->send()` defaults to
`ensured = true`, so every NaviCore send is retransmitted until the target ACKs
(`ETM_MAX_RETRIES` = 3). Only `WCB_PENDING_MAX` = 10 of those can be in flight at once.
When the table saturates, `_findFreePending()` reclaims **only** a best-effort slot or an
ensured slot that has already completed — a still-outstanding ensured delivery is never
dropped. If every slot is outstanding, `_sendPacket()` transmits the new command
best-effort once and **returns `false`**.

So the contract is: *an ensured send never silently loses a guaranteed command; it
degrades to best-effort and tells you.* That `false` is the only signal that a command
went out unguaranteed — a caller that discards `send()`'s return value on a burst is
discarding it. `rcExecuteActionNow()` does exactly that on both `RA_WCB_UNICAST` and
`RA_WCB_BROADCAST`, which is acceptable for animation traffic (a lost pose or sound is
recoverable and the next trigger supersedes it) but is the wrong default for anything
that must land exactly once.

**The one-hop cap applies to implicit routing only.** A mesh-arrived command is not
re-routed onward — but only where the receiving board has to *decide* which board hosts
the device. Those paths, and only those, gate on `lastReceivedViaESPNOW`:

| Verb | Gate |
|---|---|
| `;A` MP3 · `;D` DFPlayer · `;H` HCR | `routeStoredOrCap()` — `WCB.ino` ≈5382, called ≈5413/5416/5419 |
| `;M` Maestro | `sendMaestroCommand()` — `WCB_Maestro.cpp` ≈143 |
| `;L` WLED | remote proxy — `WCB_WLED.cpp` ≈151 |
| `;C` / `;SEQ` | `recallStoredCommand()` — `WCB.ino` ≈5672 |
| any re-**broadcast** | `sendESPNowMessage()` — `WCB.ino` ≈2145, gated on `target == 0` |

**Explicit `;w<n>` routing is *not* capped.** `processCommandCharcter()` dispatches `;w`
with no gate (≈5404); a `;w` aimed at the receiving board runs locally (≈5586) and one
aimed elsewhere re-forwards by unicast (≈5604), which `sendESPNowMessage()` allows because
its cap covers broadcast only. `;s<n>` (local serial write) and `;P` (local PWM) never
route at all.

So in a `^`-chain sent **unicast**, a part is dropped only when it is an implicitly-routed
verb *and* its device lives on a board other than the target. `;w3;s4:PP100^;w3;s4:PL5`
is delivered correctly whichever board you aim it at; `;M316^<CA1022>` aimed at WCB1 loses
the Maestro trigger if that Maestro is hosted on WCB3. A `^`-chain **broadcast** is always
fine — every board hears it and runs its own parts. NaviCore does not split or validate
chains in firmware; the config tool warns at authoring time, keyed on that verb list.

---

## 2. USB serial JSON protocol

Newline-delimited JSON. Every tool-originated message carries an additive `"sys":1` so the
WCB Wizard can mute the tool's chatter when both share a port.

**Framing and dispatch are separate**, both in [`NaviCore.ino`](../NaviCore.ino):
`handleSerialInput()` reads bytes off `Serial` and assembles lines;
**`processInputLine(const String&)` is the transport-agnostic dispatcher** and is where every
message above is actually handled. Anything else that can produce a complete line calls
`processInputLine()` directly rather than duplicating the dispatch.

`naviws::drain()` is the second caller (the WebSocket endpoint,
[`navicore_wsserver.h`](../navicore_wsserver.h)). Its shape is the constraint worth
understanding before adding a third: **the httpd handler runs on Core 0**, alongside the
ESP-NOW receive callback, so it may only copy the frame, enqueue it, and return — exactly
what `drainRemoteCli()` does for mesh-relayed CLI lines, and for the same reason.
`processInputLine()` writes flash and NVS and drives bit-banged serial; none of that may
happen on Core 0. The reply goes back via `httpd_ws_send_frame_async()`, the API documented
for sends "out of the scope of current request", because by then the request is long over.

Its `bool` return means *"yield to `loop()` before handling more input"* and is not cosmetic:
the `?` CLI branch and a JSON parse failure both need the caller to stop draining so
heartbeats keep running between the host's ACK-paced OTA chunks. A caller that ignores it and
keeps draining reintroduces the stall the early return exists to prevent. The caller also owns
the buffer — the line is passed by `const&` and cleared by the caller, because a `SET_CONFIG`
payload approaches 98 KB and copying it per line is real cost.

### Requests (tool → board)

| Message | Reply | Notes |
|---|---|---|
| `PING` / `{"type":"PING"}` | `{"type":"PONG","version":"<FW_VERSION>"}` | Also clears a stale calibration mute |
| `{"type":"GET_CONFIG"}` | `{"type":"CONFIG","data":{…}}` | Full `rcConfigToJSON()` |
| `{"type":"SET_CONFIG","data":{…}}` | `{"type":"ACK","ok":true}` | Deserialised un-filtered; re-applies bauds, SBUS-out, board profile live |
| `{"type":"GET_CMDLIB"}` | `{"type":"CMDLIB","size":N,"hash":H,"data":{…}}` | Command library stored on the droid, opaque to firmware |
| `{"type":"GET_CMDLIB_META"}` | `{"type":"CMDLIB_META","size":N,"hash":H}` | Cheap change-check so a connect can skip the pull |
| `{"type":"SET_CMDLIB","data":{…}}` | `ACK` | Raw value pulled by substring, stored verbatim |
| `{"type":"START_MONITOR"}` | streams `PWM_UPDATE` every 50 ms | |
| `{"type":"STOP_MONITOR"}` | `ACK` | Also force-clears calibration mute |
| `{"type":"CALIB","on":bool}` | `ACK` | Mutes **all** action dispatch while on |
| `{"type":"RESET_DEFAULTS"}` | `ACK` | Reloads factory defaults |
| `{"type":"TEST_ACTION","action":{…}}` | `{"type":"ACK","of":"TEST_ACTION","ok":bool}` | Fires one action without saving it. `action` is re-parsed from the raw line (the header filter strips nested objects) |
| `{"type":"REBOOT"}` | `ACK`, restart after 250 ms | |
| `{"type":"TRIGGER","mode":M,"btn":B,"tap":T}` | — | Virtual button press. `tap` 1–4; **4 = long press** (tier `t4`), which always dispatches exclusively |
| `{"type":"WCB_SEND","target":N,"cmd":"…"}` | — | `target` 0 = broadcast |
| `{"type":"FORGET_PEER","id":N}` / `"all":true` | — | id 0 or `all` = drop every learned peer |
| `{"type":"SET_DEBUG_FLAGS","flags":N}` | — | See the debug bitmask below |
| `{"type":"GET_WCB_STATUS"}` | `{"type":"WCB_STATUS",…}` | See §4 “Bridged status and metadata” |
| `{"type":"GET_WCB_SEQ","wcb":N}` | `{"type":"WCB_SEQ",…}` **async** | One board's stored-sequence key names, pulled off the mesh. See §4 “Stored sequences” |
| `{"type":"GET_WCB_SEQVAL","wcb":N,"key":"K"}` | `{"type":"WCB_SEQVAL",…}` **async** | ONE sequence's contents. Same section |
| `{"type":"GET_MESH_STATS"}` | `{"type":"MESH_STATS",…}` | ESP-NOW delivery counters. See below |
| `{"type":"RESET_MESH_STATS"}` | `{"type":"ACK","of":"RESET_MESH_STATS","ok":true}` | Zero every counter without a reboot. **Deferred to `loop()`** — `resetStats()` takes the pending-table lock and must not run on the Core-0 receive callback. ACK is immediate; re-read after a beat |

`{"type":"ERROR","msg":"JSON parse failed (…)","rxLen":N}` comes back on a malformed
line; `rxLen` lets the host detect USB RX truncation.

> **Header parsing gotcha.** The board parses the message header through an ArduinoJson
> **Filter whitelist** (a full `SET_CONFIG` would exhaust the small header doc). Any new
> top-level field a non-`SET_CONFIG` handler reads **must be added to that filter**, or it
> is silently stripped and reads as its default.

### `MESH_STATS` — ESP-NOW delivery counters

```json
{"type":"MESH_STATS","pg":0,"self":20,"upMs":812340,
 "agg":{"sent":412,"ackd":408,"rty":11,"fail":3,"ung":0,"bcast":96,"recv":530},
 "peers":[[1,140,140,0,0,0,88],[3,131,127,9,3,0,102]],"last":1}
```

Built by `rcTelemetry::buildMeshStatsPage()` for **both** transports, so the two payload
shapes cannot drift. `"sys":1` is present on the bridged reply only (see §4). `"pg"` and
`"last"` drive the paging described below — a Direct USB reply is always a single
`"pg":0` page with `"last":1`.

Peer rows are **flat arrays** and the `agg` keys are **abbreviated** — both to fit the
budget below, not for style: `[id, sent, ackd, rty, fail, ung, recv]`. The tool supplies
every human-readable name, so nothing user-facing is abbreviated.

`sent`/`ackd`/`rty`/`fail`/`ung` and `bcast` come from `WCB_Client`
(`getAggregateStats()` / `getPeerStats()` / `getBroadcastSent()`) and are **outbound only**.
`recv` has no library counter — it is `g_meshRxCount` / `g_meshRxFrom[]`, incremented in
`onWCBCommand()`, so it counts COMMANDs delivered to the application and **not** raw-packet
traffic (OTA, bulk chunks) which never reaches that callback.

`upMs` is reported alongside because these counters are RAM-only and reset on reboot — a
ratio only means something against the uptime that produced it.

`sent - (ackd + fail + ung)` is **in flight**. The library guarantees it is never negative;
a negative value is a library bug (a pending slot settled twice), not a lost packet.

**The aggregate and the per-peer rows are sampled a moment apart, so they can disagree by
one.** `agg` comes from the library's own counters; `peers[]` is walked from the peer table
afterwards. An ACK landing between the two reads leaves the aggregate *behind* the rows it is
supposed to contain — observed in the field as the tool's derived "not currently listed"
remainder showing `Ack -1` against `Sent 0`. **This is not a counter bug and there is nothing
to fix in the firmware.** Do not "correct" it by making one side authoritative or by
recomputing the aggregate from the rows: the aggregate legitimately covers all 20 library peer
slots, including boards the roster no longer lists, which is the whole reason a remainder row
exists. The tool clamps the remainder at zero for display (`renderMeshStats`); a genuine
remainder is still shown, a −1 of skew is not.

#### Paging — how the full roster crosses the bridge

The relay's `WCB_Client` cannot reassemble fragments, so a bridged reply must fit **185 B**.
The aggregate alone is ~140 B, so **no per-board data for a real fleet fits alongside it** —
measured at six boards: ~229 B with full rows, and still ~202 B stripped to `[id, ack%, fail]`.

So the reply is **paged**. `buildMeshStatsPage()` fills rows until the next would exceed the
budget, then reports where to resume; `tick()` sends one page per pass, `FRAG_PACING_MS`
apart. Six boards is three packets, ~300 ms, once per poll:

| Page | Carries | Typical | Large counters |
|---|---|---|---|
| 0 | header + `agg` + `upMs`, `"pg":0` | 164 B | 164 B |
| 1… | lean header + rows only | 166 B | 149 B |
| last | + `"last":1` | 87 B | 158 B |

**Page 0 usually carries no rows at all**, and that is deliberate: the aggregate plus one
row of five-digit counters overruns the frame. Emitting page 0 empty costs one extra packet;
force-writing the row would push the page over budget, `tick()` would refuse to send it, and
the whole cycle would abort — no stats at all.

Direct USB is the degenerate case: one page with a budget larger than any roster, so paging
never triggers and the whole fleet arrives in one line marked `"last":1`.

**The consumer merges by board id and only promotes a set it saw `"last":1` for**, and the
pages must be **contiguous**. Both matter: a lost middle page followed by a later page
carrying `last` would otherwise promote a set missing the dropped boards, and those boards
would render "no traffic" — a wrong answer that looks like a real one. A gap abandons the
cycle, leaving the previous complete snapshot on screen until the next poll rebuilds from
page 0. A duplicate page (an ETM retry landing twice) is ignored rather than double-listed.

This is also why the `agg` keys are short. Spelled out (`retries`/`failed`/`unguaranteed`)
the aggregate is ~146 B, which eats the budget the rows need.

### Outbound stats report (`?STATS,RPT`) — board → one WCB

When `statsReport.enabled`, every 30 s NaviCore unicasts **one** command to `statsReport.wcb`:

```
?STATS,RPT,<from>,<sent>,<ackd>,<retries>,<failed>,<unguaranteed>,<bcast>,<recv>
```

It carries **only this board's own counters**; every other node reports its own the same
way, so the receiving board accumulates a fleet view without anyone computing a roll-up.

**`?` is load-bearing.** `executeCommand()` routes a `?` command to `processLocalCommand()`
and returns (`WCB.ino` ≈3964), so the report is handled locally on the receiving board and
can never fall through to `processBroadcastCommand()` — it is never written to that board's
serial ports and never re-broadcast to the mesh. A `;` verb would need those exclusions
added by hand.

`<from>` is in the payload because `processLocalCommand()` receives no `sourceID`
(`WCB.ino` ≈3965). The receiver stores it in `reportedStats[]` (RAM-only, cleared by
`?STATS,RESET` or a reboot) and lists it under `?STATS` → *Reported by Other Nodes*. A
report with fewer than 8 fields is dropped whole rather than stored partially.

| Counterpart | Where |
|---|---|
| Receiver + storage | `WCB.ino` `storeReportedStats()`, `reportedStats[]` |
| Display | `WCB.ino` `buildStatsString()` |
| Sender | `rcTelemetry::reportMeshStats()` |

### Streams (board → tool)

| Message | Rate | Contents |
|---|---|---|
| `PWM_UPDATE` | 50 ms while monitoring | 24 channels, decoded mode/button, SBUS health |
| `rc_trig` | on every dispatch | `id, mode, btn, tap` — **the same JSON shape `rcDispatch()` broadcasts to the mesh**, printed to USB as well |
| `[CLIPDL:*]` | reply to `?REC,EDITLOAD` | Clip event stream — see **Ranged clip download** below |

### Ranged clip download

```
?REC,EDITLOAD,<name>                     legacy — whole clip, events UNindexed, byte-identical to before
?REC,EDITLOAD,<name>,<from>[,<count>]    ranged — events carry their absolute index
                                           count omitted → to end of clip
?REC,EDITLOAD,<name>,<from>,<count>,B    ranged + BATCHED keyframes (see below)
```

```
[CLIPDL:BEGIN]{"count":N,"durationMs":M,"mode":m,"from":f,"n":k,"fp":"8hex","fc":F,"nm":"<clip>"}
[CLIPDL:EV,<absIdx>]{…}  × k     ranged only
[CLIPDL:EVB,<firstIdx>]{"e":[[t,k,a,b,c],…]}   batched, ",B" only
[CLIPDL:EV]{…}           × k     legacy only
[CLIPDL:END]{"from":f,"n":k,"fp":"8hex","nm":"<clip>"}
```

**The compatibility rule for this whole family:** anything an old tool can trigger keeps its
exact bytes. New shapes appear only in replies to a request an old tool cannot make. `BEGIN`
is the one safe exception — the tool reads only `durationMs`/`mode`/`count` from it, so added
keys are inert. `[CLIPDL:ERR]` in particular must keep its exact prefix and offset, because
the tool matches it with `startsWith` and a fixed `slice`.

**`nm` makes the stream self-identifying, and that is not optional.** The reply carries no
request id, and `_clipRange` in the tool is a single global, so a range that times out while
the board is still sending leaves that stream to be adopted by the NEXT request. `from`/`n`
alone cannot separate two clips asked the same range — and a backup probes **every** clip with
`(0,0)`, so the probe is the most exposed request of all. Without `nm` a stale `END` finishes the
new session before its header arrives, a stale `BEGIN` overwrites its `count`/`fp`, and stale
`[CLIPDL:EV,i]` land under a foreign clip's indices — which produced a **negative** shortfall
("-32 of 16 events") and cascaded through the rest of the backup. `nm` costs ~39 B on two control
lines per range (BEGIN ~95 B → ~134 B, inside the 160 B RTERM cap) and nothing per event.
A tool that does not know the key ignores it; firmware that does not send it reads as
"cannot tell", which is exactly the old behaviour, so the tool must also drain the stream to
silence between requests rather than rely on `nm` alone.

**Batched keyframes (`,B`).** One event per line is ~62 B carrying 8 B of information, and over
the mesh every line costs a whole RTERM packet — so PACKET COUNT, not the board, is what makes a
mesh download slow (`editStream`'s pacing allows ~4000 ev/s). With `,B`, consecutive **keyframes**
pack into one line as bare tuples, indices implicit and consecutive from `<firstIdx>`:

| k | shape | meaning |
|---|---|---|
| 1 | `[t,1,slot,ch,pos]` | `REC_KF_MAESTRO` |
| 2 | `[t,2,chan,vol,0]` | `REC_KF_HCRVOL` (padded to arity 5 so the parser stays trivial) |

Actions are **never** batched — one can reach ~211 B alone — and an action (or a full line) FLUSHES
the run first, because a gap in a positional run would silently shift every index after it. The
body budget is 120 B, keeping tag+wrapper+body inside one 160 B packet, so a batch can never cost
MORE packets than the per-event form. Measured on a realistic 300-event clip: **300 packets →
60 (5.0×), 17,678 B → 6,996 B (2.5×)**, longest line 133 B.

Compatible in both directions with no handshake: the old parser read `count` with `String::toInt()`,
which stops at the comma and never saw the flag, so a new tool gets the per-event form back from old
firmware — and the tool accepts **both** shapes always. An old tool omits `,B` and is unaffected.

Five fields carry the integrity guarantee:

- **`count`** keeps its legacy meaning (total events resident), so an old tool's arithmetic
  still reads correctly.
- **`from`/`n`** say which slice this is; their *presence* is the capability probe.
- **`fp`** is FNV-1a over the resident buffer, repeated on `END`. Ranges taken from two
  different buffer contents must never be spliced, and a count alone cannot catch an edit
  that preserves the event count. Repeating it on `END` also catches a clip that changed
  *during* a stream.
- **`fc`** is the **file header's** count. `loadClip()` truncates silently when
  `h.count > _cap`, so `count` alone would report a short read as a complete clip.
  **`fc != count` means the clip must not be banked into a backup.**

**Index goes in the marker, not the JSON.** A maximal action line (~211 B with a full
`cmd[96]`) already hard-wraps across two RTERM packets at `RTERM_TEXT_SIZE = 160`, so a
marker-borne index lands on the first fragment and is readable before the JSON reassembles.
It also avoids the tool's `const {t, k, ...action} = ev` rest-spread swallowing an `"i"` key
into the `RcAction`.

**Residency.** A ranged download assembles across several requests, and the board sits at
`ST_IDLE` between them — the same state the Record and Play triggers require. So a pilot
touching a transmitter button mid-assembly can replace the shared `_buf` underneath.
`navirec::_loadedName` tracks what is resident and is cleared by **every** buffer mutator, so
staleness fails closed; a range whose clip is no longer resident reloads. Reloading blindly
per range instead would re-read `_count × 140 B` (~420 KB for a 3000-event clip) from
LittleFS each time, stalling `loop()` far longer than the streaming itself.

The relayed-path size refusal now applies only to the **legacy whole-clip** form — a ranged
request is the answer to that problem, so it is allowed at any clip size.

`rc_trig` goes out on **both** transports because `rcTelemetry::emitTrig()` returns early
without a ready WCB, so a Direct-USB tool previously saw nothing at all for a local button
press — no way to distinguish "the tier fired and did nothing visible" from "the tier never
fired". The tool uses it to flash the exact tier row that fired in the assignment cards.
The mesh copy is a broadcast, so the tool filters it by `id`; the USB copy arrives down the
wire from the one board it is talking to, so `id` is not checked there (and must not be —
a board with a non-default `deviceId` would have every event dropped).

Calibration is driven entirely by the tool: it sends `CALIB` on/off to mute dispatch and
reads channel values out of the `PWM_UPDATE` stream. The board emits no calibration message
of its own.

### Debug bitmask (`SET_DEBUG_FLAGS`)

| Bit | Constant | Category |
|---|---|---|
| 0 | `DBG_MAESTRO` | Maestro dispatch |
| 1 | `DBG_WCB` | WCB unicast + broadcast |
| 2 | `DBG_WLED` | `;L<id>` dispatch |
| 3 | `DBG_HCR` | HCR |
| 4 | `DBG_MP3` | MP3 |
| 5 | `DBG_SERIAL` | Aux serial TX/RX |
| 6 | `DBG_DFP` | DFPlayer Mini `;D` dispatch |

Default 0 — every `[DISPATCH]` log is compiled in but costs nothing until enabled. Log
sites use `dlog(BIT, fmt, …)`, which wraps `vlogf()` and drops the line rather than block
when the USB TX buffer is full.

---

## 3. CLI commands

Typed on the USB console **or** relayed from the tool's terminal over the mesh — both run
through `execCliLine()`, so behaviour is identical. Case-insensitive.

### Diagnostics

| Command | Effect |
|---|---|
| `#L01` | Board identity line |
| `#L02` | Restart |
| `#L09` | SBUS state dump |
| `#L10` | Toggle 1 Hz live SBUS dump |
| `#L11` | WCB device id, quantity, per-board up/down (`*` = auto-discovered) |
| `#L12` | Mode + decoded matrix button + raw value |
| `#L13` | Raw SBUS frame hex with byte-offset annotations |
| `#L20` / `#L21` | Send a fixed HCR `SetEmotion(HAPPY,80)` straight to S3 / S4 — bypasses config **and** mapping, so it separates wiring faults from config faults |

### Maestro (`?MAE,…`)

| Form | Meaning |
|---|---|
| `?MAE,<slot>,<ch>,<pos>` | Set target (¼ µs), slot 1–8, ch 0–31 |
| `?MAE,FREE,<slot>,<ch>` | speed = 0, accel = 0 (used by the timeline editor's live scrub) |
| `?MAE,GET,<slot>,<ch>` | Get Position → `[MAE:<slot>]{"q":"pos",…}` |
| `?MAE,MOVING,<slot>` | Get Moving State → `{"q":"mov",…}` |
| `?MAE,ERR,<slot>` | Get Errors → `{"q":"err",…}` (**clears errors on read**) |

Local slots reply synchronously off Serial2; remote slots broadcast the query and the
hosting WCB relays a `:MQR` reply asynchronously, surfaced by `maePumpRemoteEmits()`.

### Record / replay (`?REC,…`)

`START` · `STOP` · `PLAY[,name]` · `SAVE[,name]` · `LOAD,name` · `LS` · `RM,name` ·
`RENAME,from,to` · `CLEAR` · `INFO` (bare `?REC` = INFO).

Timeline-editor transport: `EDITLOAD,<name>` · `EDITBEGIN` · `EDITEV,<idx>,<json>` ·
`EDITEND,<name>` · `EDITCANCEL`.

### Other

| Command | Meaning |
|---|---|
| `?FORGET,<id>` / `?FORGET,ALL` | Drop a learned peer from the ESP-NOW table + NVS |
| `?OTALOCAL,*` | Direct-USB firmware OTA |
| `?OTA,*` | Mesh-relayed firmware OTA — `DATA` carries an optional CRC-32, see below |

An unrecognised `?` command replies `Unknown command: …` rather than being dropped.

### Machine markers on the terminal

The tool parses these out of the text stream:

| Marker | Emitted by | Carries |
|---|---|---|
| `[MAE:<slot>]{…}` | Maestro queries | `{"q":"pos"\|"mov"\|"err", …}` |
| `[CLIPFS]{"total":N,"used":N}` | `?REC,LS` | clips-partition storage |
| `[CLIPDL:BEGIN]` / `[CLIPDL:EV]` / `[CLIPDL:END]` / `[CLIPDL:ERR]` | `?REC,EDITLOAD` | clip download stream |
| `[CLIPUL:BEGIN,…]` / `[CLIPUL:ACK,<idx>]` / `[CLIPUL:NAK,…]` / `[CLIPUL:END,…]` / `[CLIPUL:RENAME,OK\|ERR]` | clip upload | per-event acknowledgement, indexed so a retry cannot duplicate |
| `[TERM:<sourceWCB>]<text>` | a bridge WCB re-emitting an RTERM packet | remote CLI output |

`?REC,EDITLOAD` over the mesh refuses clips above **3000 events** (each line is an RTERM
packet paced at 2 ms; a dense clip would stall the loop for tens of seconds). Direct USB
has no such cap.

---

## 4. The Via-WCB bridge

### Envelope

The tool wraps outbound JSON as `;w20,<json>` and writes it to the tethered bridge WCB.
`;w<id>,` is the WCB's own "relay this" syntax — `;<id>,` is *not* a substitute (it hits a
different dispatcher and returns "Invalid Serial Command"). Slot **20** is NaviCore's fixed
special-peer id; the bridge WCB needs `specialPeerEnabled` with `WCB_SPECIAL_PEER_ID == 20`.

Inbound `rc_*` messages are filtered by `id == 20` so other mesh peers do not bleed in.

`SET_CONFIG` and `GET_CONFIG` **do** work over the bridge via fragmentation, but a
very large config or command library still needs Direct USB (see the caps below).

### Fragmentation

Payloads over 187 B are split. Envelope, both directions:

```json
{"f":<1-based index>,"of":<total>,"sid":<session id>,"s":"<slice>"}
```

| Parameter | Value | Where |
|---|---|---|
| Slice size — download (RC → tool) | **adaptive**, filled to a measured envelope of ≤180 B (~115–150 B of JSON typical) | `FRAG_ESC_BUDGET` / `FRAG_CHUNK_MAX_BYTES` |
| Slice size — upload (tool → RC) and CMDLIB file sends | **80 bytes** of underlying JSON | `FRAG_CHUNK_BYTES` |
| Upload cap (tool → RC) | **192 fragments ≈ 15 KB** | `FRAG_MAX_PARTS` — the RC's static receive pool |
| Download cap (RC → tool) | **512 fragments = 40 KB** | `FRAG_SEND_MAX_PARTS` / tool `FRAG_MAX_PARTS_RECV` |
| Concurrent sessions | 3 | `FRAG_POOL_SIZE` |
| Reassembly timeout | 5000 ms **idle**, refreshed by any fragment incl. duplicates | `FRAG_TIMEOUT_MS` |
| Inter-fragment pacing — upload (tool → RC) | `max(line wire time × 2, 100 ms)` | `FRAG_PACE_FLOOR_MS` / `FRAG_PACE_LINK_MULT` |
| Inter-fragment pacing — download (RC → tool) | 150 ms | `FRAG_PACING_MS` |

**Why the download slice is adaptive.** A fragment costs a fixed `FRAG_PACING_MS` no matter how
much it carries, so an under-filled fragment is wasted *wall-clock*, not just wasted bytes. The
fixed 80 was sized for the worst case — 80 bytes that were all quotes would escape to ~187 — but
real config JSON is ~22% quote characters, so fragments ran ~60% full and the worst envelope
observed was 138 B against a 187 B cap. The sender now grows each slice while the *measured*
escaped envelope stays within `FRAG_ENV_TARGET` (180 B), counting each byte at its true escaped
cost (`_jsonEscCost`: 2 for a quote or backslash, 2 for the five short control escapes, 6 for any
other control char, 1 otherwise) and admitting each UTF-8 codepoint whole. Measured on a 24 KB
config: **301 → 206 fragments (1.46×)**, worst envelope exactly 180 B, none over the cap.
`_sendFragment` still serialises and rejects anything over 187, so that check remains the backstop
if the estimate ever disagrees with ArduinoJson's escaper.

The **upload** path keeps the fixed 80 — a full config already lands near the 192-fragment receive
cap, and adaptive fill there would be the cheapest way to buy headroom if it starts refusing.

**The two directions no longer share a pacing constant.** The upload path crosses the bridge
WCB's UART, which is a real 115200 link (the WCB builds without `CDCOnBoot=cdc`, so its
`Serial` is UART0 — unlike NaviCore, where native USB-CDC ignores baud), so the tool derives
its delay from each line's actual wire time and takes the larger of that and a floor. The
download path originates on the RC and does not cross that UART, so it keeps a flat
`FRAG_PACING_MS`. Do not "restore symmetry" by making one match the other.

`FRAG_TIMEOUT_MS` measures **idle**, not total transfer time — the receiver refreshes the
deadline on any fragment for the session, duplicates included. Gating that refresh on
*new* fragments only is a trap: a sender retransmitting a lost fragment for >5 s looks
identical to a sender that vanished, and the expiry sweep in `_findOrAllocSession()` runs
**before** the sid match, so the next fragment silently claims a fresh slot, `got` restarts,
and the transfer can never complete — with no error on either side.

Rules baked into the implementation, each for a reason that cost real debugging:

- **Slices split on UTF-8 codepoint boundaries.** The receiver concatenates raw slices, so
  a split multi-byte character corrupts the JSON.
- **`sid` never 0** — 0 is the free-slot sentinel. It wraps 65535 → 1.
- **Sessions key on `(sid, senderID)`** — two tools both start at `sid = 1`.
- **Expired slots are reclaimed before the sid match**, so a wrapped sid cannot merge into
  stale parts.
- **Receive pool is static DRAM.** Raising `FRAG_MAX_PARTS` to 384 crash-loops the board
  (heap starvation during the config-load's ~96 KB document). A larger bridged payload
  needs a heap-allocated buffer sized to the transfer, not a bigger array.
- **The send machine is single-flight.** `CONFIG`, `CMDLIB`, and `WCB_META` share one
  `OutboundSend`; `OutboundKind` tags who a job serves so completing one does not clear
  another's pending request.
- The tool sets `_bridgeUploadInFlight` for the whole multi-fragment send to quiesce
  background polling — interleaved single packets otherwise drop a fragment.

### Bulk transfer (large command library → droid flash)

Streams straight to LittleFS by byte offset — **O(1) RAM**. Implemented in `WCB_Client`
(≥ 1.10.0) with NaviCore providing the file sink (`rcTelemetry::bulkBegin/bulkChunk/bulkComplete`).

| Direction | Frame |
|---|---|
| tool → RC | `{"bb":sid,"n":chunks,"t":totalLen,"g":"cmdlib","h":hash}` |
| tool → RC | `{"bc":sid,"q":seq,"s":"<base64 of ≤96 raw bytes>"}` — **no offset on the wire** |
| tool → RC | `{"bd":sid,"r":round}` |
| RC → tool | `{"bs":sid,"got":N,"r":round,"miss":[…]}` — selective NACK; `miss[]` is advisory |
| RC → tool | `{"bs":sid,"nb":1}` — need-BEGIN: the RC lost the session, resend BEGIN + every chunk (rate-limited to 400 ms) |
| RC → tool | `{"bs":sid,"done":1,"ok":0\|1,"hash":H,"r":round}` |

`BULK_CHUNK_RAW = 96`, `BULK_MAX_CHUNKS = 512`, envelope cap 187 B, 150 ms pacing. Hash is
FNV-1a over the bytes. Staging file: `/cmdlib.json.bulk.tmp`, cleared at boot.

**A chunk's byte offset is derived, never carried.** The receiver computes
`q * BULK_CHUNK_RAW` itself, so a corrupt or hostile frame cannot make it seek anywhere the
index does not allow — do not add an offset field back.

**`r` is the retry round**, and both sides need it. The sender re-sends DONE each round and
discards any STATUS carrying an older round, so a late reply from a previous round can't be
read as progress. A `nb` reply is the only recovery from a droid that rebooted mid-transfer:
without handling it a sender simply stalls, because the RC will never accept another chunk.

### Bridged status and metadata

`WCB_STATUS` must fit **one** ESP-NOW packet whenever the relay might predate `WCB_Client`
**1.16.0**. Before that release the library could *send* an oversized command (`send()` falls
through to `_sendFragmented`) but never *reassemble* one — only the WCB firmware could — so a
relay that is itself a `WCB_Client` host, such as a MgmtRelay, would have received a split reply
and dropped it. 1.16.0 adds inbound reassembly and closes that, but NaviCore cannot tell which
version is relaying for it, so the reply is still built to fit. The builder shrinks it in order
until it fits ~185 B: drop board aliases → drop the relay's friendly name → **switch to the
sparse roster** → and only then trim.

```json
{"sys":1,"type":"WCB_STATUS","quantity":N,"self":20,"relay":R,"relayName":"…",
 "online":[…],"known":[…],"clients":[…],"aliases":[…]}          ← positional (preferred)

{"sys":1,"type":"WCB_STATUS","quantity":N,"self":20,"relay":R,"relayName":"…",
 "rows":[[id,online,client,temporary],…]}                       ← sparse (overflow)
```

**Why sparse exists.** The positional arrays cost a slot for every id up to the highest known,
not per real board. A management relay sits at **id 19**, so a three-board mesh produced a
19-slot reply — measured at **280 B** against the 185 B budget — and the only remedy was a trim
that drops the **highest id first**. That is the relay you are bridging *through*, so it was
always the first thing to vanish, and only ever on this path: the same poll over Direct USB has
no packet budget and showed it correctly. The failure therefore read as "the board is gone"
rather than "the reply did not fit". The sparse form costs one row per REAL board — the same
mesh measures **121 B** — so nothing is dropped, and it is the **only** bridged form that
carries `temporary` at all (the positional form never emitted the key). `known` is implied by a
row's presence. The tool expands `rows[]` into the same positional arrays the renderer already
consumes, and an older tool falls back to `1..quantity` rather than breaking.

The data that does not fit comes from `GET_WCB_META` → a **fragmented** `WCB_META` with
`aliases[]`, `portLabels[][5]` and `seqHash[]`, which the tool caches. The relay itself is
deliberately excluded from the positional arrays (it is a transport hop, not a managed board).

### Stored sequences

`GET_WCB_SEQ` asks one WCB for the **names** of its stored `?SEQ` sequences, so the config
tool's command library can offer the sequences a board actually holds instead of a
free-text key box. NaviCore pulls them off the mesh with `WCB_Client`'s
`requestSequenceNames()`.

```json
→ {"type":"GET_WCB_SEQ","wcb":2}
← {"sys":1,"type":"WCB_SEQ","ok":true,"wcb":2,"hash":H,"names":["wave","dance"]}
← {"sys":1,"type":"WCB_SEQ","ok":false,"wcb":2,"msg":"no reply"}

→ {"type":"GET_WCB_SEQVAL","wcb":2,"key":"wave"}
← {"sys":1,"type":"WCB_SEQVAL","ok":true,"wcb":2,"key":"wave","status":0,
   "value":";M1,1***open dome^;S5,<CA1021>"}
```

`status` is a real answer, not an error: **0** OK, **1** NOTFOUND (no such key on that
board), **2** TOOBIG (the stored value exceeds what one reply can carry). The target
distinguishes these explicitly rather than answering with silence — silence is what makes
the config-pull path unusable. The `value` is passed through verbatim: its `^`
delimiters and `***` comments are what the consumer renders, so nothing in the firmware
may reformat it.

Four things about it are load-bearing:

- **The reply is asynchronous.** The verb only starts a mesh pull; the `WCB_SEQ` line
  arrives when the board answers. A pull that gets no answer is reported as
  `ok:false,"msg":"no reply"` after `WCB_SEQ_TIMEOUT_MS` (6 s) — the library abandons its
  own request silently at ~4 s, so without this the requester would wait forever.
- **One pull at a time, mesh-wide — and the two share that slot.** `WCB_Client` allows a
  single request in flight across `requestSequenceNames` *and* `requestSequence`, and
  rejects a second rather than queueing it. So the firmware's slot is tagged with its
  **kind**: the reply callbacks and the timeout all check it, because a names answer
  emitted as a value one (or a failure carrying the wrong `type`) settles the wrong
  request in the tool and leaves the real one hanging. A request arriving while one is
  live gets `ok:false,"msg":"busy"`.
- **Bodies one key at a time, on demand, never as a set.** A single value is bounded
  (~1800 chars) but the whole set is not — pulling every body would recreate the failure
  the WCB's own config pull already has, where the reply exceeds its chunk budget and the
  target then sends *nothing*. `GET_WCB_SEQ` is therefore names-only, and a consumer
  walks that list fetching what it actually needs.
- **`wcb` and `key` must be in the filter whitelist.** `handleSerialInput()` parses the
  header with an ArduinoJson filter; an un-whitelisted field is stripped, so `wcb` reads
  as 0 ("wcb out of range") and `key` reads as empty ("key required").

Both transports carry a per-board **`seqHash[]`** — the WDP SEQHASH fingerprint from
`WCBNeighbor::seqHash`, which covers sequence names *and* contents and so moves on any
save, rename, edit or erase. It rides `WCB_STATUS` on Direct USB and `WCB_META` over the
bridge (mirroring how `portLabels` is split), and lets the tool cache a key list and
re-pull only when it actually changed. A hash of **0 means "not yet known"**, not "no
sequences" — an empty inventory hashes non-zero.

On the bridge the reply is **fragmented** on the shared `_outSend` machine
(`OS_WCB_SEQ`), and the pull itself is issued from `tick()` on Core 1: both the request
and every error reply are ESP-NOW transmits, and `handle()` runs in the Core-0 receive
callback.

### Outbound telemetry

| Message | Rate | Fields |
|---|---|---|
| `rc_hb` | 0.5 Hz, broadcast | `id, fw, up, mode, model, sbusFps, sbusAge, sbusLost, sbusFail` |
| `rc_ch` | `chRateHz` (default 5, range 1–20), **only while a subscriber is heard** | `id, ch[24]` |
| `rc_trig` | on every trigger — local, USB, or remote | `id, mode, btn, tap` (`tap` 4 = long press) |
| `rc_mode` | on mode change | `id, mode` |
| `wcb_alias` | on alias learn | board id + name |

Both periodic emits are best-effort (unacked): a missed heartbeat is covered by the next
one, and an acked 5 Hz channel stream would thrash the ETM pending table.

---

## 5. ESP-NOW raw-packet formats

`WCB_Client` routes raw packets **by struct size**, so every size must stay unique across
the whole mesh ecosystem.

| Type | Struct | Size | Purpose |
|---|---|---|---|
| 7 | `espnow_struct_remote_term` | **204 B** | RTERM — one line of CLI output |
| 20 | `espnow_struct_ota_ctrl` | **55 B** | OTA BEGIN |
| 21 | `espnow_struct_ota_data` | **243 B** | OTA fragment (192 B payload) |
| 22 | ctrl | 55 B | OTA ACK (write cursor + status) |
| 23 | ctrl | 55 B | OTA END (finalize, verify, reboot) |
| 24 | ctrl | 55 B | OTA ABORT |

Sizes already claimed elsewhere in the ecosystem: **43, 204, 226, 230, 249, 252**.
`static_assert`s in [`navicore_ota.h`](../navicore_ota.h) and
[`navicore_rterm.h`](../navicore_rterm.h) fail the build if a struct drifts — **do not
suppress them**; pick a different size instead.

Every packet begins with `char structPassword[40]`, matched against
`rcConfig.wcbNetwork.password`.

**OTA safety model.** Writes always target the inactive slot
(`esp_ota_get_next_update_partition`); the image is SHA-verified by `esp_ota_end` *before*
the boot pointer moves; any failure or timeout calls `esp_ota_abort`, which never switches.
An interrupted transfer always leaves the board on its current firmware.
`esp_ota_*` blocks (BEGIN erases ~1.2 MB) — **never call it from the receive callback**;
`otaRawPacketHook()` only enqueues.

**RTERM.** NaviCore only ever *sends* these. The format is byte-identical to the WCB's, so
an unmodified bridge WCB re-emits each packet to its USB as `[TERM:<sourceWCB>]<text>\n`
with no WCB firmware change. Lines are capped at 160 chars and hard-wrapped.

---

## 6. Device command syntax (what actions actually emit)

| Device | Wire form | Example |
|---|---|---|
| WCB command | `;`-prefixed, chainable with `^` | `:PP100`, `;h,play,a,1,fadein,4^;t6000` |
| WCB → specific board+port | `;W<id>;S<port>,<cmd>` | raw serial forward |
| Maestro (NaviCore action `cmd`) | `setTarget,ch,pos` · `goHome` · `stopScript` · `restartScript,n` · `setSpeed,ch,v` · `setAccel,ch,v` | `pos` in ¼ µs |
| Maestro (Pololu bytes) | `0x84` target · `0x87` speed · `0x89` accel · `0xA2` home · `0xA4` stop · `0xA7` restart-sub · `0x90` get-pos · `0x93` moving · `0xA1` errors | built by `WcbCmd`/`WcbMaestro` |
| MP3 Trigger | `;A,<VERB>[,arg]` | `;A,PLAY,3`, `;A,VOL,20` (0 = loudest, 64 = silent) |
| DFPlayer Mini | `;D,<VERB>[,arg[,arg]]` | `;D,PLAY,3`, `;D,FOLDER,1,2`, `;D,VOL,20` (**0 = silent, 30 = loudest**) |
| WLED | `;L<id>,<verb>` | bare `;L,` (id 0) targets the lowest-id local slot |
| HCR | angle-bracket frames | `<OH80,QEH>` |

`;D` is the DFPlayer's own verb letter — the WCB dispatch's taken letters are
`S W C M P A H L V D`. On the wire a DFPlayer command is a 10-byte binary frame
(`7E FF 06 CMD ACK PH PL CK CK EF`), built by `WcbCmd`'s `DfPlayerCodec`; the `;D` text
form only ever travels **between** boards, never down the wire to the module. Full verb
table in [DFPLAYER_DESIGN.md §3](DFPLAYER_DESIGN.md#3-wire-format).

Maestro speed/accel are **sticky limits**, not moves: they shape every later move on that
channel until reset to 0. See [MAESTRO_ACTIONS.md](MAESTRO_ACTIONS.md) for the unit maths.

---

## 7. Changing a protocol — checklist

1. **Find the counterpart.** Firmware constant ↔ tool constant pairs are listed in
   [CONFIG_SCHEMA.md §6](CONFIG_SCHEMA.md#6-cross-file-invariants). Change both.
2. **Respect the 187-byte cap** for anything crossing the mesh. Compute the *escaped,
   wrapped, UTF-8 byte* length, not the string length.
3. **Keep raw-struct sizes unique** and update the `static_assert` message list.
4. **Add new header fields to the ArduinoJson filter whitelist** in `handleSerialInput()`.
5. **Defer anything that touches flash, NVS, or droid hardware to Core 1** (§8 of
   [ARCHITECTURE.md](ARCHITECTURE.md)).
6. **Version the mesh side.** Wire-format changes to `WCB_Client` ship only when pushed to
   `greghulette/WCBClient` master — see [BUILD_AND_RELEASE.md](BUILD_AND_RELEASE.md).

---

## Revision log

Newest first. Add a row whenever a code change alters what this page describes — same commit
as the code. Page body stays present-tense; history lives here.

| Date | Commit | Change |
| 2026-08-28 | _(uncommitted)_ | **Added a WebSocket command transport** (`navicore_wsserver.h`, `ws://<softAPIP>/ws`), optional and off by default behind `wifiEnabled`. It carries the same newline-delimited JSON as Direct USB because it feeds the same `processInputLine()` — no second command surface. No 187-byte cap and no fragmentation: it is a direct link, so a large SET_CONFIG crosses in one frame. The load-bearing constraint is the split: the httpd handler runs on **Core 0** beside the ESP-NOW callback and may only copy-enqueue-return, with the command executed from `loop()` on Core 1, because `processInputLine()` writes flash and NVS and drives bit-banged serial. Reply via `httpd_ws_send_frame_async()` (the documented out-of-request API). A line up to 98 KB is queued by POINTER from PSRAM, not by value like `RemoteCliMsg` — ownership transfers with the pointer, and the handler frees it if the queue send fails. Costs +33.5 KB flash / +1.4 KB static RAM; app slot 59.1%. |
| 2026-08-28 | _(uncommitted)_ | **Split framing from dispatch in the serial input path.** `handleSerialInput()` now only reads bytes and assembles lines; the ~430-line dispatch moved verbatim into `processInputLine(const String&)`, which any transport can call. Pure refactor — verified by normalising both bodies and diffing: 274 executable lines, byte-identical. Two contracts are load-bearing and are now stated at the function: the `bool` return means "yield to `loop()` now" (the `?` branch and a parse failure both need the caller to stop draining, so heartbeats survive ACK-paced OTA chunks), and the caller owns the buffer — the line is `const&` and cleared by the caller, because copying a ~98 KB `SET_CONFIG` per line is real cost. |
| 2026-08-28 | _(uncommitted)_ | Documented that the `MESH_STATS` aggregate and its per-peer rows are sampled a moment apart and can disagree by one — `agg` from the library counters, `peers[]` walked afterwards, so an ACK in between leaves the aggregate behind the rows it contains. Seen in the field as the derived "not currently listed" remainder rendering `Ack -1` against `Sent 0`. Recorded as NOT a firmware bug, with the two tempting wrong fixes named (making one side authoritative, or recomputing the aggregate from the rows — the aggregate covers all 20 library slots including unlisted boards, which is why the remainder row exists at all). The tool now clamps the remainder at zero: a real remainder still shows, a skew of -1 does not. |
| 2026-08-26 | _(uncommitted)_ | Added `RESET_MESH_STATS` (both transports) — zero the ESP-NOW counters without rebooting. Deferred to `loop()` on both paths because `WCB_Client::resetStats()` takes the pending-table lock and races the RX task, so the library forbids calling it from a receive callback. |
|---|---|---|
| 2026-08-27 | _(uncommitted)_ | Fragment pacing is no longer one constant shared by both directions. **Upload** (tool → RC) crosses the bridge WCB’s UART — a *real* 115200 link, since the WCB builds without `CDCOnBoot=cdc` and its `Serial` is UART0, unlike NaviCore where native USB-CDC ignores baud — so the tool now paces each line by `max(wire time × FRAG_PACE_LINK_MULT, FRAG_PACE_FLOOR_MS)` (2×, 100 ms) instead of a flat 150 ms, and skips the delay after the final fragment. **Download** (RC → tool) never crosses that UART and keeps `FRAG_PACING_MS` = 150. Do not re-symmetrise them. Also documented that `FRAG_TIMEOUT_MS` is an **idle** timeout: the receiver now refreshes the deadline on *any* fragment including duplicates, because gating it on new-only made a live retransmit indistinguishable from a dead sender — the expiry sweep runs before the sid match, so the session was reclaimed mid-transfer and silently restarted at `got=1`. |
| 2026-08-28 | _(uncommitted)_ | **`WCB_Client` 1.16.0 reassembles inbound fragments**, so `send()` is symmetric between any two peers. Previously it auto-fragmented on transmit but only the WCB *firmware* could rebuild a session, so client→client oversized traffic transmitted perfectly and delivered nothing. The one-packet `WCB_STATUS` budget is kept regardless, because NaviCore cannot tell whether the relay answering for it predates 1.16.0 — and the sparse roster is smaller anyway. |
| 2026-08-27 | _(uncommitted)_ | **Bridged `WCB_STATUS` gained a sparse `rows[]` form** — `[[id,online,client,temporary],…]`, chosen when the positional arrays will not fit one ESP-NOW packet, BEFORE the roster trim rather than after. The positional form costs a slot per id, not per board, so a mgmt relay at id 19 made a three-board mesh 280 B against a 185 B budget; the trim then dropped the highest id first, i.e. the relay being bridged through, and only on this path (Direct USB has no budget and showed it). Sparse is 121 B for the same mesh, and is also the only bridged form that carries `temporary` — the positional builder never emitted the key. `known` is implied by presence; the tool expands rows into the existing positional arrays; an older tool falls back to `1..quantity`. |
| 2026-08-27 | _(uncommitted)_ | **Batched keyframe download** — `?REC,EDITLOAD,…,<count>,B` opts into `[CLIPDL:EVB,<firstIdx>]{"e":[[t,k,a,b,c],…]}`, packing consecutive keyframes into one line with positional indices. Actions are never batched and flush the run. 5.0× fewer mesh packets / 2.5× fewer bytes on a 300-event clip, longest line 133 B (inside the 160 B cap, so no new fragmentation). Round-trip verified to reconstruct byte-identically to the per-event form across seven slices including ones straddling actions. Backward compatible both ways — `toInt()` stops at the comma, and the tool accepts both shapes unconditionally. |
| 2026-08-27 | _(uncommitted)_ | **`?OTA,DATA` carries an optional CRC-32**, as a suffix on the offset field: `?OTA,DATA,<t>,<s>,<offset>:<crc32>,<b64>`, computed over `"<offset>,<b64>"` as transmitted. A relay that fails the check DROPS the line. Placed on the offset field, not appended, because `String::toInt()` stops at the `:` — so a relay predating the check reads the offset unchanged and a sender predating it just omits the suffix; appending a field would have been swept into the base64 and failed every packet. Verified by BOTH relays (`navicore_ota.h` `otaCrc32`, WCB `calculateCRC32`) against the tool's `_crc32Hex` — all three are the same reflected CRC-32 (poly `0xEDB88320`) and were cross-checked. Guards the USB→relay SERIAL hop only; 802.11 already CRCs the ESP-NOW hop in hardware. Also `Serial.setRxBufferSize` 4096 → 8192 and the sender's `PACE_MS` 12 → 25. |
| 2026-08-27 | _(uncommitted)_ | **`[CLIPDL:BEGIN]`/`[CLIPDL:END]` gained `nm`** — the clip the buffer actually holds. The reply stream was anonymous, so a timed-out range's still-arriving lines were adopted by the next request: a stale `END` finished it early ("no reply from the board"), a stale `BEGIN` replaced its `count`/`fp` ("the clip changed on the board mid-download"), and stale events landed under foreign indices, pushing `got.size` past `total` and reporting a NEGATIVE shortfall ("-32 of 16 events"). One timeout then cascaded through the remaining clips of a backup. `nm` costs ~39 B on two control lines per range and nothing per event; `[CLIPDL:EV]` is unchanged. Request format is unchanged — no new argument — so an older tool is unaffected. |
| 2026-08-25 | _(uncommitted)_ | **Ranged clip download.** `?REC,EDITLOAD,<name>,<from>[,<count>]` streams a bounded slice with each event carrying its absolute index in the MARKER (`[CLIPDL:EV,<i>]`), so a client can detect exactly which events are missing and re-request only those. `BEGIN`/`END` gained `from`/`n`/`fp` (FNV-1a buffer fingerprint) and `fc` (the FILE header count — `loadClip` truncates silently, so `count` alone would report a short read as complete). Legacy unranged form is byte-identical. `[CLIPITEM]` gained `n` (event count). The relayed 3000-event refusal now applies only to the legacy whole-clip form. |
| 2026-08-24 | _(uncommitted)_ | `rc_trig` is now emitted on **USB as well as the mesh**. `emitTrig()` returns early without a ready WCB, so a Direct-USB tool saw no dispatch events at all for a local button press. Same JSON shape on both transports; the tool filters the mesh copy by `id` but not the USB copy, which arrives from the one board it is connected to. |
| 2026-08-24 | `083207c` | `tap` now spans 1–4 on both `TRIGGER` (USB + mesh) and `rc_trig`, where **4 = long press**. No wire-format change — the existing `tap` field carries it, so the WCB bridge and `WcbCmd` are untouched. Three separate bounds enforce it: the USB handler, the mesh clamp in `rcTelemetry::handle()`, and the clamp in `drainRemoteTriggers()`. |
| 2026-08-18 | _(uncommitted)_ | §4 bulk-transfer frames corrected to the wire as implemented: CHUNK carries **no** `"o"` offset (the receiver derives `q * BULK_CHUNK_RAW` and deliberately never trusts one), DONE/STATUS/FINAL all carry `"r":round`, and the `{"bs":sid,"nb":1}` need-BEGIN reply — previously undocumented, and the only recovery from a droid that reboots mid-transfer — now has a row. |
| 2026-08-17 | _(uncommitted)_ | Added `GET_WCB_SEQVAL` → `WCB_SEQVAL` (ONE stored sequence's contents by key, on `WCB_Client` 1.15.0's `requestSequence()`), so the command library can show what a chosen sequence does. `status` is a real answer — 0 OK / 1 NOTFOUND / 2 TOOBIG — not an error, and the `value` is passed through verbatim because its `^` delimiters and `***` comments are what the consumer renders. The load-bearing part: the library allows **one** request in flight across names *and* values, so the firmware's pull slot is now tagged with its kind and every reply and timeout checks it — a names answer emitted as a value one, or a failure carrying the wrong `type`, settles the wrong request in the tool and leaves the real one hanging. `key` joins `wcb` in the `handleSerialInput()` filter whitelist. |
| 2026-08-17 | _(uncommitted)_ | Added `GET_WCB_SEQ` → `WCB_SEQ` (a WCB's stored-sequence key names, pulled off the mesh with `WCB_Client` 1.15.0's `requestSequenceNames()`) and the per-board `seqHash[]` fingerprint, carried in `WCB_STATUS` on USB and `WCB_META` over the bridge. Four constraints are load-bearing: the reply is **async** (6 s `no reply` timeout, because the library abandons its own pull silently at ~4 s); **one pull at a time mesh-wide** (a second is rejected, not queued); **names only, never bodies** (the whole set has no ceiling — the same failure the WCB config pull already has); and `wcb` must be in the `handleSerialInput()` filter whitelist or it is stripped and read as 0. Bridge replies fragment on `OS_WCB_SEQ`; the pull and every error reply are issued from `tick()` on Core 1 because both are ESP-NOW transmits. Also corrected the `GET_WCB_STATUS` row's stale "See §5" — that content is §4. |
| 2026-08-13 | _(uncommitted)_ | `MESH_STATS` is now **paged** (`"pg"`/`"last"`) so the full per-board roster crosses the bridge — measured: no per-board data for a 6-board fleet fits one 185 B frame alongside the aggregate at all, so the earlier shed-tier approach could only ever drop boards. Page 0 deliberately carries no rows. Consumer merges by id and promotes only a contiguous set ending in `last`. `agg` keys abbreviated to `rty`/`fail`/`ung` to buy row space. `buildMeshStatsPage()` is the single builder for USB **and** bridged. |
| 2026-08-12 | _(uncommitted)_ | Added `GET_MESH_STATS` → `MESH_STATS` (ESP-NOW delivery counters, flat per-peer rows, bridged replies shed `peers` to fit one frame). Noted that `recv` is NaviCore's own counter — `WCB_Client` 1.13.0's statistics are outbound-only. |
| 2026-08-12 | _(uncommitted)_ | §1: documented the **ensured-send degradation contract** (`_findFreePending` never evicts an outstanding ensured slot; `_sendPacket` degrades to best-effort and returns `false`, which `rcExecuteActionNow` deliberately discards) and the **one-hop cap** — which gates *implicit* routing only (`;A`/`;D`/`;H`, `;M`, `;L`, `;C`/`;SEQ`, and any re-broadcast). Explicit `;w<n>` is not capped: self-target runs local (`WCB.ino` ≈5586), remote re-forwards by unicast (≈5604), and `sendESPNowMessage` caps `target == 0` only (≈2145). So a unicast `^`-chain loses a part only when it is an implicitly-routed verb whose device is hosted off-target. |
| 2026-08-05 | _(uncommitted)_ | Added the `;D` DFPlayer verb to the device-command table (10-byte binary frame on the wire; `;D` text only travels between boards) and `DBG_DFP` = bit 6 to the debug bitmask. |
| 2026-08-04 | _(uncommitted)_ | Initial version. |
| 2026-08-28 | _(uncommitted)_ | **Download fragments are filled adaptively** instead of at a fixed 80 B. A fragment costs a fixed  (150 ms) regardless of payload, so under-filling wasted wall-clock: a 171-fragment config took 25.6 s while the worst envelope produced was only 138 B against the 187 B cap. The sender now grows each slice while the *measured* escaped envelope stays inside  (180 B), exact per byte-escape-cost and per UTF-8 codepoint. 301 → 206 fragments (1.46×) on a 24 KB config, worst envelope exactly 180, none over cap.  keeps its >187 rejection as the backstop. Upload and CMDLIB file sends still use the fixed . |
