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

Newline-delimited JSON. Handled by `handleSerialInput()` in
[`NaviCore.ino`](../NaviCore.ino). Every tool-originated message carries an additive
`"sys":1` so the WCB Wizard can mute the tool's chatter when both share a port.

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
| `{"type":"TRIGGER","mode":M,"btn":B,"tap":T}` | — | Virtual button press |
| `{"type":"WCB_SEND","target":N,"cmd":"…"}` | — | `target` 0 = broadcast |
| `{"type":"FORGET_PEER","id":N}` / `"all":true` | — | id 0 or `all` = drop every learned peer |
| `{"type":"SET_DEBUG_FLAGS","flags":N}` | — | See the debug bitmask below |
| `{"type":"GET_WCB_STATUS"}` | `{"type":"WCB_STATUS",…}` | See §4 “Bridged status and metadata” |
| `{"type":"GET_WCB_SEQ","wcb":N}` | `{"type":"WCB_SEQ",…}` **async** | One board's stored-sequence key names, pulled off the mesh. See §4 “Stored-sequence inventory” |
| `{"type":"GET_MESH_STATS"}` | `{"type":"MESH_STATS",…}` | ESP-NOW delivery counters. See below |

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
| `?OTA,*` | Mesh-relayed firmware OTA |

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
| Slice size | **80 bytes** of underlying JSON | `FRAG_CHUNK_BYTES` (both sides) |
| Upload cap (tool → RC) | **192 fragments ≈ 15 KB** | `FRAG_MAX_PARTS` — the RC's static receive pool |
| Download cap (RC → tool) | **512 fragments = 40 KB** | `FRAG_SEND_MAX_PARTS` / tool `FRAG_MAX_PARTS_RECV` |
| Concurrent sessions | 3 | `FRAG_POOL_SIZE` |
| Reassembly timeout | 5000 ms | `FRAG_TIMEOUT_MS` (both sides) |
| Inter-fragment pacing | 150 ms | `FRAG_PACING_MS` (both sides) |

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
| tool → RC | `{"bc":sid,"q":seq,"o":offset,"s":"<base64 of 96 raw bytes>"}` |
| tool → RC | `{"bd":sid}` |
| RC → tool | `{"bs":sid,"got":N,"miss":[…]}` — selective NACK |
| RC → tool | `{"bs":sid,"done":1,"ok":0\|1,"hash":H}` |

`BULK_CHUNK_RAW = 96`, `BULK_MAX_CHUNKS = 512`, envelope cap 187 B, 150 ms pacing. Hash is
FNV-1a over the bytes. Staging file: `/cmdlib.json.bulk.tmp`, cleared at boot.

### Bridged status and metadata

`WCB_STATUS` must fit **one** ESP-NOW packet (the relay cannot reassemble). The builder
shrinks the reply in order — drop board aliases, then the relay's friendly name, then trim
the roster — until it fits ~185 B.

```json
{"sys":1,"type":"WCB_STATUS","quantity":N,"self":20,"relay":R,"relayName":"…",
 "online":[…],"known":[…],"clients":[…],"aliases":[…]}
```

The data that does not fit comes from `GET_WCB_META` → a **fragmented** `WCB_META` with
`aliases[]`, `portLabels[][5]` and `seqHash[]`, which the tool caches. The relay itself is
deliberately excluded from the positional arrays (it is a transport hop, not a managed board).

### Stored-sequence inventory

`GET_WCB_SEQ` asks one WCB for the **names** of its stored `?SEQ` sequences, so the config
tool's command library can offer the sequences a board actually holds instead of a
free-text key box. NaviCore pulls them off the mesh with `WCB_Client`'s
`requestSequenceNames()`.

```json
→ {"type":"GET_WCB_SEQ","wcb":2}
← {"sys":1,"type":"WCB_SEQ","ok":true,"wcb":2,"hash":H,"names":["wave","dance"]}
← {"sys":1,"type":"WCB_SEQ","ok":false,"wcb":2,"msg":"no reply"}
```

Four things about it are load-bearing:

- **The reply is asynchronous.** The verb only starts a mesh pull; the `WCB_SEQ` line
  arrives when the board answers. A pull that gets no answer is reported as
  `ok:false,"msg":"no reply"` after `WCB_SEQ_TIMEOUT_MS` (6 s) — the library abandons its
  own request silently at ~4 s, so without this the requester would wait forever.
- **One pull at a time, mesh-wide.** `WCB_Client` allows a single inventory request in
  flight and rejects a second rather than queueing it, so a consumer walking several
  boards must go one at a time. A request arriving while one is live gets
  `ok:false,"msg":"busy"`.
- **Names only, never bodies.** `WCB_Client` can also fetch a sequence's contents
  (`requestSequence`), but one at a time and with no ceiling on the total — pulling every
  body would recreate the failure the WCB's own config pull already has, where the reply
  exceeds its chunk budget and the target then sends *nothing*.
- **`wcb` must be in the filter whitelist.** `handleSerialInput()` parses the header with
  an ArduinoJson filter; an un-whitelisted field is stripped, read as 0, and the pull is
  rejected as out of range.

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
| `rc_trig` | on every trigger — local, USB, or remote | `id, mode, btn, tap` |
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
|---|---|---|
| 2026-08-17 | _(uncommitted)_ | Added `GET_WCB_SEQ` → `WCB_SEQ` (a WCB's stored-sequence key names, pulled off the mesh with `WCB_Client` 1.15.0's `requestSequenceNames()`) and the per-board `seqHash[]` fingerprint, carried in `WCB_STATUS` on USB and `WCB_META` over the bridge. Four constraints are load-bearing: the reply is **async** (6 s `no reply` timeout, because the library abandons its own pull silently at ~4 s); **one pull at a time mesh-wide** (a second is rejected, not queued); **names only, never bodies** (the whole set has no ceiling — the same failure the WCB config pull already has); and `wcb` must be in the `handleSerialInput()` filter whitelist or it is stripped and read as 0. Bridge replies fragment on `OS_WCB_SEQ`; the pull and every error reply are issued from `tick()` on Core 1 because both are ESP-NOW transmits. Also corrected the `GET_WCB_STATUS` row's stale "See §5" — that content is §4. |
| 2026-08-13 | _(uncommitted)_ | `MESH_STATS` is now **paged** (`"pg"`/`"last"`) so the full per-board roster crosses the bridge — measured: no per-board data for a 6-board fleet fits one 185 B frame alongside the aggregate at all, so the earlier shed-tier approach could only ever drop boards. Page 0 deliberately carries no rows. Consumer merges by id and promotes only a contiguous set ending in `last`. `agg` keys abbreviated to `rty`/`fail`/`ung` to buy row space. `buildMeshStatsPage()` is the single builder for USB **and** bridged. |
| 2026-08-12 | _(uncommitted)_ | Added `GET_MESH_STATS` → `MESH_STATS` (ESP-NOW delivery counters, flat per-peer rows, bridged replies shed `peers` to fit one frame). Noted that `recv` is NaviCore's own counter — `WCB_Client` 1.13.0's statistics are outbound-only. |
| 2026-08-12 | _(uncommitted)_ | §1: documented the **ensured-send degradation contract** (`_findFreePending` never evicts an outstanding ensured slot; `_sendPacket` degrades to best-effort and returns `false`, which `rcExecuteActionNow` deliberately discards) and the **one-hop cap** — which gates *implicit* routing only (`;A`/`;D`/`;H`, `;M`, `;L`, `;C`/`;SEQ`, and any re-broadcast). Explicit `;w<n>` is not capped: self-target runs local (`WCB.ino` ≈5586), remote re-forwards by unicast (≈5604), and `sendESPNowMessage` caps `target == 0` only (≈2145). So a unicast `^`-chain loses a part only when it is an implicitly-routed verb whose device is hosted off-target. |
| 2026-08-05 | _(uncommitted)_ | Added the `;D` DFPlayer verb to the device-command table (10-byte binary frame on the wire; `;D` text only travels between boards) and `DBG_DFP` = bit 6 to the debug bitmask. |
| 2026-08-04 | _(uncommitted)_ | Initial version. |
