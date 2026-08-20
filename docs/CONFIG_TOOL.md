# NaviCore — Config Tool

The browser GUI at [`config_tool/index.html`](../config_tool/index.html). ~16 500 lines of
HTML + CSS + inline JavaScript in **one file**, no build step, no framework, no bundler.
Published to GitHub Pages and openable straight from disk.

Related: [PROTOCOLS.md](PROTOCOLS.md) · [CONFIG_SCHEMA.md](CONFIG_SCHEMA.md)

---

## 1. Why one file

The tool must run from `file://`, from GitHub Pages, and from a droid owner's laptop with
no toolchain. Everything is inline; the only siblings are:

| File | Role |
|---|---|
| [`flasher.js`](../config_tool/flasher.js) | ESP32-S3 flashing via esptool-js (loaded on demand from CDN) |
| [`serial-hub.js`](../config_tool/serial-hub.js) | `WcbSerialHub` — shares one Web Serial port across same-origin tabs |
| [`cmdlib/`](../config_tool/cmdlib/) | Vendored command library: `droidnet/` (third-party boards) + `navicore/` (native verbs) |

`index-Old.html` and `index1.html` are frozen snapshots nothing loads.

**There is no build step, so a syntax slip silently breaks all event wiring.** After every
edit run:

```
node C:\Users\ghulette\tools\jscheck.js config_tool/index.html
```

It compiles each inline `<script>` with `vm.Script` (parse only), catching brace/quote
imbalances and the `</script>`-inside-a-string mistake that the HTML parser turns into a
premature script close.

---

## 2. Connecting

Three transports, chosen in the connect modal:

| Mode | Function | What it talks to |
|---|---|---|
| **Direct USB** | `connectDirect()` | A NaviCore over Web Serial |
| **Via WCB** | `connectViaWcbOpt()` | A tethered bridge WCB that relays to NaviCore at slot 20 |
| **Shared port** | `connectSharedPort()` | A port already owned by another same-origin tab |

Transport is auto-detected at connect: the tool pings, and `_pongSeen` decides whether it
is talking to a NaviCore directly or needs the bridge wrapper.

**`WcbSerialHub`** (`serial-hub.js`) exists because a Web Serial port can be open in exactly
one browsing context. One tab wins a Web Lock and becomes leader, owning the physical port;
followers proxy raw bytes over a `BroadcastChannel` and the leader mirrors reads back.
Leader failover rides the lock queue. Two constraints are deliberate:

- **All tabs must be same-origin** — that is the entire reason the Pages deploy publishes
  the tool under `greghulette.github.io`, alongside the WCB Wizard.
- **DTR is never asserted and there is no visibility handoff.** A WCB reboots on DTR, so
  either would reset the bridge board mid-session.

Flashing needs the raw port, so only the leader tab can flash. Firmware-flash buttons are
disabled while Via WCB is active.

**Page teardown releases the port.** A `pagehide` handler (`_releaseSerialOnUnload()`) deasserts DTR/RTS and closes, because a refresh otherwise abandons an open port and the CDC control lines settle wherever the driver leaves them — and on a NaviCore v2 (native USB, no bridge chip) those lines are what the USB Serial/JTAG peripheral watches to reset the chip. It also calls `sharedHub.leave()`, whose cross-tab `bye` posts synchronously so a follower can take a shared port over without waiting out the Web Lock.

Link loss (sleep, unplug) is caught by `handleLinkLost()` → `tryAutoReconnect()`, gated on
`_wantAutoReconnect` so a deliberate disconnect never re-opens the port. It is reached from
**both** directions: a fatal read error in `startReading()`, and a fatal write error in
`sendLine()` — a `read()` on a lost device can hang forever without ever rejecting, so the
write side is often the only thing that notices.

---

## 3. Layout

Eleven tabs (`data-tab=…`): **general, channels, logical, transmitter, wcb, audio,
maestro, wled, smoothing, serial, firmware**, plus modal editors:

**audio** holds all three sound destinations — HCR vocalizer, MP3 Trigger, DFPlayer Mini —
in one pane. It replaced the separate `hcr` and `mp3` tabs; `setConfigTab()` remaps those
stored names (and `dfplayer`) to `audio`, because a stale `rcConfigLastTab` in localStorage
would otherwise hit the unknown-name fallback and dump the user on **channels**.

| Editor | Entry point |
|---|---|
| Button / switch / knob action editor | `openModal()` → `buildTierCard()` → `buildActionRow()` |
| Command library browser | `openCmdLibrary()` |
| Clips library | `openClipsModal()` |
| Timeline editor | `openTimelineEditor()` |
| Cheat sheet | `openCheatSheet()` |
| Calibration wizard | sends `CALIB` on/off to mute dispatch, reads values from `PWM_UPDATE` |

The terminal is a resizable split pane; in Via WCB mode it grows a second column for raw
CLI to the bridge WCB. Debug chips map to the firmware's `SET_DEBUG_FLAGS` bitmask, so
turning a category off actually silences the board rather than hiding lines client-side.

---

## 4. Function map

Where to look when changing a given area:

| Area | Key functions |
|---|---|
| Inbound message routing | `handleBoardMessage()` — the switchboard for every board reply |
| Read loop / framing | `startReading()` |
| Outbound | `sendLine()` (raw, serialised, 512 B USB chunking) · `sendJSON()` (adds `sys:1`, wraps `;w20,`, fragments) |
| Config load | `applyConfig()` — also the migration point for older config shapes |
| Config save | `saveConfigToBoard()` + `_diffConfigBranches()` / `_diffMappings()` |
| Live monitor | `updatePWMDisplay()`, `updateChannelsGrid()`, `updateTransmitterAnimation()`, `markActiveChannels()` |
| Mesh status | `renderWcbStatus()`, `startWcbStatusPoll()`, `_maybeRequestWcbMeta()`, `_wcbPortTooltip()` (hover = that board's serial-port map) |
| Mesh stats | `_meshStatsChipLine()` (sidebar per-board), `_meshStatsGlanceHtml()` (footer), `renderMeshStats()` (modal table), `_meshStatsMergePage()` (paged reply), `requestMeshStats()` · `_statsReport()`, `renderStatsReport()` (the saved `?STATS,RPT` push setting) |
| Action rows | `buildActionRow()`, `renderArgs()`, `_renderMaestroActionArgs()`, `renderHcrParamFields()`, `renderMp3ArgField()` |
| Wire-command row (command + "Send to") | `_appendCommandView()` — shared by the tier rows *and* the timeline popover; `readActionFromFid()` reads it back |
| Command library | `ncCommandLibrary()`, `ncEncodeCommand()`, `ncDecodeCommand()`, `_cmdlibDestFieldHtml()` · live sequence source: `_wcbSeqRefresh()`, `_wcbSeqOptionsHtml()`, `_wcbSeqFieldHtml()`, `_cmdlibApplySeqSource()` |
| Maestro panel | `renderMaestroLocations()`, `_importMaestroFile()` (Control Center XML), `_maestroChInfo()` |
| Smoothing | `renderSmoothingPane()`, `_smoothProfiles()` |
| Clips | `clipsRefresh()`, `_clipListFeed()`, `renderClips()`, `clipRename()`, `clipDelete()` |
| Timeline editor | everything prefixed `_tl*` |
| Fragmentation (receive) | fragment envelope handling in `handleBoardMessage()`, `_fragProgress()` |
| Bulk push | `_bulk*` family |
| Flashing | `flasher.js` → `flashFirmware(port, callbacks)` |

---

## 5. Save flow

Saving is diff-based, and the guard rails exist because each failure mode actually happened:

1. **`_configLoaded`** must be true. Before the first `CONFIG` arrives, `config.*` holds the
   file's static defaults; saving then would push `deviceId=3` and cut the bridge cord.
2. **`_configBaseline`** is a deep clone taken every time `CONFIG` arrives and refreshed
   after each successful save.
3. **`_stringifyStable()`** sorts object keys before comparing — plain `JSON.stringify`
   preserves insertion order, which differs between the spread-rebuilt config and its clone
   and produced a false diff on every save right after a load.
4. **`_diffConfigBranches()`** ships only changed top-level branches, and **sub-diffs
   `mappings` per button**. Without the sub-diff, one action edit marks all 108 mappings
   dirty — ~40 fragments over the bridge, enough to trip the board's task watchdog.
   Firmware rebuilds each *present* button key wholesale and leaves absent ones untouched,
   so a partial object is correct. A deleted button ships as `{}` to clear the slot.
5. **`_hwSetupBaseline`** is snapshotted when the Config window opens, so render-time
   padding cannot fabricate a phantom "unsaved changes" prompt on close.
6. **`_postFlashReload`** re-snapshots after a flash reconnect for the same reason.
7. The result is reported by a **toast driven by the board's actual ACK**, not by a
   terminal line the user never reads.

---

## 6. Command library

A data-driven catalog of droid-board commands, so users pick verbs instead of typing wire
strings.

```
cmdlib/
  droidnet/manifest.json  + boards/*.json   third-party boards (FlthyHPs, MagicPanel,
                                            RSeriesLogic, HCR, MP3, WLED, Maestro,
                                            RoamADome, UppitySpinner, AstroPixels, WCB config +
                                            sequences, …)
  navicore/manifest.json  + navicore.json   NaviCore-native verbs (record / play / stop)
```

`cmdlib/droidnet/` is a **vendored MPL-2.0 snapshot** — NaviCore's own boards never go in
there. They live inline in `NC_CMDLIB_SEED` in `index.html`; `wcb-dfplayer` (the `;D` verb
set) is one of them. Two local deltas to the snapshot are described below: the `source`
field, and the `wcb-sequences` board split out of `wcb-native`. A param's `enum` field is a
**string id** into the library-level `enums` map, not an inline list.

Each command declares `id, name, safety, encoder, template, params[], examples,
commentLabel, category`, plus routing metadata (`class`, `nativeWrapper`,
`durationSuffix`). `ncEncodeCommand()` renders a template into a wire string;
`ncDecodeCommand()` reverses it so an existing action re-opens in the picker instead of
appearing as opaque text.

Users can add **private boards**, stored in `localStorage` and pushable to the droid's
`/cmdlib.json` — small libraries over `SET_CMDLIB`, large ones over the bulk-transfer path
(`_bulk*`). The droid stores the JSON opaquely; only size + FNV-1a hash are interpreted, so
the tool can skip re-pulling an unchanged library.

### Picker layout

The library builds **actions** — what a pilot fires from a transmitter switch. Boards of
setup/config verbs are therefore **hidden** (`NC_CMDLIB_HIDDEN`): nobody binds a button to
"set the hardware version" or "erase NVS", and those belong in the WCB Wizard, which is
built for them. `wcb-native` (71 verbs) is hidden today.

Hiding rather than deleting keeps the vendored snapshot byte-for-byte upstream apart from
the sequence split, and means a re-fetch can't bring the board back. Un-hide by removing
the id: its `NC_CMDLIB_ORDER_TOP` entry is still there, so it returns to its old slot.
**Known consequence** — `ncDecodeCommand()` only searches boards that are present, so an
action row already holding one of those commands keeps working (the wire string is stored
in the config and sent unchanged) but loses its `📚 Board · Command` hint and re-opens at
the list instead of the composer.

What survives renders collapsed. Related boards fold one level further into a **group**
(`NC_CMDLIB_GROUPS`), so a cluster is one row instead of six:

| Group | Members |
|---|---|
| **WCB** | Stored Sequences, HCR Vocalizer, MP3 Trigger, DFPlayer Mini, Maestro, WLED Lighting |
| **AstroPixels** | General, Sound, PSI, Logics, Holo, Servo |

A board joins by **id prefix** (`wcb-`, `astropixels-`) *or* by an explicit **`ids`**
listing — either is enough. `ids` exists for the member a prefix can't reach:
`nc-maestro-wcb` (the `;M` sequence + servo verbs) is routed by the WCB exactly like the
rest of the group, but it is one of NaviCore's own seed boards so its id doesn't start
with `wcb-`. Its sibling `nc-maestro` is deliberately **out** — raw Pololu bytes to a
configured Maestro slot is a controller concern, not a WCB one.

Each group's `sub()` shortens a member's label inside it, since the shared part is now the
group header — `"WCB · HCR Vocalizer"` → `HCR Vocalizer`, `"Maestro (via WCB)"` →
`Maestro` (the qualifier only exists to tell it from the Pololu board outside, and the
group header already says WCB). Naming a new board
`WCB · <thing>` is what makes it fold in cleanly without touching `ids`.

Two ordering facts are easy to trip over:

- **A group renders at the position of its FIRST member**, so pinning a group means
  putting its members first *and contiguous* in `NC_CMDLIB_ORDER_TOP`. That is what puts
  WCB at the very top of the picker. A gap in that run — easy to introduce, since
  `nc-maestro-wcb` sits in the middle of it and does not look like it belongs — splits
  the cluster into two separate group rows. `_cmdlibNormalize()` does the sort, and the
  sort is stable, so unlisted boards keep their manifest order in the middle.
- **Order inside a group is that same array.** Stored Sequences leads the WCB group,
  because that is the one you reach for.

A search auto-expands every section with a match, so the extra level costs nothing when
you know what you are looking for.

### Live param sources

A param normally offers a fixed `enum` or a free-text box. It can instead declare a
**`source`** — a set of values that only exists on the droid, resolved at composer-render
time. One today:

| `source` | Field | Fed by |
|---|---|---|
| `wcb.sequences` | Dropdown of stored `?SEQ` keys, grouped by the board holding them, with a ⟳ that re-reads the mesh and a manual-entry escape — plus the chosen sequence's **contents** rendered underneath | `GET_WCB_SEQ` → `WCB_SEQ`, `GET_WCB_SEQVAL` → `WCB_SEQVAL` ([PROTOCOLS.md](PROTOCOLS.md#stored-sequences)) |

It is a `source` and not an `enum` because the values are not knowable to the library —
they are whatever the boards happen to store today. That also keeps the board file
portable: a consumer that does not implement the source sees an unknown extra field and
falls back to the plain text input, which is what makes it shippable upstream rather than
a NaviCore-only fork. Board files therefore keep the param's `pattern` as the validation
and fallback.

Things worth knowing before touching it:

- **The list is per board, and the destination is not chosen in the composer.** The action
  row picks where a command goes, so the picker offers every board's keys grouped by board
  rather than filtering to one — `;C<key>` is routinely broadcast, and a key that exists
  only on WCB 3 is still a valid broadcast.
- **The current value is always offered**, even when no board reports it. Re-opening the
  composer on an existing action must not rewrite its key just because that board is
  offline or the key was typed by hand.
- **Cached in memory only, keyed by board, validated against `seqHash`.** This is live
  mesh state, not a setting; a list that outlived the droid it came from would offer
  sequences that no longer exist. A board's hash moving invalidates its list
  (`_applyWcbSeqHash`), so an edit made in the Wizard shows up without polling.
- **Pulls are sequential** — `WCB_Client` allows one request in flight mesh-wide, across
  names *and* values, and rejects a second rather than queueing it. The name walk
  (`_wcbSeqRefresh`) goes board by board, and a value fetch waits out any pull already
  running. Both in-flight markers **self-clear on timeout**: they were cleared only by the
  reply handler, which never runs if the port closes mid-pull, and the stale marker then
  blocked every later pull for the rest of the session.
- **The chosen sequence's body is shown underneath**, one command per line with its
  `***` comment after it. `_seqValueToLines()` is a **port of the WCB Wizard's
  `seqValueToLines()`** and is deliberately behaviour-identical — a sequence has to read
  the same in both tools, and the Wizard is where it is authored. Keep them in step if
  either changes; a test asserts they agree line-for-line.
- **Which board's copy is not recoverable from the key.** The same key can exist on
  several boards with *different* contents, so each `<option>` carries `data-wcb`. When
  the composer re-opens on an existing action there is no option behind the value — the
  row stores only the wire string — so it falls back to the boards whose inventory holds
  that key, and says which one it is showing.
- **The delimiter is assumed to be `^`.** A WCB's is configurable but is not advertised
  over WDP, so an overridden one is not knowable here — the same assumption the tool's
  `^`-chain warning already makes.
- **Two `_cmdlibNormalize()` steps keep a library re-fetch from undoing this.** The
  vendored `wcb-sequences.json` carries the `source` field, and the six sequence verbs
  live there rather than in `wcb-native` — but "check online for a newer library" (and a
  user importing their own copy of the DroidNet files) replaces a board *wholesale*, from
  a snapshot that still has neither. So `_cmdlibApplySeqSource()` re-asserts the field,
  and `_cmdlibDropMovedCmds()` strips the six from `wcb-native` whenever `wcb-sequences`
  is present — without it every sequence command would list twice, and only one copy
  would have the picker. `_cmdlibDropMovedCmds()` is deliberately a no-op when the new
  board is absent, so it can never strand the commands. Delete both once the split and
  the field are upstream.

---

## 7. Timeline editor

Opens a recorded clip as an editable timeline: per-channel servo curves with draggable
keyframes and Photoshop-style tangent handles, discrete action markers, zoom/fit/trim,
RDP-based smoothing, easing insertion, undo history, and **live preview** that drives the
real servos to the cursor position via `?MAE` writes.

Transport is `?REC,EDITLOAD` down (`[CLIPDL:*]` lines) and
`EDITBEGIN` / `EDITEV,<idx>,<json>` / `EDITEND` up, with indexed ACKs so a timeout retry
cannot duplicate an event. It can also export a clip as Maestro **script source**.

---

## 8. Persistence in the browser

| `localStorage` key | Purpose |
|---|---|
| `rcDisplayUnit` | `sbus` \| `us` (labelled "PWM") |
| `rcTerminalAutoScroll`, `rcTerminalTimestamps` | Terminal prefs |
| `rcCalibAutoAdvance` | Calibration wizard |
| `rcConfigLastTab` | Reopen on the last tab |
| `rc_via_wcb_on` | Remember bridge mode |
| `nc_cmdlib_droid_sig` | Cached droid library signature |
| `rc_fw_branch` | **Dev only** — flash from a non-`main` branch |

**Nothing in browser storage may affect the configuration.** Every key above is a UI
preference or a cache. The droid is the only home for config: `/config.json` for the
`RcConfig` object and `/cmdlib.json` for the custom command library.

Two keys used to break that rule and were removed — `rc_maestro_channels` (imported Maestro
channel names and travel endpoints) and `navicore-wcb-profiles-v1` (pre-config WCB credential
profiles). Both had a one-time fold that wrote a browser copy *into* the config when the board
carried none. Both folds are gone and the keys are purged at startup by
`_purgeLegacyConfigKeys()`. The `_maestroCh` mirror still exists but is **in-memory only** —
`_syncMaestroChFromConfig()` rebuilds it from `config.maestros[]` on every load, so it is
derived from the droid and never writes back.

The one deliberate exception is the custom command library. It is homed on the droid in
`/cmdlib.json` (see the Revision log for 2026-07-31), but a browser copy is retained as the
edit buffer and as the store for **bridge mode** — auto-syncing the library over the WCB bridge
is a multi-KB fragment transfer that suppressed `rc_hb`/`rc_ch` for its whole duration and
took config, live channel and SBUS down with it. Library sync is Direct-USB only for that
reason; do not "fix" it by re-enabling the bridged pull.

---

## 9. Two extras worth knowing

**The `^`-chain warning.** A wire-command row shows a red line when the command is a `^`
chain, the destination is a single board, **and** at least one part starts with an
implicitly-routed verb (`;M` `;L` `;H` `;A` `;D` `;C`/`;SEQ` — the `IMPLICIT_ROUTED` regex).
Those are the only verbs the one-hop cap can drop; explicit `;w<n>` routing is not capped
and `;s`/`;P` never route, so `;w3;s4:PP100^;w3;s4:PL5` is correct and stays quiet. Full
rule in [PROTOCOLS.md §1](PROTOCOLS.md#1-transport-overview).

The check lives in `refreshChainWarn()` inside `_appendCommandView()` and re-runs on both
triggers that can change the answer — typing in the command box, and changing "Send to".
It keys on the hidden `-destsel` value (a bare board number means unicast), so it stays
correct as the destination dropdowns rewrite the action type. It is deliberately a
**warning, not an error**: a chain whose devices all live on the target board is valid, and
the tool cannot always know where a device is hosted. Authoring time is still the only
cheap place to catch this — the symptom in the field reads as "half my action works".

**Mesh Stats — config and view live apart, deliberately.** The **checkbox + Target WCB** are
*saved config* (`config.statsReport` → `rcConfig.statsReport`) and stay in **General**; the
numbers themselves are a *live diagnostic* that saves nothing and must never dirty the
config. They are shown in two places at two depths:

| Where | What | Function |
|---|---|---|
| **Sidebar**, under each WCB Status chip | A per-board line — `140 sent · 100% ack`, with `↻`/`✗` counts when non-zero — plus an all-links footer | `_meshStatsChipLine()`, `_meshStatsGlanceHtml()` |
| **📊 modal** (from the footer, or General) | Full per-board table, uptime, refresh, 5 s auto-poll | `renderMeshStats()` |

Every board gets numbers, not just unhealthy ones: an aggregate alone was too vague to act
on, and *which board* is the first thing you want to know. The modal adds the columns the
sidebar has no room for (retries, unguaranteed, recv, in-flight).

The sidebar stats are shown when the droid's own config says it uses them (`statsReport.enabled`), seeded on every config load — so they are there **from connect**, already covering everything since the board booted. The `stats` checkbox in the WCB Status header is a **session override** on top of that, for getting the compact list back without editing anything. Deliberately not persisted in localStorage: the config is where this belongs, so it travels with the droid and rides backups. The toggle gates the **poll** as well as the render, so a droid that does not use stats costs no mesh traffic.

Every peer gets a line, **including temporary ones** (a mgmt relay): they are real link targets whose sent/failed land in the aggregate, and hiding them once made the totals unexplainable. Only self is skipped. The modal adds a **"not currently listed"** row when the per-board rows do not sum to the aggregate — the library keeps counters for all 20 slots, so a board that drops off the roster would otherwise take its share of the totals with no row to hang it on.

`_meshStatsMergePage()` reassembles the bridged **paged** reply (see
[PROTOCOLS.md §2](PROTOCOLS.md#2-usb-serial-json-protocol)). It stages pages and promotes
only a contiguous set ending in `"last":1`, so a dropped page leaves the previous complete
snapshot up rather than rendering boards as "no traffic" that simply went missing.

**The badge wording is load-bearing.** These are *our* counters for *our* links: `⚠3` beside
WCB3 means "this board's link to WCB3", which could equally be our own radio. The section
reads **"This board's links"** and the tooltip is directional (`WCB 20 → WCB 3`), so nobody
power-cycles WCB3 over our antenna.

Polling rides the existing 3 s status timer at **1-in-5** (~15 s) rather than owning a
timer — these counters move slowly and a bridged poll competes with SBUS. The modal's 5 s
auto-poll is the "watching it right now" case and stops when the modal closes.

`renderMeshStats()` handles all three reply shapes: full rows, `pfilt` (only problem boards
survived the bridged shed), and no `peers` at all. Each says so — without that, a short list
reads as "every other board is idle" and a totals-only table as "no peer has any traffic".

**Cheat sheet QR.** The 📱 button publishes a generated cheat-sheet page through a live
Cloudflare Worker relay (source in [`tools/cheatsheet-relay-worker.js`](../tools/cheatsheet-relay-worker.js))
and shows a QR code for it.

**Export / Import.** The **Export** button (`exportConfigJson`) downloads the **complete**
config as JSON (`{navicore, savedAt, note, config}`) — every branch, including knob/servo
passthrough outputs. This is the real backup; the file interchanges with the cloud modal's
⤓ Download / 📂 Load. **Import** (`handleConfigImportFile`) sniffs the file: a leading `{`
takes the lossless JSON path (`_cfgExtractConfig` → `applyConfig`, left unsaved for review);
otherwise it falls back to the legacy **CSV** parser. `exportConfigCsv` still exists but is a
**partial, human-editable spreadsheet** export — it cannot represent the variable-length knob
output lists (per-mode passthrough), `peerEvent` actions, etc., so it is **not** a full backup.

**Cloud config backup.** There is **no visible button** — click the "NaviCore" wordmark
four times quickly to open it. A **username + password** pair (independent of the WCB
password) both addresses and encrypts your backups: the slot base is
`SHA-256(slotSalt | user | pw)` and the AES-GCM-256 key is `PBKDF2(user | pw)` under a
distinct salt (`_cfgSlotBase` / `_cfgKey`, ≈16733/16738). Each pair therefore sees only its
own rolling ring of 10 backups (`CFG_BACKUP_MAX`) in Cloudflare KV — anyone can keep their
own configs under their own credentials, and forgetting either orphans them. Each backup
carries an optional note; "remember on this device" stores the pair in `localStorage`
(`CFG_CREDS_LS`). Beyond restore, each row has a **⤓ Download** to a decrypted, readable
`{navicore, savedAt, note, config}` JSON file; the modal also has **⤒ Upload** (encrypt a
config file from disk straight into a cloud slot) and **📂 Load** (apply a config file into
the tool, left unsaved for review). It's a convenience copy, not a vault — keep Export files
too.

**WCB credential profiles.** On the WCB Network tab, save the current network credentials
(MAC octets, password, quantity, device id, channel) as a named profile and radio-toggle
between them — e.g. a *dev* mesh and an *in-droid* mesh — so one functional config can target
either. Stored **in the config** (`config.wcbProfiles` → firmware `rcConfig.wcbProfiles`, up to
`WCB_MAX_PROFILES` = 6), so they travel with the droid and ride along in cloud backup, the JSON
export, **and the CSV Export/Import** (`wcbProfile<N>_<field>` rows, incl. passwords). Selecting
a profile makes the WCB Network fields **edit that profile** — change a value and it's captured
back into the profile on the next profile switch or Save (`_snapshotLiveIntoSelectedProfile`);
editing the active identity that merely *matches* a profile (nothing selected) never rewrites
it. Loading a profile writes `config.wcbNetwork`, and because WCB Network changes apply over
**Direct USB only**, it still needs a Save over USB to switch the droid's mesh. Profiles saved
in the earlier localStorage build (`navicore-wcb-profiles-v1`) are auto-imported into the config
on the next Load (then Save to store them on the droid).

---

## 10. Adding a feature — checklist

1. Add the UI (HTML + CSS inline, matching the surrounding idiom).
2. Load it in `applyConfig()`; include it in the save path so the diff picks it up.
3. Route any new board reply in `handleBoardMessage()`.
4. If it crosses the mesh, check the 187-byte envelope budget and whether it needs
   fragmentation or bulk transfer.
5. Keep firmware/tool constant pairs in sync —
   [CONFIG_SCHEMA.md §6](CONFIG_SCHEMA.md#6-cross-file-invariants).
6. Syntax-check: `node C:\Users\ghulette\tools\jscheck.js config_tool/index.html`.
7. Push — the Pages workflow deploys `main` to `/config_tool` and any other branch to
   `/dev/<branch>/config_tool` automatically.
8. Update the wiki if the feature is user-visible.

---

## Revision log

Newest first. Add a row whenever a code change alters what this page describes — same commit
as the code. Page body stays present-tense; history lives here.

| Date | Commit | Change |
|---|---|---|
| 2026-08-19 | _(uncommitted)_ | `sendLine()` treats a `NetworkError`/`InvalidStateError` write failure as link loss and calls `handleLinkLost()`, so a dead port is detected even when the parked `read()` never rejects. |
| 2026-08-18 | _(uncommitted)_ | Browser storage no longer holds anything that affects the configuration. Removed `_foldLocalMaestroChIntoConfig()` and `_foldLegacyWcbProfiles()` — the two pre-config migrations that let a localStorage copy write into the config when the board carried none — and purge `rc_maestro_channels` / `navicore-wcb-profiles-v1` at startup. `_maestroCh` is now in-memory only, rebuilt from the config by `_syncMaestroChFromConfig()`. The custom command library keeps its browser copy deliberately: it is the edit buffer and the bridge-mode store, because a bridged library pull kills telemetry. |
| 2026-08-18 | _(uncommitted)_ | Calibration wizard: the manual-channel box is pre-populated, so it always outranked auto-detect and the "move the control, then Capture" path was unreachable. `renderCalibrationStep()` now records `calibrationState.manualSeed` and `captureCalibrationValue()` treats the box as an override only when the user changed it. New-peer action editor: `renderPeerEventEditor()` now calls `syncPeerEventFromDom()` first, so edits held only in the DOM survive a tab switch / profile load / modal reopen instead of being silently discarded. |
| 2026-08-17 | _(uncommitted)_ | Added `NC_CMDLIB_HIDDEN` and hid `wcb-native` (71 setup/config/routing/system/power verbs). The library builds ACTIONS — things a pilot fires from a transmitter switch — and none of those are: nobody binds a button to "set the hardware version" or "erase NVS", and the WCB Wizard is built for them. Hidden rather than deleted so the vendored snapshot stays byte-for-byte upstream apart from the sequence split, and so a re-fetch cannot bring it back; the board keeps its `NC_CMDLIB_ORDER_TOP` entry so removing the id restores its old slot. Known consequence: `ncDecodeCommand()` only searches present boards, so an action row already holding one of those commands still works but loses its 📚 hint and re-opens at the list. The WCB group is six members now, ending at WLED. |
| 2026-08-17 | _(uncommitted)_ | Added `NC_CMDLIB_HIDDEN` and hid `wcb-native` (71 setup/config/routing/system/power verbs). The library builds ACTIONS — things a pilot fires from a transmitter switch — and none of those are: nobody binds a button to "set the hardware version" or "erase NVS", and the WCB Wizard is built for them. Hidden rather than deleted so the vendored snapshot stays byte-for-byte upstream apart from the sequence split, and so a re-fetch cannot bring it back; the board keeps its `NC_CMDLIB_ORDER_TOP` entry so removing the id restores its old slot. Known consequence: `ncDecodeCommand()` only searches present boards, so an action row already holding one of those commands still works but loses its `📚` hint and re-opens at the list. The WCB group is six members now, ending at WLED. |
| 2026-08-17 | _(uncommitted)_ | The command library now shows **what a chosen sequence does**: its body is pulled with `GET_WCB_SEQVAL` and rendered under the field, one command per line with its `***` comment after it. `_seqValueToLines()` is a behaviour-identical port of the WCB Wizard's function of the same name (tested line-for-line against it) so a sequence reads the same in both tools. Each `<option>` now carries `data-wcb`, because the same key can exist on several boards with different contents and the board is not recoverable from the key alone. Bodies are cached per `<wcb>/<key>` and dropped when that board's `seqHash` moves — the fingerprint covers values, so an in-place edit that leaves the name list identical still invalidates. Also fixed: both in-flight markers now self-clear on timeout, where previously a port closed mid-pull left one set and blocked every later pull for the session. |
| 2026-08-17 | _(uncommitted)_ | Picker groups take an explicit **`ids`** list as well as an id prefix, so `nc-maestro-wcb` (the `;M` sequence + servo verbs) joins the **WCB** group despite being one of NaviCore's own seed boards. `nc-maestro` stays out — raw Pololu bytes to a configured Maestro slot is a controller concern, not a WCB one. `sub()` renders the member as plain `Maestro`, since the qualifier only exists to tell it from the Pololu board outside the group. Note the ordering trap this creates: `nc-maestro-wcb` now sits in the MIDDLE of the WCB run in `NC_CMDLIB_ORDER_TOP` and does not look like it belongs there, but moving it out of the run splits the cluster into two group rows. |
| 2026-08-17 | _(uncommitted)_ | Picker layout, so the sequence verbs are findable: the six stored-sequence commands split out of the 77-command `wcb-native` into their own vendored board (`wcb-sequences`, a pure move — that is the second local delta to the snapshot, see NOTICE.md), and all six `wcb-*` boards now fold into one **WCB** group. Two ordering facts are load-bearing and easy to trip over — a group renders at the position of its **first** member (so pinning WCB to the top means putting its members first *and contiguous* in `NC_CMDLIB_ORDER_TOP`), and order inside a group is that same array (Sequences leads, the 71-command native config trails). `NC_CMDLIB_ORDER_BOTTOM` is now empty. Added `_cmdlibDropMovedCmds()`: upstream still ships the six inside `wcb-native`, so a "check online" fetch re-adds them and every sequence command would list twice with only one copy carrying the live picker — it is a deliberate no-op when `wcb-sequences` is absent, so it can never strand them. |
| 2026-08-17 | _(uncommitted)_ | Command library: params can declare a **`source`** — values resolved from the droid at render time rather than a fixed `enum`. First one is `wcb.sequences`, a dropdown of the `?SEQ` keys the boards actually hold (grouped by board, ⟳ to re-read the mesh, manual-entry escape), fed by `GET_WCB_SEQ`. The vendored `wcb-native.json` carries it on `wcb.runSeq` / `wcb.runSeqLong` / `wcb.seqClear` — the snapshot's only local delta, and the shape of the upstream PR — so the note that it is *unmodified* no longer holds. `_cmdlibApplySeqSource()` re-asserts it after every merge because a fetch or import replaces that board wholesale and would otherwise silently drop the picker back to a text box; delete it once the field ships upstream. Lists are cached in memory only (live mesh state, not a setting) and invalidated by a board's `seqHash` moving. |
| 2026-08-13 | _(uncommitted)_ | Added `_releaseSerialOnUnload()` on `pagehide`: deasserts DTR/RTS, closes the port, and calls `sharedHub.leave()`. The Direct USB path had no teardown at all, so a refresh abandoned an open port. |
| 2026-08-13 | _(uncommitted)_ | Hovering a WCB Status chip now shows that board's serial-port map, from the WDP labels already cached in `wcbPortLabels` (`_wcbPortTooltip()`). Real WCBs only — a client device has no WCB ports. Distinguishes "nothing advertised yet" from "ports are empty". |
| 2026-08-13 | _(uncommitted)_ | Sidebar stats are now seeded from the droid config (`statsReport.enabled`) on every config load, so they appear **from connect** rather than needing a box ticked; the header checkbox is a session override and is no longer persisted in localStorage. |
| 2026-08-13 | _(uncommitted)_ | Sidebar stats put behind a `stats` checkbox in the WCB Status header, **off by default**, gating the background poll as well as the render so the default costs no mesh traffic. View-only state in `rcShowMeshStats`. Temporary peers now get a per-board line (their counters were in the aggregate with no row to explain them), and the modal shows a "not currently listed" remainder row so the columns always reconcile. |
| 2026-08-13 | _(uncommitted)_ | Mesh Stats split by depth: **per-board numbers under every chip** in the sidebar (not just a badge on unhealthy ones) plus an all-links footer, with the full table in a **📊 modal**. General keeps only the saved toggle + Target WCB. `_meshStatsMergePage()` reassembles the paged bridged reply. Polling folded into the 3 s status tick at 1-in-5 (~15 s). Per-board wording is directional so a link failure is not read as the remote board being broken. |
| 2026-08-12 | _(uncommitted)_ | Added the **Mesh Stats** section to the General tab: a saved `statsReport` toggle + Target WCB (the 30 s `;V` push) and a live `GET_MESH_STATS` readout with optional 5 s auto-poll. The live view saves nothing; the shared renderer notes when a bridged reply has shed its per-board rows. |
| 2026-08-12 | _(uncommitted)_ | Wire-command rows now warn (`refreshChainWarn()` in `_appendCommandView()`) when a `^`-chain is aimed at a **single** board **and** a part starts with an implicitly-routed verb (`IMPLICIT_ROUTED` = `;A` `;D` `;H` `;M` `;L` `;C`/`;SEQ`) — only those can be dropped by the WCB one-hop cap. Explicit `;w<n>` chains stay quiet. Re-evaluated on both the command text and the "Send to" destination. Indexed `_appendCommandView()` in the function map. |
| 2026-08-12 | _(uncommitted)_ | **Export** now downloads a **complete JSON backup** (was the lossy CSV, which silently dropped knob/servo passthrough outputs); **Import** auto-detects JSON vs legacy CSV. `exportConfigCsv` is retained as a partial spreadsheet export only. Added an 👁 show/hide toggle to the cloud-backup password field. The live monitor now **auto-re-subscribes** (re-sends `START_MONITOR` from `_sbusStaleTick` when the SBUS panel goes stale while a link is open), so it self-heals after a board reboot drops `wsMonitorActive`. |
| 2026-08-11 | _(uncommitted)_ | WCB profiles are now **edit-in-place**: selecting a profile makes the WCB Network fields edit that profile (captured back on switch/Save via `_snapshotLiveIntoSelectedProfile`), so changing a selected profile's password sticks to it instead of being lost. Profiles are also included in **CSV Export/Import** (`wcbProfile<N>_<field>` rows). |
| 2026-08-11 | _(uncommitted)_ | WCB profiles now live **in the config** (`config.wcbProfiles` ↔ firmware `rcConfig.wcbProfiles`, cap `WCB_MAX_PROFILES`=6) instead of browser localStorage — they travel with the droid + backups; legacy localStorage profiles auto-migrate on first Load. Also fixed a curly-quote (`”`) in the profile-select `querySelectorAll` that silently no-op'd the force-check. |
| 2026-08-11 | _(uncommitted)_ | Fixes: WCB-profile radios use `onclick` (not `onchange`) and force the clicked profile checked after the list rebuild, so selecting a profile whose creds match another's no longer snaps back. Mode Report drops in the `;V,MODE,{mode}` default whenever it is enabled with a blank command — on load too, not only when toggled on — so a config saved enabled-but-empty (firmware default template is empty) still shows the default. |
| 2026-08-11 | _(uncommitted)_ | Cloud backup: corrected the crypto model to the **username + password** pair (was still documented as WCB-password-derived); documented per-row **⤓ Download** to a decrypted config JSON, **⤒ Upload** a config file into a cloud slot, and **📂 Load** a config file into the tool. Added **WCB credential profiles** (dev / in-droid radio-toggle, `localStorage`, Direct-USB-only apply). |
| 2026-08-05 | _(uncommitted)_ | Separate `hcr` + `mp3` tabs merged into one **audio** tab (twelve tabs → eleven), with a stale-`rcConfigLastTab` remap; noted that NaviCore's own command-library boards live in `NC_CMDLIB_SEED`, never in the vendored MPL-2.0 snapshot, and that a param's `enum` is a string id. |
| 2026-08-04 | _(uncommitted)_ | Initial version. |
