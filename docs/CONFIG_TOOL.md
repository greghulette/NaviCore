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

Link loss (sleep, unplug) is caught by `handleLinkLost()` → `tryAutoReconnect()`, gated on
`_wantAutoReconnect` so a deliberate disconnect never re-opens the port.

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
| Mesh status | `renderWcbStatus()`, `startWcbStatusPoll()`, `_maybeRequestWcbMeta()` |
| Action rows | `buildActionRow()`, `renderArgs()`, `_renderMaestroActionArgs()`, `renderHcrParamFields()`, `renderMp3ArgField()` |
| Command library | `ncCommandLibrary()`, `ncEncodeCommand()`, `ncDecodeCommand()`, `_cmdlibDestFieldHtml()` |
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
                                            RoamADome, UppitySpinner, AstroPixels, …)
  navicore/manifest.json  + navicore.json   NaviCore-native verbs (record / play / stop)
```

`cmdlib/droidnet/` is a **vendored, unmodified MPL-2.0 snapshot** — NaviCore's own boards
never go in there. They live inline in `NC_CMDLIB_SEED` in `index.html`; `wcb-dfplayer`
(the `;D` verb set) is one of them. A param's `enum` field is a **string id** into the
library-level `enums` map, not an inline list.

Each command declares `id, name, safety, encoder, template, params[], examples,
commentLabel, category`, plus routing metadata (`class`, `nativeWrapper`,
`durationSuffix`). `ncEncodeCommand()` renders a template into a wire string;
`ncDecodeCommand()` reverses it so an existing action re-opens in the picker instead of
appearing as opaque text.

Users can add **private boards**, stored in `localStorage` and pushable to the droid's
`/cmdlib.json` — small libraries over `SET_CMDLIB`, large ones over the bulk-transfer path
(`_bulk*`). The droid stores the JSON opaquely; only size + FNV-1a hash are interpreted, so
the tool can skip re-pulling an unchanged library.

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
| `rc_maestro_channels` | Imported Maestro channel metadata (also folded into the config) |
| `nc_cmdlib_droid_sig` | Cached droid library signature |
| `rc_fw_branch` | **Dev only** — flash from a non-`main` branch |

---

## 9. Two extras worth knowing

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
| 2026-08-12 | _(uncommitted)_ | **Export** now downloads a **complete JSON backup** (was the lossy CSV, which silently dropped knob/servo passthrough outputs); **Import** auto-detects JSON vs legacy CSV. `exportConfigCsv` is retained as a partial spreadsheet export only. Added an 👁 show/hide toggle to the cloud-backup password field. The live monitor now **auto-re-subscribes** (re-sends `START_MONITOR` from `_sbusStaleTick` when the SBUS panel goes stale while a link is open), so it self-heals after a board reboot drops `wsMonitorActive`. |
| 2026-08-11 | _(uncommitted)_ | WCB profiles are now **edit-in-place**: selecting a profile makes the WCB Network fields edit that profile (captured back on switch/Save via `_snapshotLiveIntoSelectedProfile`), so changing a selected profile's password sticks to it instead of being lost. Profiles are also included in **CSV Export/Import** (`wcbProfile<N>_<field>` rows). |
| 2026-08-11 | _(uncommitted)_ | WCB profiles now live **in the config** (`config.wcbProfiles` ↔ firmware `rcConfig.wcbProfiles`, cap `WCB_MAX_PROFILES`=6) instead of browser localStorage — they travel with the droid + backups; legacy localStorage profiles auto-migrate on first Load. Also fixed a curly-quote (`”`) in the profile-select `querySelectorAll` that silently no-op'd the force-check. |
| 2026-08-11 | _(uncommitted)_ | Fixes: WCB-profile radios use `onclick` (not `onchange`) and force the clicked profile checked after the list rebuild, so selecting a profile whose creds match another's no longer snaps back. Mode Report drops in the `;V,MODE,{mode}` default whenever it is enabled with a blank command — on load too, not only when toggled on — so a config saved enabled-but-empty (firmware default template is empty) still shows the default. |
| 2026-08-11 | _(uncommitted)_ | Cloud backup: corrected the crypto model to the **username + password** pair (was still documented as WCB-password-derived); documented per-row **⤓ Download** to a decrypted config JSON, **⤒ Upload** a config file into a cloud slot, and **📂 Load** a config file into the tool. Added **WCB credential profiles** (dev / in-droid radio-toggle, `localStorage`, Direct-USB-only apply). |
| 2026-08-05 | _(uncommitted)_ | Separate `hcr` + `mp3` tabs merged into one **audio** tab (twelve tabs → eleven), with a stale-`rcConfigLastTab` remap; noted that NaviCore's own command-library boards live in `NC_CMDLIB_SEED`, never in the vendored MPL-2.0 snapshot, and that a param's `enum` is a string id. |
| 2026-08-04 | _(uncommitted)_ | Initial version. |
