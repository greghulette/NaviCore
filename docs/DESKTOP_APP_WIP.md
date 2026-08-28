# Desktop App / WiFi — change tracker

**Purpose: make this work cleanly removable.** It is being built on `main` rather than a
branch, so this page is the substitute for that branch — the record of every commit and every
touched site, kept current as the work proceeds.

**If you are here to rip it out, go to [§1](#1-how-to-remove-it).** Everything else is context.

Status: **in progress.** The SoftAP comes up and a laptop can associate. Nothing listens on it
yet — the comms server is not written, so the AP is currently an open door to an empty room.

Related: [CONFIG_SCHEMA.md](CONFIG_SCHEMA.md) · [CONFIG_TOOL.md](CONFIG_TOOL.md) ·
[TROUBLESHOOTING.md](TROUBLESHOOTING.md)

---

## 1. How to remove it

### The clean path

Every commit in this effort is listed in [§2](#2-commits) and touches nothing else. Newest first:

```bash
git revert --no-commit 2c0e788 327ae1d 5958d70
git commit -m "Remove the WiFi/desktop-app work"
```

Revert in that order (newest → oldest); the reverse conflicts, because each commit builds on the
last. **`a2b64b8` is NOT part of this work** — it is the fragment-pacing and channel-clamp fix
that happened to land in the same session. Do not revert it.

### If the commits have become tangled with later work

Remove by symbol instead. These names appear nowhere else in the tree, so a grep is exhaustive:

```
wifiEnabled   wifiSsid   wifiPassword          # firmware + tool
cfg-wifi-                                      # tool DOM ids
"wifi"  "wifien"                               # NVS keys
[WIFI]                                         # firmware boot log prefix
```

Per-site inventory is in [§3](#3-every-touched-site). Line numbers drift — the symbols do not.

### What removal leaves behind

- **NVS keys `wifien` and `wifi` stay on every board that ran this firmware.** Reverting the
  code does not erase them. They are inert once nothing reads them, and a `RESET_DEFAULTS` or a
  full wipe clears them. Harmless, but they exist.
- **`wifiEnabled` / `wifiSsid` / `wifiPassword` remain in saved config JSON and cloud backups**
  taken while this was live. Reverted firmware ignores unknown keys, so old backups still
  restore — they just carry three dead fields.
- **A board left with `wifiEnabled: true` keeps its AP until reflashed.** Reverting the source
  does nothing to a board in the field. Turn WiFi off *before* rolling back, or reflash it.

### One-line kill switch (no revert)

To disable everything without touching git, force the default off and ignore stored state —
`rc_config.h`, in `rcConfigDefaults()`:

```c
rcConfig.wifiEnabled = false;   // and delete the prefs.isKey("wifien") restore at ~2401
```

With the restore gone the flag can never become true, so the `if (rcConfig.wifiEnabled)` block
in `setup()` is dead and the AP cannot come up. Useful for bisecting whether WiFi is implicated
in some other symptom.

---

## 2. Commits

| Commit | What it did | Files |
|---|---|---|
| `5958d70` | `wifiEnabled` bool, default false, six sites. Hidden toggle in the cloud-backup modal (reusing the existing 4×-wordmark gesture rather than adding a second secret). **Inert** — nothing read the flag | `rc_config.h`, `config_tool/index.html`, `docs/CONFIG_SCHEMA.md`, `docs/CONFIG_TOOL.md` |
| `327ae1d` | `wifiSsid[33]` + `wifiPassword[64]` — the flag alone could not name or secure an AP. Own NVS key `wifi`. AP name/password fields beside the toggle. Still inert | `rc_config.h`, `config_tool/index.html`, `docs/CONFIG_SCHEMA.md`, `docs/CONFIG_TOOL.md` |
| `2c0e788` | **The flag became live.** `setup()` raises the SoftAP before `wcb->begin()`. Removed the now-false "No WiFi AP or web server" comment | `NaviCore.ino`, `config_tool/index.html`, `docs/CONFIG_SCHEMA.md`, `docs/TROUBLESHOOTING.md` |

`fw_version.h` also changes in each — that is the pre-commit DTG stamp, not part of this work.

---

## 3. Every touched site

### `rc_config.h` — the config field (all of it additive)

| Area | What |
|---|---|
| struct `RcConfig` | `wifiEnabled`, `wifiSsid[33]`, `wifiPassword[64]` |
| `rcConfigDefaults()` | all three zeroed / false |
| `rcConfigToJSON()` | three `doc[...]` writes |
| `rcConfigFromJSON()` | three reads; `wifiEnabled` uses `\| false` so an older config can never enable the radio |
| NVS save | `putBool("wifien")` + a `"wifi"` JSON blob (`ssid`, `pw`) |
| NVS load | `isKey("wifien")` restore + `isKey("wifi")` blob restore |

Modelled site-for-site on `sbusOutEnabled` — diff the two if anything looks out of place.

### `NaviCore.ino` — the bring-up (one block)

One `if (rcConfig.wifiEnabled) { … }` in `setup()`, immediately before `wcb = new WCB_Client(...)`.
Deleting the block restores the previous behaviour exactly; nothing else in the file references it.

Three properties are load-bearing, and matter to anyone moving rather than deleting this:

1. **It must run before `wcb->begin()`.** `WCB_Client` checks `WiFi.getMode()` on entry and,
   finding an AP up, selects `WIFI_AP_STA` and keeps it instead of forcing `WIFI_STA` and tearing
   the AP down (`WCB_Client.cpp` ≈89-110).
2. **The channel is passed explicitly** from `wcbNetwork.channel`. `softAP()`'s 3rd parameter
   defaults to 1, and once an AP owns the radio `WCB_Client` only *warns* on a mismatch — so a
   defaulted channel against a mesh on any other channel is a silent total blackout.
3. **It fails closed.** Empty or <8-char password refuses to start rather than falling back to an
   open AP. This command surface has no per-command auth (`REBOOT`, `RESET_DEFAULTS` dispatch on a
   bare `type`), so an open AP is an unauthenticated command channel to the whole mesh.

### `config_tool/index.html` — three clusters

| Cluster | What |
|---|---|
| default `config` object | `wifiEnabled` / `wifiSsid` / `wifiPassword` |
| `applyConfig()` | three reads from the board's `CONFIG` |
| `cfgOpenCloudModal()` | the hidden block: markup (`cfg-wifi-enable`, `-ssid`, `-pw`, `-pw-eye`, `-note`), the `_wifiNote()` helper, and four listeners |

No existing function was modified beyond those insertions — no transport code, no save path, no
message dispatch. `_diffConfigBranches()` picks the fields up generically; nothing was special-cased
for them.

### Deliberately NOT touched

- **The CSV cheat-sheet export.** A user-facing artifact; listing the field there would defeat the
  hiding. Consequence: a CSV round-trip does not carry the WiFi settings.
- **The Via-WCB strip in `saveConfigToBoard()`.** It strips `wcbNetwork` transport fields only, so
  WiFi settings save over either transport. Deliberate — they do not define the transport in use.
- **The ArduinoJson filter whitelist.** Not needed: `SET_CONFIG` deserialises un-filtered
  (`NaviCore.ino` ≈3725). The whitelist governs fields read by *non*-`SET_CONFIG` handlers.

---

## 4. Verified vs assumed

| | |
|---|---|
| **Verified on hardware** | Save → NVS → boot → `GET_CONFIG` round-trip of all three fields. SoftAP comes up; a laptop associates |
| **Verified by compiler only** | Everything else. 1,128,539 B, 57% of the 1,966,080 B app slot |
| **NOT verified** | **ESP-NOW surviving alongside the AP.** The `WCB_Client` coexistence path (`WIFI_AP_STA`, `WIFI_PS_NONE`, channel deferral) has never been exercised by anything in this ecosystem. Confirm `WCB<n> ONLINE` and the roll call still land with WiFi on |
| **Known gap** | Nothing listens on the AP. The comms server is unwritten |

---

## 5. Decisions worth not relitigating

- **Toggle lives in the cloud-backup modal** — that modal is already behind the 4×-wordmark
  gesture, so this inherits an existing hiding place instead of adding a second secret.
- **Hiding is obscurity, not security.** The tool is public and readable. The real gate is
  `wifiEnabled` defaulting false with the bring-up unreachable while it is. Corollary: a droid
  *reporting* WiFi on should show that in visible chrome even to a user who cannot see the switch —
  hide the control, never the state.
- **Never reuse `wcbNetwork.password` as the AP password.** It rides in cleartext in every ESP-NOW
  packet the droid emits; it is public by construction. Hence the separate NVS key.
- **Empty SSID derives `NaviCore-<deviceId>`** so a droid always has a distinguishable name.

---

## 6. Still to come

Each will be added to [§2](#2-commits) as it lands.

1. **The comms server** — `esp_http_server` (ships in the pinned core, `CONFIG_HTTPD_WS_SUPPORT=y`,
   measured +32.8 KB flash / +8 B static RAM). Gated on the same flag.
2. **A firmware transport seam.** `handleSerialInput()` (≈3692-4172) reads `Serial` directly and has
   no line-level entry point, so a WebSocket cannot reuse its dispatcher. Extracting
   `processInputLine(const String&)` is the fix — but it modifies the primary input path rather than
   adding to it, which makes it the *first* change in this effort that is not cleanly revertible.
   Land it as its own commit, separate from anything else, for exactly that reason.
3. **The desktop app** — Python (reusing the ESP-Flasher-Companion pipeline), talking to a local
   WebSocket so serial and board-WiFi are one code path in the page.

---

## Revision log

| Date | Commit | Change |
|---|---|---|
| 2026-08-28 | _(uncommitted)_ | Created. Tracks the WiFi/desktop-app work on `main` in place of a feature branch: revert recipe, symbol-level inventory for when commits get tangled, what removal leaves behind (NVS keys, backup fields, boards already in the field), and the one-line kill switch. Records that the ESP-NOW-alongside-AP path is still unverified, and flags the coming `handleSerialInput()` extraction as the first non-additive change. |
