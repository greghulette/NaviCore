# NaviCore — Build, Verify, Release

How a change gets from an edit to a flashed board, and how to verify work before pushing.

Related: [ARCHITECTURE.md](ARCHITECTURE.md) · [CONFIG_TOOL.md](CONFIG_TOOL.md)

---

## 1. What ships where

| Change to | Ships via | Lands as |
|---|---|---|
| `*.ino` / `*.h` | `.github/workflows/build-firmware.yml` | `firmware/*.bin` auto-committed to the branch |
| `config_tool/**` | `.github/workflows/pages-deploy.yml` | `gh-pages:/config_tool` (main) or `gh-pages:/dev/<branch>/config_tool` |
| `WCB_Client` / `WcbCmd` library | push to **their own repos** | pulled fresh at CI build time |
| Wiki | separate repo, branch `master` | published immediately on push |

---

## 2. Dependencies

| Library | Source | Pinned |
|---|---|---|
| `ArduinoJson` | Library Manager | **7.4.3** |
| `EspSoftwareSerial` | Library Manager | 8.1.0 |
| `Adafruit NeoPixel` | Library Manager | 1.15.4 |
| `PololuMaestro` | Library Manager | floating |
| `WCB_Client` (+ `WCBStream`) | `git-url https://github.com/greghulette/WCBClient.git` | **master, unpinned** |
| `WcbCmd` | `git-url https://github.com/greghulette/WcbCmd.git` | **master, unpinned** |
| ESP32 core | `esp32:esp32@3.3.4` | pinned; bump in lock-step with the WCB repo |

`HCRVocalizer` is deliberately **not** a dependency — NaviCore formats HCR byte strings
itself in `hcrFormatCommand()`.

### The library-source rule

> **A `WCB_Client` change only reaches a deployed NaviCore build once it is pushed to
> `greghulette/WCBClient` master.**

CI runs `arduino-cli lib install --git-url …` and compiles with no `--libraries` override,
so it uses that clone. The copy at
`C:\Users\ghulette\Documents\GitHub\Arduino-Code\libraries\WCB_Client` is the **local
sketchbook** — it drives local bench builds only and is a dead end for CI. Nothing enforces
that the two stay in sync; they silently diverge once either is edited.

A library change therefore ships as: **push WCBClient master → push NaviCore → CI clones the
updated library → new `.bin` → flash.**

---

## 3. Verifying a firmware change

The compiler is the only real check — there are no firmware unit tests.

**Local compile** (`arduino-cli` 1.5.1 is installed; sketchbook is
`C:\Users\ghulette\Documents\GitHub\Arduino-Code`):

```bash
arduino-cli compile \
  --fqbn "esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=cdc,PartitionScheme=custom,FlashSize=16M,PSRAM=opi" \
  NaviCore.ino
```

> **The sketchbook's `WCB_Client` shadows the real one** in every local compile, while CI
> builds against `greghulette/WCBClient` master. A copy that lags can fail the compile or,
> worse, pass against old library code. Before trusting a local build, this must print
> nothing:
>
> ```bash
> git -C C:/Users/ghulette/Documents/GitHub/WCBClient pull
> diff -rq C:/Users/ghulette/Documents/GitHub/Arduino-Code/libraries/WCB_Client/src \
>          C:/Users/ghulette/Documents/GitHub/WCBClient/src
> ```
>
> If it prints anything, mirror `src/` across first, or treat **CI as the authoritative
> compile** — push and watch the workflow.

Every FQBN field is load-bearing:

| Field | Why |
|---|---|
| `USBMode=hwcdc,CDCOnBoot=cdc` | `rc_serial.h` `#error`s without it; the OTG port needs it |
| `PSRAM=opi` | Without it `ps_calloc` returns null and the board halts on a red LED |
| `PartitionScheme=custom` | Uses [`partitions.csv`](../partitions.csv) — the 12 MB `clips` partition |
| `FlashSize=16M` | The `clips` partition starts at 0x400000 |

**The HIL hook build.** The WCB repo's hardware-in-the-loop harness builds a test image with
the same FQBN plus one define:

```bash
arduino-cli compile --fqbn "<the FQBN above>" --build-path <a private folder> \
  --build-property "compiler.cpp.extra_flags=-DNAVICORE_HIL_HOOKS=1" NaviCore.ino
```

(`compiler.cpp.extra_flags` is empty by default in the esp32 3.3.4 recipe, so nothing else
changes.) That compiles in [`navicore_hil.h`](../navicore_hil.h): `DBG_WIRE` and the
`#L90`–`#L93` fault verbs ([PROTOCOLS.md §3](PROTOCOLS.md#hil-test-hooks-navicore_hil_hooks-builds-only)).
They are fault injectors — a corrupted config file, a failed save — so **CI, the release bins
and the flasher never define it**, and without it the image is byte-for-byte what it would be
if the hooks did not exist: every hook site sits inside `#ifdef NAVICORE_HIL_HOOKS`, and
`HIL_TAP(port)` is the port itself. A hook image names itself in its boot banner
(`[HIL] NAVICORE_HIL_HOOKS build: …`) and knows `#L90`; any other answers
`Unknown #L code 90`. The harness builds both through `tests/hil/hil/ncflash.py`
(`build(tag, hooks=True)`), whose `BUILD.json` records which.

**Config-tool check** (no compiler, so this is the substitute):

```
node C:\Users\ghulette\tools\jscheck.js config_tool/index.html
```

---

## 4. Versioning

[`fw_version.h`](../fw_version.h) is the single source of truth:

```c
#define FW_VERSION_BASE  "v0.2.0"          // bump BY HAND for a release
#define FW_VERSION_DTG   "040901QAUG26"    // stamped automatically — do not edit
#define FW_VERSION       FW_VERSION_BASE "_" FW_VERSION_DTG
```

`tools/git-hooks/pre-commit` stamps the DTG on every commit, into **two** places that must
stay in lock-step: `FW_VERSION_DTG` and the `#footer-dtg` span in the config tool.

- Format: `DDHHMM<TZ>MMMYY` (e.g. `211520QMAY26`) — colon-free because it goes into `.bin`
  filenames, and `:` is illegal in Windows paths. The UI footer uses the readable
  `DD.HH:MM.TZ.MMM.YYYY` variant.
- POSIX `/bin/sh`, so GitHub Desktop on Windows can run it without WSL.
- Non-blocking: a failure still lets the commit through.
- Activate per clone: `git config core.hooksPath tools/git-hooks` (already set in this one).

**The version names a commit, not an image.** Every build of one commit — or of an uncommitted
tree on top of it — reports the same `FW_VERSION`. The image itself is named by
`App SHA256: <16 hex>`, which the board prints in its boot banner and in `?OTALOCAL,STATUS`: the
first 8 bytes of the build's ELF SHA-256, the value elf2image stamps at image offset `0xB0`. Match
it against `sha256sum NaviCore.ino.elf` of a build to know which one a board runs, and decode a
backtrace only with the `.elf` whose hash it starts — `addr2line` against any other build prints
plausible, wrong function names.

---

## 5. Firmware CI

Triggers on a push touching `**.ino`, `**.h`, `**.cpp`, `tools/build-firmware.sh`, or the
workflow itself — on **any** branch. `fw_version.h` is excluded (the hook stamps it every
commit, so including it would rebuild on every commit). The auto-commit carries
`[skip ci]`. A bare version-base bump needs a manual `workflow_dispatch` run.

The commit step uses `git add -A firmware/` — the build deletes prior DTG-tagged bins, and a
bare glob would stage only additions, leaving stale bins that the flasher's alphabetical
`.find()` would then lock onto. Push is rebase-and-retry up to 5 times, because a commit
landing during the ~2-minute build must not silently drop the binaries.

Three bins per build:

| Suffix | Flash address | Contents |
|---|---|---|
| `_ESP32S3.bin` | `0x10000` | Application (`ota_0`) |
| `_ESP32S3_boot.bin` | *not flashed* | Per-build bootloader artifact — see below |
| `_ESP32S3_part.bin` | `0x8000` | Partition table, built from [`partitions.csv`](../partitions.csv) |

The flasher finds the app by its `_ESP32S3.bin` **suffix**, then requires a `_part.bin` carrying the
IDENTICAL version prefix — an app and a table from two different builds can never be paired (a table
without the `clips` row flashes silently and only shows up later as an unmounted clips FS).

**The bootloader at `0x0` is not the per-build `_ESP32S3_boot.bin`.** `flasher.js` fetches
`firmware/WCB_S3_custom_bootloader_16MB_wdt3s.bin` by that **fixed** name — the custom
short-WDT 16 MB bootloader (cold-boot auto-retry), the matched pair of the in-app boot guard.
The name is fixed precisely so a per-build `_boot.bin` can never shadow it: `build-firmware.ps1`
copies the custom bootloader under that name while `build-firmware.sh` (the CI path) copies
arduino-cli's stock one, and neither is ever written to a board. See
[firmware/README.md](../firmware/README.md).

---

## 6. Pages CI

Every branch except `gh-pages` and `claude/**` deploys when `config_tool/**` changes. Each
run rewrites only its own slice of `gh-pages`, so production and branch previews coexist:

```
https://greghulette.github.io/NaviCore/config_tool/                 (main)
https://greghulette.github.io/NaviCore/dev/<branch>/config_tool/    (preview)
```

Publishing here is what puts the tool on the **same origin** as the WCB Wizard, which is the
precondition for `WcbSerialHub`'s cross-tab port sharing. The workflow also discovers
`../Images/<file>` references dynamically and deploys only those images.

---

## 7. Flashing a board

**In-browser (Config → Firmware).** Reads `firmware/` on `main` through the GitHub Contents
API, so a fresh CI build is available to users the moment the workflow finishes.

| Button | Effect |
|---|---|
| **⬆ Update Firmware** | Routine update. NVS at `0x9000` untouched. Writes bootloader + partition table + app **unconditionally** — there is deliberately no read-back to decide app-only, because `readFlash()` over the S3's native USB wedges the esptool stub and times out the *next* write |
| **⚠ Full Wipe & Flash** | First-time programming or recovery. Also erases NVS (`0x9000`, 20 KB: learned mesh peers, the legacy pre-LittleFS config copy) and OTA data (`0xE000`, 8 KB). **Does not erase `/config.json`**, the command library or the clips: neither button writes the LittleFS partitions. Resetting the config is Restore Defaults then Save. The tool's texts must say so (D-NC34) |

A serial app-flash preserves `/config.json` (it lives in LittleFS, not NVS). Blank boards
need the full set including the 16 MB custom bootloader.

From a live session, the tool fetches everything the flash needs from the network first —
esptool-js and CryptoJS, and the image set it checks (`prepareFirmwareFlash()` in
`flasher.js`) — and only then releases the port to esptool-js. A refusal at that stage (an
incomplete set on GitHub, an unreachable CDN) leaves the session connected; only a completed
flash reconnects on its own, so a refusal after the teardown would strand the user
disconnected from a board nothing had touched.

**OTA.** `?OTALOCAL,*` over USB, or `?OTA,*` relayed through a tethered board over the mesh
(windowed/pipelined, roughly 3 minutes per MB). After the restart, `?OTALOCAL,STATUS` confirms
the flash: `Running` is the slot that was `Next`, and `App SHA256` starts the new build's ELF
SHA-256 (§4).

**Offline builds.** `tools/build-firmware.ps1` (Windows) or `tools/build-firmware.sh` — same
FQBN and pruning logic as CI; you commit and push the bins yourself. The Arduino IDE also
works: ESP32S3 Dev Module, custom partition scheme, **PSRAM: OPI PSRAM**, then Export
Compiled Binary and rename to the three suffixes.

---

## 8. Git conventions

- Remote `origin` = `https://github.com/greghulette/NaviCore.git`, default branch `main`.
- Firmware and tool binaries are committed intentionally: `.gitignore` blocks `*.bin` but
  re-allows `firmware/*.bin` and `model/*.bin`.
- **Pushing is outward-facing** — it triggers CI, republishes the public tool, and can
  change what users flash. Confirm before pushing unless already told to proceed.
- Wiki pushes are separate and equally outward-facing:
  `C:\Users\ghulette\Documents\GitHub\NaviCore.wiki`, branch **`master`**.

---

## Revision log

Newest first. Add a row whenever a code change alters what this page describes — same commit
as the code. Page body stays present-tense; history lives here.

| Date | Commit | Change |
|---|---|---|
| 2026-10-04 | _(pending)_ | §7: a flash from a live session fetches the flash tool and the image set before the teardown (`prepareFirmwareFlash()`), so a refused set no longer leaves the session disconnected (`nctool.fw_refused_flash_keeps_session`). |
| 2026-10-04 | _(pending)_ | §7 Full Wipe row corrected: it erases NVS and OTA data only — `/config.json`, the command library and the clips survive, and the tool's texts that promised the config is erased now say so (D-NC34). |
| 2026-09-28 | `703a0e7` | §3: **the HIL hook build** — `-DNAVICORE_HIL_HOOKS=1` through `compiler.cpp.extra_flags` compiles in `navicore_hil.h` (`DBG_WIRE`, `#L90`–`#L93`); CI and releases never define it, and without it the image is the same bytes as with no hook code at all (checked: 71 differing bytes against the pre-hook tree, all version stamp, compile time and hashes). |
| 2026-09-28 | `1e15601` | §4: the version names a commit, not an image — `App SHA256` (boot banner, `?OTALOCAL,STATUS`) names the image and the `.elf` that decodes its backtraces. §7: how STATUS confirms an OTA. |
| 2026-09-10 | _(uncommitted)_ | §3: **a local compile of `main` passes again** (1,169,087 B). The sketchbook's `WCB_Client` now matches `greghulette/WCBClient` master — the `diff -rq` is empty. The callout recorded a *state* ("a local compile of current `main` fails"), which went stale without anything saying so; it now gives the check instead. CLAUDE.md's build note likewise. |
| 2026-08-18 | _(uncommitted)_ | §5/§7 corrected against `flasher.js`: the per-build `_ESP32S3_boot.bin` is **never flashed** — the flasher writes the fixed-name `WCB_S3_custom_bootloader_16MB_wdt3s.bin` at `0x0`, which is why that name is fixed (a per-build `_boot.bin` must not shadow it) — and **Update Firmware** writes bootloader + partition table + app unconditionally rather than auto-detecting, because reading flash back over the S3's native USB wedges the esptool stub. Also recorded the app/table version pairing rule. |
| 2026-08-04 | _(uncommitted)_ | Initial version. |
