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

> **Known state: a local compile of current `main` fails.** The sketchbook's `WCB_Client`
> is **1.10.0** and has no `setPortLabel()`, which `rcAdvertiseSerialLabels()`
> ([`NaviCore.ino`](../NaviCore.ino) ≈ L3180) calls — that needs **≥ 1.11.0**. Refresh the
> sketchbook copy from `greghulette/WCBClient` master before trusting a local build:
>
> ```bash
> git -C C:/Users/ghulette/Documents/GitHub/WCBClient pull
> # then mirror src/ into Arduino-Code/libraries/WCB_Client
> ```
>
> Until that is done, **CI is the authoritative compile** — push and watch the workflow.

Every FQBN field is load-bearing:

| Field | Why |
|---|---|
| `USBMode=hwcdc,CDCOnBoot=cdc` | `rc_serial.h` `#error`s without it; the OTG port needs it |
| `PSRAM=opi` | Without it `ps_calloc` returns null and the board halts on a red LED |
| `PartitionScheme=custom` | Uses [`partitions.csv`](../partitions.csv) — the 12 MB `clips` partition |
| `FlashSize=16M` | The `clips` partition starts at 0x400000 |

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
| `_ESP32S3_boot.bin` | `0x0` | Second-stage bootloader |
| `_ESP32S3_part.bin` | `0x8000` | Partition table |

The flasher matches by **suffix**, so the version prefix can change freely.

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
| **⬆ Update Firmware** | Routine update. NVS at `0x9000` untouched; auto-detects whether app-only is safe or a full bootloader+partition+app write is needed |
| **⚠ Full Wipe & Flash** | First-time programming or recovery. Also erases NVS (`0x9000`, 20 KB) and OTA data (`0xE000`, 8 KB). **Erases all saved settings** |

A serial app-flash preserves `/config.json` (it lives in LittleFS, not NVS). Blank boards
need the full set including the 16 MB custom bootloader.

**OTA.** `?OTALOCAL,*` over USB, or `?OTA,*` relayed through a tethered board over the mesh
(windowed/pipelined, roughly 3 minutes per MB).

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
| 2026-08-04 | _(uncommitted)_ | Initial version. |
