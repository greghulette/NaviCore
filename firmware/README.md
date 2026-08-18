# Firmware Binaries

This folder hosts the compiled NaviCore firmware for the **WCB v3.2**
hardware (ESP32-S3). The browser-based flasher in `config_tool/index.html`
(Config → Firmware tab) pulls the latest set from here on `main` via the
GitHub Contents API.

## Versioning

The firmware version is stored in `fw_version.h` at the repo root:

```c
#define FW_VERSION_BASE  "v0.2.0"         // bump by hand for releases
#define FW_VERSION_DTG   "211520QMAY26"   // auto-stamped by pre-commit hook
#define FW_VERSION       FW_VERSION_BASE "_" FW_VERSION_DTG
```

- **`FW_VERSION_BASE`** — semver version (MAJOR.MINOR.PATCH).  Edit by hand
  when cutting a new release: `v0.2.0` → `v0.2.1` → `v0.3.0` → `v1.0.0`.
  The build scripts treat the field as opaque — any quoted string works,
  so two-digit forms (`v0.1`) still parse if you need them.
- **`FW_VERSION_DTG`** — stamped automatically by `tools/git-hooks/pre-commit`
  on every commit, same DTG format as the UI footer in `config_tool/index.html`.
  Both files always reflect the same commit time.
- **`FW_VERSION`** — convenience macro the firmware uses to report itself
  in the `PONG` reply and at boot.

The build script reads both `#define`s and embeds them in the bin filename
(e.g. `NaviCore_v0.2.0_211520QMAY26_ESP32S3.bin`), so the file on disk,
the file's reported version at runtime, and the source header all match.

## File naming

Three files per release, all ending with these stable suffixes:

| File suffix             | Flash address | What it is                |
|-------------------------|---------------|---------------------------|
| `_ESP32S3.bin`          | `0x10000`     | Application image (`ota_0`)|
| `_ESP32S3_boot.bin`     | *not flashed* | Per-build bootloader artifact (see below)|
| `_ESP32S3_part.bin`     | `0x8000`      | Partition table (from `partitions.csv`)|

The bootloader written at `0x0` is **not** the per-build `_ESP32S3_boot.bin`.
`config_tool/flasher.js` fetches `firmware/WCB_S3_custom_bootloader_16MB_wdt3s.bin`
by that FIXED name — the custom short-WDT 16 MB bootloader (cold-boot auto-retry),
the matched pair of the in-app boot guard in `NaviCore.ino`.  The name is fixed
precisely so a per-build `_ESP32S3_boot.bin` can never shadow it.  What that
per-build file actually contains differs by builder — `tools/build-firmware.ps1`
copies the custom bootloader under that name, while `tools/build-firmware.sh`
(the CI path) copies arduino-cli's stock `NaviCore.ino.bootloader.bin` — but
neither is ever flashed, so changing it changes nothing on the board.

The flasher matches by **suffix**, so the version prefix can change every
build without touching the page. Example set:

```
NaviCore_201500RMAY26_ESP32S3.bin
NaviCore_201500RMAY26_ESP32S3_boot.bin
NaviCore_201500RMAY26_ESP32S3_part.bin
```

The Config → Firmware tab offers two buttons:

- **⬆ Update Firmware** — routine update. NVS at `0x9000` is never touched,
  so the user's saved configuration survives.  The flasher ALWAYS writes
  bootloader + partition table + app — it never reads the flash back to
  decide whether app-only would do.  That read is slow and flaky over the
  S3's native USB, and when it stalls it wedges the esptool stub so the
  *following* write times out (observed in the field).  Boot + partition
  table are ~23 KB next to the ~1 MB app, so always writing them is nearly
  free and works on blank and programmed boards alike.  See the comment at
  `flashFirmware()` step 3b in `config_tool/flasher.js`.
- **⚠ Full Wipe & Flash** — initial push / recovery.  Writes the full image
  AND erases NVS (`0x9000`, 20 KB) and OTA data (`0xE000`, 8 KB), returning
  the board to factory-fresh state.  Use this for first-time programming on
  a new board, or to recover from corrupted config / bad partition state.
  **Erases all saved settings.**

## How to update the binaries

**Default: GitHub Actions does it automatically.**  Every push to any
branch triggers `.github/workflows/build-firmware.yml`, which runs
`tools/build-firmware.sh` — compiling `NaviCore.ino` for ESP32-S3 with
`PartitionScheme=custom,FlashSize=16M` (the table in `partitions.csv`),
reading `FW_VERSION_BASE` + `FW_VERSION_DTG` out of `fw_version.h`, and
committing the three resulting bins back to `firmware/` (overwriting older
versioned files) under an auto-commit tagged `[skip ci]`.  Once that
commit lands on `main`, the Config → Firmware tab can flash any
connected board.

The two manual options below remain available for offline work or for
producing a one-off bin without going through CI.

### Option A — Arduino IDE (zero scripting)

1. Open `NaviCore.ino` in Arduino IDE.
2. Tools → Board → **ESP32S3 Dev Module** (or whatever you normally use for WCB v3.2).
3. Tools → Partition Scheme → **Custom** — uses `partitions.csv` from the
   sketch folder.  Do **not** pick "Minimal SPIFFS": rows 0–5 match, but the
   stock table has no `clips` partition, so the record/replay LittleFS never
   mounts and every clip save is refused ("clips FS not mounted") while the
   rest of the firmware looks perfectly healthy.
4. Tools → Flash Size → **16MB (128Mb)** — **required**; the `clips` partition
   spans `0x400000`–`0x1000000` and is unaddressable at any smaller size.
5. Tools → PSRAM → **OPI PSRAM** — **required**; the firmware allocates its
   runtime config in external PSRAM and halts at boot (solid red status LED)
   without it.
6. Sketch → **Export Compiled Binary** (Ctrl/Cmd+Alt+S).
7. The IDE writes three files into `build/<fqbn>/`:
   - `NaviCore.ino.bin`
   - `NaviCore.ino.bootloader.bin`
   - `NaviCore.ino.partitions.bin`
8. Copy them into `firmware/`, renaming with a version prefix + the suffixes
   in the table above. Example:
   ```
   cp build/.../NaviCore.ino.bin             firmware/NaviCore_<DTG>_ESP32S3.bin
   cp build/.../NaviCore.ino.bootloader.bin  firmware/NaviCore_<DTG>_ESP32S3_boot.bin
   cp build/.../NaviCore.ino.partitions.bin  firmware/NaviCore_<DTG>_ESP32S3_part.bin
   ```
   (Old versioned files can be deleted or left as history — the flasher only
   looks at suffixes, so it always picks one of each.)

   `NaviCore.ino.bootloader.bin` is copied for completeness only — the flasher
   writes the fixed `WCB_S3_custom_bootloader_16MB_wdt3s.bin` at `0x0`, so an
   IDE-built bootloader never reaches the board.
9. Commit to `main`. The Config → Firmware tab will pick them up automatically.

### Option B — `tools/build-firmware.ps1` (Windows) or `build-firmware.sh` (Linux/macOS/WSL)

Wraps `arduino-cli` so steps 2–7 above happen in one command.  Same
logic the CI workflow uses, just running locally.

```powershell
# Windows:
pwsh tools/build-firmware.ps1
```
```bash
# Linux / macOS / WSL:
tools/build-firmware.sh
```

Both read `FW_VERSION_BASE` + `FW_VERSION_DTG` from `fw_version.h`,
compile with the right FQBN + partition scheme, prune older bins, and
drop the new ones into `firmware/` with the matching version prefix.
You commit + push manually after a local build.

Prereqs: `arduino-cli` on `PATH`, the `esp32` core installed
(`arduino-cli core install esp32:esp32@3.3.4`), and the same library set
the CI workflow installs (see `.github/workflows/build-firmware.yml`).

## Notes

- Only **WCB v3.2** (ESP32-S3) is supported by the in-browser flasher right
  now. If support for older boards is added later, mirror the WCB Wizard's
  per-variant suffix scheme (`_ESP32.bin`, `_ESP32_boot.bin`, etc.) and add
  a board-variant dropdown to the Firmware tab.
- The flasher writes bootloader + partition table + app on every flash,
  blank board or not.  The only difference between Update and Full Wipe is
  whether NVS (`0x9000`) and otadata (`0xE000`) are also erased.
- App-only is a fallback, not a mode: it happens only when the bootloader
  and partition files are both absent from GitHub.  If exactly one is
  present the flasher aborts rather than write a partial set, because
  app-only onto a blank board leaves it unbootable.
- The page fetches from `main` by default. Developers can override the
  branch by setting `localStorage.rc_fw_branch = '<branch-name>'` in DevTools.
