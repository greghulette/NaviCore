This directory vendors an unmodified snapshot of the **DroidNet Command Library**
data files (`manifest.json` + `boards/*.json`), so NaviCore's config-tool command
picker has a full command catalog available immediately, without a live fetch to
GitHub.

- Source: https://github.com/travisccook/Droidnet-Command-Library
- License: Mozilla Public License 2.0 (see `LICENSE` in this directory — DroidNet's
  own, unmodified) — the license permits combining with proprietary code; only the
  DroidNet files themselves (this directory) stay under MPL-2.0.
- Snapshot: `libraryVersion` 4.1.0 (19 boards), fetched 2026-07-10 from the `main`
  branch. 4.1.0 added `wcb-mp3` + `wcb-native` and now sources the WCB 6.1.5
  firmware, so its WCB boards (wcb-hcr / wcb-mp3 / wcb-native) supersede NaviCore's
  small curated seed of the same ids on merge.
- These files are used **as-is, unmodified** — read at runtime by NaviCore's
  command-library picker (`config_tool/index.html`, `_cmdlibImportFromManifest`),
  the same code path used for a live "check for updates" fetch. NaviCore's own
  command data (WCB/HCR/MP3/WLED/Maestro, curated from the real WCB firmware) is
  separate and lives inline in `index.html` as `NC_CMDLIB_SEED`.
- To refresh this snapshot, use the "Check online for the latest library" option
  in the command-library picker, or re-download `manifest.json` + every file it
  lists under `boards/` from the source repo above.
