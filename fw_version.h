// =============================================================================
//  fw_version.h — NaviCore firmware version string
//
//  Single source of truth for the firmware version, formatted as:
//      <BASE>_<DTG>     e.g.  v0.2.0_211520QMAY26
//
//  • FW_VERSION_BASE  — bumped MANUALLY when cutting a new release.
//                       Three-digit semver (MAJOR.MINOR.PATCH) is preferred:
//                       e.g. "v0.2.0" → "v0.2.1" → "v0.3.0" → "v1.0.0".
//                       Edit by hand.
//
//  • FW_VERSION_DTG   — stamped AUTOMATICALLY by the pre-commit hook
//                       (tools/git-hooks/pre-commit), same Date-Time-Group
//                       format as the UI footer in config_tool/index.html.
//                       Format:  DDHHMM<TZ>MMMYY  (e.g. 211520QMAY26).
//                       Don't edit by hand — the hook will overwrite it.
//
//  • FW_VERSION       — convenient combined string used by the firmware
//                       when reporting itself (PONG reply, boot banner).
//
//  The build script (tools/build-firmware.ps1) also reads the two defines
//  to embed the version into the .bin filename, so a flashed board's
//  reported version always matches the file it came from.
// =============================================================================

#pragma once

#define FW_VERSION_BASE  "v0.2.0"
#define FW_VERSION_DTG   "021354QJUL26"
#define FW_VERSION       FW_VERSION_BASE "_" FW_VERSION_DTG
