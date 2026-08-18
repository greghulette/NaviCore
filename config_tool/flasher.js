// ════════════════════════════════════════════════════════════════
//  NaviCore Firmware Flasher
//
//  Browser-based ESP32-S3 flasher for the WCB v3.2 hardware variant.
//  This is a simplified port of the Wireless_Communication_Board-WCB
//  Wizard flasher (same upstream library, same flash addresses,
//  same NVS-preservation logic) — but stripped to a single board
//  variant since the NaviCore firmware only targets WCB v3.2.
//
//  Uses esptool-js (Espressif's official browser flash library)
//  loaded on-demand from CDN, plus CryptoJS for MD5 verification.
//
//  Firmware binaries are hosted in the NaviCore repo under
//  /firmware/ on GitHub.  The Contents API is used to list the
//  directory so a freshly built binary with a versioned filename
//  (e.g. NaviCore_v0.2.0_172203QAUG26_ESP32S3.bin) is picked up
//  automatically — the NaviCore_<version>_ESP32S3 name SHAPE is
//  what has to stay stable, not any one version.
//
//  Public surface:
//    flashFirmware(port, callbacks)  → Promise<void>
//
//  port is a WebSerial SerialPort that MUST be closed before calling.
// ════════════════════════════════════════════════════════════════

const ESPTOOL_CDN  = 'https://cdn.jsdelivr.net/npm/esptool-js@0.4.7/+esm';
const CRYPTOJS_CDN = 'https://cdnjs.cloudflare.com/ajax/libs/crypto-js/4.2.0/crypto-js.min.js';

// ── Firmware source config ───────────────────────────────────────
// Binaries live in /firmware/ on GitHub.  Three files are read:
//   NaviCore_<version>_ESP32S3.bin       — application image  → 0x10000
//   NaviCore_<version>_ESP32S3_part.bin  — partition table    → 0x8000
//   WCB_S3_custom_bootloader_16MB_wdt3s.bin — bootloader      → 0x0
//
// The bootloader is a FIXED name, not a per-build one: CI also emits a stock
// NaviCore_<version>_ESP32S3_boot.bin, which is NOT the one we want (see the
// pairing comment in fetchFirmwareImages).
//
// Only the application image is required.  If boot+part are missing
// we fall back to an app-only flash (works on already-programmed
// boards; blank boards need a full set).
const GITHUB_OWNER          = 'greghulette';
const GITHUB_REPO           = 'NaviCore';
const GITHUB_BRANCH_DEFAULT = 'main';
const GITHUB_BIN_PATH       = 'firmware';

// Branch override (Advanced/dev only) — set via localStorage to test
// unreleased branches without recompiling the page.
function getFirmwareBranch() {
  try {
    const b = (localStorage.getItem('rc_fw_branch') || '').trim();
    return b || GITHUB_BRANCH_DEFAULT;
  } catch (_) {
    return GITHUB_BRANCH_DEFAULT;
  }
}

// ── Latest-version check ─────────────────────────────────────────
// Lists /firmware/ (the SAME GitHub Contents API the flasher uses) and parses
// the version out of the NaviCore app image's filename
// (NaviCore_<version>_ESP32S3.bin). A flashed board reports the SAME string
// (FW_VERSION = base_DTG) in its PONG, so the two compare directly. This is the
// exact image the "Update Firmware" button would write, so it's the right
// notion of "latest". Returns { version, filename, branch }; throws on
// network / API error / missing image.
async function fetchLatestFirmwareVersion() {
  const branch = getFirmwareBranch();
  const apiUrl = `https://api.github.com/repos/${GITHUB_OWNER}/${GITHUB_REPO}/contents/${GITHUB_BIN_PATH}?ref=${branch}`;
  const resp = await fetch(apiUrl);
  if (!resp.ok) throw new Error(`GitHub API: HTTP ${resp.status}`);
  const files = await resp.json();
  if (!Array.isArray(files)) throw new Error('unexpected GitHub API response');
  // Match ONLY the NaviCore app image — not _boot/_part, not RC-Controller_*.
  const app = files.find(f => f.type === 'file' && /^NaviCore_.+_ESP32S3\.bin$/.test(f.name));
  if (!app) throw new Error('no NaviCore app image (NaviCore_*_ESP32S3.bin) in firmware/');
  const m = app.name.match(/^NaviCore_(.+)_ESP32S3\.bin$/);
  return { version: m ? m[1] : null, filename: app.name, branch };
}

// ── Script loader ────────────────────────────────────────────────
function loadScript(src) {
  return new Promise((resolve, reject) => {
    if (document.querySelector(`script[src="${src}"]`)) { resolve(); return; }
    const s    = document.createElement('script');
    s.src      = src;
    s.onload   = resolve;
    s.onerror  = () => reject(new Error(`Failed to load CDN script: ${src}`));
    document.head.appendChild(s);
  });
}

// ── Binary fetching ──────────────────────────────────────────────
// Returns [{ buf, address }, ...] in ascending-address order.
//
// Flash map (ESP32-S3, 16 MB, the custom table in partitions.csv):
//   boot   → 0x0       (bootloader)
//   part   → 0x8000    (partition table)
//   nvs    → 0x9000    (NVS — NOT touched here so saved config survives)
//   app0   → 0x10000   (application)
function _otaSleep(ms) { return new Promise(res => setTimeout(res, ms)); }
// Fetch a URL, retrying through GitHub's transient rate-limits (HTTP 429, and
// the 403 GitHub uses for abuse throttling) and 5xx. Honors Retry-After when
// present, else exponential backoff. GitHub throttles heavy/rapid access to raw
// content + the API per-IP; a short wait almost always clears it, so a few
// automatic retries beat failing the whole OTA on a transient throttle.
async function _fetchRetry(url, onLog, label) {
  const MAX_ATTEMPTS = 4;
  for (let attempt = 1; attempt <= MAX_ATTEMPTS; attempt++) {
    let r = null;
    try {
      r = await fetch(url);
    } catch (netErr) {
      if (attempt === MAX_ATTEMPTS) throw netErr;
      await _otaSleep(1500 * attempt);
      continue;
    }
    if (r.ok) return r;
    const retryable = (r.status === 429 || r.status === 403 || r.status >= 500);
    if (retryable && attempt < MAX_ATTEMPTS) {
      const ra = parseInt(r.headers.get('retry-after') || '', 10);
      const waitMs = Number.isFinite(ra) ? Math.min(30000, ra * 1000)
                                         : Math.min(8000, 1500 * Math.pow(2, attempt - 1));
      if (onLog) onLog(`GitHub throttled ${label || 'download'} (HTTP ${r.status}) — retrying in ${Math.round(waitMs / 1000)}s… (${attempt}/${MAX_ATTEMPTS - 1})`);
      await _otaSleep(waitMs);
      continue;
    }
    return r;   // non-retryable status (or out of attempts) — caller reports it
  }
}

async function fetchFirmwareImages(onLog) {
  const branch = getFirmwareBranch();
  const apiUrl = `https://api.github.com/repos/${GITHUB_OWNER}/${GITHUB_REPO}/contents/${GITHUB_BIN_PATH}?ref=${branch}`;

  if (branch !== GITHUB_BRANCH_DEFAULT)
    onLog(`⚠ Firmware source branch: ${branch} (not the released 'main')`);
  onLog(`Scanning ${GITHUB_BIN_PATH}/ on GitHub (${branch})…`);

  const listResp = await _fetchRetry(apiUrl, onLog, 'firmware list');
  if (!listResp.ok) throw new Error(`GitHub API: HTTP ${listResp.status}`);
  const files = await listResp.json();

  // Select by ANCHORED name, never by bare suffix. firmware/ is a shared
  // directory: it also holds another product's bins (RC-Controller_*_ESP32S3*.bin,
  // which the build scripts' NaviCore_-scoped prune never removes) and may hold
  // older NaviCore builds (build-firmware.ps1 -KeepOld leaves them). An
  // endsWith() match resolves to whichever name sorts first in the API listing,
  // which has already locked onto the wrong (oldest) image once — see the note
  // in .github/workflows/build-firmware.yml. DTG tags do not sort chronologically,
  // so alphabetical order is not a safety net.
  function pick(re) {
    return files.find(f => f.type === 'file' && re.test(f.name)) || null;
  }
  function pickExact(name) {
    return files.find(f => f.type === 'file' && f.name === name) || null;
  }

  async function download(entry, required, what) {
    if (!entry) {
      if (required) throw new Error(`No ${what} found in ${GITHUB_BIN_PATH}/`);
      return null;
    }
    onLog(`Found: ${entry.name}`);
    const r = await _fetchRetry(entry.download_url, onLog, entry.name);
    if (!r.ok) throw new Error(`HTTP ${r.status} fetching ${entry.name}`);
    const buf = await r.arrayBuffer();
    if (buf.byteLength === 0) throw new Error(`${entry.name} is empty`);
    return buf;
  }

  // App is required; boot + part are optional (paired — either both or neither).
  // Same strict regex as fetchLatestFirmwareVersion() so the "Latest on GitHub"
  // line and the image actually written can never disagree.
  const appEntry = pick(/^NaviCore_.+_ESP32S3\.bin$/);
  const appBuf   = await download(appEntry, true, 'NaviCore app image (NaviCore_*_ESP32S3.bin)');
  const images   = [{ buf: appBuf, address: 0x10000 }];

  // Version captured from the app filename — the partition table is then required
  // to carry the IDENTICAL version, so an app and a table from two different
  // builds can never be paired. A mismatched table flashes silently and the
  // damage only shows later (e.g. a table without the `clips` row leaves
  // clipsFS unmounted and every record/replay save fails).
  const fwVersion = appEntry.name.match(/^NaviCore_(.+)_ESP32S3\.bin$/)[1];

  // Bootloader + partition table are a PAIR — flash both or neither. Fetch each
  // as optional (non-throwing) and inspect them independently:
  //   • both present  → full flash (boot + part + app)
  //   • both absent   → app-only (fine for re-flashing an already-provisioned
  //                     board; a blank board still needs a one-time IDE flash)
  //   • exactly one   → a corrupted/partial firmware upload. Do NOT silently
  //                     fall back to app-only: app-only onto a blank board (no
  //                     partition table) leaves it unbootable. Abort loudly.
  // Bootloader = the CUSTOM short-WDT 16MB bootloader (cold-boot auto-retry),
  // committed under a FIXED name so CI's stock per-build _ESP32S3_boot.bin can
  // never shadow it. It is the matched pair of the firmware's in-app boot guard.
  // Partition table stays the tagged per-build _ESP32S3_part.bin, matched to the
  // app's exact version (a table from a different build is treated as missing →
  // the "exactly one" abort below, not a silent mismatched flash).
  // Sequential (not Promise.all) so we don't fire concurrent large raw-content
  // fetches at GitHub — a burst is more likely to trip its per-IP rate limit.
  const partName = `NaviCore_${fwVersion}_ESP32S3_part.bin`;
  const bootBuf  = await download(pickExact('WCB_S3_custom_bootloader_16MB_wdt3s.bin'), false);
  const partBuf  = await download(pickExact(partName), false);
  let hasBootPart = false;
  if (bootBuf && partBuf) {
    images.unshift(
      { buf: bootBuf, address: 0x0    },
      { buf: partBuf, address: 0x8000 },
    );
    hasBootPart = true;
  } else if (bootBuf || partBuf) {
    const missing = bootBuf ? `partition table (${partName})`
                            : 'custom bootloader (WCB_S3_custom_bootloader_16MB_wdt3s.bin)';
    throw new Error(`Incomplete firmware on GitHub: the ${missing} is missing while ` +
      `its pair is present. Refusing to flash a partial set — app-only onto a blank ` +
      `board would leave it unbootable. Re-run the firmware build/upload, then retry.`);
  } else {
    onLog('Note: bootloader/partition files not on GitHub — flashing app only.');
    onLog('      A blank board will need a one-time full flash via Arduino IDE.');
  }

  const totalKB = Math.round(images.reduce((s, i) => s + i.buf.byteLength, 0) / 1024);
  onLog(`Loaded ${totalKB} KB (${hasBootPart ? 'boot + partitions + app' : 'app only'})`);
  return images;
}

// ── Buffer → Latin1 string (esptool-js wants strings, not Uint8Array) ─
function bufToLatin1(buf) {
  const u8 = new Uint8Array(buf);
  let s = '';
  const CHUNK = 65536;
  for (let i = 0; i < u8.length; i += CHUNK) {
    s += String.fromCharCode.apply(null, u8.subarray(i, i + CHUNK));
  }
  return s;
}

// Windows detection — CP210x driver has a known issue with mid-flash
// baud-rate changes, so we keep 115200 there.  Mac/Linux can use 460800.
const _isWindowsPlatform = /Win/i.test(navigator.platform || '');

// ════════════════════════════════════════════════════════════════
//  flashFirmware
//
//  port      — WebSerial SerialPort, MUST be CLOSED before calling
//  options   — {
//                onProgress(written, total),
//                onLog(msg),
//                onStatus(msg),
//                eraseNvs  : bool   — true → full wipe (factory state)
//              }
//
//  Both modes write bootloader + partition table + app unconditionally.
//  There is NO read-back / auto-detect of what is already on the board —
//  see Step 3b for why reading flash over the S3's native USB was removed.
//  The ONLY difference between the two is what gets erased:
//
//    eraseNvs = false  (default — "Update" path):
//      Erase otadata (0xE000, 8 KB) so the boot selector returns to app0.
//      NVS (0x9000) is left alone, so the saved config survives.
//
//    eraseNvs = true   ("Full Wipe / Initial Push" path):
//      Erase otadata AND NVS (0x9000, 20 KB).  Use for first-time
//      programming, recovery from a bricked board, or whenever a
//      factory-fresh config is wanted.
// ════════════════════════════════════════════════════════════════
async function flashFirmware(port, { onProgress, onLog, onStatus, eraseNvs = false }) {

  // ── Step 1: load CDN dependencies ──────────────────────────────
  onStatus('Loading flash tool…');
  onLog('Loading CryptoJS…');
  try { await loadScript(CRYPTOJS_CDN); }
  catch (e) { throw new Error(`Could not load CryptoJS from CDN — are you online?\n${e.message}`); }

  onLog('Loading esptool-js…');
  let ESPLoader, Transport;
  try { ({ ESPLoader, Transport } = await import(ESPTOOL_CDN)); }
  catch (e) { throw new Error(`Could not load esptool-js from CDN — are you online?\n${e.message}`); }
  onLog('Flash tool loaded.');

  // ── Step 2: fetch firmware ──────────────────────────────────────
  onStatus('Downloading firmware…');
  let flashImages;
  try { flashImages = await fetchFirmwareImages(onLog); }
  catch (e) { throw new Error(`Firmware download failed: ${e.message}`); }

  // ── Step 3: connect to ESP bootloader ──────────────────────────
  onStatus('Connecting to bootloader…');
  onLog('Connecting to ESP32-S3 bootloader…');
  onLog('► Hold BOOT, tap RST, release BOOT — then watch for sync below.');

  // Route esptool-js internal messages into our log so the user can see sync progress.
  const terminal = {
    clean:     ()    => {},
    writeLine: (msg) => { if (msg?.trim()) onLog(`[esptool] ${msg.trim()}`); },
    write:     (msg) => { if (msg?.trim()) onLog(`[esptool] ${msg.trim()}`); },
  };

  const transport = new Transport(port, false);
  const loader    = new ESPLoader({
    transport,
    baudrate:    _isWindowsPlatform ? 115200 : 460800,
    romBaudrate: 115200,
    enableTracing: false,
    terminal,
  });

  let chip;
  try {
    chip = await loader.main();
    onLog(`Chip identified: ${chip}`);
  } catch (e) {
    try { await transport.disconnect(); } catch (_) {}
    throw new Error(
      `Bootloader connection failed: ${e.message}\n\n` +
      `To enter bootloader mode: hold BOOT, press RST, release BOOT, then click Flash again.`
    );
  }

  // ── Step 3b: decide what to flash ──────────────────────────────
  // We DELIBERATELY do not read the flash to choose app-only vs. full.
  // readFlash() over the ESP32-S3 native USB is slow and flaky, and when it
  // stalls it wedges the esptool stub so the SUBSEQUENT write times out
  // (observed in the field: "Flash read check failed… → Flash write failed:
  // Timeout"). Racing the read against a timeout doesn't help — it doesn't
  // cancel the underlying operation, so the stub stays wedged.
  //
  // Instead: ALWAYS write bootloader + partition table + app. The bootloader
  // (~20 KB) and partition table (~3 KB) are tiny next to the ~1 MB app and
  // are identical on every build (the fixed custom table in partitions.csv),
  // so always writing them costs almost nothing and is reliable on BOTH blank and
  // already-programmed boards. The only difference between Update and Full
  // Wipe is whether we also erase NVS/otadata (Step 3c below) — Update never
  // touches NVS at 0x9000, so saved config is preserved.
  let imagesToFlash = flashImages.slice();
  onLog(eraseNvs
    ? 'Full wipe — flashing bootloader + partitions + app (NVS will be erased).'
    : 'Update — flashing bootloader + partitions + app (NVS preserved).');

  // ── Step 3c: optionally prepend NVS + otadata erase images ────
  // NaviCore partition layout (PartitionScheme=custom + FlashSize=16M — the
  // table in partitions.csv; rows 0-5 are byte-identical to stock min_spiffs):
  //   nvs      @ 0x9000,   size 0x5000   (20 KB)
  //   otadata  @ 0xE000,   size 0x2000   (8 KB — two 4 KB flash sectors)
  //   app0     @ 0x10000,  size 0x1E0000 (~1.9 MB)
  //   app1     @ 0x1F0000, size 0x1E0000
  //   spiffs   @ 0x3D0000, size 0x20000  (config LittleFS — /config.json)
  //   coredump @ 0x3F0000, size 0x10000
  //   clips    @ 0x400000, size 0xC00000 (12 MB record/replay LittleFS)
  //
  // Writing 0xFF buffers causes esptool to erase then rewrite those sectors,
  // returning them to factory-fresh state. Both otadata sectors MUST be
  // erased: if either still holds a stale OTA state pointing to ota_1, the
  // bootloader will try to boot ota_1, fail (nothing there after a fresh
  // flash to ota_0), and the OTA rollback watchdog fires — endless reboot loop.
  // OTA-data (the boot selector) is ALWAYS reset to ota_0 on an esptool flash:
  // we always write the app to ota_0, so if a prior OTA had flipped the boot
  // selector to ota_1, leaving otadata alone would make the board boot the stale
  // (now-overwritten) slot → rollback watchdog → reboot loop. NVS (saved config)
  // is only wiped on a Full Wipe. Both are written as 0xFF so esptool erases then
  // rewrites the sectors back to factory-fresh.
  {
    const otadataBlank = new ArrayBuffer(0x2000);  // otadata: 8 KB @ 0xE000 (two 4 KB sectors)
    new Uint8Array(otadataBlank).fill(0xFF);
    const prepend = [{ buf: otadataBlank, address: 0xE000 }];
    if (eraseNvs) {
      const nvsBlank = new ArrayBuffer(0x5000);    // NVS: 20 KB @ 0x9000
      new Uint8Array(nvsBlank).fill(0xFF);
      prepend.unshift({ buf: nvsBlank, address: 0x9000 });   // ascending address order
      onLog('NVS (0x9000, 20 KB) and OTA data (0xE000, 8 KB) will be erased.');
    } else {
      onLog('OTA boot selector (0xE000, 8 KB) reset to ota_0 (config preserved).');
    }
    imagesToFlash = [...prepend, ...imagesToFlash];
  }

  const totalBytes = imagesToFlash.reduce((sum, img) => sum + img.buf.byteLength, 0);

  // ── Step 4: write flash ────────────────────────────────────────
  onStatus(`Flashing ${chip}…`);
  onLog(`Writing ${Math.round(totalBytes / 1024)} KB across ${imagesToFlash.length} region(s)…`);
  onProgress(0, totalBytes);

  const writeFlashFn = loader.writeFlash ?? loader.write_flash;
  if (typeof writeFlashFn !== 'function') {
    try { await transport.disconnect(); } catch (_) {}
    throw new Error('esptool-js: writeFlash method not found — unexpected library version.');
  }

  let bytesWritten = 0;
  try {
    await writeFlashFn.call(loader, {
      fileArray: imagesToFlash.map(img => ({ data: bufToLatin1(img.buf), address: img.address })),
      flashSize: 'keep',
      flashMode: 'keep',
      flashFreq: 'keep',
      eraseAll:  false,
      compress:  true,
      reportProgress: (_fileIdx, written, total) => {
        // esptool-js resets written/total per file; accumulate for overall progress.
        onProgress(bytesWritten + written, totalBytes);
        if (written === total) bytesWritten += total;
      },
      calculateMD5Hash: (img) =>
        CryptoJS.MD5(CryptoJS.enc.Latin1.parse(img)).toString(),
    });
  } catch (e) {
    try { await transport.disconnect(); } catch (_) {}
    throw new Error(`Flash write failed: ${e.message}`);
  }

  // ── Step 5: reset into firmware ────────────────────────────────
  onLog('Resetting board into firmware…');
  onStatus('Resetting…');
  onProgress(totalBytes, totalBytes);

  const afterFlashFn = loader.afterFlash ?? loader.after_flash;
  try { if (afterFlashFn) await afterFlashFn.call(loader, 'hard_reset'); } catch (_) {}
  try { await transport.disconnect(); }                                   catch (_) {}

  onLog('Flash complete — board rebooting.');
  onStatus('Flash complete!');
}

// Expose globally so index.html's inline JS can call it without a module loader.
window.flashFirmware = flashFirmware;
