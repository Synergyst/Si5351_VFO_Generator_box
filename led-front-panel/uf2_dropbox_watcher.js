
#!/usr/bin/env node
// uf2_dropbox_watcher.js
// Watches a directory for *.uf2 files, waits until each file is stable,
// waits POST_WAIT_MS, then:
//   - sends "BOOTSEL\n" to SERIAL_TTY (if present)
//   - if serial was sent: wait up to BOOTSEL_TIMEOUT_MS for BOOTSEL (picotool info)
//   - if serial not available or failed: do a single picotool info check
//   - if neither path indicates BOOTSEL, assume no RP device connected and skip (retry later)
//   - else flash via: picotool load --verify --update --execute <file.uf2>
//   - delete UF2 on success
//
// Env vars (set in the systemd unit if desired):
//   UF2_DIR=/root/uf2-dropbox
//   PICOTOOL=/usr/local/bin/picotool
//   SERIAL_TTY=/dev/ttyACM0 or /dev/serial/by-id/...
//   SCAN_MS=500
//   STABLE_MS=1500
//   POST_WAIT_MS=2000
//   BOOTSEL_SERIAL_SLEEP_MS=2000
//   BOOTSEL_TIMEOUT_MS=10000
//   LOAD_TIMEOUT_MS=30000

const fs = require('fs');
const fsp = fs.promises;
const path = require('path');
const { exec, execFile } = require('child_process');

const DROP_DIR = process.env.UF2_DIR || '/root/uf2-dropbox';
const SCAN_MS = parseInt(process.env.SCAN_MS || '500', 10);
const STABLE_MS = parseInt(process.env.STABLE_MS || '1500', 10);
const POST_WAIT_MS = parseInt(process.env.POST_WAIT_MS || '2000', 10);
const BOOTSEL_SERIAL_SLEEP_MS = parseInt(process.env.BOOTSEL_SERIAL_SLEEP_MS || '2000', 10);
const BOOTSEL_TIMEOUT_MS = parseInt(process.env.BOOTSEL_TIMEOUT_MS || '10000', 10);
const LOAD_TIMEOUT_MS = parseInt(process.env.LOAD_TIMEOUT_MS || '30000', 10);

let PICOTOOL = process.env.PICOTOOL || 'picotool';
let SERIAL_TTY = process.env.SERIAL_TTY || '/dev/ttyACM0';

function log(...args) { console.log(new Date().toISOString(), ...args); }
function sleep(ms) { return new Promise(r => setTimeout(r, ms)); }

function execAsync(cmd, opts = {}) {
  return new Promise(resolve => {
    exec(cmd, { timeout: 2500, ...opts }, (err, stdout, stderr) => {
      resolve({ ok: !err, code: err ? err.code : 0, stdout: String(stdout || ''), stderr: String(stderr || '') });
    });
  });
}
function execFileAsync(cmd, args, opts = {}) {
  return new Promise(resolve => {
    execFile(cmd, args, { timeout: 25000, ...opts }, (err, stdout, stderr) => {
      resolve({ ok: !err, code: err ? err.code : 0, stdout: String(stdout || ''), stderr: String(stderr || '') });
    });
  });
}

async function resolvePicotool() {
  if (PICOTOOL.startsWith('/')) {
    try { fs.accessSync(PICOTOOL, fs.constants.X_OK); } catch {}
  } else {
    const r = await execAsync(`command -v ${PICOTOOL}`);
    if (r.ok && r.stdout.trim()) PICOTOOL = r.stdout.trim();
  }
  log('Using picotool at:', PICOTOOL);
}

function sanitizeTty(p) {
  if (typeof p !== 'string') return null;
  if (!p.startsWith('/dev/')) return null;
  if (!/^[-_./A-Za-z0-9]+$/.test(p)) return null;
  return p;
}

async function findConfiguredTty() {
  const s = sanitizeTty(SERIAL_TTY);
  if (!s) return null;
  try { await fsp.access(s, fs.constants.W_OK); return s; } catch { return null; }
}

async function picotoolPresent() {
  const r = await execFileAsync(PICOTOOL, ['version'], { timeout: 2000 });
  return r.ok;
}

async function bootselReady() {
  // ok only when a BOOTSEL RP device is present
  const r = await execFileAsync(PICOTOOL, ['info'], { timeout: 1500 });
  return r.ok;
}

async function sendBootselSerial() {
  const tty = await findConfiguredTty();
  if (!tty) {
    log('INFO: SERIAL_TTY not present; will check if already in BOOTSEL.');
    return { attempted: false, ok: false, tty: null };
  }
  const cmd = `/bin/sh -lc 'printf "BOOTSEL\\n" > ${tty}'`;
  const r = await execAsync(cmd, { timeout: 2000 });
  if (!r.ok) {
    log(`WARN: failed to write BOOTSEL to ${tty}:`, r.stderr || r.stdout);
    return { attempted: true, ok: false, tty };
  }
  log(`Sent BOOTSEL to ${tty}, waiting ${BOOTSEL_SERIAL_SLEEP_MS}ms`);
  await sleep(BOOTSEL_SERIAL_SLEEP_MS);
  return { attempted: true, ok: true, tty };
}

async function waitForBootsel(timeoutMs) {
  const deadline = Date.now() + timeoutMs;
  while (Date.now() < deadline) {
    if (await bootselReady()) return true;
    await sleep(300);
  }
  return false;
}

async function flashUf2(uf2Path) {
  log('Flashing', uf2Path);
  await resolvePicotool();
  const okPT = await picotoolPresent();
  if (!okPT) {
    log('ERROR: picotool not available. Set Environment=PICOTOOL=/full/path in the service. Keeping file:', uf2Path);
    return false;
  }

  // 1) Try serial-triggered BOOTSEL if SERIAL_TTY exists
  const serial = await sendBootselSerial();

  // 2) If we sent serial, poll for BOOTSEL up to timeout.
  if (serial.attempted && serial.ok) {
    const bootOk = await waitForBootsel(BOOTSEL_TIMEOUT_MS);
    if (!bootOk) {
      log('ERROR: Did not enter BOOTSEL after serial trigger; will retry later.');
      return false;
    }
  } else if (!serial.attempted) {
    // 3) No serial available: do a single BOOTSEL check now; if not ready, assume no device plugged in.
    const inBootsel = await bootselReady();
    if (!inBootsel) {
      log('INFO: No SERIAL_TTY and not already in BOOTSEL; assuming no RP device connected. Will retry later.');
      return false;
    }
  } else if (serial.attempted && !serial.ok) {
    // Serial write failed; check once for BOOTSEL, else bail.
    const inBootsel = await bootselReady();
    if (!inBootsel) {
      log('INFO: Serial trigger failed and not in BOOTSEL; will retry later.');
      return false;
    }
  }

  // 4) Flash with your preferred flags
  const r = await execFileAsync(PICOTOOL, ['load', '--verify', '--update', '--execute', uf2Path], { timeout: LOAD_TIMEOUT_MS });
  if (!r.ok) {
    log('ERROR: picotool load failed:', r.stderr || r.stdout);
    return false;
  }
  log('picotool load OK.');
  return true;
}

async function ensureDir(dir) { await fsp.mkdir(dir, { recursive: true }).catch(() => {}); }

const tracking = new Map(); // file -> { size, mtimeMs, since }
const queue = [];
let working = false;

async function scanOnce() {
  let entries = [];
  try { entries = await fsp.readdir(DROP_DIR, { withFileTypes: true }); } catch (e) {
    log('WARN: cannot read dir', DROP_DIR, e.message); return;
  }
  const seen = new Set();
  for (const ent of entries) {
    if (!ent.isFile()) continue;
    if (!ent.name.toLowerCase().endsWith('.uf2')) continue;
    const full = path.join(DROP_DIR, ent.name);
    seen.add(full);
    if (queue.includes(full)) continue;

    let st;
    try { st = await fsp.stat(full); } catch { tracking.delete(full); continue; }
    const cur = tracking.get(full);
    if (!cur) {
      tracking.set(full, { size: st.size, mtimeMs: st.mtimeMs, since: Date.now() });
      continue;
    }
    if (st.size !== cur.size || st.mtimeMs !== cur.mtimeMs) {
      tracking.set(full, { size: st.size, mtimeMs: st.mtimeMs, since: Date.now() });
      continue;
    }
    if (Date.now() - cur.since >= STABLE_MS) {
      queue.push(full);
      tracking.delete(full);
    }
  }
  for (const k of Array.from(tracking.keys())) if (!seen.has(k)) tracking.delete(k);

  if (!working && queue.length > 0) {
    working = true;
    processQueue().finally(() => { working = false; });
  }
}

async function processQueue() {
  while (queue.length > 0) {
    const file = queue.shift();
    try { await fsp.access(file, fs.constants.R_OK); } catch { log('File gone, skipping', file); continue; }

    log('File stable, waiting', POST_WAIT_MS, 'ms:', file);
    await sleep(POST_WAIT_MS);

    const ok = await flashUf2(file);
    if (ok) {
      try { await fsp.unlink(file); log('Deleted', file); }
      catch (e) { log('WARN: flashed but could not delete', file, e.message); }
    } else {
      log('Flash failed; will retry later for', file);
    }
  }
}

(async function main() {
  await ensureDir(DROP_DIR);
  log(`Watching for UF2 files in ${DROP_DIR} (scan ${SCAN_MS}ms, stable ${STABLE_MS}ms)`);
  setInterval(() => { scanOnce().catch(e => log('scan error', e.message)); }, SCAN_MS);
})();