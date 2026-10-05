import { execFileSync } from 'node:child_process';
import { existsSync, readFileSync } from 'node:fs';
import { dirname, join, resolve } from 'node:path';
import { pathToFileURL } from 'node:url';
import { parseArgs } from 'node:util';

// Flashing is explicit; require a restoration image and a specific probe.
const { values } = parseArgs({ options: {
  elf: { type: 'string' }, restore: { type: 'string' },
  platform: { type: 'string' }, probe: { type: 'string' },
} });
for (const name of ['elf', 'restore', 'platform', 'probe']) {
  if (!values[name]) throw new Error(`Missing --${name}`);
}
const elf = resolve(values.elf), restore = resolve(values.restore);
if (elf === restore) throw new Error('Restoration image must differ from test image');
for (const path of [elf, restore]) {
  if (!existsSync(path)) throw new Error(`ELF not found: ${path}`);
}
const cache = readFileSync(join(dirname(elf), 'CMakeCache.txt'), 'utf8');
if (!/^PNX_BOARD:STRING=h723_dmmc02\r?$/m.test(cache)) {
  throw new Error('This target test requires the H723 board');
}
const nm = cache.match(/^CMAKE_NM:FILEPATH=(.+)\r?$/m)?.[1].trim();
if (!nm) throw new Error('CMAKE_NM missing from test build');
const symbols = execFileSync(nm, [elf], { encoding: 'utf8', windowsHide: true });
const address = symbols.match(/^([0-9a-fA-F]+) [Bb] pnx_core_test\r?$/m)?.[1];
if (!address) throw new Error('pnx_core_test not found in test ELF');
const { DapSession, listProbes } = await import(pathToFileURL(
  join(resolve(values.platform), 'src/dap.mjs')));
const probes = await listProbes();
const probe = probes.find(item => item.selector === values.probe || item.serialNumber === values.probe);
if (!probe) throw new Error('Selected probe is not connected');
const fields = ['stage', 'checks', 'failures', 'failed_groups', 'done',
  'boot_count', 'retained_marker', 'prior_boot', 'logger_cycles', 'clock_hz'];
const session = new DapSession(() => {}, message => console.error(message));
let flashAttempted = false;

async function result() {
  const memory = await session.request('readMemory', { memoryReference: `0x${address}`, count: 40 });
  const data = Buffer.from(memory.data, 'base64');
  if (data.length !== 40) throw new Error('Incomplete target result');
  return Object.fromEntries(fields.map((field, index) => [field, data.readUInt32LE(index * 4)]));
}

async function completed() {
  // Allow startup after reset before inspecting the prior RAM address.
  await new Promise(resolve => setTimeout(resolve, 100));
  for (let attempt = 0; attempt < 100; ++attempt) {
    const value = await result();
    if (value.done) return value;
    await new Promise(resolve => setTimeout(resolve, 100));
  }
  throw new Error(`Test timed out: ${JSON.stringify(await result())}`);
}

function passed(value) {
  return value.done === 1 && value.stage === 8 && value.checks > 0 && value.failures === 0;
}

try {
  await session.start({ chip: 'STM32H723VGTx', elf, probe: probe.selector,
    speedKHz: 4000, rate: 1, allowDebug: true, allowFlash: true });
  flashAttempted = true;
  await session.flashAndRun(elf);
  const first = await completed();
  console.log(`FIRST_RUN ${JSON.stringify(first)}`);
  await session.request('restart');
  await session.debug('continue');
  const reset = await completed();
  console.log(`RESET_RUN ${JSON.stringify(reset)}`);
  if (!passed(first) || !passed(reset) || reset.retained_marker !== 1 ||
      reset.prior_boot !== first.boot_count || reset.boot_count !== first.boot_count + 1) {
    throw new Error('Target assertions or reset retention failed');
  }
} finally {
  try {
    if (flashAttempted) {
      await session.flashAndRun(restore);
      console.log(`RESTORED ${restore}`);
    }
  } finally {
    await session.stop();
  }
}
