import { execFile, execFileSync } from 'node:child_process';
import { existsSync, readFileSync } from 'node:fs';
import { dirname, join, resolve } from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';
import { parseArgs, promisify } from 'node:util';

// Flashing is explicit; require a restoration image and a specific probe.
const { values } = parseArgs({ options: {
  elf: { type: 'string' }, restore: { type: 'string' },
  platform: { type: 'string' }, probe: { type: 'string' },
  'usart-port': { type: 'string' },
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
const board = cache.match(/^PNX_BOARD:STRING=(.+)\r?$/m)?.[1].trim();
const targets = {
  h723_dmmc02: { chip: 'STM32H723VGTx', deviceId: 0x483, idAddress: '0x5C001000' },
  f407_cboard: { chip: 'STM32F407IGHx', deviceId: 0x413, idAddress: '0xE0042000' },
};
const target = targets[board];
if (!target) throw new Error(`Unsupported target test board: ${board}`);
const restoreCache = readFileSync(join(dirname(restore), 'CMakeCache.txt'), 'utf8');
if (restoreCache.match(/^PNX_BOARD:STRING=(.+)\r?$/m)?.[1].trim() !== board) {
  throw new Error('Restoration image must use the same board as the test image');
}
const nm = cache.match(/^CMAKE_NM:FILEPATH=(.+)\r?$/m)?.[1].trim();
if (!nm) throw new Error('CMAKE_NM missing from test build');
const symbols = execFileSync(nm, [elf], { encoding: 'utf8', windowsHide: true });
const usart = Boolean(values['usart-port']);
const resultSymbol = usart ? 'pnx_usart_test' : 'pnx_core_test';
const address = symbols.match(new RegExp(`^([0-9a-fA-F]+) [Bb] ${resultSymbol}\\r?$`, 'm'))?.[1];
if (!address) throw new Error(`${resultSymbol} not found in test ELF`);
const { DapSession, listProbes } = await import(pathToFileURL(
  join(resolve(values.platform), 'src/dap.mjs')));
const probes = await listProbes();
const probe = probes.find(item => item.selector === values.probe || item.serialNumber === values.probe);
if (!probe) throw new Error('Selected probe is not connected');
const fields = usart ? ['ready', 'failures', 'rx_events', 'rx_bytes', 'tx_bytes', 'overflows',
  'checks', 'busy_checks', 'command', 'acknowledged', 'rx_address', 'clock_hz', 'full_tx'] : ['stage', 'checks', 'failures', 'failed_groups', 'done',
  'boot_count', 'retained_marker', 'prior_boot', 'logger_cycles', 'clock_hz'];
const session = new DapSession(() => {}, message => console.error(message));
let flashAttempted = false;

async function result() {
  const memory = await session.request('readMemory', { memoryReference: `0x${address}`, count: fields.length * 4 });
  const data = Buffer.from(memory.data, 'base64');
  if (data.length !== fields.length * 4) throw new Error('Incomplete target result');
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

async function checkTimebase(image) {
  const imageSymbols = execFileSync(nm, [image], { encoding: 'utf8', windowsHide: true });
  const names = ['uwTick', '_tx_timer_system_clock'];
  const addresses = names.map(name => {
    const value = imageSymbols.match(new RegExp(`^([0-9a-fA-F]+) [BbDd] ${name}\\r?$`, 'm'))?.[1];
    if (!value) throw new Error(`${name} missing from ${image}`);
    return `0x${value}`;
  });
  async function read() {
    const ticks = [];
    for (const memoryReference of addresses) {
      const memory = await session.request('readMemory', { memoryReference, count: 4 });
      const data = Buffer.from(memory.data, 'base64');
      if (data.length !== 4) throw new Error('Incomplete timebase read');
      ticks.push(data.readUInt32LE(0));
    }
    return ticks;
  }
  const before = await read();
  await new Promise(resolve => setTimeout(resolve, 200));
  const after = await read();
  console.log(`TIMEBASE ${JSON.stringify({ image, names, before, after })}`);
  if (after.some((value, index) => ((value - before[index]) >>> 0) === 0)) {
    throw new Error('HAL or ThreadX timebase did not advance during idle');
  }
}

async function testUsart() {
  let value;
  for (let attempt = 0; attempt < 100; ++attempt) {
    await new Promise(resolve => setTimeout(resolve, 100));
    value = await result();
    if (value.ready) break;
  }
  if (value.ready !== 1 || value.failures) throw new Error(`USART startup failed: ${JSON.stringify(value)}`);
  await checkTimebase(elf);
  let bytes = 0;
  for (let phase = 0; phase < 3; ++phase) {
    if (phase) {
      // Stop and restart only while the host is idle, then await target acknowledgement.
      for (let restart = 0; restart < 5; ++restart) {
        const command = (phase - 1) * 5 + restart + 1;
        const data = Buffer.alloc(4);
        data.writeUInt32LE(command);
        await session.request('writeMemory', {
          memoryReference: `0x${(parseInt(address, 16) + 32).toString(16)}`, data: data.toString('base64'),
        });
        for (let attempt = 0; attempt < 100; ++attempt) {
          value = await result();
          if (value.acknowledged === command) break;
          await new Promise(resolve => setTimeout(resolve, 10));
        }
        if (value.acknowledged !== command || value.failures) {
          throw new Error(`USART restart failed: ${JSON.stringify(value)}`);
        }
      }
    }
    const { stdout } = await promisify(execFile)('python', [
      join(dirname(fileURLToPath(import.meta.url)), 'usart.py'),
      '--port', values['usart-port'], '--phase', String(phase),
    ], { encoding: 'utf8', windowsHide: true, timeout: 60000 });
    const host = JSON.parse(stdout);
    bytes += host.bytes;
    console.log(`USART_HOST ${JSON.stringify(host)}`);
  }
  value = await result();
  console.log(`USART_TARGET ${JSON.stringify(value)}`);
  if (value.ready !== 1 || value.failures || value.overflows || value.busy_checks !== 1 ||
      value.rx_bytes !== bytes || value.tx_bytes !== bytes || value.acknowledged !== 10 || !value.full_tx) {
    throw new Error('USART DMA assertions or byte counts failed');
  }
}

try {
  await session.start({ chip: target.chip, elf, probe: probe.selector,
    speedKHz: 4000, rate: 1, allowDebug: true, allowFlash: true });
  const identity = await session.request('readMemory', { memoryReference: target.idAddress, count: 4 });
  const identityData = Buffer.from(identity.data, 'base64');
  if (identityData.length !== 4 || (identityData.readUInt32LE(0) & 0xfff) !== target.deviceId) {
    throw new Error('Connected MCU does not match the test board');
  }
  console.log(`TARGET ${board} ${target.chip} ${probe.serialNumber}`);
  flashAttempted = true;
  await session.flashAndRun(elf);
  if (usart) {
    await testUsart();
  } else {
    const first = await completed();
    console.log(`FIRST_RUN ${JSON.stringify(first)}`);
    await checkTimebase(elf);
    await session.debug('pause');
    const stack = await session.request('stackTrace', { threadId: 1, startFrame: 0, levels: 1 });
    if (!stack.stackFrames?.length) throw new Error('Debugger did not return a halted stack frame');
    console.log(`DEBUG_HALT ${JSON.stringify(stack.stackFrames[0])}`);
    await session.debug('continue');
    await session.request('restart');
    await session.debug('continue');
    const reset = await completed();
    console.log(`RESET_RUN ${JSON.stringify(reset)}`);
    if (!passed(first) || !passed(reset) || reset.retained_marker !== 1 ||
        reset.prior_boot !== first.boot_count || reset.boot_count !== first.boot_count + 1) {
      throw new Error('Target assertions or reset retention failed');
    }
  }
} finally {
  try {
    if (flashAttempted) {
      await session.flashAndRun(restore);
      console.log(`RESTORED ${restore}`);
      await new Promise(resolve => setTimeout(resolve, 100));
      await checkTimebase(restore);
    }
  } finally {
    await session.stop();
  }
}
