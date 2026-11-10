const fs = require('node:fs');
const path = require('node:path');
const assert = require('node:assert/strict');
const base = path.resolve(__dirname, '../..');
if (!process.argv[2]) {
  console.error('Usage: node docs/diagnostics/reproduce-version.cjs <Betaflight-2026.6.1-source-directory>');
  process.exit(2);
}
const clientRoot = path.resolve(process.argv[2]);
const firmware = fs.readFileSync(path.join(base, 'lib/Espfc/src/Connect/MspProcessor.cpp'), 'utf8');
const client = fs.readFileSync(path.join(clientRoot, 'src/js/msp/MSPHelper.js'), 'utf8');
const backend = fs.readFileSync(path.join(clientRoot, 'src/js/serial_backend.js'), 'utf8');
const versionResponse = firmware.match(/case MSP_FC_VERSION:([\s\S]*?)break;/)[1];
assert.equal((versionResponse.match(/r\.writeU8/g) || []).length, 3);
assert(!versionResponse.includes('writePString'));
assert(firmware.includes('MSP_FC_VERSION_YEAR = 2026 - 2000'));
const decoderBody = client.match(/case MSPCodes\.MSP_FC_VERSION: \{([\s\S]*?)break;/)[1];
const decode = new Function('data', 'FC', decoderBody);
function decodeVersion(payload) {
  let offset = 0;
  const data = {
    readU8: () => offset < payload.length ? payload[offset++] : null,
    readU16() { const lo = this.readU8(); const hi = this.readU8(); return lo === null || hi === null ? null : lo | hi << 8; },
  };
  const helper = { getText(data) { const length = data.readU8() || 0; let result = ''; for (let i = 0; i < length; i++) result += String.fromCharCode(data.readU8()); return result; } };
  const FC = { CONFIG: {} };
  decode.call(helper, data, FC);
  return FC.CONFIG.flightControllerVersion;
}
assert(backend.includes('FC.CONFIG.flightControllerVersion !== "" && !isCliOnlyMode()'));
assert(client.includes('FC.FEATURE_CONFIG.features.setMask(data.readU32())'));
assert.equal(decodeVersion([26, 6, 0]), '');
const label = '2026.6.0';
assert.equal(decodeVersion([26, 6, 0, label.length, ...Buffer.from(label)]), label);
assert.equal(decodeVersion([4, 5, 0]), '4.5.0');
console.log('Confirmed against exact client decoder: current firmware version payload decodes to an empty string.');
console.log('Empty version skips Features/Beeper initialization; feature reads then invoke setMask on the numeric initial value.');
console.log('Control cases passed: length-prefixed calendar version and legacy semantic version decode correctly.');
