// Decode the actual native firmware packet with the pinned client source.
const fs = require('node:fs');
const path = require('node:path');
const assert = require('node:assert/strict');
if (!process.argv[2] || !process.argv[3]) {
  console.error('Usage: node docs/diagnostics/reproduce-version.cjs CLIENT NATIVE-PACKETS.json');
  process.exit(2);
}
const client = fs.readFileSync(path.join(process.argv[2], 'src/js/msp/MSPHelper.js'), 'utf8');
const match = client.match(/case MSPCodes\.MSP_FC_VERSION:\s*(?:\{)?([\s\S]*?)break;/);
assert(match, 'Version decoder not found in supplied client');
const decode = new Function('data', 'FC', match[1]);
function version(payload) {
  let offset = 0;
  const data = {
    readU8: () => offset < payload.length ? payload[offset++] : null,
    readU16() { const lo=this.readU8(), hi=this.readU8(); return lo===null||hi===null ? null : lo|hi<<8; }
  };
  const helper = {getText(data) {const n=data.readU8()||0;let s='';for(let i=0;i<n;++i)s+=String.fromCharCode(data.readU8());return s;}};
  const FC={CONFIG:{}};decode.call(helper,data,FC);return FC.CONFIG.flightControllerVersion;
}
const packets=JSON.parse(fs.readFileSync(process.argv[3], 'utf8'));
const packet=packets.find(p=>p.code===3).payload;
assert.equal(packet.length,4+packet[3]);
assert.equal(Buffer.from(packet.slice(4)).toString(), '2026.6.1');
const modern=client.includes('const major = data.readU8()');
assert.equal(version(packet), modern ? '2026.6.1' : '26.6.1');
if(modern) assert.equal(version(packet.slice(0,3)), '');
console.log('Actual firmware version packet decoded correctly:', version(packet));
