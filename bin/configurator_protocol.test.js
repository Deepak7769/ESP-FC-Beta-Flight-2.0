// Copied into the pinned Configurator's test/ directory by check_configurator.py.
// The packets are emitted by the real native MspProcessor, not a JS firmware mock.
import { readFileSync, writeFileSync } from 'node:fs';
import { expect, it, vi } from 'vitest';
vi.mock('../src/js/Analytics', () => ({ tracking: {
    sendChangeEvents: vi.fn(), EVENT_CATEGORIES: { FLIGHT_CONTROLLER: 'fc' }
} }));
import MspHelper from '../src/js/msp/MSPHelper';
import MSPCodes from '../src/js/msp/MSPCodes';
import FC from '../src/js/fc';
import Features from '../src/js/Features';
import Beepers from '../src/js/Beepers';
import '../src/js/injected_methods';

it('decodes real firmware packets and serializes supported edits', () => {
    FC.resetState();
    const helper = new MspHelper();
    const packets = JSON.parse(readFileSync(process.env.ESPFC_MSP_FIXTURES, 'utf8'));
    const feed = packet => helper.process_data({ code: packet.code,
        dataView: new DataView(Uint8Array.from(packet.payload).buffer), crcError: false,
        unsupported: false, callbacks: [] });
    for (const code of [MSPCodes.MSP_API_VERSION, MSPCodes.MSP_FC_VARIANT,
        MSPCodes.MSP_FC_VERSION, MSPCodes.MSP_BOARD_INFO, MSPCodes.MSP_BUILD_INFO]) {
        feed(packets.find(p => p.code === code));
    }
    expect(FC.CONFIG.apiVersion).toBe('1.46.0');
    expect(FC.CONFIG.flightControllerVersion).toBe('2026.6.1');
    // Mirrors the serial_backend initialization gate responsible for the regression.
    if (FC.CONFIG.flightControllerVersion !== '') {
        FC.FEATURE_CONFIG.features = new Features(FC.CONFIG);
        FC.BEEPER_CONFIG.beepers = new Beepers(FC.CONFIG);
        FC.BEEPER_CONFIG.dshotBeaconConditions = new Beepers(FC.CONFIG, ['RX_LOST','RX_SET']);
    }
    for (const packet of packets) feed(packet);
    expect(FC.FEATURE_CONFIG.features.isEnabled('RX_SERIAL')).toBe(true);
    expect(FC.RX_CONFIG.serialrx_provider).toBe(7); // Betaflight SERIALRX_IBUS
    expect(FC.CONFIG.buildOptions).toContain('USE_SERIALRX_IBUS');
    expect(FC.MOTOR_CONFIG.motor_count).toBe(4);
    expect(FC.RC.active_channels).toBe(14);
    expect(FC.RC.channels.slice(0,4)).toEqual([1000,1050,1100,1150]);
    expect(FC.MODE_RANGES_EXTRA.length).toBe(8);
    expect(FC.RC_TUNING.rates_type).toBe(3);

    if (process.env.ESPFC_MSP_WRITES) {
        FC.RX_CONFIG.serialrx_provider=7;
        FC.RX_CONFIG.stick_min=1060;
        FC.RC_DEADBAND_CONFIG.deadband=4;
        FC.TUNING_SLIDERS.slider_pids_mode=0; // manual PID editing disables gain sliders
        FC.PIDS[0][0]=37;
        FC.MOTOR_CONFIG.maxthrottle=1950;
        FC.PID_ADVANCED_CONFIG.fast_pwm_protocol=0;
        FC.PID_ADVANCED_CONFIG.use_unsyncedPwm=1;
        FC.PID_ADVANCED_CONFIG.motor_pwm_rate=400;
        FC.PID_ADVANCED_CONFIG.motorIdle=4.5;
        FC.RX_CONFIG.rcSmoothingFeedforwardCutoff=25;
        const codes=[MSPCodes.MSP_SET_FEATURE_CONFIG,MSPCodes.MSP_SET_RX_CONFIG,
            MSPCodes.MSP_SET_RX_MAP,MSPCodes.MSP_SET_RSSI_CONFIG,MSPCodes.MSP_SET_RC_DEADBAND,
            MSPCodes.MSP_SET_RC_TUNING,MSPCodes.MSP_SET_PID,MSPCodes.MSP_SET_PID_ADVANCED,
            MSPCodes.MSP_SET_FILTER_CONFIG,MSPCodes.MSP_SET_MIXER_CONFIG,
            MSPCodes.MSP_SET_MOTOR_CONFIG,MSPCodes.MSP_SET_ADVANCED_CONFIG,
            MSPCodes.MSP_SET_BATTERY_CONFIG,MSPCodes.MSP_SET_FAILSAFE_CONFIG,
            MSPCodes.MSP_SET_SIMPLIFIED_TUNING,MSPCodes.MSP_SET_BEEPER_CONFIG,
            MSPCodes.MSP_SET_ARMING_CONFIG,MSPCodes.MSP_SET_SENSOR_CONFIG,
            MSPCodes.MSP_SET_SENSOR_ALIGNMENT,MSPCodes.MSP_SET_BOARD_ALIGNMENT_CONFIG,
            MSPCodes.MSP_SET_BLACKBOX_CONFIG,MSPCodes.MSP_SET_GPS_CONFIG];
        writeFileSync(process.env.ESPFC_MSP_WRITES,codes.map(code => {
            const payload=helper.crunch(code);
            expect(payload).toBeTruthy();
            return [code,payload.length,...payload].join(' ');
        }).join('\n')+'\n');
    }
    if (process.env.ESPFC_VERIFY_READBACK) {
        expect(FC.RX_CONFIG.stick_min).toBe(1060);
        expect(FC.RC_DEADBAND_CONFIG.deadband).toBe(4);
        expect(FC.PIDS[0][0]).toBe(37);
        expect(FC.MOTOR_CONFIG.maxthrottle).toBe(1950);
        expect(FC.PID_ADVANCED_CONFIG.fast_pwm_protocol).toBe(0);
        expect(FC.PID_ADVANCED_CONFIG.motor_pwm_rate).toBe(400);
        expect(FC.PID_ADVANCED_CONFIG.motorIdle).toBe(4.5);
        expect(FC.RX_CONFIG.rcSmoothingFeedforwardCutoff).toBe(25);
    }
});
