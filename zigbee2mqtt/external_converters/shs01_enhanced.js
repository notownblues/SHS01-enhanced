/**
 * @file shs01_enhanced.js
 * @brief Zigbee2MQTT External Converter for SHS01 Presence Sensor
 *
 * Features:
 * - Occupancy/Moving/Static target detection
 * - Distance measurements (moving, static, detection)
 * - Energy/confidence values (0-100)
 * - Configuration: cooldown, delay, sensitivity, detection range
 *
 * Place this file in: zigbee2mqtt/data/external_converters/shs01_enhanced.js
 */

const fz = require('zigbee-herdsman-converters/converters/fromZigbee');
const tz = require('zigbee-herdsman-converters/converters/toZigbee');
const exposes = require('zigbee-herdsman-converters/lib/exposes');
const reporting = require('zigbee-herdsman-converters/lib/reporting');
const e = exposes.presets;
const ea = exposes.access;

// Endpoints
const EP_LIGHT = 1;
const EP_OCC = 2;

// Cluster IDs
const CLUSTER_CONFIG = 0xFDCD;
const CLUSTER_DISTANCE = 0xFDCE;

// Config cluster attributes (0xFDCD on EP1)
const ATTR_MOVING_COOLDOWN = 0x0001;
const ATTR_OCCUPANCY_DELAY = 0x0002;
const ATTR_MOVING_SENSITIVITY = 0x0003;
const ATTR_STATIC_SENSITIVITY = 0x0004;
const ATTR_MOVING_MAX_GATE = 0x0005;
const ATTR_STATIC_MAX_GATE = 0x0006;

// Distance cluster attributes (0xFDCE on EP2)
const ATTR_MOVING_DISTANCE = 0x0001;
const ATTR_STATIC_DISTANCE = 0x0002;
const ATTR_DETECTION_DISTANCE = 0x0003;
const ATTR_MOVING_ENERGY = 0x0004;
const ATTR_STATIC_ENERGY = 0x0005;

// Occupancy cluster manufacturer-specific attributes
const ATTR_MS_MOVING = 0xF001;
const ATTR_MS_STATIC = 0xF002;
const ATTR_MS_MOVING_DISTANCE = 0xF003;
const ATTR_MS_STATIC_DISTANCE = 0xF004;
const ATTR_MS_DETECTION_DISTANCE = 0xF005;
const ATTR_MS_MOVING_ENERGY = 0xF006;
const ATTR_MS_STATIC_ENERGY = 0xF007;

// Data types
const U16 = 0x21;

// Helpers
const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, Number(v)));
const M_PER_GATE = 0.75;
const gateToM = (g) => Number((Math.max(0, Math.min(8, Number(g))) * M_PER_GATE).toFixed(2));

// ============================================================================
// FROM ZIGBEE CONVERTERS (sensor -> Z2M)
// ============================================================================

const fzLocal = {
    // Occupancy states from EP2
    occupancy_states: {
        cluster: 'msOccupancySensing',
        type: ['attributeReport', 'readResponse'],
        convert: (model, msg, publish, options, meta) => {
            const result = {};

            // Standard occupancy
            if (msg.data.hasOwnProperty('occupancy')) {
                result.occupancy = (msg.data.occupancy & 1) === 1;
            }

            // Manufacturer-specific moving/static
            if (msg.data.hasOwnProperty(ATTR_MS_MOVING)) {
                result.moving_target = msg.data[ATTR_MS_MOVING] === 1 || msg.data[ATTR_MS_MOVING] === true;
            }
            if (msg.data.hasOwnProperty(ATTR_MS_STATIC)) {
                result.static_target = msg.data[ATTR_MS_STATIC] === 1 || msg.data[ATTR_MS_STATIC] === true;
            }

            // Distance (cm to m)
            if (msg.data.hasOwnProperty(ATTR_MS_MOVING_DISTANCE)) {
                result.moving_distance = Number((msg.data[ATTR_MS_MOVING_DISTANCE] / 100).toFixed(2));
            }
            if (msg.data.hasOwnProperty(ATTR_MS_STATIC_DISTANCE)) {
                result.static_distance = Number((msg.data[ATTR_MS_STATIC_DISTANCE] / 100).toFixed(2));
            }
            if (msg.data.hasOwnProperty(ATTR_MS_DETECTION_DISTANCE)) {
                result.detection_distance = Number((msg.data[ATTR_MS_DETECTION_DISTANCE] / 100).toFixed(2));
            }

            // Energy
            if (msg.data.hasOwnProperty(ATTR_MS_MOVING_ENERGY)) {
                result.moving_energy = msg.data[ATTR_MS_MOVING_ENERGY];
            }
            if (msg.data.hasOwnProperty(ATTR_MS_STATIC_ENERGY)) {
                result.static_energy = msg.data[ATTR_MS_STATIC_ENERGY];
            }

            return result;
        },
    },

    // Configuration cluster (EP1)
    config_cluster: {
        cluster: CLUSTER_CONFIG,
        type: ['attributeReport', 'readResponse'],
        convert: (model, msg, publish, options, meta) => {
            const result = {};

            if (msg.data.hasOwnProperty(ATTR_MOVING_COOLDOWN)) {
                result.moving_cooldown = msg.data[ATTR_MOVING_COOLDOWN];
            }
            if (msg.data.hasOwnProperty(ATTR_OCCUPANCY_DELAY)) {
                result.occupancy_delay = msg.data[ATTR_OCCUPANCY_DELAY];
            }
            if (msg.data.hasOwnProperty(ATTR_MOVING_SENSITIVITY)) {
                result.moving_sensitivity = msg.data[ATTR_MOVING_SENSITIVITY];
            }
            if (msg.data.hasOwnProperty(ATTR_STATIC_SENSITIVITY)) {
                result.static_sensitivity = msg.data[ATTR_STATIC_SENSITIVITY];
            }
            if (msg.data.hasOwnProperty(ATTR_MOVING_MAX_GATE)) {
                result.movement_detection_range = msg.data[ATTR_MOVING_MAX_GATE];
            }
            if (msg.data.hasOwnProperty(ATTR_STATIC_MAX_GATE)) {
                result.static_detection_range = msg.data[ATTR_STATIC_MAX_GATE];
            }

            return result;
        },
    },

    // Distance cluster (EP2)
    distance_cluster: {
        cluster: CLUSTER_DISTANCE,
        type: ['attributeReport', 'readResponse'],
        convert: (model, msg, publish, options, meta) => {
            const result = {};

            if (msg.data.hasOwnProperty(ATTR_MOVING_DISTANCE)) {
                result.moving_distance = Number((msg.data[ATTR_MOVING_DISTANCE] / 100).toFixed(2));
            }
            if (msg.data.hasOwnProperty(ATTR_STATIC_DISTANCE)) {
                result.static_distance = Number((msg.data[ATTR_STATIC_DISTANCE] / 100).toFixed(2));
            }
            if (msg.data.hasOwnProperty(ATTR_DETECTION_DISTANCE)) {
                result.detection_distance = Number((msg.data[ATTR_DETECTION_DISTANCE] / 100).toFixed(2));
            }
            if (msg.data.hasOwnProperty(ATTR_MOVING_ENERGY)) {
                result.moving_energy = msg.data[ATTR_MOVING_ENERGY];
            }
            if (msg.data.hasOwnProperty(ATTR_STATIC_ENERGY)) {
                result.static_energy = msg.data[ATTR_STATIC_ENERGY];
            }

            return result;
        },
    },
};

// ============================================================================
// TO ZIGBEE CONVERTERS (Z2M -> sensor)
// ============================================================================

const tzLocal = {
    config_settings: {
        key: [
            'moving_cooldown', 'occupancy_delay',
            'moving_sensitivity', 'static_sensitivity',
            'movement_detection_range', 'static_detection_range',
        ],
        convertSet: async (entity, key, value, meta) => {
            const endpoint = meta.device.getEndpoint(EP_LIGHT);
            const attrMap = {
                'moving_cooldown': ATTR_MOVING_COOLDOWN,
                'occupancy_delay': ATTR_OCCUPANCY_DELAY,
                'moving_sensitivity': ATTR_MOVING_SENSITIVITY,
                'static_sensitivity': ATTR_STATIC_SENSITIVITY,
                'movement_detection_range': ATTR_MOVING_MAX_GATE,
                'static_detection_range': ATTR_STATIC_MAX_GATE,
            };

            const attrId = attrMap[key];
            if (attrId !== undefined) {
                let val = parseInt(value);

                if (key.includes('sensitivity')) {
                    val = clamp(val, 0, 10);
                } else if (key === 'movement_detection_range') {
                    val = clamp(val, 0, 8);
                } else if (key === 'static_detection_range') {
                    val = clamp(val, 2, 8);
                } else if (key === 'moving_cooldown') {
                    val = clamp(val, 0, 300);
                }

                await endpoint.write(CLUSTER_CONFIG, {[attrId]: {value: val, type: U16}}, {
                    manufacturerCode: null,
                    disableDefaultResponse: true,
                });

                return {state: {[key]: val}};
            }
        },
        convertGet: async (entity, key, meta) => {
            const endpoint = meta.device.getEndpoint(EP_LIGHT);
            const attrMap = {
                'moving_cooldown': ATTR_MOVING_COOLDOWN,
                'occupancy_delay': ATTR_OCCUPANCY_DELAY,
                'moving_sensitivity': ATTR_MOVING_SENSITIVITY,
                'static_sensitivity': ATTR_STATIC_SENSITIVITY,
                'movement_detection_range': ATTR_MOVING_MAX_GATE,
                'static_detection_range': ATTR_STATIC_MAX_GATE,
            };

            const attrId = attrMap[key];
            if (attrId !== undefined) {
                await endpoint.read(CLUSTER_CONFIG, [attrId]);
            }
        },
    },
};

// ============================================================================
// DEVICE DEFINITION
// ============================================================================

const definition = {
    zigbeeModel: ['SHS01', 'SHS01_Enhanced'],
    model: 'SHS01_Enhanced',
    vendor: 'SmartHomeScene',
    description: 'DIY Zigbee mmWave Presence Sensor with distance and energy data',

    fromZigbee: [
        fz.on_off,
        fzLocal.occupancy_states,
        fzLocal.config_cluster,
        fzLocal.distance_cluster,
    ],

    toZigbee: [
        tz.on_off,
        tzLocal.config_settings,
    ],

    exposes: [
        // ====== PRESENCE STATES ======
        e.binary('occupancy', ea.STATE, true, false)
            .withDescription('Combined presence detection (moving OR static)'),
        e.binary('moving_target', ea.STATE, true, false)
            .withDescription('Moving target detected'),
        e.binary('static_target', ea.STATE, true, false)
            .withDescription('Static target detected'),

        // ====== DISTANCE MEASUREMENTS ======
        exposes.numeric('moving_distance', ea.STATE)
            .withUnit('m')
            .withDescription('Distance to moving target'),
        exposes.numeric('static_distance', ea.STATE)
            .withUnit('m')
            .withDescription('Distance to static target'),
        exposes.numeric('detection_distance', ea.STATE)
            .withUnit('m')
            .withDescription('General detection distance'),

        // ====== ENERGY VALUES ======
        exposes.numeric('moving_energy', ea.STATE)
            .withValueMin(0).withValueMax(100)
            .withDescription('Moving target energy/confidence (0-100)'),
        exposes.numeric('static_energy', ea.STATE)
            .withValueMin(0).withValueMax(100)
            .withDescription('Static target energy/confidence (0-100)'),

        // ====== CONFIGURATION ======
        exposes.numeric('moving_cooldown', ea.ALL)
            .withValueMin(0).withValueMax(300)
            .withUnit('s')
            .withDescription('Moving detection cooldown (seconds)'),
        exposes.numeric('occupancy_delay', ea.ALL)
            .withValueMin(0).withValueMax(65535)
            .withUnit('s')
            .withDescription('Occupancy clear delay - time sensor waits before reporting no presence'),
        exposes.numeric('moving_sensitivity', ea.ALL)
            .withValueMin(0).withValueMax(10)
            .withDescription('Global moving sensitivity (0=off, 10=max)'),
        exposes.numeric('static_sensitivity', ea.ALL)
            .withValueMin(0).withValueMax(10)
            .withDescription('Global static sensitivity (0=off, 10=max)'),
        exposes.numeric('movement_detection_range', ea.ALL)
            .withValueMin(0).withValueMax(8)
            .withDescription('Movement detection range in gates (0-8, each gate = 0.75m, max 6m)'),
        exposes.numeric('static_detection_range', ea.ALL)
            .withValueMin(2).withValueMax(8)
            .withDescription('Static detection range in gates (2-8, each gate = 0.75m, max 6m)'),

        // ====== ONBOARD LED SWITCH ======
        e.switch().withEndpoint('light'),
    ],

    endpoint: (device) => {
        return {
            'light': EP_LIGHT,
            'occupancy': EP_OCC,
        };
    },

    configure: async (device, coordinatorEndpoint, definition, options) => {
        const endpoint2 = device.getEndpoint(EP_OCC);
        await reporting.bind(endpoint2, coordinatorEndpoint, ['msOccupancySensing']);

        try {
            await endpoint2.configureReporting('msOccupancySensing', [{
                attribute: {ID: 0x0000, type: 0x18},
                minimumReportInterval: 0,
                maximumReportInterval: 600,
                reportableChange: 1,
            }]);
        } catch (e) {
            // Ignore
        }

        // Read initial config
        const endpoint1 = device.getEndpoint(EP_LIGHT);
        try {
            await endpoint1.read(CLUSTER_CONFIG, [
                ATTR_MOVING_COOLDOWN, ATTR_OCCUPANCY_DELAY,
                ATTR_MOVING_SENSITIVITY, ATTR_STATIC_SENSITIVITY,
                ATTR_MOVING_MAX_GATE, ATTR_STATIC_MAX_GATE,
            ]);
        } catch (e) {
            // Ignore
        }

        // Read initial distance/energy values
        try {
            await endpoint2.read('msOccupancySensing', [
                ATTR_MS_MOVING_DISTANCE, ATTR_MS_STATIC_DISTANCE,
                ATTR_MS_DETECTION_DISTANCE, ATTR_MS_MOVING_ENERGY, ATTR_MS_STATIC_ENERGY,
            ]);
        } catch (e) {
            // Ignore
        }
    },

    meta: {
        multiEndpoint: true,
    },
};

module.exports = definition;
