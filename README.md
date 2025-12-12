# SHS01 Enhanced - DIY Zigbee mmWave Presence Sensor

Enhanced firmware for the SmartHomeScene SHS01 ESP32-C6 + LD2410C presence sensor.

## New Features (vs Original SHS01)

| Feature | Original | Enhanced |
|---------|----------|----------|
| Occupancy states | Yes (3) | Yes (3) |
| **Distance measurements** | No | **Yes** (moving, static, detection in cm) |
| **Energy values** | No | **Yes** (0-100 confidence) |
| **Per-gate energy** | No | **Yes** (9 gates, engineering mode) |
| **Per-gate sensitivity** | No | **Yes** (configurable 0-100 per gate) |
| **Firmware version** | No | **Yes** (read from LD2410) |
| **Engineering mode toggle** | No | **Yes** |
| Zigbee clusters | 1 custom | 3 custom (config, distance, gates) |

## Hardware Requirements

- ESP32-C6 development board
- LD2410C mmWave radar sensor
- Wiring:
  - LD2410C TX -> ESP32-C6 GPIO4 (RX)
  - LD2410C RX -> ESP32-C6 GPIO5 (TX)
  - LD2410C VCC -> 3.3V or 5V
  - LD2410C GND -> GND

## Building the Firmware

### Prerequisites

1. Install ESP-IDF v5.0 or later
2. Set up the ESP-IDF environment

### Build Commands

```bash
# Navigate to project directory
cd SHS01-enhanced

# Set target to ESP32-C6
idf.py set-target esp32c6

# Build the project
idf.py build

# Flash to device
idf.py -p /dev/ttyUSB0 flash monitor
```

### VS Code with ESP-IDF Extension

1. Open the `SHS01-enhanced` folder in VS Code
2. Install the "ESP-IDF" extension
3. Press F1 -> "ESP-IDF: Set Espressif Device Target" -> Select ESP32-C6
4. Press F1 -> "ESP-IDF: Build your Project"
5. Press F1 -> "ESP-IDF: Flash your Project"

## Zigbee2MQTT Integration

### Install External Converter

1. Copy `zigbee2mqtt/external_converters/shs01_enhanced.js` to your Zigbee2MQTT data folder:
   ```bash
   cp shs01_enhanced.js /path/to/zigbee2mqtt/data/external_converters/
   ```

2. Add to your `configuration.yaml`:
   ```yaml
   external_converters:
     - shs01_enhanced.js
   ```

3. Restart Zigbee2MQTT

### Exposed Properties

#### States (read-only)
- `occupancy` - Combined presence (moving OR static)
- `moving_target` - Moving target detected
- `static_target` - Static target detected

#### Distance (read-only, NEW)
- `moving_distance` - Distance to moving target (cm)
- `static_distance` - Distance to static target (cm)
- `detection_distance` - General detection distance (cm)

#### Energy (read-only, NEW)
- `moving_energy` - Moving confidence (0-100)
- `static_energy` - Static confidence (0-100)

#### Configuration (read/write)
- `moving_cooldown` - Moving detection cooldown (0-300s)
- `occupancy_delay` - Occupancy clear delay (0-65535s)
- `moving_sensitivity` - Global moving sensitivity (0-10)
- `static_sensitivity` - Global static sensitivity (0-10)
- `moving_max_gate` - Max moving detection gate (0-8)
- `static_max_gate` - Max static detection gate (2-8)
- `sensor_timeout` - Sensor built-in timeout
- `engineering_mode` - Enable per-gate energy data

#### Per-Gate Sensitivity (read/write, NEW)
- `gate_0_move_sensitivity` through `gate_8_move_sensitivity` (0-100)
- `gate_0_still_sensitivity` through `gate_8_still_sensitivity` (0-100)

#### Per-Gate Energy (read-only, engineering mode, NEW)
- `gate_0_move_energy` through `gate_8_move_energy` (0-100)
- `gate_0_still_energy` through `gate_8_still_energy` (0-100)

## Use Cases

### Bed Exclusion Zone
Configure gates 0-2 with low sensitivity to ignore bed movement while detecting room entry.

### Distance-Based Automation
Use `moving_distance` or `static_distance` to trigger different automations based on proximity.

### Sensitivity Tuning
Enable engineering mode to see real-time per-gate energy, then tune individual gate sensitivities.

## Factory Reset

Hold the BOOT button for 6 seconds to factory reset the Zigbee configuration and rejoin the network.

## Project Structure

```
SHS01-enhanced/
├── CMakeLists.txt              # ESP-IDF project file
├── sdkconfig.defaults          # Build configuration
├── partitions.csv              # Flash partition table
├── main/
│   ├── CMakeLists.txt
│   ├── idf_component.yml       # Component dependencies
│   ├── shs01.h                 # Main header with definitions
│   └── shs01.c                 # Main application
├── components/
│   ├── ld2410_enhanced/        # Enhanced LD2410 driver
│   │   ├── include/ld2410_enhanced.h
│   │   └── src/ld2410_enhanced.c
│   ├── light_driver/           # LED driver
│   └── zcl_utility/            # ZCL helper
└── zigbee2mqtt/
    └── external_converters/
        └── shs01_enhanced.js   # Z2M converter
```

## Credits

- Original SHS01 firmware: [SmartHomeScene](https://github.com/SmartHomeScene/zigbee-esp32)
- Enhanced by Claude for Thibault
- LD2410 protocol reference: HLK-LD2410 Serial Communication Protocol v1.02
# SHS01-enhanced
