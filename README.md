# SHS01 Enhanced

Enhanced firmware for the [SmartHomeScene SHS01](https://github.com/SmartHomeScene/zigbee-esp32) ESP32-C6 + LD2410C presence sensor.

## What's New

- **Distance measurements** - Moving, static, and detection distance in meters
- **Energy/confidence values** - 0-100 scale for moving and static targets
- **Auto-zeroing** - Distances show 0 when no target detected
- **Renamed settings** - "Movement Detection Range" and "Static Detection Range" (clearer than "max gate")

All original features preserved.

## Z2M Properties

| Property | Type | Description |
|----------|------|-------------|
| `occupancy` | bool | Combined presence (moving OR static) |
| `moving_target` | bool | Moving target detected |
| `static_target` | bool | Static target detected |
| `moving_distance` | number | Distance to moving target (m) |
| `static_distance` | number | Distance to static target (m) |
| `detection_distance` | number | General detection distance (m) |
| `moving_energy` | 0-100 | Moving target confidence |
| `static_energy` | 0-100 | Static target confidence |
| `moving_cooldown` | 0-300s | Delay before moving clears |
| `occupancy_delay` | seconds | Delay before occupancy clears |
| `moving_sensitivity` | 0-10 | Global moving sensitivity |
| `static_sensitivity` | 0-10 | Global static sensitivity |
| `movement_detection_range` | 0-8 | Max range for movement (gates, 0.75m each) |
| `static_detection_range` | 2-8 | Max range for static (gates, 0.75m each) |

## Quick Start

```bash
# Build
idf.py set-target esp32c6
idf.py build
idf.py flash monitor

# Z2M: Copy shs01_enhanced.js to external_converters/
```

## Credits

- Original: [SmartHomeScene/zigbee-esp32](https://github.com/SmartHomeScene/zigbee-esp32)
- LD2410 protocol: HLK-LD2410 Serial Communication Protocol v1.02
