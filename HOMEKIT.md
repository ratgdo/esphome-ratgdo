# HomeKit (HAP) Support

This project now includes HomeKit Accessory Protocol (HAP) support for ESP32 boards, allowing direct control of your garage door through Apple HomeKit without requiring Home Assistant as an intermediary.

## Requirements

- ESP32 board (ESP8266 is not supported by HAP-ESPHome)
- Compatible RATGDO hardware (v2.5 or v3.2 boards)
- Apple Home app on iOS/macOS

## Supported Features

When using HAP-enabled configurations, the following RATGDO features are exposed to HomeKit:

- **Garage Door**: Exposed as a switch that toggles the garage door
- **Garage Light**: Exposed as a light that can be turned on/off
- **Remote Lock**: Exposed as a lock for controlling remote access
- **Motion Sensor**: Reports motion detection in the garage
- **Obstruction Sensor**: Reports when the garage door is obstructed

## Available Configurations

### ESP32 HomeKit Configurations

- [V2.5 Board ESP32 D1 Mini with HomeKit](https://github.com/ratgdo/esphome-ratgdo/blob/main/static/v25board_esp32_d1_mini_hap.yaml)
- [V32 Board with HomeKit](https://github.com/ratgdo/esphome-ratgdo/blob/main/static/v32board_hap.yaml)

## Setup Instructions

### 1. Flash HAP-Enabled Firmware

Use the [Web Installer](https://ratgdo.github.io/esphome-ratgdo/) and select one of the HomeKit-enabled configurations for your ESP32 board.

### 2. Initial Setup

1. Power on your RATGDO device
2. Connect to the setup WiFi network created by the device
3. Configure your WiFi credentials through the captive portal
4. Wait for the device to connect to your WiFi network

### 3. Add to HomeKit

1. Open the Apple Home app on your iOS device
2. Tap the "+" button to add an accessory
3. Select "More Options" when prompted
4. Look for "RATGDO HomeKit Bridge" in the list of nearby accessories
5. When prompted for the setup code, enter: **159-35-728**
6. Follow the on-screen instructions to complete pairing
7. Assign the accessories to a room and configure as desired

### 4. Using HomeKit Controls

Once paired, you'll see these accessories in the Home app:

- **Garage Door**: Toggle switch to open/close the garage door
- **Garage Light**: Light control for the garage light
- **Remote Lock**: Lock to enable/disable remote controls
- **Motion Sensor**: Shows motion detection status
- **Obstruction Sensor**: Shows if the door path is obstructed

## Troubleshooting

### HomeKit Pairing Issues

If you have trouble pairing:

1. Ensure your iOS device is on the same WiFi network as the RATGDO
2. Try moving closer to the device during pairing
3. Use the "Reset HomeKit Pairings" button in the RATGDO web interface
4. Restart the RATGDO device and try pairing again

### Garage Door as Switch

The garage door appears as a switch in HomeKit rather than a garage door accessory. This is because:
- HAP-ESPHome doesn't currently support native garage door accessories
- A switch provides the same core functionality (open/close toggle)
- Future updates may add proper garage door accessory support

### Factory Reset HomeKit Pairings

You can reset HomeKit pairings using:
- The "Reset HomeKit Pairings" button in the web interface
- The physical reset button on your RATGDO board
- Re-flashing the firmware

## Technical Details

### HomeKit Setup Code
- Default setup code: `159-35-728`
- Setup ID: `RGDO`

### Network Configuration
- HomeKit runs on port 32042 (configurable)
- Requires mDNS for discovery
- Uses ESP-IDF framework with HomeKit SDK

### Security
- All communication is encrypted using HomeKit's security protocols
- Device keys are generated and stored securely on the ESP32
- Pairing must be done locally (same network)

## Customization

To customize the HomeKit configuration, you can:

1. Fork this repository
2. Modify `base_hap.yaml` to change accessory names, metadata, or configuration
3. Create your own board-specific configuration files
4. Build and flash your custom firmware

See the [HAP-ESPHome documentation](https://github.com/rednblkx/HAP-ESPHome) for more details on customization options.