
# ratgdo for ESPHome

[ESPHome](https://esphome.io/) component for [ratgdo](https://ratcloud.llc) — a WiFi control board for garage door openers that works over your local network. Compatible with most residential Chamberlain and LiftMaster openers, with dry contact support for other brands.

Purchase boards at [ratcloud.llc](https://ratcloud.llc).

## Web Installer

Flash the ESPHome based firmware using the [Web Installer](https://ratgdo.github.io/esphome-ratgdo/).

## First use after adding to Home Assistant

The ESPHome firmware will allow you to open the door to any position after calibration. To calibrate the door, open and close it once without stopping.

<img width="560" alt="position_demo" src="https://github.com/RATGDO/esphome-ratgdo/assets/663432/22a9873e-67bb-4b2f-bb32-70047cfe666d">

## ESPHome configs

| Board | Chip | Protocol | Config |
|-------|------|----------|--------|
| v2.0 | ESP8266 D1 Mini | Security+ 2.0 | [v2board_esp8266_d1_mini.yaml](static/v2board_esp8266_d1_mini.yaml) |
| v2.0 | ESP8266 D1 Mini lite | Security+ 2.0 | [v2board_esp8266_d1_mini_lite.yaml](static/v2board_esp8266_d1_mini_lite.yaml) |
| v2.0 | ESP32 D1 Mini | Security+ 2.0 | [v2board_esp32_d1_mini.yaml](static/v2board_esp32_d1_mini.yaml) |
| v2.0 | ESP32 Lolin S2 Mini | Security+ 2.0 | [v2board_esp32_lolin_s2_mini.yaml](static/v2board_esp32_lolin_s2_mini.yaml) |
| v2.5 | ESP8266 D1 Mini | Security+ 2.0 | [v25board_esp8266_d1_mini.yaml](static/v25board_esp8266_d1_mini.yaml) |
| v2.5 | ESP8266 D1 Mini | Security+ 1.0 | [v25board_esp8266_d1_mini_secplusv1.yaml](static/v25board_esp8266_d1_mini_secplusv1.yaml) |
| v2.5 | ESP8266 D1 Mini lite | Security+ 2.0 | [v25board_esp8266_d1_mini_lite.yaml](static/v25board_esp8266_d1_mini_lite.yaml) |
| v2.5 | ESP8266 D1 Mini lite | Security+ 1.0 | [v25board_esp8266_d1_mini_lite_secplusv1.yaml](static/v25board_esp8266_d1_mini_lite_secplusv1.yaml) |
| v2.5 | ESP32 D1 Mini | Security+ 2.0 | [v25board_esp32_d1_mini.yaml](static/v25board_esp32_d1_mini.yaml) |
| v2.5 | ESP32 D1 Mini | Security+ 1.0 | [v25board_esp32_d1_mini_secplusv1.yaml](static/v25board_esp32_d1_mini_secplusv1.yaml) |
| v2.5i/2.52i/2.53i | ESP32 | Security+ 2.0 | [v25iboard.yaml](static/v25iboard.yaml) |
| v2.5i/2.52i/2.53i | ESP32 | Security+ 1.0 | [v25iboard_secplusv1.yaml](static/v25iboard_secplusv1.yaml) |
| v2.5i/2.52i/2.53i | ESP32 | Dry Contact | [v25iboard_drycontact.yaml](static/v25iboard_drycontact.yaml) |
| v3.2 | ESP32 | Security+ 2.0 | [v32board.yaml](static/v32board.yaml) |
| v3.2 | ESP32 | Security+ 1.0 | [v32board_secplusv1.yaml](static/v32board_secplusv1.yaml) |
| v3.2 | ESP32 | Dry Contact | [v32board_drycontact.yaml](static/v32board_drycontact.yaml) |
| v3.2 Disco | ESP32 | Security+ 2.0 | [v32disco.yaml](static/v32disco.yaml) |
| v3.2 Disco | ESP32 | Security+ 1.0 | [v32disco_secplusv1.yaml](static/v32disco_secplusv1.yaml) |
| v3.2 Disco | ESP32 | Dry Contact | [v32disco_drycontact.yaml](static/v32disco_drycontact.yaml) |

![Home Assistant Screen Shot](static/hass.png)

## ESP32 Framework

Most ESP32 boards use the **ESP-IDF** framework. The project originally depended on Arduino, but PR [#577](https://github.com/ratgdo/esphome-ratgdo/pull/577) replaced the SoftwareSerial dependency with hardware UART and RMT peripherals, eliminating the need for Arduino on ESP32. Removing the Arduino layer saves ~1.5KB RAM and ~44KB flash since Arduino is built as an IDF component on top of ESP-IDF, and the smaller firmware means faster OTA updates.

The **v3.2 Disco** board uses the Arduino framework because its VL53L4CX distance sensor library and Wire I2C library require it.

ESP8266 boards continue to use the Arduino framework as ESPHome requires it on that platform.

## Troubleshooting

### False obstruction events

Starting with the v32 board series the obstruction sensor input uses circuitry that causes logic levels to be inverted. If the input pins for the obstruction sensor are not configured correctly this can cause misreads on the obstruction sensor in some instances.

This is accounted for in the configurations provided in this repository. However, if you are creating a custom ESPHome configuration you need to ensure that the pin connected to the obstruction sensor is configured with the `mode` set to `INPUT_PULLUP` and the `inverted` flag set to `true` for these newer boards:

```yaml
ratgdo:
  input_obst_pin:
    number: GPIO4
    mode: INPUT_PULLUP
    inverted: true
```
> [!NOTE]
> There were revisions in which this behavior was controlled using the `obst_sleep_low` flag on the `ratgdo` entry. This setting has been removed and the abovementioned configuration should be used instead. The example shown above is equivalent to `obst_sleep_low` being set to `false` when used with an ESP32 and `obst_sleep_low` being set to `true` when used with an ESP8266.
>
> If you were using `obst_sleep_low` to fix an issue previously, chances are that you don't have to configure anything additionally and you can just drop this setting from your config.
