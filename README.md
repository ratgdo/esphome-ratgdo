
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

## Dry contact toggle behavior

A dry contact opener on the single button input only understands toggle, and what a toggle does depends on the opener. These optional settings describe it; without them, behavior is unchanged.

```yaml
ratgdo:
  id: ratgdov25i
  toggle_while_opening: stop          # ignore, stop or reverse
  toggle_while_closing: stop          # ignore, stop or reverse
  toggle_while_stopped: reverse       # ignore or reverse (opposite to the last direction)
  obstruction_while_opening: ignore   # ignore, stop or reverse
  obstruction_while_closing: reverse  # ignore, stop or reverse
```

The example matches a Genie CM7600IC screw-drive opener.

The three `toggle_while_*` settings go together. Open, Close and Stop then send as many toggles as needed; on the example opener, Close while opening sends one toggle to stop and a second to close. On the example opener, a toggle from a stopped door moves it opposite its last direction, so a request to continue in the last direction first moves the door briefly the wrong way, then stops and reverses it. A toggle that starts the door closing waits for the closing delay, if set. Stop is ignored if the opener stops on a toggle in neither direction. Pressing the wall button while the door moves can leave the inferred status wrong.

`obstruction_while_*` requires the obstruction input. It describes what the opener does by itself when the sensor trips, and the inferred status follows: on the example opener, an obstruction while closing changes the status to opening.

These settings are not available with an encoder.

## Troubleshooting

### False obstruction events on ESP32 v2.5 boards

Some v2.5 boards using ESP32 D1 Mini controllers may report spurious obstruction events. This is caused by the obstruction sensor "asleep" detection logic being inverted on these boards.

To fix this, add the following to your ESPHome config:

```yaml
ratgdo:
  id: ratgdov25
  obst_sleep_low: true
```

The `id` must match the `id` of the existing `ratgdo` component in your config (e.g. `ratgdov25` for v2.5 boards). This setting tells the obstruction sensor to treat LOW as the "asleep" state instead of the ESP32 default of HIGH.
