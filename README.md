
# ratgdo for ESPHome

This is a port of the ratgdo software for the v2.0/v2.5 board to ESPHome.

[Visit the github.io page to purchase boards](https://paulwieland.github.io/ratgdo/#order)

## Installation

- Flash the ESPHome based firmware using the [Web Installer](https://ratgdo.github.io/esphome-ratgdo/)

It is no longer necessary to save the rolling code counter when switching between firmware.

## First use after adding to Home Assistant

The ESPHome firmware will allow you to open the door to any position after calibration. To calibrate the door, open and close it once without stopping.

<img width="560" alt="position_demo" src="https://github.com/RATGDO/esphome-ratgdo/assets/663432/22a9873e-67bb-4b2f-bb32-70047cfe666d">

## ESPHome config

- [ESPHome config for v2.0 board with ESP8266 D1 Mini lite](https://github.com/RATGDO/esphome-ratgdo/blob/main/static/v2board_esp8266_d1_mini_lite.yaml)
- [ESPHome config for v2.0 board with ESP32 D1 Mini](https://github.com/RATGDO/esphome-ratgdo/blob/main/static/v2board_esp32_d1_mini.yaml)
- [ESPHome config for v2.0 board with ESP32 Lolin D2 Mini](https://github.com/RATGDO/esphome-ratgdo/blob/main/static/v2board_esp32_lolin_s2_mini.yaml)
- [ESPHome config for v2.5 board with ESP8266 D1 Mini lite](https://github.com/RATGDO/esphome-ratgdo/blob/main/static/v25board_esp8266_d1_mini_lite.yaml)
- [ESPHome config for v2.5 board with ESP32 D1 Mini](https://github.com/RATGDO/esphome-ratgdo/blob/main/static/v25board_esp32_d1_mini.yaml)
- [Web Installer](https://ratgdo.github.io/esphome-ratgdo/)

![Home Assistant Screen Shot](static/hass.png)
