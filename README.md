
# ratgdo

This is a port of the ratgdo software for the v2 board to esphome.

> **ratgdo shields available to order**
> Shields are available and shipping domestic USA via USPS.
>
> * [ratgdo shield only](https://square.link/u/xNP2Orez) $15
> * [ratgdo shield with ESP8266 D1 Clone](https://square.link/u/JaMwtjLL) $30

# [Visit the github.io page for instructions](https://paulwieland.github.io/ratgdo/).
[ratgdo on GitHub.io](https://paulwieland.github.io/ratgdo/)

# ESPHome config

```yaml
---
substitutions:
  id_prefix: ratgdo
  friendly_name: "Garage"
  wifi_ssid: <FILL IN SSID>
  wifi_password: <FILL IN PASSWORD>


esphome:
  name: ${id_prefix}
  platform: ESP8266
  board: esp01_1m

api:
  id: api_server

web_server:

external_components:
  - source:
      type: git
      url: https://github.com/bdraco/esphome-ratgdo
    refresh: 1s

ratgdo:
  id: ${id_prefix}

binary_sensor:
  - platform: ratgdo
    type: motion
    id: ${id_prefix}_motion
    ratgdo_id: ${id_prefix}
    name: "${friendly_name} Motion"
    device_class: motion
  - platform: ratgdo
    type: obstruction
    id: ${id_prefix}_obstruction
    ratgdo_id: ${id_prefix}
    name: "${friendly_name} Obstruction"
    device_class: problem

number:
  - platform: ratgdo
    id: ${id_prefix}_rolling_code_counter
    type: rolling_code_counter
    entity_category: config
    ratgdo_id: ${id_prefix}
    name: "${friendly_name} Rolling Code Counter"

cover:
  - platform: ratgdo
    id: ${id_prefix}_garage
    device_class: garage
    name: ${friendly_name}
    ratgdo_id: ${id_prefix}


light:
  - platform: ratgdo
    id: ratgdo_light
    name: "${friendly_name} Light"
    ratgdo_id: ${id_prefix}

uart:
  tx_pin: 
      number: 2
      inverted: true
  rx_pin:
      number: 4
      inverted: true
  baud_rate: 9600

wifi:
  ssid: ${wifi_ssid}
  password: ${wifi_password}


logger:
  level: VERBOSE

ota:

button:
  - platform: restart
    name: "${friendly_name} Restart"

# Sync time with Home Assistant.
time:
  - platform: homeassistant
    id: homeassistant_time


```
