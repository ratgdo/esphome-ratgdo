
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

web_server:

packages:
  # Git repo examples
  remote_package:
    url: https://github.com/bdraco/esphome-ratgdo
    files: [base.yml]
    refresh: 1s # optional

# Sync time with Home Assistant.
time:
  - platform: homeassistant
    id: homeassistant_time

api:
  id: api_server

ota:

wifi:
  ssid: ${wifi_ssid}
  password: ${wifi_password}

logger:
  level: VERBOSE

```
