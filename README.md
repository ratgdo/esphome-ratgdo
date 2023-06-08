
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

web_server:

dashboard_import:
  package_import_url: github://esphome-ratgdo/esphome-ratgdo/ratgdo.yaml@main

packages:
  # Git repo examples
  remote_package:
    url: https://github.com/esphome-ratgdo/esphome-ratgdo
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
  ap:

logger:
  level: VERBOSE
```
