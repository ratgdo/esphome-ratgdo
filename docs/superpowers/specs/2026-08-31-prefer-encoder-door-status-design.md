# Design: Prefer Encoder Door Status Switch

## Overview
This design adds a configurable runtime switch (`prefer_encoder_status`) to the ratgdo ESPHome component for encoder-enabled configurations (Security+ 1.0, Security+ 2.0, and Dry Contact).

Currently, in Security+ 1.0 and 2.0 encoder builds, the door status is primarily driven by the opener/wall panel wire protocol. Encoder updates only override protocol state if motion continues for over 500ms without protocol acknowledgement (latched as manual operation). This feature allows users to select the physical encoder as the authoritative source for door status directly from the ratgdo web interface and Home Assistant.

Additionally, this change ensures the `reverse_encoder` switch is exposed in `base_secplusv1_enc.yaml` and `base_enc.yaml`, aligning them with `base_drycontact_enc.yaml`.

## Architecture & Components

### 1. Switch Component (`components/ratgdo/switch/`)
- **Enum Extension**: Add `RATGDO_PREFER_ENCODER_STATUS` to `SwitchType` in `ratgdo_switch.h`.
- **Python Codegen**: Add `"prefer_encoder_status": SwitchType.RATGDO_PREFER_ENCODER_STATUS` in `components/ratgdo/switch/__init__.py`.
- **Preference Storage**: In `RATGDOSwitch::setup()` and `write_state()`, persist the boolean setting to NVS flash via `global_preferences->make_preference<bool>(fnv1_hash("ratgdo_prefer_encoder_status"))`, defaulting to `false`.
- **Runtime Forwarding**: Forward state changes to `RATGDOComponent::set_prefer_encoder_status(bool)`.

### 2. Door State Arbitration (`components/ratgdo/ratgdo.{h,cpp}`)
- **Component State**: Add `uint8_t prefer_encoder_status : 1;` to `flags_` in `RATGDOComponent` under `#ifdef RATGDO_USE_ENCODER`, with setter `set_prefer_encoder_status(bool)`.
- **Protocol Update Handling (`received(DoorState)`)**:
  - When `flags_.prefer_encoder_status` is true and `encoder_sensor_ != nullptr`, store `protocol_door_state_ = door_state` for diagnostics/internal tracking, but do not call `set_resolved_door_state()`.
- **Encoder Update Handling (`encoder_received(DoorState)`)**:
  - When `flags_.prefer_encoder_status` is true, immediately forward `door_state` to `set_resolved_door_state(door_state)`.
  - When `flags_.prefer_encoder_status` is false, retain existing manual operation fallback logic.
- **Sync/Startup Handling (`sync()`)**:
  - When `flags_.prefer_encoder_status` is true and the encoder is calibrated, compute initial door state from saved calibrated boundaries.

### 3. YAML Base Files Update
Expose both `reverse_encoder` and `prefer_encoder_status` switches in:
- `base_secplusv1_enc.yaml`
- `base_enc.yaml`
- `base_drycontact_enc.yaml`

```yaml
switch:
  - platform: ratgdo
    id: ${id_prefix}_reverse_encoder
    type: reverse_encoder
    name: "Encoder Reverse"
    entity_category: config

  - platform: ratgdo
    id: ${id_prefix}_prefer_encoder_status
    type: prefer_encoder_status
    name: "Prefer Encoder Door Status"
    entity_category: config
```

## Error Handling & Edge Cases
- **Uncalibrated Encoder**: If `prefer_encoder_status` is enabled on first boot before the encoder has learned open/close limits, door state remains `UNKNOWN` until the first calibration cycle or motion detection.
- **Reboot Persistence**: NVS flash storage ensures the toggle preference survives power cycles and reboots.
- **Web UI & Home Assistant**: Switch entities marked with `entity_category: config` are automatically published to both the ESPHome web dashboard and Home Assistant via native API.

## Testing & Verification Plan
1. **Static Validation**:
   - `pre-commit run --all-files` (or `ruff` + `clang-format`).
   - `pytest tests/ -v`.
   - `esphome config` on all encoder board YAMLs:
     - `v25iboard_secplusv1_enc.yaml`
     - `v32board_secplusv1_enc.yaml`
     - `v32disco_secplusv1_enc.yaml`
     - `v25iboard_enc.yaml`
     - `v32board_enc.yaml`
     - `v32disco_enc.yaml`
     - `v25iboard_drycontact_enc.yaml`
     - `v32board_drycontact_enc.yaml`
     - `v32disco_drycontact_enc.yaml`
2. **Hardware Verification (by Operator)**:
   - Flash binary to ratgdo board.
   - Verify presence of "Prefer Encoder Door Status" and "Encoder Reverse" in web UI.
   - Test door operation with toggle ON (door state follows encoder ticks directly).
   - Test door operation with toggle OFF (door state follows wall panel protocol).
