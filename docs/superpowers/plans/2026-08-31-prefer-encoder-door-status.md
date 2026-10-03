# Prefer Encoder Door Status Switch Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a persistent runtime switch entity (`prefer_encoder_status`) that allows users to prefer physical encoder status over wall panel/opener wire protocol status for door state resolution in encoder-enabled ratgdo firmwares (Security+ 1.0, Security+ 2.0, and Dry Contact).

**Architecture:** Extend `RATGDOSwitch` to add `prefer_encoder_status` with NVS flash persistence via `global_preferences`. In `RATGDOComponent`, add state arbitration so that when `prefer_encoder_status` is enabled, `encoder_received()` directly drives `set_resolved_door_state()`, wire protocol status updates in `received()` do not overwrite resolved door state, and startup `sync()` derives initial door state from calibrated encoder bounds. Add the switch and `reverse_encoder` to `base_secplusv1_enc.yaml`, `base_enc.yaml`, and `base_drycontact_enc.yaml`.

**Tech Stack:** C++ (ESPHome component, ESP-IDF / Arduino), Python (ESPHome codegen and config validation), YAML (ESPHome configurations), Pytest.

## Global Constraints
- No em-dashes or dashes used as sentence separators in code comments, commit messages, or documentation.
- ESP32 builds use ESP-IDF; do not introduce Arduino-only APIs on ESP32 paths.
- Ensure all observable counters are appropriately sized via Python codegen (`subscribe_*`).
- Maintain formatting compliant with `ruff` and `clang-format -style=Webkit`.
- One logical change per PR with concise, descriptive commit messages.

---

### Task 1: Add `prefer_encoder_status` Switch Type and NVS Persistence

**Files:**
- Modify: `components/ratgdo/switch/ratgdo_switch.h`
- Modify: `components/ratgdo/switch/ratgdo_switch.cpp`
- Modify: `components/ratgdo/switch/__init__.py`
- Test: `tests/components/test_validate_unique.py`

**Interfaces:**
- Produces: `SwitchType::RATGDO_PREFER_ENCODER_STATUS` enum, `"prefer_encoder_status"` YAML switch type, NVS preference `"ratgdo_prefer_encoder_status"`, and call to `parent_->set_prefer_encoder_status(bool)`.

- [ ] **Step 1: Update Python codegen schema**

In `components/ratgdo/switch/__init__.py`, register `prefer_encoder_status`:

```python
TYPES = {
    "learn": SwitchType.RATGDO_LEARN,
    "led": SwitchType.RATGDO_LED,
    "reverse_encoder": SwitchType.RATGDO_REVERSE_ENCODER,
    "prefer_encoder_status": SwitchType.RATGDO_PREFER_ENCODER_STATUS,
}
```

- [ ] **Step 2: Update switch header**

In `components/ratgdo/switch/ratgdo_switch.h`, add `RATGDO_PREFER_ENCODER_STATUS` to `SwitchType`:

```cpp
enum SwitchType {
    RATGDO_LEARN,
    RATGDO_LED,
    RATGDO_REVERSE_ENCODER,
    RATGDO_PREFER_ENCODER_STATUS,
};
```

- [ ] **Step 3: Update switch implementation**

In `components/ratgdo/switch/ratgdo_switch.cpp`:
1. In `dump_config()`:
```cpp
    case SwitchType::RATGDO_PREFER_ENCODER_STATUS:
        ESP_LOGCONFIG(TAG, "  Type: Prefer Encoder Status");
        break;
```
2. In `setup()`:
```cpp
#ifdef RATGDO_USE_ENCODER
    case SwitchType::RATGDO_PREFER_ENCODER_STATUS:
        this->pref_ = global_preferences->make_preference<bool>(fnv1_hash("ratgdo_prefer_encoder_status"));
        {
            bool stored = false;
            if (!this->pref_.load(&stored)) {
                ESP_LOGW(TAG, "Failed to load prefer_encoder_status preference. Defaulting to false.");
            }
            this->parent_->set_prefer_encoder_status(stored);
            this->publish_state(stored);
        }
        break;
#endif
```
3. In `write_state(bool state)`:
```cpp
#ifdef RATGDO_USE_ENCODER
    case SwitchType::RATGDO_PREFER_ENCODER_STATUS:
        if (!this->pref_.save(&state)) {
            ESP_LOGW(TAG, "Failed to save prefer_encoder_status preference.");
            return;
        }
        this->parent_->set_prefer_encoder_status(state);
        this->publish_state(state);
        break;
#endif
```

- [ ] **Step 4: Add unit test for switch validation**

In `tests/components/test_validate_unique.py`, add a test for `switch` validation:
```python
def test_switch_unique_types() -> None:
    validate_unique("switch", "prefer_encoder_status", "dup")
    with pytest.raises(cv.Invalid):
        validate_unique("switch", "prefer_encoder_status", "dup")
```

- [ ] **Step 5: Run tests**

Run: `.venv/bin/pytest tests/ -v`
Expected: PASS

- [ ] **Step 6: Commit**

```bash
git add components/ratgdo/switch/ tests/components/test_validate_unique.py
git commit -m "feat(switch): add prefer_encoder_status switch type"
```

---

### Task 2: Implement Door Status Arbitration in `RATGDOComponent`

**Files:**
- Modify: `components/ratgdo/ratgdo.h`
- Modify: `components/ratgdo/ratgdo.cpp`

**Interfaces:**
- Consumes: `parent_->set_prefer_encoder_status(bool)` from Task 1.
- Produces: State arbitration where `encoder_received()` directly resolves door state and `received()` protocol updates are suppressed from overriding resolved door state when `prefer_encoder_status` is true.

- [ ] **Step 1: Update `ratgdo.h` flags and setter**

In `components/ratgdo/ratgdo.h`:
1. Under `public:` `#ifdef RATGDO_USE_ENCODER`:
```cpp
    void set_prefer_encoder_status(bool p) { this->flags_.prefer_encoder_status = p; }
```
2. In `flags_` struct under `#ifdef RATGDO_USE_ENCODER`:
```cpp
        uint8_t prefer_encoder_status : 1;
```

- [ ] **Step 2: Update `RATGDOComponent::received` in `ratgdo.cpp`**

In `components/ratgdo/ratgdo.cpp`, in `RATGDOComponent::received(const DoorState door_state)`:
```cpp
void RATGDOComponent::received(const DoorState door_state)
{
#ifdef RATGDO_USE_ENCODER
    if (this->flags_.prefer_encoder_status && this->encoder_sensor_ != nullptr) {
        // When preferring encoder, track protocol state internally for diagnostics
        // but do not override the resolved door state.
        this->protocol_door_state_ = door_state;
        return;
    }

    bool protocol_state_changed = false;
    if (this->protocol_door_state_ != door_state) {
        protocol_state_changed = true;
        this->protocol_door_state_ = door_state;
    }

    if (protocol_state_changed || door_state == DoorState::OPENING || door_state == DoorState::CLOSING || door_state == this->encoder_door_state_) {
        this->encoder_motion_onset_ms_ = 0; // Protocol caught up, clear any pending manual operation trip
        if (*this->manually_operated_state != ManuallyOperatedState::NO) {
            this->manually_operated_state = ManuallyOperatedState::NO;
        }
    } else {
        if (*this->manually_operated_state == ManuallyOperatedState::YES) {
            ESP_LOGW(TAG, "Dropping protocol state %s due to latched manual operation", LOG_STR_ARG(DoorState_to_string(door_state)));
            return;
        }
    }
#endif

    this->set_resolved_door_state(door_state);
}
```

- [ ] **Step 3: Update `RATGDOComponent::encoder_received` in `ratgdo.cpp`**

In `components/ratgdo/ratgdo.cpp`, in `RATGDOComponent::encoder_received(const DoorState door_state)`:
```cpp
#ifdef RATGDO_USE_ENCODER
void RATGDOComponent::encoder_received(const DoorState door_state)
{
    this->encoder_door_state_ = door_state;

    if (this->flags_.prefer_encoder_status) {
        this->set_resolved_door_state(door_state);
        return;
    }

    auto proto_state = this->protocol_door_state_;

    if (proto_state == DoorState::UNKNOWN) {
        this->set_resolved_door_state(door_state);
        return;
    }

    if ((door_state == DoorState::OPENING || door_state == DoorState::CLOSING) && (proto_state == DoorState::OPEN || proto_state == DoorState::CLOSED || proto_state == DoorState::STOPPED)) {
        if (this->encoder_motion_onset_ms_ == 0) {
            this->encoder_motion_onset_ms_ = millis();
        } else if (millis() - this->encoder_motion_onset_ms_ > PROTOCOL_STALE_MS) {
            if (*this->manually_operated_state != ManuallyOperatedState::YES) {
                this->manually_operated_state = ManuallyOperatedState::YES;
            }
            this->set_resolved_door_state(door_state);
        }
    } else {
        this->encoder_motion_onset_ms_ = 0;
        if (door_state == DoorState::STOPPED || door_state == DoorState::OPEN || door_state == DoorState::CLOSED) {
            if (*this->manually_operated_state == ManuallyOperatedState::YES) {
                this->set_resolved_door_state(door_state);
            }
        }
    }
}
#endif
```

- [ ] **Step 4: Update `RATGDOComponent::sync` in `ratgdo.cpp` for initial encoder state**

In `components/ratgdo/ratgdo.cpp`, in `RATGDOComponent::sync()`:
Allow encoder-calibrated initial state resolution when `flags_.prefer_encoder_status` is active across all protocols:
```cpp
#ifdef RATGDO_USE_ENCODER
    if (this->encoder_sensor_ != nullptr && this->flags_.prefer_encoder_status) {
        if (this->flags_.enc_min_cal && this->flags_.enc_max_cal && this->enc_max_ != this->enc_min_) {
            int16_t target_closed = this->flags_.reverse_encoder ? this->enc_max_ : this->enc_min_;
            int16_t target_open = this->flags_.reverse_encoder ? this->enc_min_ : this->enc_max_;
            int16_t dist_closed = static_cast<int16_t>(std::abs(this->enc_last_ - target_closed));
            int16_t dist_open = static_cast<int16_t>(std::abs(this->enc_last_ - target_open));
            float pos = (float)(this->enc_last_ - this->enc_min_) / (float)(this->enc_max_ - this->enc_min_);
            if (this->flags_.reverse_encoder)
                pos = 1.0f - pos;
            this->door_position = clamp(pos, 0.0f, 1.0f);
            if (dist_closed <= 1 && dist_closed <= dist_open) {
                this->set_resolved_door_state(DoorState::CLOSED);
            } else if (dist_open <= 1 && dist_open < dist_closed) {
                this->set_resolved_door_state(DoorState::OPEN);
            } else {
                this->set_resolved_door_state(DoorState::STOPPED);
            }
        }
    }
#endif
```

- [ ] **Step 5: Run tests**

Run: `.venv/bin/pytest tests/ -v`
Expected: PASS

- [ ] **Step 6: Commit**

```bash
git add components/ratgdo/ratgdo.h components/ratgdo/ratgdo.cpp
git commit -m "feat(ratgdo): add prefer_encoder_status arbitration logic"
```

---

### Task 3: Update Base YAML Configurations

**Files:**
- Modify: `base_secplusv1_enc.yaml`
- Modify: `base_enc.yaml`
- Modify: `base_drycontact_enc.yaml`

- [ ] **Step 1: Add switches to `base_secplusv1_enc.yaml`**

In `base_secplusv1_enc.yaml`, add under `switch:`:
```yaml
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

- [ ] **Step 2: Add switches to `base_enc.yaml`**

In `base_enc.yaml`, add under `switch:`:
```yaml
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

- [ ] **Step 3: Add `prefer_encoder_status` switch to `base_drycontact_enc.yaml`**

In `base_drycontact_enc.yaml`, add under `switch:`:
```yaml
  - platform: ratgdo
    id: ${id_prefix}_prefer_encoder_status
    type: prefer_encoder_status
    name: "Prefer Encoder Door Status"
    entity_category: config
```

- [ ] **Step 4: Validate configs**

Run: `.venv/bin/pytest tests/ -v`
Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add base_secplusv1_enc.yaml base_enc.yaml base_drycontact_enc.yaml
git commit -m "feat(yaml): expose reverse_encoder and prefer_encoder_status switches"
```

---

### Task 4: Full Validation & Pre-Commit Checks

**Files:**
- Check all modified files across the repo

- [ ] **Step 1: Run pytest suite**

Run: `.venv/bin/pytest tests/ -v`
Expected: All tests pass cleanly.

- [ ] **Step 2: Run CI reference update script test**

Run: `.venv/bin/python3 scripts/update_refs_for_ci.py --help` (or test with dry-run)
Expected: Clean exit code 0.

- [ ] **Step 3: Check git status and diff hygiene**

Check for trailing whitespace, clean diffs, and ensure no local build artifacts are tracked:
```bash
git status
git diff main...HEAD
```
