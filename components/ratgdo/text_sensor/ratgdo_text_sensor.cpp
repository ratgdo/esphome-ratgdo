#include "ratgdo_text_sensor.h"
#include "esphome/core/log.h"
#include "esphome/core/progmem.h"

namespace esphome::ratgdo {

static const char* const TAG = "ratgdo.text_sensor";

// User-facing labels for TtcState, kept separate from TtcState_to_string()
// (which stays technical/enum-shaped, since it's also used for logging).
//
// IMPORTANT: if you add a name longer than "Init (disabled)", update
// TTC_STATE_DISPLAY_NAME_BUF_SIZE below too.
static ESPHOME_PGM_P ttc_state_display_name(TtcState state)
{
    switch (state) {
    case TtcState::TTC_ENABLED_READY:
        return ESPHOME_PSTR("Ready");
    case TtcState::TTC_ENABLED_COUNTING:
        return ESPHOME_PSTR("Counting");
    case TtcState::TTC_ENABLED_HOLDING:
        return ESPHOME_PSTR("Holding");
    case TtcState::TTC_DISABLED:
        return ESPHOME_PSTR("Disabled");
    case TtcState::TTC_INITIALIZING_ENABLED:
        return ESPHOME_PSTR("Init (enabled)");
    case TtcState::TTC_INITIALIZING_DISABLED:
        return ESPHOME_PSTR("Init (disabled)");
    case TtcState::TTC_CLOSING_ALERT:
        return ESPHOME_PSTR("Closing alert");
    case TtcState::TTC_UNKNOWN:
    default:
        return ESPHOME_PSTR("Unknown");
    }
}

static constexpr size_t TTC_STATE_DISPLAY_NAME_BUF_SIZE = sizeof("Init (disabled)");

void RATGDOTextSensor::setup()
{
    switch (this->ratgdo_text_sensor_type_) {
    case RATGDOTextSensorType::RATGDO_TTC_STATE: {
        auto publish_ttc_state = [this](TtcState state) {
            // ttc_state_display_name() returns a PROGMEM-backed pointer on
            // ESP8266, so copy it into a regular char buffer on the stack
            // before publishing.
            char buf[TTC_STATE_DISPLAY_NAME_BUF_SIZE];
            ESPHOME_strncpy_P(buf, ttc_state_display_name(state), sizeof(buf) - 1);
            buf[sizeof(buf) - 1] = '\0';
            this->publish_state(buf);
        };

        // The subscriber will only be called for future changes, not initial values.
        // So publish once here too so the UI is initialized.
        publish_ttc_state(*this->parent_->ttc_state);

        // subscribe to future changes too.
        this->parent_->subscribe_ttc_state(publish_ttc_state);
        break;
    }
    default:
        break;
    }
}

void RATGDOTextSensor::dump_config()
{
    LOG_TEXT_SENSOR("", "RATGDO Text Sensor", this);
    switch (this->ratgdo_text_sensor_type_) {
    case RATGDOTextSensorType::RATGDO_TTC_STATE:
        ESP_LOGCONFIG(TAG, "  Type: TTC State");
        break;
    default:
        break;
    }
}

} // namespace esphome::ratgdo
