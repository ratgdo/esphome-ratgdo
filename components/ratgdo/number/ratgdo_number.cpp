#include "ratgdo_number.h"
#include "../ratgdo_state.h"
#include "esphome/core/log.h"

namespace esphome {
namespace ratgdo {

    static const char* const TAG = "ratgdo.number";

    void RATGDONumber::dump_config()
    {
        LOG_NUMBER("", "RATGDO Number", this);
        if (this->number_type_ == RATGDO_ROLLING_CODE_COUNTER) {
            ESP_LOGCONFIG(TAG, "  Type: Rolling Code Counter");
        } else if (this->number_type_ == RATGDO_OPENING_DURATION) {
            ESP_LOGCONFIG(TAG, "  Type: Opening Duration");
        } else if (this->number_type_ == RATGDO_CLOSING_DURATION) {
            ESP_LOGCONFIG(TAG, "  Type: Closing Duration");
        }
    }

    void RATGDONumber::setup()
    {
        if (this->number_type_ == RATGDO_ROLLING_CODE_COUNTER) {
            this->parent_->subscribe_rolling_code_counter([=](uint32_t value) {
                this->publish_state(value);
            });
        } else if (this->number_type_ == RATGDO_OPENING_DURATION) {
            this->parent_->subscribe_opening_duration([=](float value) {
                this->publish_state(value);
            });
        } else if (this->number_type_ == RATGDO_CLOSING_DURATION) {
            this->parent_->subscribe_closing_duration([=](float value) {
                this->publish_state(value);
            });
        }
    }

    void RATGDONumber::set_number_type(NumberType number_type_)
    {
        this->number_type_ = number_type_;
        if (this->number_type_ == RATGDO_OPENING_DURATION || this->number_type_ == RATGDO_CLOSING_DURATION) {
            this->traits.set_step(0.1);
            this->traits.set_min_value(0.0);
            this->traits.set_max_value(300.0);
        }
    }

    void RATGDONumber::control(float value)
    {
        if (this->number_type_ == RATGDO_ROLLING_CODE_COUNTER) {
            this->parent_->setRollingCodeCounter(value);
        } else if (this->number_type_ == RATGDO_OPENING_DURATION) {
            this->parent_->setOpeningDuration(value);
        } else if (this->number_type_ == RATGDO_CLOSING_DURATION) {
            this->parent_->setClosingDuration(value);
        }
    }

} // namespace ratgdo
} // namespace esphome
