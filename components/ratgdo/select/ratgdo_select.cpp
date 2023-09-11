#include "ratgdo_select.h"
#include "../ratgdo_state.h"
#include "esphome/core/log.h"

namespace esphome {
namespace ratgdo {

    static const char* const TAG = "ratgdo.select";

    void RATGDOSelect::dump_config()
    {
        LOG_SELECT("", "RATGDO Select", this);
        ESP_LOGCONFIG(TAG, "  Type: Time to close (TTC)");
    }

    void RATGDOSelect::setup()
    {
        this->parent_->subscribe_ttc_seconds([=](uint16_t value) {
            if (value == 0) {
                this->publish_state(std::string("Off"));
            } else if (value == 60) {
                this->publish_state(std::string("1 Minute"));
            } else if (value == 300) {
                this->publish_state(std::string("5 Minutes"));
            } else if (value == 600) {
                this->publish_state(std::string("10 Minutes"));
            }
        });
    }

    void RATGDOSelect::control(const std::string &value) 
    {
        if (value.compare("Off") == 0) {
            this->parent_->turn_ttc_off();
            this->publish_state(value);
        } else if (value.compare("1 Minute") == 0) {
            this->parent_->set_ttc_sec(60);
            this->publish_state(value);
        } else if (value.compare("5 Minutes") == 0) {
            this->parent_->set_ttc_sec(300);
            this->publish_state(value);
        } else if (value.compare("10 Minutes") == 0) {
            this->parent_->set_ttc_sec(600);
            this->publish_state(value);
        } else {
            ESP_LOGW(TAG, "Invalid value %s", value.c_str());
        }
    }

} // namespace ratgdo
} // namespace esphome
