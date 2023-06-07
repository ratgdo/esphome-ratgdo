#include "ratgdo_number.h"
#include "../ratgdo_state.h"
#include "esphome/core/log.h"

namespace esphome {
namespace ratgdo {

    static const char* const TAG = "ratgdo.number";

    void RATGDONumber::dump_config()
    {
        LOG_NUMBER("", "RATGDO Number", this);
        ESP_LOGCONFIG(TAG, "  Type: Rolling Code Counter");
    }

    void RATGDONumber::on_rolling_code_change(uint32_t rollingCodeCounter)
    {
        this->publish_state(rollingCodeCounter);
    }
    void RATGDONumber::control(float value)
    {
        ESP_LOGD(TAG, "name: %s this->type_:%d control: %f", this->get_name(), this->number_type_, value);
        this->parent_->setRollingCodeCounter(value);
        this->publish_state(value);
    }

} // namespace ratgdo
} // namespace esphome
