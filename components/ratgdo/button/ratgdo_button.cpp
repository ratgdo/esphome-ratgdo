#include "ratgdo_button.h"
#include "../ratgdo_state.h"
#include "esphome/core/log.h"

namespace esphome {
namespace ratgdo {

    static const char* const TAG = "ratgdo.button";

    void RATGDOButton::dump_config()
    {
        LOG_NUMBER("", "RATGDO Button", this);
        ESP_LOGCONFIG(TAG, "  Type: Sync");
    }

    void RATGDOButton::control(float value)
    {
        ESP_LOGD(TAG, "name: %s this->type_:%d", this->get_name(), this->button_type_);
        this->parent_->sync();
    }

} // namespace ratgdo
} // namespace esphome
