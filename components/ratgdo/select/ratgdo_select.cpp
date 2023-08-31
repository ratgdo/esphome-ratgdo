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
        // this->parent_->subscribe_openings([=](uint16_t value) {
        //     this->publish_state(value);
        // });
    }

    void RATGDOSelect::control(const std::string &value) 
    {
        // if (this->optimistic_)
             this->publish_state(value);

        // auto idx = this->index_of(value);
        // if (idx.has_value()) {
        //     uint8_t mapping = this->mappings_.at(idx.value());
        //     ESP_LOGV(TAG, "Setting %u datapoint value to %u:%s", this->select_id_, mapping, value.c_str());
        //     this->parent_->set_enum_datapoint_value(this->select_id_, mapping);
        //     return;
        // }

        ESP_LOGW(TAG, "Invalid value %s", value.c_str());
    }

} // namespace ratgdo
} // namespace esphome
