#include "observable.h"
#include "esphome/core/log.h"

namespace esphome {
namespace ratgdo {

    static const char* const TAG = "ratgdo.observable";

    void log_multiple_subscribers()
    {
        ESP_LOGE(TAG, "single_observable already has a subscriber! This will overwrite the existing subscriber.");
    }

} // namespace ratgdo
} // namespace esphome
