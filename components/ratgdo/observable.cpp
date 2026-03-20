#include "observable.h"
#include "esphome/core/log.h"

namespace esphome::ratgdo {

static const char* const TAG = "ratgdo.observable";

void log_multiple_subscribers()
{
    ESP_LOGE(TAG, "single_observable already has a subscriber! This will overwrite the existing subscriber.");
}

void log_observer_overflow()
{
    ESP_LOGE(TAG, "observable has too many subscribers! Ignoring new subscriber.");
}

} // namespace esphome::ratgdo
