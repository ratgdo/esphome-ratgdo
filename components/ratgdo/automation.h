
#pragma once

#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "ratgdo.h"

namespace esphome {
namespace ratgdo {

    class SyncFailed : public Trigger<> {
    public:
        explicit SyncFailed(RATGDOComponent* parent)
        {
            parent->subscribe_sync_failed([this](bool state) {
                if (state)
                    this->trigger();
            });
        }
    };

    class TTC_Failed : public Trigger<> {
    public:
        explicit TTC_Failed(RATGDOComponent* parent)
        {
            parent->subscribe_ttc_failed([this](bool state) {
                if (state)
                    this->trigger();
            });
        }
    };    

} // namespace ratgdo
} // namespace esphome
