
#pragma once

#include "esphome/core/component.h"
#include "esphome/core/automation.h"
#include "ratgdo.h"

namespace esphome {
namespace ratgdo {

class SyncFailed: public Trigger<> {
public:
  explicit SyncFailed(RATGDOComponent *parent) {
    parent->subscribe_sync_failed([this](bool state) {
      if (state)
        this->trigger();
    });
  }
};

}
}