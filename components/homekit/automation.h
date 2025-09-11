#pragma once
#include "esphome/core/automation.h"

namespace esphome {
namespace homekit {
  class HKAuthTrigger : public Trigger<std::string, std::string> {
   public:
    explicit HKAuthTrigger() = default;
  };
  
  class HKFailTrigger : public Trigger<> {
   public:
    explicit HKFailTrigger() = default;
  };
}
}