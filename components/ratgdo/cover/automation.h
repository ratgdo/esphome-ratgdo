#pragma once

#include "esphome/core/component.h"
#include "esphome/core/automation.h"
#include "esphome/components/cover/cover.h"

namespace esphome {
namespace ratgdo {

class CoverOpeningTrigger : public Trigger<> {
 public:
  CoverOpeningTrigger(cover::Cover *a_cover) {
    a_cover->add_on_state_callback([this, a_cover]() {
      if (a_cover->current_operation == cover::COVER_OPERATION_OPENING) {
        this->trigger();
      }
    });
  }
};

class CoverClosingTrigger : public Trigger<> {
 public:
  CoverClosingTrigger(cover::Cover *a_cover) {
    a_cover->add_on_state_callback([this, a_cover]() {
      if (a_cover->current_operation == cover::COVER_OPERATION_CLOSING) {
        this->trigger();
      }
    });
  }
};

} // namespace ratgdo
} // namespace esphome
