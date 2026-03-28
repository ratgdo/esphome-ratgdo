#pragma once

#include "../ratgdo.h"
#include "../ratgdo_state.h"
#include "esphome/components/lock/lock.h"
#include "esphome/core/component.h"

namespace esphome::ratgdo {

class RATGDOLock : public lock::Lock, public RATGDOClient, public Component {
public:
    void dump_config() override;
    void setup() override;

    void on_lock_state(LockState state);
    void control(const lock::LockCall& call) override;
};

} // namespace esphome::ratgdo
