#include "ratgdo_lock.h"
#include "../ratgdo_state.h"
#include "esphome/core/log.h"

namespace esphome {
namespace ratgdo {

    static const char* const TAG = "ratgdo.lock";

    void RATGDOLock::dump_config()
    {
        LOG_LOCK("", "RATGDO Lock", this);
        ESP_LOGCONFIG(TAG, "  Type: Lock");
    }

    void RATGDOLock::setup()
    {
        this->parent_->subscribe_lock_state([=](LockState state) {
            this->on_lock_state(state);
        });
    }

    void RATGDOLock::on_lock_state(LockState state)
    {
        if (state == LockState::LOCKED && this->state == lock::LockState::LOCK_STATE_LOCKED) {
            return;
        }
        if (state == LockState::UNLOCKED && this->state == lock::LockState::LOCK_STATE_UNLOCKED) {
            return;
        }

        auto call = this->make_call();
        if (state == LockState::LOCKED) {
            call.set_state(lock::LockState::LOCK_STATE_LOCKED);
        } else if (state == LockState::UNLOCKED) {
            call.set_state(lock::LockState::LOCK_STATE_UNLOCKED);
        }
        this->control(call);
    }

    void RATGDOLock::control(const lock::LockCall& call)
    {
        auto state = *call.get_state();

        if (state == lock::LockState::LOCK_STATE_LOCKED) {
            this->parent_->lock();
        } else if (state == lock::LockState::LOCK_STATE_UNLOCKED) {
            this->parent_->unlock();
        }

        this->publish_state(state);
    }

} // namespace ratgdo
} // namespace esphome
