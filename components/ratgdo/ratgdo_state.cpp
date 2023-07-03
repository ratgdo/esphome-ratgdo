#include "ratgdo_state.h"

namespace esphome {
namespace ratgdo {

    LightState light_state_toggle(LightState state)
    {
        switch (state) {
        case LightState::OFF:
            return LightState::ON;
        case LightState::ON:
            return LightState::OFF;
            // 2 and 3 appears sometimes
        case LightState::UNKNOWN:
        default:
            return LightState::UNKNOWN;
        }
    }

    LockState lock_state_toggle(LockState state)
    {
        switch (state) {
        case LockState::UNLOCKED:
            return LockState::LOCKED;
        case LockState::LOCKED:
            return LockState::UNLOCKED;
            // 2 and 3 appears sometimes
        case LockState::UNKNOWN:
        default:
            return LockState::UNKNOWN;
        }
    }

} // namespace ratgdo
} // namespace esphome
