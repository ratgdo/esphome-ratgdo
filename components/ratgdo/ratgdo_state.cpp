#include "ratgdo_state.h"

#include "esphome/core/log.h"

namespace esphome {
namespace ratgdo {

    const char* door_state_to_string(DoorState state)
    {
        switch (state) {
        case DOOR_STATE_OPEN:
            return "OPEN";
        case DOOR_STATE_CLOSED:
            return "CLOSED";
        case DOOR_STATE_STOPPED:
            return "STOPPED";
        case DOOR_STATE_OPENING:
            return "OPENING";
        case DOOR_STATE_CLOSING:
            return "CLOSING";
        case DOOR_STATE_UNKNOWN:
        default:
            return "UNKNOWN";
        }
    }

    const char* light_state_to_string(LightState state)
    {
        switch (state) {
        case LIGHT_STATE_OFF:
            return "OFF";
        case LIGHT_STATE_ON:
            return "ON";
            // 2 and 3 appears sometimes
        case LIGHT_STATE_UNKNOWN:
        default:
            return "UNKNOWN";
        }
    }

    const char* lock_state_to_string(LockState state)
    {
        switch (state) {
        case LOCK_STATE_UNLOCKED:
            return "UNLOCKED";
        case LOCK_STATE_LOCKED:
            return "LOCKED";
        case LOCK_STATE_UNKNOWN:
        default:
            return "UNKNOWN";
        }
    }

    const char* motion_state_to_string(MotionState state)
    {
        switch (state) {
        case MOTION_STATE_CLEAR:
            return "CLEAR";
        case MOTION_STATE_DETECTED:
            return "DETECTED";
        default:
            return "CLEAR";
        }
    }

    const char* obstruction_state_to_string(ObstructionState state)
    {
        switch (state) {
        case OBSTRUCTION_STATE_CLEAR:
            return "CLEAR";
        case OBSTRUCTION_STATE_OBSTRUCTED:
            return "OBSTRUCTED";
        case OBSTRUCTION_STATE_UNKNOWN:
        default:
            return "UNKNOWN";
        }
    }

} // namespace ratgdo
} // namespace esphome
