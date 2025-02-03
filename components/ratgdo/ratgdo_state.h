/************************************
 * Rage
 * Against
 * The
 * Garage
 * Door
 * Opener
 *
 * Copyright (C) 2022  Paul Wieland
 *
 * GNU GENERAL PUBLIC LICENSE
 ************************************/

#pragma once
#include "macros.h"
#include <cstdint>

namespace esphome {
namespace ratgdo {

    ENUM(DoorState, uint8_t,
        (UNKNOWN, 0),
        (OPEN, 1),
        (CLOSED, 2),
        (STOPPED, 3),
        (OPENING, 4),
        (CLOSING, 5))

    ENUM(DoorActionDelayed, uint8_t,
        (NO, 0),
        (YES, 1))

    /// Enum for all states a the light can be in.
    ENUM(LightState, uint8_t,
        (OFF, 0),
        (ON, 1),
        (UNKNOWN, 2))
    LightState light_state_toggle(LightState state);

    /// Enum for all states a the lock can be in.
    ENUM(LockState, uint8_t,
        (UNLOCKED, 0),
        (LOCKED, 1),
        (UNKNOWN, 2))
    LockState lock_state_toggle(LockState state);

    /// MotionState for all states a the motion can be in.
    ENUM(MotionState, uint8_t,
        (CLEAR, 0),
        (DETECTED, 1),
        (UNKNOWN, 2))

    /// Enum for all states a the obstruction can be in.
    ENUM(ObstructionState, uint8_t,
        (OBSTRUCTED, 0),
        (CLEAR, 1),
        (UNKNOWN, 2))

    /// Enum for all states a the motor can be in.
    ENUM(MotorState, uint8_t,
        (OFF, 0),
        (ON, 1),
        (UNKNOWN, 2))

    /// Enum for all states the button can be in.
    ENUM(ButtonState, uint8_t,
        (PRESSED, 0),
        (RELEASED, 1),
        (UNKNOWN, 2))

    ENUM(BatteryState, uint8_t,
        (UNKNOWN, 0),
        (CHARGING, 0x6),
        (FULL, 0x8))

    /// Enum for learn states.
    ENUM(LearnState, uint8_t,
        (INACTIVE, 0),
        (ACTIVE, 1),
        (UNKNOWN, 2))
    LearnState learn_state_toggle(LearnState state);

    ENUM(PairedDevice, uint8_t,
        (ALL, 0),
        (REMOTE, 1),
        (KEYPAD, 2),
        (WALL_CONTROL, 3),
        (ACCESSORY, 4),
        (UNKNOWN, 0xff))

    // actions
    ENUM(LightAction, uint8_t,
        (OFF, 0),
        (ON, 1),
        (TOGGLE, 2),
        (UNKNOWN, 3))

    ENUM(LockAction, uint8_t,
        (UNLOCK, 0),
        (LOCK, 1),
        (TOGGLE, 2),
        (UNKNOWN, 3))

    ENUM(DoorAction, uint8_t,
        (CLOSE, 0),
        (OPEN, 1),
        (TOGGLE, 2),
        (STOP, 3),
        (UNKNOWN, 4))

    ENUM(VehicleDetectedState, uint8_t,
        (NO, 0),
        (YES, 1))

    ENUM(VehicleArrivingState, uint8_t,
        (NO, 0),
        (YES, 1))

    ENUM(VehicleLeavingState, uint8_t,
        (NO, 0),
        (YES, 1))

    struct Openings {
        uint16_t count;
        uint8_t flag;
    };

    struct PairedDeviceCount {
        PairedDevice kind;
        uint16_t count;
    };

    struct TimeToClose {
        uint16_t seconds;
    };

} // namespace ratgdo
} // namespace esphome
