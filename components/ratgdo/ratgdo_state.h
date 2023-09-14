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
#include "enum.h"
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

    /// Enum for all states the light can be in.
    ENUM(LightState, uint8_t,
        (OFF, 0),
        (ON, 1),
        (UNKNOWN, 2))
    LightState light_state_toggle(LightState state);

    /// Enum for all states the lock can be in.
    ENUM(LockState, uint8_t,
        (UNLOCKED, 0),
        (LOCKED, 1),
        (UNKNOWN, 2))
    LockState lock_state_toggle(LockState state);

    /// Enum for all states the TTC hold open can be in.
    ENUM(HoldState, uint8_t,
        (HOLD_DISABLED, 0),
        (HOLD_ENABLED, 1),
        (UNKNOWN, 2))
    HoldState hold_state_toggle(HoldState state);

    /// MotionState for all states the motion can be in.
    ENUM(MotionState, uint8_t,
        (CLEAR, 0),
        (DETECTED, 1),
        (UNKNOWN, 2))

    /// Enum for all states the obstruction can be in.
    ENUM(ObstructionState, uint8_t,
        (OBSTRUCTED, 0),
        (CLEAR, 1),
        (UNKNOWN, 2))

    /// Enum for all states the motor can be in.
    ENUM(MotorState, uint8_t,
        (OFF, 0),
        (ON, 1),
        (UNKNOWN, 2))

    /// Enum for all states the button can be in.
    ENUM(ButtonState, uint8_t,
        (PRESSED, 0),
        (RELEASED, 1),
        (UNKNOWN, 2))

} // namespace ratgdo
} // namespace esphome