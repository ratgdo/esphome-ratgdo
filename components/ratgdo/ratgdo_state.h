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
#include "esphome/core/component.h"
#include "esphome/core/gpio.h"
#include "esphome/core/log.h"
#include "esphome/core/preferences.h"

namespace esphome {
namespace ratgdo {

    /// Enum for all states a the door can be in.
    enum DoorState : uint8_t {
        DOOR_STATE_UNKNOWN = 0,
        DOOR_STATE_OPEN = 1,
        DOOR_STATE_CLOSED = 2,
        DOOR_STATE_STOPPED = 3,
        DOOR_STATE_OPENING = 4,
        DOOR_STATE_CLOSING = 5
    };
    const char* door_state_to_string(DoorState state);

    /// Enum for all states a the light can be in.
    enum LightState : uint8_t {
        LIGHT_STATE_OFF = 0,
        LIGHT_STATE_ON = 1,
        LIGHT_STATE_UNKNOWN = 2,
    };
    const char* light_state_to_string(LightState state);

    /// Enum for all states a the lock can be in.
    enum LockState : uint8_t {
        LOCK_STATE_UNLOCKED = 0,
        LOCK_STATE_LOCKED = 1,
        LOCK_STATE_UNKNOWN = 2,
    };
    const char* lock_state_to_string(LockState state);

    /// Enum for all states a the motion can be in.
    enum MotionState : uint8_t {
        MOTION_STATE_CLEAR = 0,
        MOTION_STATE_DETECTED = 1,
    };
    const char* motion_state_to_string(MotionState state);

    /// Enum for all states a the obstruction can be in.
    enum ObstructionState : uint8_t {
        OBSTRUCTION_STATE_OBSTRUCTED = 0,
        OBSTRUCTION_STATE_CLEAR = 1,
        OBSTRUCTION_STATE_UNKNOWN = 2,
    };
    const char* obstruction_state_to_string(ObstructionState state);

    /// Enum for all states a the motor can be in.
    enum MotorState : uint8_t {
        MOTOR_STATE_OFF = 0,
        MOTOR_STATE_ON = 1,
    };

} // namespace ratgdo
} // namespace esphome