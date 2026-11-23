#pragma once

#include <cstdint>

namespace esphome::ratgdo {

// Remembers which way the door was last commanded to travel so the encoder can
// spot an opener that responds by moving the wrong way.
//
// The intent has to expire. A command does not always produce movement: dry
// contact builds swallow OPEN when the door is already at the open limit, and
// ROW openers without obstruction sensors ignore an explicit CLOSE. Without an
// expiry the intent stays armed indefinitely, and the next manual operation of
// the door in the other direction looks like a wrong-way response, so the
// firmware stops the door and reverses it.
//
// The window is refreshed while the door keeps moving the intended way, so a
// mid-travel reversal is still caught on a long door.
class EncoderIntent {
public:
    // How long an intent survives without movement in the intended direction.
    // An opener starts moving within a second or so of a command; anything
    // slower than this is treated as a command that was never acted on. The
    // same window covers the gap between encoder pulses during travel, so it
    // has to stay comfortably above the encoder stopped watchdog, which already
    // declares the door stopped after two seconds without a pulse.
    static constexpr uint32_t TIMEOUT_MS = 5000;

    // Arm the intent. dir is +1 to open, -1 to close. now is millis().
    void arm(int8_t dir, uint32_t now)
    {
        this->dir_ = dir;
        this->stamp_ = now;
    }

    void clear() { this->dir_ = 0; }

    // Restart the window without changing the direction. Used when the command
    // is dispatched later than it was requested (closing delay) and while the
    // door travels the intended way. An intent whose window has already lapsed
    // is dropped rather than revived.
    void refresh(uint32_t now)
    {
        if (this->active(now) != 0)
            this->stamp_ = now;
    }

    // Armed direction, or 0 when nothing is armed. Does not consider expiry;
    // use active() to act on an intent.
    int8_t direction() const { return this->dir_; }

    // Armed direction, or 0 once the window has lapsed. An expired intent is
    // dropped here rather than merely reported as inactive.
    int8_t active(uint32_t now)
    {
        if (this->dir_ != 0 && now - this->stamp_ >= TIMEOUT_MS)
            this->dir_ = 0;
        return this->dir_;
    }

protected:
    uint32_t stamp_ { 0 };
    int8_t dir_ { 0 };
};

} // namespace esphome::ratgdo
