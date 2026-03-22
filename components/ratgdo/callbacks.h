#pragma once
#include "observable.h"
#include <cstdint>
#include <utility>

namespace esphome::ratgdo {

void log_once_callbacks_overflow(uint8_t max);

template <typename... X>
class OnceCallbacks;

template <typename... Ts>
class OnceCallbacks<void(Ts...)> {
public:
    // Runtime max is 1 for all current usage (door_state waits, command_sent waits).
    // Set to 2 for safety margin.
    static constexpr uint8_t MAX_CALLBACKS = 2;

    template <typename F>
    void operator()(F&& callback)
    {
        if (this->count_ >= MAX_CALLBACKS) {
            log_once_callbacks_overflow(MAX_CALLBACKS);
            return;
        }
        this->callbacks_[this->count_++] = Callback<Ts...>::create(std::forward<F>(callback));
    }

    // Re-entrant safe: count_ is zeroed before invoking callbacks,
    // so callbacks can queue new entries during trigger().
    void trigger(Ts... args)
    {
        uint8_t count = this->count_;
        this->count_ = 0;
        for (uint8_t i = 0; i < count; i++) {
            this->callbacks_[i].call(args...);
        }
    }

    void clear() { this->count_ = 0; }

protected:
    Callback<Ts...> callbacks_[MAX_CALLBACKS] { };
    uint8_t count_ { 0 };
};

} // namespace esphome::ratgdo
