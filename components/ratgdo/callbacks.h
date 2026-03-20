#pragma once
#include <cstdint>
#include <functional>
#include <utility>
#include <vector>

namespace esphome::ratgdo {

template <typename... X>
class OnceCallbacks;

template <typename... Ts>
class OnceCallbacks<void(Ts...)> {
public:
    template <typename Callback>
    void operator()(Callback&& callback) { this->callbacks_.push_back(std::forward<Callback>(callback)); }

    void trigger(Ts... args)
    {
        for (auto& cb : this->callbacks_)
            cb(args...);
        this->callbacks_.clear();
    }

protected:
    std::vector<std::function<void(Ts...)>> callbacks_;
};

} // namespace esphome::ratgdo
