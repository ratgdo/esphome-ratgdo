#pragma once
#include <functional>
#include <utility>
#include <vector>

namespace esphome {
namespace ratgdo {

    template <typename... X>
    class OnceCallbacks;

    template <typename... Ts>
    class OnceCallbacks<void(Ts...)> {
    public:
        template <typename Callback>
        void then(Callback&& callback) { this->callbacks_.push_back(std::forward<Callback>(callback)); }

        void operator()(Ts... args)
        {
            for (auto& cb : this->callbacks_)
                cb(args...);
            this->callbacks_.clear();
        }

    protected:
        std::vector<std::function<void(Ts...)>> callbacks_;
    };

} // namespace ratgdo
} // namespace esphome
