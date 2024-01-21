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
        void operator()(Callback&& callback) { this->callbacks_.push_back(std::forward<Callback>(callback)); }

        void trigger(Ts... args)
        {
            for (auto& cb : this->callbacks_) {
                cb(args...);
            }
            this->callbacks_.clear();
        }

    protected:
        std::vector<std::function<void(Ts...)>> callbacks_;
    };

    template <typename... X>
    class ExpiringCallbacks;

    template <typename... Ts>
    class ExpiringCallbacks<void(Ts...)> {
    public:
        template <typename Callback>
        void operator()(uint32_t expiration, Callback&& callback)
        {
            this->callbacks_.push_back(std::make_pair(expiration, std::forward<Callback>(callback)));
        }

        void trigger(uint32_t now, Ts... args)
        {
            for (auto& cb : this->callbacks_) {
                if (cb.first >= now) {
                    cb.second(args...);
                }
            }
            this->callbacks_.clear();
        }

        bool is_expired(uint32_t now) const
        {
            bool expired = true;
            for (const auto& cb : this->callbacks_) {
                if (cb.first >= now) {
                    expired = false;
                }
            }
            return expired;
        }

        void clear()
        {
            this->callbacks_.clear();
        }

    protected:
        std::vector<std::pair<uint32_t, std::function<void(Ts...)>>> callbacks_;
    };

} // namespace ratgdo
} // namespace esphome
