#pragma once
#include <functional>
#include <utility>
#include <vector>
#include "esphome/core/log.h"

namespace esphome {
namespace ratgdo {

    template <typename T>
    class observable {
    public:
        observable(const T& value)
            : value_(value)
        {
        }

        template <typename U>
        observable& operator=(U value)
        {
            if (value != this->value_) {
                this->value_ = value;
                this->notify();
            }
            return *this;
        }

        T const* operator&() const { return &this->value_; }
        T const& operator*() const { return this->value_; }

        template <typename Observer>
        void subscribe(Observer&& observer)
        {
            this->observers_.push_back(std::forward<Observer>(observer));
            ESP_LOGD("XXX","The observers vector is %d in length",this->observers_.size());
        }

        void notify() const
        {
            ESP_LOGD("YYY","NOTIFY %d subscribers",this->observers_.size());

            for (const auto& observer : this->observers_) {
                observer(this->value_);
            }
        }

    private:
        T value_;
        std::vector<std::function<void(T)>> observers_;
    };

} // namespace ratgdo
} // namespace esphome
