#pragma once
#include <functional>
#include <utility>
#include <vector>

namespace esphome {
namespace ratgdo {

    void log_multiple_subscribers();

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
        }

        void notify() const
        {
            for (const auto& observer : this->observers_) {
                observer(this->value_);
            }
        }

    private:
        T value_;
        std::vector<std::function<void(T)>> observers_;
    };

    template <typename T>
    class single_observable {
    public:
        single_observable(const T& value)
            : value_(value)
            , observer_(nullptr)
        {
        }

        template <typename U>
        single_observable& operator=(U value)
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
            if (this->observer_ != nullptr) {
                log_multiple_subscribers();
            }
            this->observer_ = std::forward<Observer>(observer);
        }

        void notify() const
        {
            if (this->observer_) {
                this->observer_(this->value_);
            }
        }

    private:
        T value_;
        std::function<void(T)> observer_;
    };

} // namespace ratgdo
} // namespace esphome
