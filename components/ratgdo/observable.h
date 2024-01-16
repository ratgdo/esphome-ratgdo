#pragma once
#include <functional>
#include <utility>
#include <vector>

namespace esphome {
namespace ratgdo {

    template <typename T>
    class distinct_observable;



    template <typename T>
    class observable_base {
    public:
        template <typename Observer>
        void subscribe(Observer&& observer)
        {
            this->observers_.push_back(std::forward<Observer>(observer));
        }

        void notify(T value) const
        {
            for (const auto& observer : this->observers_) {
                observer(value);
            }
        }

        distinct_observable<T> distinct()
        {
            return std::make_shared(this);
        }

    private:
        std::vector<std::function<void(T)>> observers_;
    };

    template <typename T>
    class observable : public observable_base<T> {
    public:
        observable(const T& value) : value_(value) {}

        template <typename U>
        observable& operator=(U value)
        {
            if (value != this->value_) {
                this->value_ = value;
                this->notify(value);
            }
            return *this;
        }

        T const* operator&() const { return &this->value_; }
        T const& operator*() const { return this->value_; }

    private:
        T value_;
    };



    template <typename T>
    class distinct_observable : public observable<T> {
    public:
        distinct_observable(std::shared_ptr<observable<T>> inner) : inner_(inner) {
            inner.subscribe([=] (T value) {
                if (value != this->value_) {
                    this->value_ = value;
                    this->notify(value);
                }
            });
        }

    private:
        std::shared_ptr<observable<T>> inner_;
        T value_;
    };

} // namespace ratgdo
} // namespace esphome
