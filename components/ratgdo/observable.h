#pragma once
#include <cstdint>
#include <new>
#include <type_traits>
#include <utility>

namespace esphome {
namespace ratgdo {

    void log_multiple_subscribers();
    void log_observer_overflow();

    // Lightweight type-erased callback (16 bytes on 32-bit).
    // For small trivially-copyable callables (like [this], [this, f], or [this, f, id] lambdas),
    // stores the callable inline — zero heap allocation.
    // Supports up to 3 * sizeof(void*) bytes (12 bytes on 32-bit, 24 on 64-bit).
    template <typename... Ts>
    struct Callback {
        static constexpr size_t STORAGE_SIZE = 3 * sizeof(void*);

        using fn_t = void (*)(const void*, Ts...);
        fn_t fn_ { nullptr };
        alignas(void*) uint8_t storage_[STORAGE_SIZE] { };

        void call(Ts... args) const { this->fn_(this->storage_, args...); }
        explicit operator bool() const { return this->fn_ != nullptr; }

        template <typename F>
        static Callback create(F&& f)
        {
            Callback cb;
            using Decay = std::decay_t<F>;
            static_assert(std::is_trivially_copyable_v<Decay>, "Observable callbacks must be trivially copyable (e.g. [this] lambdas)");
            static_assert(sizeof(Decay) <= STORAGE_SIZE, "Observable callbacks must fit in storage (capture at most 3 pointers)");
            cb.fn_ = [](const void* storage, Ts... args) {
                alignas(Decay) char buf[sizeof(Decay)];
                __builtin_memcpy(buf, storage, sizeof(Decay));
                (*std::launder(reinterpret_cast<Decay*>(buf)))(args...);
            };
            __builtin_memcpy(cb.storage_, &f, sizeof(Decay));
            return cb;
        }
    };

    template <typename T, uint8_t MaxObservers = 4>
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

        template <typename F>
        void subscribe(F&& observer)
        {
            if (this->count_ >= MaxObservers) {
                log_observer_overflow();
                return;
            }
            this->observers_[this->count_++] = Callback<T>::create(std::forward<F>(observer));
        }

        void notify() const
        {
            for (uint8_t i = 0; i < this->count_; i++) {
                this->observers_[i].call(this->value_);
            }
        }

    private:
        T value_;
        Callback<T> observers_[MaxObservers] { };
        uint8_t count_ { 0 };
    };

    template <typename T>
    class single_observable {
    public:
        single_observable(const T& value)
            : value_(value)
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

        template <typename F>
        void subscribe(F&& observer)
        {
            if (this->observer_) {
                log_multiple_subscribers();
            }
            this->observer_ = Callback<T>::create(std::forward<F>(observer));
        }

        void notify() const
        {
            if (this->observer_) {
                this->observer_.call(this->value_);
            }
        }

    private:
        T value_;
        Callback<T> observer_ { };
    };

} // namespace ratgdo
} // namespace esphome
