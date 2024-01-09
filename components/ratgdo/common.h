#pragma once
#include <cstdint>
#include <functional>
#include "observable.h"

#define ESP_LOG1 ESP_LOGD
#define ESP_LOG2 ESP_LOGD


namespace esphome {
namespace ratgdo {

struct SetRollingCodeCounter { uint32_t counter; };
struct GetRollingCodeCounter {};
struct RollingCodeCounter { observable<uint32_t>* counter; };
struct SetClientID { uint64_t client_id; };
struct ActivateLearn {};
struct InactivateLearn {};


// a poor man's sum-type, because C++
class ProtocolArgs {
public:
    union {
       SetRollingCodeCounter set_rolling_code_counter;
       GetRollingCodeCounter get_rolling_code_counter;
       RollingCodeCounter rolling_code_counter;
       SetClientID set_client_id;
       ActivateLearn activate_learn;
       InactivateLearn inactivate_learn;
    } value;

    enum class Tag {
        set_rolling_code_counter,
        get_rolling_code_counter,
        rolling_code_counter,
        set_client_id,
        activate_learn,
        inactivate_learn,
        void_,
    } tag;

    ProtocolArgs(): tag(Tag::void_) {
    }

    ProtocolArgs(GetRollingCodeCounter&& arg): tag(Tag::get_rolling_code_counter) {
        value.get_rolling_code_counter = std::move(arg);
    }
    ProtocolArgs(SetRollingCodeCounter&& arg): tag(Tag::set_rolling_code_counter) {
        value.set_rolling_code_counter = std::move(arg);
    }
    ProtocolArgs(RollingCodeCounter&& arg): tag(Tag::rolling_code_counter) {
        value.rolling_code_counter = std::move(arg);
    }
    ProtocolArgs(SetClientID&& arg): tag(Tag::set_client_id) {
        value.set_client_id = std::move(arg);
    }
    ProtocolArgs(ActivateLearn&& arg): tag(Tag::activate_learn) {
        value.activate_learn = std::move(arg);
    }
    ProtocolArgs(InactivateLearn&& arg): tag(Tag::inactivate_learn) {
        value.inactivate_learn = std::move(arg);
    }
};


} // namespace ratgdo
} // namespace esphome
