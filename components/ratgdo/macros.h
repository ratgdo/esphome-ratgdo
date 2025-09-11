

#include "esphome/core/log.h"

#define PARENS ()

// Rescan macro tokens 256 times
#define EXPAND(...) EXPAND4(EXPAND4(EXPAND4(EXPAND4(__VA_ARGS__))))
#define EXPAND4(...) EXPAND3(EXPAND3(EXPAND3(EXPAND3(__VA_ARGS__))))
#define EXPAND3(...) EXPAND2(EXPAND2(EXPAND2(EXPAND2(__VA_ARGS__))))
#define EXPAND2(...) EXPAND1(EXPAND1(EXPAND1(EXPAND1(__VA_ARGS__))))
#define EXPAND1(...) __VA_ARGS__

#define FOR_EACH(macro, name, ...) \
    __VA_OPT__(EXPAND(FOR_EACH_HELPER(macro, name, __VA_ARGS__)))
#define FOR_EACH_HELPER(macro, name, a1, ...) \
    macro(name, a1)                           \
        __VA_OPT__(FOR_EACH_AGAIN PARENS(macro, name, __VA_ARGS__))
#define FOR_EACH_AGAIN() FOR_EACH_HELPER

#define ENUM_VARIANT0(name, val) name = val,
#define ENUM_VARIANT(name, tuple) ENUM_VARIANT0 tuple

#define TUPLE(x, y) x, y

#define LPAREN (

#ifdef USE_ESP8266
#define TO_STRING_CASE0(type, name, val) \
    case type::name:                     \
        return LOG_STR(#name);
#else
#define TO_STRING_CASE0(type, name, val) \
    case type::name:                     \
        return #name;
#endif
#define TO_STRING_CASE(type, tuple) TO_STRING_CASE0 LPAREN type, TUPLE tuple)

#define FROM_INT_CASE0(type, name, val) \
    case val:                           \
        return type::name;
#define FROM_INT_CASE(type, tuple) FROM_INT_CASE0 LPAREN type, TUPLE tuple)

#ifdef USE_ESP8266
#define ENUM(name, type, ...)                           \
    enum class name : type {                            \
        FOR_EACH(ENUM_VARIANT, name, __VA_ARGS__)       \
    };                                                  \
    inline const esphome::LogString*                    \
    name##_to_string(name _e)                           \
    {                                                   \
        switch (_e) {                                   \
            FOR_EACH(TO_STRING_CASE, name, __VA_ARGS__) \
        default:                                        \
            return LOG_STR("UNKNOWN");                  \
        }                                               \
    }                                                   \
    inline name                                         \
    to_##name(type _t, name _unknown)                   \
    {                                                   \
        switch (_t) {                                   \
            FOR_EACH(FROM_INT_CASE, name, __VA_ARGS__)  \
        default:                                        \
            return _unknown;                            \
        }                                               \
    }
#else
#define ENUM(name, type, ...)                           \
    enum class name : type {                            \
        FOR_EACH(ENUM_VARIANT, name, __VA_ARGS__)       \
    };                                                  \
    inline const char*                                  \
    name##_to_string(name _e)                           \
    {                                                   \
        switch (_e) {                                   \
            FOR_EACH(TO_STRING_CASE, name, __VA_ARGS__) \
        default:                                        \
            return "UNKNOWN";                           \
        }                                               \
    }                                                   \
    inline name                                         \
    to_##name(type _t, name _unknown)                   \
    {                                                   \
        switch (_t) {                                   \
            FOR_EACH(FROM_INT_CASE, name, __VA_ARGS__)  \
        default:                                        \
            return _unknown;                            \
        }                                               \
    }
#endif

#define SUM_TYPE_UNION_MEMBER0(type, var) type var;
#define SUM_TYPE_UNION_MEMBER(name, tuple) SUM_TYPE_UNION_MEMBER0 tuple

#define SUM_TYPE_ENUM_MEMBER0(type, var) var,
#define SUM_TYPE_ENUM_MEMBER(name, tuple) SUM_TYPE_ENUM_MEMBER0 tuple

#define SUM_TYPE_CONSTRUCTOR0(name, type, val) \
    name(type&& arg)                           \
        : tag(Tag::val)                        \
    {                                          \
        value.val = std::move(arg);            \
    }
#define SUM_TYPE_CONSTRUCTOR(name, tuple) SUM_TYPE_CONSTRUCTOR0 LPAREN name, TUPLE tuple)

#define SUM_TYPE(name, ...)                                    \
    class name {                                               \
    public:                                                    \
        union {                                                \
            FOR_EACH(SUM_TYPE_UNION_MEMBER, name, __VA_ARGS__) \
        } value;                                               \
        enum class Tag {                                       \
            void_,                                             \
            FOR_EACH(SUM_TYPE_ENUM_MEMBER, name, __VA_ARGS__)  \
        } tag;                                                 \
                                                               \
        name()                                                 \
            : tag(Tag::void_)                                  \
        {                                                      \
        }                                                      \
        FOR_EACH(SUM_TYPE_CONSTRUCTOR, name, __VA_ARGS__)      \
    };
