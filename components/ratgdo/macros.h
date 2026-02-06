

#include <cstddef>
#include <cstdint>

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

// String blob helpers for packed enum-to-string lookup tables
#define STR_BLOB_ENTRY0(type, name, val) #name "\0"
#define STR_BLOB_ENTRY(type, tuple) STR_BLOB_ENTRY0 LPAREN type, TUPLE tuple)

#define COUNT_ONE0(type, name, val) +1
#define COUNT_ONE(name, tuple) COUNT_ONE0 LPAREN name, TUPLE tuple)

namespace esphome {
namespace ratgdo {
    namespace detail {

        template <size_t N>
        struct EnumStringOffsets {
            uint8_t data[N];
        };

        template <size_t Count, size_t BlobSize>
        constexpr EnumStringOffsets<Count> compute_enum_string_offsets(const char (&blob)[BlobSize])
        {
            EnumStringOffsets<Count> result {};
            result.data[0] = 0;
            size_t entry = 1;
            for (size_t i = 0; i < BlobSize - 1 && entry < Count; ++i) {
                if (blob[i] == '\0') {
                    result.data[entry++] = static_cast<uint8_t>(i + 1);
                }
            }
            return result;
        }

    } // namespace detail
} // namespace ratgdo
} // namespace esphome

// Platform-specific helpers for enum string return types
#ifdef USE_ESP8266
#define ENUM_STR_RET const esphome::LogString*
#define ENUM_STR_UNKNOWN LOG_STR("UNKNOWN")
#define ENUM_BLOB_ATTR PROGMEM
#define ENUM_BLOB_RETURN(blob, offset) reinterpret_cast<const esphome::LogString*>(&(blob)[offset])
#else
#define ENUM_STR_RET const char*
#define ENUM_STR_UNKNOWN "UNKNOWN"
#define ENUM_BLOB_ATTR
#define ENUM_BLOB_RETURN(blob, offset) (&(blob)[offset])
#endif

// ENUM: packed string blob with O(1) offset lookup (for contiguous 0-based enums)
#define ENUM(name, type, ...)                                                                          \
    enum class name : type {                                                                           \
        FOR_EACH(ENUM_VARIANT, name, __VA_ARGS__)                                                      \
    };                                                                                                 \
    inline ENUM_STR_RET                                                                                \
    name##_to_string(name _e)                                                                          \
    {                                                                                                  \
        static constexpr size_t _n = (0 FOR_EACH(COUNT_ONE, name, __VA_ARGS__));                       \
        static const char _b[] ENUM_BLOB_ATTR = FOR_EACH(STR_BLOB_ENTRY, name, __VA_ARGS__) "UNKNOWN"; \
        static constexpr auto _o = ::esphome::ratgdo::detail::compute_enum_string_offsets<_n + 1>(     \
            FOR_EACH(STR_BLOB_ENTRY, name, __VA_ARGS__) "UNKNOWN");                                    \
        auto _i = static_cast<uint8_t>(_e);                                                            \
        if (_i >= _n)                                                                                  \
            _i = static_cast<uint8_t>(_n);                                                             \
        return ENUM_BLOB_RETURN(_b, _o.data[_i]);                                                      \
    }                                                                                                  \
    inline name                                                                                        \
    to_##name(type _t, name _unknown)                                                                  \
    {                                                                                                  \
        switch (_t) {                                                                                  \
            FOR_EACH(FROM_INT_CASE, name, __VA_ARGS__)                                                 \
        default:                                                                                       \
            return _unknown;                                                                           \
        }                                                                                              \
    }

// ENUM_SPARSE: switch-based lookup (for non-contiguous enum values)
#define ENUM_SPARSE(name, type, ...)                    \
    enum class name : type {                            \
        FOR_EACH(ENUM_VARIANT, name, __VA_ARGS__)       \
    };                                                  \
    inline ENUM_STR_RET                                 \
    name##_to_string(name _e)                           \
    {                                                   \
        switch (_e) {                                   \
            FOR_EACH(TO_STRING_CASE, name, __VA_ARGS__) \
        default:                                        \
            return ENUM_STR_UNKNOWN;                    \
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
