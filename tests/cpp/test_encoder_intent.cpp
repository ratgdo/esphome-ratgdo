// Host test for EncoderIntent, the bounded wrong-direction intent used by the
// encoder builds. Built and run by tests/components/test_encoder_intent.py.

#include "components/ratgdo/encoder_intent.h"

#include <cassert>
#include <cstdio>

using esphome::ratgdo::EncoderIntent;

static constexpr uint32_t TIMEOUT = EncoderIntent::TIMEOUT_MS;

static void test_starts_disarmed()
{
    EncoderIntent intent;
    assert(intent.direction() == 0);
    assert(intent.active(0) == 0);
    assert(intent.active(1000000) == 0);
}

static void test_arm_is_active_within_window()
{
    EncoderIntent intent;
    intent.arm(1, 1000);
    assert(intent.direction() == 1);
    assert(intent.active(1000) == 1);
    assert(intent.active(1000 + TIMEOUT - 1) == 1);
}

static void test_arm_expires_after_window()
{
    EncoderIntent intent;
    intent.arm(-1, 1000);
    assert(intent.active(1000 + TIMEOUT) == 0);
    // Expiry is sticky: the intent is gone, not merely reported as inactive.
    assert(intent.direction() == 0);
}

static void test_expired_intent_cannot_be_refreshed()
{
    EncoderIntent intent;
    intent.arm(1, 1000);
    assert(intent.active(1000 + TIMEOUT) == 0);
    intent.refresh(1000 + TIMEOUT);
    assert(intent.active(1000 + TIMEOUT) == 0);
}

static void test_refresh_extends_window()
{
    EncoderIntent intent;
    intent.arm(1, 1000);
    // Door keeps moving the intended way, one refresh per encoder pulse.
    uint32_t now = 1000;
    for (int i = 0; i < 20; i++) {
        now += TIMEOUT - 1;
        assert(intent.active(now) == 1);
        intent.refresh(now);
    }
    // Movement stops; the window lapses from the last refresh.
    assert(intent.active(now + TIMEOUT) == 0);
}

static void test_refresh_on_disarmed_does_nothing()
{
    EncoderIntent intent;
    intent.refresh(50000);
    assert(intent.direction() == 0);
    assert(intent.active(50000) == 0);
}

static void test_clear()
{
    EncoderIntent intent;
    intent.arm(-1, 1000);
    intent.clear();
    assert(intent.direction() == 0);
    assert(intent.active(1000) == 0);
}

static void test_rearm_resets_window()
{
    EncoderIntent intent;
    intent.arm(1, 1000);
    intent.arm(-1, 1000 + TIMEOUT);
    assert(intent.active(1000 + TIMEOUT) == -1);
    assert(intent.active(1000 + 2 * TIMEOUT - 1) == -1);
    assert(intent.active(1000 + 2 * TIMEOUT) == 0);
}

static void test_survives_millis_wraparound()
{
    EncoderIntent intent;
    const uint32_t before_wrap = 0xFFFFFFFFu - 1000;
    intent.arm(1, before_wrap);
    // Unsigned arithmetic keeps the elapsed time correct across the wrap.
    const uint32_t after_wrap = static_cast<uint32_t>(before_wrap + TIMEOUT - 1);
    assert(intent.active(after_wrap) == 1);
    assert(intent.active(static_cast<uint32_t>(before_wrap + TIMEOUT)) == 0);
}

int main()
{
    test_starts_disarmed();
    test_arm_is_active_within_window();
    test_arm_expires_after_window();
    test_expired_intent_cannot_be_refreshed();
    test_refresh_extends_window();
    test_refresh_on_disarmed_does_nothing();
    test_clear();
    test_rearm_resets_window();
    test_survives_millis_wraparound();
    printf("ok\n");
    return 0;
}
