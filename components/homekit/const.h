#pragma once

namespace esphome {
namespace homekit {
    enum TemperatureUnits {
        CELSIUS,
        FAHRENHEIT
    };

    enum HKFinish {
        TAN,
        GOLD,
        SILVER,
        BLACK
    };

    enum AInfo {
        NAME,
        MODEL,
        SN,
        MANUFACTURER,
        FW_REV
    };

    // Define missing HAP constants if not available from library
    #ifndef HAP_HW_FINISH_OTHER
    #define HAP_HW_FINISH_OTHER 0
    #endif
}
}
