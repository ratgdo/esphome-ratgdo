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
}
}
