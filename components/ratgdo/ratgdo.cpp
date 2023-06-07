/************************************
 * Rage
 * Against
 * The
 * Garage
 * Door
 * Opener
 *
 * Copyright (C) 2022  Paul Wieland
 *
 * GNU GENERAL PUBLIC LICENSE
 ************************************/

#include "ratgdo.h"
#include "esphome/core/log.h"

namespace esphome {
namespace ratgdo {

    static const char* const TAG = "ratgdo";
    static const int STARTUP_DELAY = 2000; // delay before enabling interrupts

    const char* door_state_to_string(DoorState state)
    {
        switch (state) {
        case DOOR_STATE_OPEN:
            return "OPEN";
        case DOOR_STATE_CLOSED:
            return "CLOSED";
        case DOOR_STATE_STOPPED:
            return "STOPPED";
        case DOOR_STATE_OPENING:
            return "OPENING";
        case DOOR_STATE_CLOSING:
            return "CLOSING";
        case DOOR_STATE_UNKNOWN:
        default:
            return "UNKNOWN";
        }
    }

    const char* light_state_to_string(LightState state)
    {
        switch (state) {
        case LIGHT_STATE_OFF:
            return "OFF";
        case LIGHT_STATE_ON:
            return "ON";
        case IGHT_STATE_UNKNOWN:
        default:
            return "UNKNOWN";
        }
    }

    const char* lock_state_to_string(LockState state)
    {
        switch (state) {
        case LOCK_STATE_UNLOCKED:
            return "UNLOCKED";
        case LOCK_STATE_LOCKED:
            return "LOCKED";
        case LOCK_STATE_UNKNOWN:
        default:
            return "UNKNOWN";
        }
    }

    const char* motion_state_to_string(MotionState state)
    {
        switch (state) {
        case MOTION_STATE_CLEAR:
            return "CLEAR";
        case MOTION_STATE_DETECTED:
            return "DETECTED";
        default:
            return "CLEAR";
        }
    }

    const char* obstruction_state_to_string(ObstructionState state)
    {
        switch (state) {
        case OBSTRUCTION_STATE_CLEAR:
            return "CLEAR";
        case OBSTRUCTION_STATE_DETECTED:
            return "DETECTED";
        case OBSTRUCTION_STATE_UNKNOWN:
        default:
            return "UNKNOWN";
        }
    }

    /*************************** DRY CONTACT CONTROL OF LIGHT & DOOR
     * ***************************/
    void IRAM_ATTR HOT RATGDOStore::isrDoorOpen(RATGDOStore* arg)
    {
        static unsigned long lastOpenDoorTime = 0;

        unsigned long currentMillis = millis();
        // Prevent ISR during the first 2 seconds after reboot
        if (currentMillis < STARTUP_DELAY)
            return;

        if (!arg->trigger_open.digital_read()) {
            // save the time of the falling edge
            lastOpenDoorTime = currentMillis;
        } else if (currentMillis - lastOpenDoorTime > 500 && currentMillis - lastOpenDoorTime < 10000) {
            // now see if the rising edge was between 500ms and 10 seconds after the
            // falling edge
            arg->dryContactDoorOpen = true;
        }
    }

    void IRAM_ATTR HOT RATGDOStore::isrDoorClose(RATGDOStore* arg)
    {
        static unsigned long lastCloseDoorTime = 0;

        unsigned long currentMillis = millis();
        // Prevent ISR during the first 2 seconds after reboot
        if (currentMillis < STARTUP_DELAY)
            return;

        if (!arg->trigger_close.digital_read()) {
            // save the time of the falling edge
            lastCloseDoorTime = currentMillis;
        } else if (currentMillis - lastCloseDoorTime > 500 && currentMillis - lastCloseDoorTime < 10000) {
            // now see if the rising edge was between 500ms and 10 seconds after the
            // falling edge
            arg->dryContactDoorClose = true;
        }
    }

    void IRAM_ATTR HOT RATGDOStore::isrLight(RATGDOStore* arg)
    {
        static unsigned long lastToggleLightTime = 0;

        unsigned long currentMillis = millis();
        // Prevent ISR during the first 2 seconds after reboot
        if (currentMillis < STARTUP_DELAY)
            return;

        if (!arg->trigger_light.digital_read()) {
            // save the time of the falling edge
            lastToggleLightTime = currentMillis;
        } else if (currentMillis - lastToggleLightTime > 500 && currentMillis - lastToggleLightTime < 10000) {
            // now see if the rising edge was between 500ms and 10 seconds after the
            // falling edge
            arg->dryContactToggleLight = true;
        }
    }

    void IRAM_ATTR HOT RATGDOStore::isrObstruction(RATGDOStore* arg)
    {
        if (arg->input_obst.digital_read()) {
            // ESP_LOGD(TAG, "isrObstruction HIGH");
            arg->lastObstructionHigh = millis();
        } else {
            // ESP_LOGD(TAG, "isrObstruction LOW");
            arg->obstructionLowCount++;
        }
    }

    void RATGDOComponent::setup()
    {
        this->pref_ = global_preferences->make_preference<int>(734874333U);
        if (!this->pref_.load(&this->rollingCodeCounter)) {
            this->rollingCodeCounter = 0;
        }

        this->output_gdo_pin_->setup();
        this->input_gdo_pin_->setup();
        this->input_obst_pin_->setup();

        this->trigger_open_pin_->setup();
        this->trigger_close_pin_->setup();
        this->trigger_light_pin_->setup();

        this->status_door_pin_->setup();
        this->status_obst_pin_->setup();

        this->store_.input_obst = this->input_obst_pin_->to_isr();

        this->store_.trigger_open = this->trigger_open_pin_->to_isr();
        this->store_.trigger_close = this->trigger_close_pin_->to_isr();
        this->store_.trigger_light = this->trigger_light_pin_->to_isr();

        this->trigger_open_pin_->pin_mode(gpio::FLAG_INPUT | gpio::FLAG_PULLUP);
        this->trigger_close_pin_->pin_mode(gpio::FLAG_INPUT | gpio::FLAG_PULLUP);
        this->trigger_light_pin_->pin_mode(gpio::FLAG_INPUT | gpio::FLAG_PULLUP);

        this->status_door_pin_->pin_mode(gpio::FLAG_OUTPUT);
        this->status_obst_pin_->pin_mode(gpio::FLAG_OUTPUT);

        this->output_gdo_pin_->pin_mode(gpio::FLAG_OUTPUT);
        this->input_gdo_pin_->pin_mode(gpio::FLAG_INPUT | gpio::FLAG_PULLUP);
        this->input_obst_pin_->pin_mode(gpio::FLAG_INPUT);

        this->check_uart_settings(9600, 1, esphome::uart::UART_CONFIG_PARITY_NONE, 8);

        this->trigger_open_pin_->attach_interrupt(RATGDOStore::isrDoorOpen, &this->store_, gpio::INTERRUPT_ANY_EDGE);
        this->trigger_close_pin_->attach_interrupt(RATGDOStore::isrDoorClose, &this->store_, gpio::INTERRUPT_ANY_EDGE);
        this->trigger_light_pin_->attach_interrupt(RATGDOStore::isrLight, &this->store_, gpio::INTERRUPT_ANY_EDGE);
        this->input_obst_pin_->attach_interrupt(RATGDOStore::isrObstruction, &this->store_, gpio::INTERRUPT_ANY_EDGE);

        ESP_LOGD(TAG, "Syncing rolling code counter after reboot...");
        sync(); // reboot/sync to the opener on startup
    }

    void RATGDOComponent::loop()
    {
        // ESP_LOGD(TAG, "loop rollingCodeCounter: %d", this->rollingCodeCounter);
        obstructionLoop();
        gdoStateLoop();
        dryContactLoop();
        statusUpdateLoop();
        // ESP_LOGD(TAG, "Door State: %s", this->doorState.c_str());
    }

    void RATGDOComponent::dump_config()
    {
        ESP_LOGCONFIG(TAG, "Setting up RATGDO...");
        LOG_PIN("  Output GDO Pin: ", this->output_gdo_pin_);
        LOG_PIN("  Input GDO Pin: ", this->input_gdo_pin_);
        LOG_PIN("  Input Obstruction Pin: ", this->input_obst_pin_);
        LOG_PIN("  Trigger Open Pin: ", this->trigger_open_pin_);
        LOG_PIN("  Trigger Close Pin: ", this->trigger_close_pin_);
        LOG_PIN("  Trigger Light Pin: ", this->trigger_light_pin_);
        LOG_PIN("  Status Door Pin: ", this->status_door_pin_);
        LOG_PIN("  Status Obstruction Pin: ", this->status_obst_pin_);
    }

    void RATGDOComponent::readRollingCode(uint8_t& door, uint8_t& light, uint8_t& lock, uint8_t& motion, uint8_t& obstruction)
    {
        uint32_t rolling = 0;
        uint64_t fixed = 0;
        uint32_t data = 0;

        uint16_t cmd = 0;
        uint8_t nibble = 0;
        uint8_t byte1 = 0;
        uint8_t byte2 = 0;

        decode_wireline(this->rxRollingCode, &rolling, &fixed, &data);

        cmd = ((fixed >> 24) & 0xf00) | (data & 0xff);

        nibble = (data >> 8) & 0xf;
        byte1 = (data >> 16) & 0xff;
        byte2 = (data >> 24) & 0xff;

        if (cmd == 0x81) {
            door = nibble;
            light = (byte2 >> 1) & 1;
            lock = byte2 & 1;
            motion = 0; // when the status message is read, reset motion state to 0|clear
            // obstruction = (byte1 >> 6) & 1; // unreliable due to the time it takes to register an obstruction

        } else if (cmd == 0x281) {
            light ^= 1; // toggle bit
        } else if (cmd == 0x84) {
        } else if (cmd == 0x285) {
            motion = 1; // toggle bit
        }
    }

    void RATGDOComponent::getRollingCode(const char* command)
    {

        uint64_t id = 0x539;
        uint64_t fixed = 0;
        uint32_t data = 0;

        if (strcmp(command, "reboot1") == 0) {
            fixed = 0x400000000;
            data = 0x0000618b;
        } else if (strcmp(command, "reboot2") == 0) {
            fixed = 0;
            data = 0x01009080;
        } else if (strcmp(command, "reboot3") == 0) {
            fixed = 0;
            data = 0x0000b1a0;
        } else if (strcmp(command, "reboot4") == 0) {
            fixed = 0;
            data = 0x01009080;
        } else if (strcmp(command, "reboot5") == 0) {
            fixed = 0x300000000;
            data = 0x00008092;
        } else if (strcmp(command, "reboot6") == 0) {
            fixed = 0x300000000;
            data = 0x00008092;
        } else if (strcmp(command, "door1") == 0) {
            fixed = 0x200000000;
            data = 0x01018280;
        } else if (strcmp(command, "door2") == 0) {
            fixed = 0x200000000;
            data = 0x01009280;
        } else if (strcmp(command, "light") == 0) {
            fixed = 0x200000000;
            data = 0x00009281;
        } else if (strcmp(command, "lock") == 0) {
            fixed = 0x0100000000;
            data = 0x0000728c;
        } else {
            ESP_LOGD(TAG, "ERROR: Invalid command");
            return;
        }

        fixed = fixed | id;

        encode_wireline(this->rollingCodeCounter, fixed, data, this->txRollingCode);

        printRollingCode();

        if (strcmp(command, "door1") != 0) { // door2 is created with same counter and should always be called after door1
            this->rollingCodeCounter = (this->rollingCodeCounter + 1) & 0xfffffff;
        }
        return;
    }

    void RATGDOComponent::printRollingCode()
    {
        ESP_LOGD(TAG, "Counter: %d Send code: [%02x][%02x][%02x][%02x][%02x][%02x][%02x][%02x][%02x][%02x][%02x][%02x][%02x][%02x][%02x][%02x][%02x][%02x][%02x]",
            this->rollingCodeCounter,
            this->txRollingCode[0],
            this->txRollingCode[1],
            this->txRollingCode[2],
            this->txRollingCode[3],
            this->txRollingCode[4],
            this->txRollingCode[5],
            this->txRollingCode[6],
            this->txRollingCode[7],
            this->txRollingCode[8],
            this->txRollingCode[9],
            this->txRollingCode[10],
            this->txRollingCode[11],
            this->txRollingCode[12],
            this->txRollingCode[13],
            this->txRollingCode[14],
            this->txRollingCode[15],
            this->txRollingCode[16],
            this->txRollingCode[17],
            this->txRollingCode[18]);
    }

    // handle changes to the dry contact state
    void RATGDOComponent::dryContactLoop()
    {
        if (this->store_.dryContactDoorOpen) {
            ESP_LOGD(TAG, "Dry Contact: open the door");
            this->store_.dryContactDoorOpen = false;
            openDoor();
        }

        if (this->store_.dryContactDoorClose) {
            ESP_LOGD(TAG, "Dry Contact: close the door");
            this->store_.dryContactDoorClose = false;
            closeDoor();
        }

        if (this->store_.dryContactToggleLight) {
            ESP_LOGD(TAG, "Dry Contact: toggle the light");
            this->store_.dryContactToggleLight = false;
            toggleLight();
        }
    }

    /*************************** OBSTRUCTION DETECTION ***************************/

    void RATGDOComponent::obstructionLoop()
    {
        long currentMillis = millis();
        static unsigned long lastMillis = 0;

        // the obstruction sensor has 3 states: clear (HIGH with LOW pulse every 7ms), obstructed (HIGH), asleep (LOW)
        // the transitions between awake and asleep are tricky because the voltage drops slowly when falling asleep
        // and is high without pulses when waking up

        // If at least 3 low pulses are counted within 50ms, the door is awake, not obstructed and we don't have to check anything else

        // Every 50ms
        if (currentMillis - lastMillis > 50) {
            // check to see if we got between 3 and 8 low pulses on the line
            if (this->store_.obstructionLowCount >= 3 && this->store_.obstructionLowCount <= 8) {
                // obstructionCleared();
                this->store_.obstructionState = 1;

                // if there have been no pulses the line is steady high or low
            } else if (this->store_.obstructionLowCount == 0) {
                // if the line is high and the last high pulse was more than 70ms ago, then there is an obstruction present
                if (this->input_obst_pin_->digital_read() && currentMillis - this->store_.lastObstructionHigh > 70) {
                    this->store_.obstructionState = 0;
                    // obstructionDetected();
                } else {
                    // asleep
                }
            }

            lastMillis = currentMillis;
            this->store_.obstructionLowCount = 0;
        }
    }

    void RATGDOComponent::gdoStateLoop()
    {
        if (!this->available()) {
            // ESP_LOGD(TAG, "No data available input:%d output:%d", this->input_gdo_pin_->get_pin(), this->output_gdo_pin_->get_pin());
            return;
        }
        uint8_t serData;
        if (!this->read_byte(&serData)) {
            ESP_LOGD(TAG, "Failed to read byte");
            return;
        }

        static uint32_t msgStart;
        static bool reading = false;
        static uint16_t byteCount = 0;

        if (!reading) {
            // shift serial byte onto msg start
            msgStart <<= 8;
            msgStart |= serData;

            // truncate to 3 bytes
            msgStart &= 0x00FFFFFF;

            // if we are at the start of a message, capture the next 16 bytes
            if (msgStart == 0x550100) {
                byteCount = 3;
                rxRollingCode[0] = 0x55;
                rxRollingCode[1] = 0x01;
                rxRollingCode[2] = 0x00;

                reading = true;
                return;
            }
        }

        if (reading) {
            this->rxRollingCode[byteCount] = serData;
            byteCount++;

            if (byteCount == 19) {
                reading = false;
                msgStart = 0;
                byteCount = 0;

                readRollingCode(this->store_.doorState, this->store_.lightState, this->store_.lockState, this->store_.motionState, this->store_.obstructionState);
            }
        }
    }

    void RATGDOComponent::statusUpdateLoop()
    {
        // initialize to unknown
        static uint8_t previousDoorState = 0;
        static uint8_t previousLightState = 2;
        static uint8_t previousLockState = 2;
        static uint8_t previousObstructionState = 2;

        if (this->store_.doorState != previousDoorState)
            sendDoorStatus();
        if (this->store_.lightState != previousLightState)
            sendLightStatus();
        if (this->store_.lockState != previousLockState)
            sendLockStatus();
        if (this->store_.obstructionState != previousObstructionState)
            sendObstructionStatus();

        if (this->store_.motionState == 1) {
            sendMotionStatus();
            this->store_.motionState = 0;
        }

        previousDoorState = this->store_.doorState;
        previousLightState = this->store_.lightState;
        previousLockState = this->store_.lockState;
        previousObstructionState = this->store_.obstructionState;
    }

    void RATGDOComponent::sendDoorStatus()
    {
        ESP_LOGD(TAG, "Door state %d", this->store_.doorState);
        this->status_door_pin_->digital_write(this->store_.doorState == 1);
    }

    void RATGDOComponent::sendLightStatus()
    {
        ESP_LOGD(TAG, "Light state %d", this->store_.lightState);
    }

    void RATGDOComponent::sendLockStatus()
    {
        ESP_LOGD(TAG, "Lock state %d", this->store_.lockState);
    }

    void RATGDOComponent::sendMotionStatus()
    {
        ESP_LOGD(TAG, "Motion state %d", this->store_.motionState);
        this->store_.motionState = 0; // reset motion state
    }

    void RATGDOComponent::sendObstructionStatus()
    {
        ESP_LOGD(TAG, "Obstruction state %d", this->store_.obstructionState);
        this->status_obst_pin_->digital_write(this->store_.obstructionState == 0);
    }

    /************************* DOOR COMMUNICATION *************************/
    /*
     * Transmit a message to the door opener over uart1
     * The TX1 pin is controlling a transistor, so the logic is inverted
     * A HIGH state on TX1 will pull the 12v line LOW
     *
     * The opener requires a specific duration low/high pulse before it will accept
     * a message
     */
    void RATGDOComponent::transmit(const char* command)
    {
        getRollingCode(command);
        this->output_gdo_pin_->digital_write(true); // pull the line high for 1305 micros so the
                                                    // door opener responds to the message
        delayMicroseconds(1305);
        this->output_gdo_pin_->digital_write(false); // bring the line low

        delayMicroseconds(1260); // "LOW" pulse duration before the message start
        this->write_array(this->txRollingCode, CODE_LENGTH);
    }

    void RATGDOComponent::sync()
    {
        transmit("reboot1");
        delay(65);

        transmit("reboot2");
        delay(65);

        transmit("reboot3");
        delay(65);

        transmit("reboot4");
        delay(65);

        transmit("reboot5");
        delay(65);

        transmit("reboot6");
        delay(65);

        this->pref_.save(&this->rollingCodeCounter);
    }

    void RATGDOComponent::openDoor()
    {
        if (this->store_.doorState == DoorState.DOOR_STATE_OPEN || this->store_.doorState == DoorState.DOOR_STATE_OPENING) {
            ESP_LOGD(TAG, "The door is already %s", door_state_to_string(this->store_.doorState));
            return;
        }
        toggleDoor();
    }

    void RATGDOComponent::closeDoor()
    {
        if (this->store_.doorState == DoorState.DOOR_STATE_CLOSED || this->store_.doorState == DoorState.DOOR_STATE_CLOSING) {
            ESP_LOGD(TAG, "The door is already %s", door_state_to_string(this->store_.doorState));
            return;
        }
        toggleDoor();
    }

    void RATGDOComponent::stopDoor()
    {
        if (this->store_.doorState == DoorState.DOOR_STATE_OPENING || this->store_.doorState == DoorState.DOOR_STATE_CLOSING) {
            toggleDoor();
        } else {
            ESP_LOGD(TAG, "The door is not moving.");
        }
    }

    void RATGDOComponent::toggleDoor()
    {
        transmit("door1");
        delay(40);
        transmit("door2");
        this->pref_.save(&this->rollingCodeCounter);
    }

    void RATGDOComponent::lightOn()
    {
        if (this->store_.lightState == LightState.LIGHT_STATE_ON) {
            ESP_LOGD(TAG, "already on");
        } else {
            toggleLight();
        }
    }

    void RATGDOComponent::lightOff()
    {
        if (this->store_.lightState == LightState.LIGHT_STATE_OFF) {
            ESP_LOGD(TAG, "already off");
        } else {
            toggleLight();
        }
    }

    void RATGDOComponent::toggleLight()
    {
        sendCommand("light");
    }

    // Lock functions
    void RATGDOComponent::lock()
    {
        if (this->store_.lockState == LockState.LOCK_STATE_LOCKED) {
            ESP_LOGD(TAG, "already locked");
        } else {
            toggleLock();
        }
    }

    void RATGDOComponent::unlock()
    {
        if (this->store_.lockState == LockState.LOCK_STATE_UNLOCKED) {
            ESP_LOGD(TAG, "already unlocked");
        } else {
            toggleLock();
        }
    }

    void RATGDOComponent::toggleLock()
    {
        sendCommand("lock");
    }

    void RATGDOComponent::sendCommand(const char* command)
    {
        transmit(command);
        this->pref_.save(&this->rollingCodeCounter);
    }

} // namespace ratgdo
} // namespace esphome
