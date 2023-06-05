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
    /*** Static Codes ***/
    static const unsigned char SYNC1[] = { 0x55, 0x01, 0x00, 0x61, 0x12, 0x49, 0x2c, 0x92, 0x5b, 0x24,
        0x96, 0x86, 0x0b, 0x65, 0x96, 0xd9, 0x8f, 0x26, 0x4a };
    static const unsigned char SYNC2[] = { 0x55, 0x01, 0x00, 0x08, 0x34, 0x93, 0x49, 0xb4, 0x92, 0x4d,
        0x20, 0x26, 0x1b, 0x4d, 0xb4, 0xdb, 0xad, 0x76, 0x93 };
    static const unsigned char SYNC3[] = { 0x55, 0x01, 0x00, 0x06, 0x1b, 0x2c, 0xbf, 0x4b, 0x6d, 0xb6,
        0x4b, 0x18, 0x20, 0x92, 0x09, 0x20, 0xf2, 0x11, 0x2c };
    static const unsigned char SYNC4[] = { 0x55, 0x01, 0x00, 0x95, 0x29, 0x36, 0x91, 0x29, 0x36, 0x9a,
        0x69, 0x05, 0x2f, 0xbe, 0xdf, 0x6d, 0x16, 0xcb, 0xe7 };

    static const unsigned char  DOOR_CODE[] = { 0x55, 0x01, 0x00, 0x94, 0x3f, 0xef, 0xbc, 0xfb, 0x7f, 0xbe,
        0xfc, 0xa6, 0x1a, 0x4d, 0xa6, 0xda, 0x8d, 0x36, 0xb3 };

    static const unsigned char  LIGHT_CODE[] = { 0x55, 0x01, 0x00, 0x94, 0x3f, 0xef, 0xbc, 0xfb, 0x7f, 0xbe,
        0xff, 0xa6, 0x1a, 0x4d, 0xa6, 0xda, 0x8d, 0x76, 0xb1 };



    /*************************** DRY CONTACT CONTROL OF LIGHT & DOOR
     * ***************************/
    void IRAM_ATTR HOT RATGDOStore::isrDoorOpen(RATGDOStore *arg) { 
        unsigned long currentMillis = millis();
        // Prevent ISR during the first 2 seconds after reboot
        if (currentMillis < 2000)
            return;

		if (!arg->trigger_open.digital_read()) {
			// save the time of the falling edge
			arg->lastOpenDoorTime = currentMillis;
		} else if (currentMillis - arg->lastOpenDoorTime > 500 && currentMillis - arg->lastOpenDoorTime < 10000) {
			// now see if the rising edge was between 500ms and 10 seconds after the
			// falling edge
			arg->dryContactDoorOpen = true;
		}
	}

    void IRAM_ATTR HOT RATGDOStore::isrDoorClose(RATGDOStore *arg) { 
        unsigned long currentMillis = millis();
        // Prevent ISR during the first 2 seconds after reboot
        if (currentMillis < 2000)
            return;
			
		if (!this->trigger_close.digital_read()) {
			// save the time of the falling edge
			arg->lastCloseDoorTime = currentMillis;
		} else if (currentMillis - arg->lastCloseDoorTime > 500 && currentMillis - arg->lastCloseDoorTime < 10000) {
			// now see if the rising edge was between 500ms and 10 seconds after the
			// falling edge
			arg->dryContactDoorClose = true;
		}	
	}

    void IRAM_ATTR HOT RATGDOStore::isrLight(RATGDOStore *arg) { 
        unsigned long currentMillis = millis();
        // Prevent ISR during the first 2 seconds after reboot
        if (currentMillis < 2000)
            return;
			
		if (!arg->trigger_light.digital_read()) {
			// save the time of the falling edge
			arg->lastToggleLightTime = currentMillis;
		} else if (currentMillis - arg->lastToggleLightTime > 500 && currentMillis - arg->lastToggleLightTime < 10000) {
			// now see if the rising edge was between 500ms and 10 seconds after the
			// falling edge
			arg->dryContactToggleLight = true;
		}		
	}

    // Fire on RISING edge of RPM1
    void IRAM_ATTR HOT RATGDOStore::isrRPM1(RATGDOStore *arg) { arg->rpm1Pulsed = true; }

    // Fire on RISING edge of RPM2
    // When RPM1 HIGH on RPM2 rising edge, door closing:
    // RPM1: __|--|___
    // RPM2: ___|--|__

    // When RPM1 LOW on RPM2 rising edge, door opening:
    // RPM1: ___|--|__
    // RPM2: __|--|___
    void IRAM_ATTR HOT RATGDOStore::isrRPM2(RATGDOStore *arg)
    {
        // The encoder updates faster than the ESP wants to process, so by sampling
        // every 5ms we get a more reliable curve The counter is behind the actual
        // pulse counter, but it doesn't matter since we only need a reliable linear
        // counter to determine the door direction
        if (millis() - arg->lastPulse < 5) {
            return;
        }

        // In rare situations, the rotary encoder can be parked so that RPM2
        // continuously fires this ISR. This causes the door counter to change value
        // even though the door isn't moving To solve this, check to see if RPM1
        // pulsed. If not, do nothing. If yes, reset the pulsed flag
        if (arg->rpm1Pulsed) {
            arg->rpm1Pulsed = false;
        } else {
            return;
        }

        arg->lastPulse = millis();

        // If the RPM1 state is different from the RPM2 state, then the door is
        // opening
		if (arg->input_rpm1.digital_read()) {
            arg->doorPositionCounter--;
        } else {
            arg->doorPositionCounter++;
        }
    }

    void IRAM_ATTR HOT RATGDOStore::isrObstruction()
    {
		if (this->input_obst.digital_read()) {
            arg->lastObstructionHigh = millis();
        } else {
            this->obstructionLowCount++;
        }
    }

    void RATGDOComponent::setup()
    {
        this->pref_ = global_preferences->make_preference<int>(734874333U);
        if (!this->pref_.load(&this->rollingCodeCounter)) {
            this->rollingCodeCounter = 0;
        }

		this->output_gdo_pin_->setup();
		this->store_.output_gdo = this->output_gdo_pin_->to_isr();
		this->trigger_open_pin_->setup();
		this->store_.trigger_open = this->trigger_open_pin_->to_isr();
		this->trigger_close_pin_->setup();
		this->store_.trigger_close = this->trigger_close_pin_->to_isr();
		this->trigger_light_pin_->setup();
		this->store_.trigger_light = this->trigger_light_pin_->to_isr();
		this->status_door_pin_->setup();
		this->store_.status_door = this->status_door_pin_->to_isr();
		this->status_obst_pin_->setup();
		this->store_.status_obst = this->status_obst_pin_->to_isr();
		this->input_rpm1_pin_->setup();
		this->store_.input_rpm1 = this->input_rpm1_pin_->to_isr();
		this->input_rpm2_pin_->setup();
		this->store_.input_rpm2 = this->input_rpm2_pin_->to_isr();
		this->input_obst_pin_->setup();

        this->swSerial.begin(9600, SWSERIAL_8N2, -1, this->output_gdo_pin_->get_pin(), true);

		this->trigger_open_pin_->pin_mode(gpio::FLAG_INPUT | gpio::FLAG_PULLUP);
		this->trigger_close_pin_->pin_mode(gpio::FLAG_INPUT | gpio::FLAG_PULLUP);
		this->trigger_light_pin_->pin_mode(gpio::FLAG_INPUT | gpio::FLAG_PULLUP);
		this->status_door_pin_->pin_mode(gpio::FLAG_OUTPUT);
		this->status_obst_pin_->pin_mode(gpio::FLAG_OUTPUT);
		this->input_rpm1_pin_->pin_mode(gpio::FLAG_INPUT | gpio::FLAG_PULLUP);  // set to pullup to add support for reed switches
		this->input_rpm2_pin_->pin_mode(gpio::FLAG_INPUT | gpio::FLAG_PULLUP);// make sure pin doesn't float when using reed switch
                           // and fire interrupt by mistake
		this->input_obst_pin_->pin_mode(gpio::FLAG_INPUT);


		this->trigger_open_pin_->attach_interrupt(RATGDOStore::isrDoorOpen, &this->_store, gpio::INTERRUPT_ANY_EDGE);
		this->trigger_close_pin_->attach_interrupt(RATGDOStore::isrDoorClose, &this->store_, gpio::INTERRUPT_ANY_EDGE);
		this->trigger_light_pin_->attach_interrupt(RATGDOStore::isrLight, &this->store_, gpio::INTERRUPT_ANY_EDGE);
		this->input_obst_pin_->attach_interrupt(RATGDOStore::isrObstruction, &this->store_, gpio::INTERRUPT_ANY_EDGE);
		this->input_rpm1_pin_->attach_interrupt(RATGDOStore::isrRPM1, &this->store_, gpio::INTERRUPT_RISING_EDGE);
		this->input_rpm2_pin_->attach_interrupt(RATGDOStore::isrRPM2, &this->store_, gpio::INTERRUPT_RISING_EDGE);

        if (this->useRollingCodes_) {
            ESP_LOGD(TAG, "Syncing rolling code counter after reboot...");
            sync(); // if rolling codes are being used (rolling code counter > 0), send
                    // reboot/sync to the opener on startup
        } else {
            ESP_LOGD(TAG, "Rolling codes are disabled.");
        }
    }

    void RATGDOComponent::loop()
    {
        obstructionLoop();
        doorStateLoop();
        dryContactLoop();
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
        } else {
            ESP_LOGD(TAG, "ERROR: Invalid command");
            return;
        }

        fixed = fixed | id;

        encode_wireline(this->rollingCodeCounter, fixed, data, this->rollingCode);

        printRollingCode();

        if (strcmp(command, "door1") != 0) { // door2 is created with same counter and should always be called after door1
            this->rollingCodeCounter = (this->rollingCodeCounter + 1) & 0xfffffff;
        }
        return;
    }

    void RATGDOComponent::printRollingCode()
    {
        for (int i = 0; i < CODE_LENGTH; i++) {
            if (this->rollingCode[i] <= 0x0f)
                ESP_LOGD(TAG, "0");
            ESP_LOGD(TAG, "%x", this->rollingCode[i]);
        }
    }

    void RATGDOComponent::set_rolling_codes(bool useRollingCodes)
    {
        this->useRollingCodes_ = useRollingCodes;
    }

    /*************************** DETECTING THE DOOR STATE
     * ***************************/
    void RATGDOComponent::doorStateLoop()
    {
        static bool rotaryEncoderDetected = false;
        static int lastDoorPositionCounter = 0;
        static int lastDirectionChangeCounter = 0;
        static int lastCounterMillis = 0;

        // Handle reed switch
        // This may need to be debounced, but so far in testing I haven't detected any
        // bounces
        if (!rotaryEncoderDetected) {
            if (!this->input_rpm1_pin_->digital_read()) {
                if (this->doorState != "reed_closed") {
                    ESP_LOGD(TAG, "Reed switch closed");
                    this->doorState = "reed_closed";
					this->status_door_pin_->digital_write(true);
                }
            } else if (this->doorState != "reed_open") {
                ESP_LOGD(TAG, "Reed switch open");
                this->doorState = "reed_open";
				this->status_door_pin_->digital_write(false);
            }
        }
        // end reed switch handling

        // If the previous and the current state of the RPM2 Signal are different,
        // that means there is a rotary encoder detected and the door is moving
        if (this->doorPositionCounter != lastDoorPositionCounter) {
            rotaryEncoderDetected = true; // this disables the reed switch handler
            lastCounterMillis = millis();

            ESP_LOGD(TAG, "Door Position: %d", doorPositionCounter);
        }

        // Wait 5 pulses before updating to door opening status
        if (this->doorPositionCounter - lastDirectionChangeCounter > 5) {
            if (this->doorState != "opening") {
                ESP_LOGD(TAG, "Door Opening...");
            }
            lastDirectionChangeCounter = this->doorPositionCounter;
            this->doorState = "opening";
        }

        if (lastDirectionChangeCounter - this->doorPositionCounter > 5) {
            if (this->doorState != "closing") {
                ESP_LOGD(TAG, "Door Closing...");
            }
            lastDirectionChangeCounter = this->doorPositionCounter;
            this->doorState = "closing";
        }

        // 250 millis after the last rotary encoder pulse, the door is stopped
        if (millis() - lastCounterMillis > 250) {
            // if the door was closing, and is now stopped, then the door is closed
            if (this->doorState == "closing") {
                this->doorState = "closed";
                ESP_LOGD(TAG, "Closed");
				this->status_door_pin_->digital_write(false);
            }

            // if the door was opening, and is now stopped, then the door is open
            if (this->doorState == "opening") {
                this->doorState = "open";
                ESP_LOGD(TAG, "Open");
				this->status_door_pin_->digital_write(true);
            }
        }

        lastDoorPositionCounter = doorPositionCounter;
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

        // the obstruction sensor has 3 states: clear (HIGH with LOW pulse every 7ms),
        // obstructed (HIGH), asleep (LOW) the transitions between awake and asleep
        // are tricky because the voltage drops slowly when falling asleep and is high
        // without pulses when waking up

        // If at least 3 low pulses are counted within 50ms, the door is awake, not
        // obstructed and we don't have to check anything else

        // Every 50ms
        if (currentMillis - lastMillis > 50) {
            // check to see if we got between 3 and 8 low pulses on the line
            if (this->obstructionLowCount >= 3 && this->obstructionLowCount <= 8) {
                obstructionCleared();

                // if there have been no pulses the line is steady high or low
            } else if (this->obstructionLowCount == 0) {
                // if the line is high and the last high pulse was more than 70ms ago,
                // then there is an obstruction present
                if (this->input_obst_pin_->digital_read() && currentMillis - this->lastObstructionHigh > 70) {
                    obstructionDetected();
                } else {
                    // asleep
                }
            }

            lastMillis = currentMillis;
            this->obstructionLowCount = 0;
        }
    }

    void RATGDOComponent::obstructionDetected()
    {
        static unsigned long lastInterruptTime = 0;
        unsigned long interruptTime = millis();
        // Anything less than 100ms is a bounce and is ignored
        if (interruptTime - lastInterruptTime > 250) {
            this->doorIsObstructed = true;
			this->status_obst_pin_->digital_write(true);
            ESP_LOGD(TAG, "Obstruction Detected");
        }
        lastInterruptTime = interruptTime;
    }

    void RATGDOComponent::obstructionCleared()
    {
        if (this->doorIsObstructed) {
            this->doorIsObstructed = false;
			this->status_obst_pin_->digital_write(false);
            ESP_LOGD(TAG, "Obstruction Cleared");
        }
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
    void RATGDOComponent::transmit(const unsigned char * payload, unsigned int length)
    {
		this->output_gdo_pin_->digital_write(true); // pull the line high for 1305 micros so the
                                        // door opener responds to the message
        delayMicroseconds(1305);
		this->output_gdo_pin_->digital_write(false); // bring the line low

        delayMicroseconds(1260); // "LOW" pulse duration before the message start
        this->swSerial.write(payload, length);
    }

    void RATGDOComponent::sync()
    {
        if (!this->useRollingCodes_)
            return;

        getRollingCode("reboot1");
        transmit(this->rollingCode, CODE_LENGTH);
        delay(45);

        getRollingCode("reboot2");
        transmit(this->rollingCode, CODE_LENGTH);
        delay(45);

        getRollingCode("reboot3");
        transmit(this->rollingCode, CODE_LENGTH);
        delay(45);

        getRollingCode("reboot4");
        transmit(this->rollingCode, CODE_LENGTH);
        delay(45);

        getRollingCode("reboot5");
        transmit(this->rollingCode, CODE_LENGTH);
        delay(45);

        getRollingCode("reboot6");
        transmit(this->rollingCode, CODE_LENGTH);
        delay(45);

        this->pref_.save(&this->rollingCodeCounter);
    }

	void RATGDOComponent::sendSyncCodes()
	{
		transmit(SYNC1, CODE_LENGTH);
		delay(45);
		transmit(SYNC2, CODE_LENGTH);
		delay(45);
		transmit(SYNC3, CODE_LENGTH);
		delay(45);
		transmit(SYNC4, CODE_LENGTH);
		delay(45);
	}						

    void RATGDOComponent::openDoor()
    {
        if (this->doorState == "open" || this->doorState == "opening") {
            ESP_LOGD(TAG, "The door is already %s", doorState);
            return;
        }

        this->doorState = "opening"; // It takes a couple of pulses to detect
                                     // opening/closing. by setting here, we can avoid
                                     // bouncing from rapidly repeated commands

        if (this->useRollingCodes) {
            getRollingCode("door1");
            transmit(this->rollingCode, CODE_LENGTH);

            delay(40);

            getRollingCode("door2");
            transmit(this->rollingCode, CODE_LENGTH);

            this->pref_.save(&this->rollingCodeCounter);
        } else {
            sendSyncCodes();
            ESP_LOGD(TAG, "door_code");
            transmit(DOOR_CODE, CODE_LENGTH);
        }
    }

    void RATGDOComponent::closeDoor()
    {
        if (this->doorState == "closed" || this->doorState == "closing") {
            ESP_LOGD(TAG, "The door is already %s", this->doorState);
            return;
        }

        this->doorState = "closing"; // It takes a couple of pulses to detect
                                     // opening/closing. by setting here, we can avoid
                                     // bouncing from rapidly repeated commands

        if (this->useRollingCodes_) {
            getRollingCode("door1");
            transmit(this->rollingCode, CODE_LENGTH);

            delay(40);

            getRollingCode("door2");
            transmit(this->rollingCode, CODE_LENGTH);

            this->pref_.save(&this->rollingCodeCounter);
        } else {
            sendSyncCodes();
            ESP_LOGD(TAG, "door_code");
            transmit(DOOR_CODE, CODE_LENGTH);
        }
    }

    void RATGDOComponent::toggleLight()
    {
        if (this->useRollingCodes) {
            getRollingCode("light");
            transmit(this->rollingCode, CODE_LENGTH);
            this->pref_.save(&this->rollingCodeCounter);
        } else {
            sendSyncCodes();
            ESP_LOGD(TAG, "light_code");
            transmit(LIGHT_CODE, CODE_LENGTH);
        }
    }



} // namespace ratgdo
} // namespace esphome
