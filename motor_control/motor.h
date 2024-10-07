#pragma once

#include "pwm_pin.h"
#include "pin_interfaces.h"
#include "types.h"

class Motor {
public:
    Motor() = delete;
    Motor(IPWMPin& enablePin, GPIOLine& input1Pin, GPIOLine& input2Pin) : 
        m_enablePin(enablePin), m_input1Pin(input1Pin), m_input2Pin(input2Pin) 
    {}

    // stop motor on shutdown (should be redundant as GPIO pins should go low)
    ~Motor() {
        stop();
    }

    bool sendCommand(double dutyCycle, Direction direction) const;
    bool stop() const;

private:
    bool setPWM(double dutyCycle) const;
    bool setDirection(Direction direction) const;

    IPWMPin& m_enablePin;
    GPIOLine& m_input1Pin;
    GPIOLine& m_input2Pin;
};
