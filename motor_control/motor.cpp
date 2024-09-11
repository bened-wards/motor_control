#include <iostream>

#include "motor.h"
#include "types.h"

bool Motor::sendCommand(double dutyCycle, Direction direction) const {
    if (!setPWM(dutyCycle)) {
        return false;
    }
    return setDirection(direction);
}

bool Motor::stop() const {
    return setPWM(0.0);
}

bool Motor::setPWM(double dutyCycle) const {
    if (dutyCycle < 0.0 || dutyCycle > 1.0) {
        return false;
    }
    return m_enablePin.set_duty_cycle(dutyCycle);
}

bool Motor::setDirection(Direction direction) const {
    bool result1 = false;
    bool result2 = false;
    if (direction == Direction::FORWARD) {
        result1 = m_input1Pin.set_value(1);
        result2 = m_input2Pin.set_value(0);
    } 
    else if (direction == Direction::REVERSE) {
        result1 = m_input1Pin.set_value(0);
        result2 = m_input2Pin.set_value(1);
    }
    return result1 && result2;
}
