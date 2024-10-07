#include "wheel.h"
#include "types.h"

bool Wheel::sendCommand(Command command) const {
    return sendCommand(command.dutyCycle, command.direction);
}

bool Wheel::sendCommand(double dutyCycle, Direction direction) const {
    return m_motor.sendCommand(dutyCycle, direction);
}
