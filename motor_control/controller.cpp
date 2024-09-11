#include "controller.h"
#include "types.h"

#include <algorithm>

template<typename T>
T clamp(T value, T min, T max) {
    return std::max(min, std::min(value, max));
}

Command Controller::getCommand(double desiredVelocity, double currentVelocity) {
    m_error = desiredVelocity - currentVelocity;
    double dutyCycle = clamp(m_Kp * m_error + m_Ki * m_errorSum, -1.0, 1.0);
    m_errorSum += m_error;

    if (dutyCycle < 0) {
        return Command{-dutyCycle, Direction::REVERSE};
    }
    else {
        return Command{dutyCycle, Direction::FORWARD};
    }
}
