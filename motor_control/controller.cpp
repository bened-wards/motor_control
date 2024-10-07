#include "controller.h"
#include "types.h"

#include <algorithm>
#include <iostream>

template<typename T>
T clamp(T value, T min, T max) {
    return std::max(min, std::min(value, max));
}

template<typename T>
T isCloseToZero(T a, T tolerance) {
    return std::abs(a) < tolerance;
}

Command Controller::getCommand(double desiredVelocity, double currentVelocity) {
    // reset controller if reached a stop
    if (isCloseToZero(desiredVelocity, 0.01) && isCloseToZero(currentVelocity, 0.01)) {
        reset();
    }
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

void Controller::reset() {
    // std::cout << "Resetting controller as reached zero speed desired and current\n";
    m_errorSum = 0;
}
