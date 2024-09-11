#pragma once

#include "types.h"

class IController {
public:
    virtual ~IController() = default;

    virtual Command getCommand(double desiredVelocity, double currentVelocity) = 0;
};

class Controller : public IController {
public:
    Controller() = delete;
    Controller(double Kp, double Ki) : 
        m_Kp(Kp), m_Ki(Ki), m_error(0), m_errorSum(0) {}

    Command getCommand(double desiredVelocity, double currentVelocity) override;

private:
    double m_Kp;
    double m_Ki;
    double m_error;
    double m_errorSum;
};
