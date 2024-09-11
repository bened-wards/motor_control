#pragma once

#include "motor.h"
#include "encoder.h"
#include "types.h"

class IWheel {
public:
    virtual ~IWheel() = default;

    virtual bool sendCommand(Command command) const = 0;
    virtual bool sendCommand(double dutyCycle, Direction direction) const = 0;
    virtual double getSpeed() const = 0;
    virtual double getRadius() const = 0;
};

class Wheel : public IWheel{
public:
    Wheel(double radius, const Motor& motor, Encoder& encoder) :     
        m_radius(radius), m_motor(motor), m_encoder(encoder) 
    {}

    bool sendCommand(Command command) const override;
    bool sendCommand(double dutyCycle, Direction direction) const override;

    double getSpeed() const override { return m_encoder.getSpeed(); }
    double getRadius() const override { return m_radius; }

private:
    double m_radius;
    const Motor& m_motor;
    Encoder& m_encoder;
};
