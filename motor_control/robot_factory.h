#pragma once

#include <iostream>
#include <vector>
#include <gpiod.hpp>
#include <thread>

#include "motor.h"
#include "encoder.h"
#include "wheel.h"
#include "robot.h"
#include "controller.h"
#include "pwm_pin.h"
#include "pin_interfaces.h"

class RobotFactory {
public:
    RobotFactory() {
        m_chip = gpiod::chip();
        m_chip.open("/dev/gpiochip0", gpiod::chip::OPEN_BY_PATH);
        
        // reserving memory so that reallocation doesn't occur
        m_gpioPins.reserve(20);
        m_pwmPins.reserve(2);
    }

    // NOTE: pinType should be of type gpiod::line_request::___
    // bad programming practices because is a cpp version of a C library
    gpiod::line createAndSetupGPIOLine(int pinNumber, int pinType) {
        auto request = gpiod::line_request();
        request.request_type = pinType; 
        gpiod::line line = m_chip.get_line(pinNumber);
        line.request(request);
        return line;
    }

    GPIOLine& moveLineToGPIOLineVector(gpiod::line line) {
        GPIOLine gpioLine { std::move(line) };
        m_gpioPins.push_back(std::move(gpioLine));
        return m_gpioPins.back();
    }

    Motor createMotor(int enablePinNumber, int input1PinNumber, int input2PinNumber) {
        assert(enablePinNumber == 0 || enablePinNumber == 1);
        std::cout << "Creating pwm pin\n";
        auto enablePin = std::make_unique<PWMPin>(0, static_cast<unsigned int>(enablePinNumber));
        m_pwmPins.push_back(std::move(enablePin));
        std::cout << "Sleeping for 2s for PWM setup\n";
        std::this_thread::sleep_for(std::chrono::seconds(2));
        // set duty cycle to zero and enable
        auto& enablePinRef = *(m_pwmPins.back());
        enablePinRef.set_duty_cycle(0.0);
        enablePinRef.set_period(500000); // 2kHz
        enablePinRef.enable();
        
        std::cout << "Creating output pins\n";
        gpiod::line input1Line = createAndSetupGPIOLine(input1PinNumber, gpiod::line_request::DIRECTION_OUTPUT);
        gpiod::line input2Line = createAndSetupGPIOLine(input2PinNumber, gpiod::line_request::DIRECTION_OUTPUT);
        GPIOLine& in1GPIOLine = moveLineToGPIOLineVector(input1Line);
        GPIOLine& in2GPIOLine = moveLineToGPIOLineVector(input2Line);
        std::cout << "Creating motor\n";
        return Motor {*(m_pwmPins.back()), in1GPIOLine, in2GPIOLine};
    }

    Encoder createLeftEncoder(int encAPinNumber, int encBPinNumber, int ticksPerRev, int deltaTMillis) {
        return createEncoder(encAPinNumber, encBPinNumber, ticksPerRev, deltaTMillis, true);
    }

    Encoder createRightEncoder(int encAPinNumber, int encBPinNumber, int ticksPerRev, int deltaTMillis) {
        return createEncoder(encAPinNumber, encBPinNumber, ticksPerRev, deltaTMillis, false);
    }

    Wheel createWheel(double radius, const Motor& motor, Encoder& encoder) {
        return Wheel(radius, motor, encoder);
    }

    Controller createController(double Kp, double Ki) {
        return Controller(Kp, Ki);
    }

    Robot createRobot(
        double wheelSeparation, double wheelSeparationScale, double wheelRadiusScale, 
        Wheel& leftWheel, Wheel& rightWheel, Controller& leftController, Controller& rightController, 
        double speedInterruptMillis) 
    {
        return Robot(wheelSeparation, wheelSeparationScale, wheelRadiusScale, leftWheel, rightWheel, leftController, rightController, speedInterruptMillis);
    }

private:
    Encoder createEncoder(int encAPinNumber, int encBPinNumber, int ticksPerRev, int deltaTMillis, bool left=false) {
        gpiod::line encAPin = createAndSetupGPIOLine(encAPinNumber, gpiod::line_request::EVENT_BOTH_EDGES);
        gpiod::line encBPin = createAndSetupGPIOLine(encBPinNumber, gpiod::line_request::EVENT_BOTH_EDGES);
        if (left) {
            m_encLeftPins.push_back(encAPin);
            m_encLeftPins.push_back(encBPin);
            m_encLeftLineBulk = gpiod::line_bulk(m_encLeftPins);
            return Encoder { m_encLeftLineBulk, ticksPerRev, deltaTMillis };
        } else {
            m_encRightPins.push_back(encAPin);
            m_encRightPins.push_back(encBPin);
            m_encRightLineBulk = gpiod::line_bulk(m_encRightPins);
            return Encoder { m_encRightLineBulk, ticksPerRev, deltaTMillis };
        }
    }

    gpiod::chip m_chip;
    std::vector<GPIOLine> m_gpioPins;
    std::vector<gpiod::line> m_encLeftPins;
    std::vector<gpiod::line> m_encRightPins;
    gpiod::line_bulk m_encLeftLineBulk;
    gpiod::line_bulk m_encRightLineBulk;
    std::vector<std::unique_ptr<PWMPin>> m_pwmPins;
};
