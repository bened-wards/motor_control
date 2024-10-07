// Example on how to use sysfs interface for hardware pwm
#include <chrono>
#include <thread>
#include <iostream>

#include "pin_interfaces.h"
#include "robot_factory.h"

using namespace std::chrono_literals;

void test_output(GPIOLine& line)
{
    line.set_value(1);
    std::cout << "Writing high for 5s" << std::endl;
    std::this_thread::sleep_for(5s);

    line.set_value(0);
    std::cout << "Writing low for 3s" << std::endl;
    std::this_thread::sleep_for(3s);

    line.set_value(1);
    std::cout << "Writing high for 5s" << std::endl;
    std::this_thread::sleep_for(5s);

    line.set_value(0);
    std::cout << "Output Test Complete" << std::endl;
}

void test_input(GPIOLine& line)
{
    std::cout << "Reading input, change voltage\n";

    for (int i = 0; i < 20; i++) {
        std::cout << "Value: " << line.get_value() << std::endl;
        std::this_thread::sleep_for(0.5s);
    }
}

void test_rising_edge(GPIOLine& line)
{
    std::cout << "Waiting for rising edge for 10s\n";
    for (int i = 0; i < 20000; i++) {
        bool eventOccurred = line.event_wait(std::chrono::microseconds(500));
        if (eventOccurred && line.event_read().event_type == gpiod::line_event::RISING_EDGE) {
            std::cout << "Got rising edge\n";
        }
    }
}


int main(int argc, char** argv)
{
    using namespace std::chrono_literals;

    RobotFactory robot;
    
    std::cout << "Setting up output pin\n";
    auto outputLine = robot.createAndSetupGPIOLine(24, gpiod::line_request::DIRECTION_OUTPUT);
    GPIOLine outputPin { std::move(outputLine) };
    std::cout << "Setting up input pin\n";
    auto inputLine = robot.createAndSetupGPIOLine(25, gpiod::line_request::DIRECTION_INPUT);
    GPIOLine inputPin { std::move(inputLine) };
    std::cout << "Setting up rising edge pin\n";
    auto risingEdgeLine = robot.createAndSetupGPIOLine(26, gpiod::line_request::EVENT_RISING_EDGE);
    GPIOLine risingEdgePin { std::move(risingEdgeLine) };


    std::cout << "Testing output pin\n";
    test_output(outputPin);
    std::cout << "Testing input pin\n";
    test_input(inputPin);
    std::cout << "Testing rising edge pin\n";
    test_rising_edge(risingEdgePin);    

    return 0;
}
