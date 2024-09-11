#include <chrono>
#include <iostream>
#include <thread>
#include <csignal>  // For signal handling
#include <cstdlib>  // For exit()

#include "robot_factory.h"
#include "config.h"

namespace {
    void signalHandler(int signum) {
        exit(signum);
    }
}

int main(int argc, char** argv)
{
    using namespace std::chrono_literals;
    // Register the signal handler for SIGINT (Ctrl+C)
    std::signal(SIGINT, signalHandler);

    // stores the GPIO chip and all pin references
    Config config("config.yaml");
    RobotFactory factory;

    std::cout << "Creating motors\n";
    std::cout << "Creating left motor\n";
    Motor leftMotor = factory.createMotor(config.enableLeft, config.in1Left, config.in2Left);
    std::cout << "Creating right motor\n";
    Motor rightMotor = factory.createMotor(config.enableRight, config.in1Right, config.in2Right);

    std::chrono::milliseconds sleepTime = 1000ms;
   
    std::cout << "LEFT MOTOR tests\n";
    std::cout << "Left forward\n";
    for (int i = 0; i <= 10; i++) {
        std::cout << "PWM: " << 0.1 * static_cast<double>(i) << std::endl;
        leftMotor.sendCommand(0.1 * static_cast<double>(i), Direction::FORWARD);
        std::this_thread::sleep_for(sleepTime);
    }
    leftMotor.stop();
    std::cout << "Left backward\n";
    for (int i = 0; i <= 10; i++) {
        std::cout << "PWM: " << 0.1 * static_cast<double>(i) << std::endl;
        leftMotor.sendCommand(0.1 * static_cast<double>(i), Direction::REVERSE);
        std::this_thread::sleep_for(sleepTime);
    }
    leftMotor.stop();

    std::cout << "RIGHT MOTOR tests\n";
    std::cout << "Right forward\n";
    for (int i = 0; i <= 10; i++) {
        std::cout << "PWM: " << 0.1 * static_cast<double>(i) << std::endl;
        rightMotor.sendCommand(0.1 * static_cast<double>(i), Direction::FORWARD);
        std::this_thread::sleep_for(sleepTime);
    }
    rightMotor.stop();
    std::cout << "Right backward\n";
    for (int i = 0; i <= 10; i++) {
        std::cout << "PWM: " << 0.1 * static_cast<double>(i) << std::endl;
        rightMotor.sendCommand(0.1 * static_cast<double>(i), Direction::REVERSE);
        std::this_thread::sleep_for(sleepTime);
    }
    rightMotor.stop();

    std::cout << "BOTH tests\n";
    std::cout << "Driving forwards\n";
    for (int i = 0; i <= 10; i++) {
        std::cout << "PWM: " << 0.1 * static_cast<double>(i) << std::endl;
        leftMotor.sendCommand(0.1 * static_cast<double>(i), Direction::FORWARD);
        rightMotor.sendCommand(0.1 * static_cast<double>(i), Direction::FORWARD);
        std::this_thread::sleep_for(sleepTime);
    }
    leftMotor.stop();
    rightMotor.stop();

    std::cout << "Turning left\n";
    for (int i = 0; i <= 10; i++) {
        leftMotor.sendCommand(0.1 * static_cast<double>(i), Direction::FORWARD);
        rightMotor.sendCommand(0.1 * static_cast<double>(i), Direction::REVERSE);
        std::this_thread::sleep_for(sleepTime);
    }
    leftMotor.stop();
    rightMotor.stop();

    std::cout << "Turning right\n";
    for (int i = 0; i <= 10; i++) {
        leftMotor.sendCommand(0.1 * static_cast<double>(i), Direction::REVERSE);
        rightMotor.sendCommand(0.1 * static_cast<double>(i), Direction::FORWARD);
        std::this_thread::sleep_for(sleepTime);
    }
    leftMotor.stop();
    rightMotor.stop();

    std::cout << "Driving backwards\n";
    for (int i = 0; i <= 10; i++) {
        std::cout << "PWM: " << 0.1 * static_cast<double>(i) << std::endl;
        leftMotor.sendCommand(0.1 * static_cast<double>(i), Direction::REVERSE);
        rightMotor.sendCommand(0.1 * static_cast<double>(i), Direction::REVERSE);
        std::this_thread::sleep_for(sleepTime);
    }
    leftMotor.stop();
    rightMotor.stop();

    std::cout << "\nMOTOR TEST COMPLETE\n";

    return 0;
}
