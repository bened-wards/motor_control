#include <chrono>
#include <iostream>
#include <thread>

#include "robot_factory.h"
#include "config.h"
#include "loops.h"

void poll(std::chrono::seconds duration, Wheel& leftWheel, int speedInterruptMillis) {
    std::cout << "Running main polling loop in main thread for " << static_cast<long>(duration.count()) << "seconds\n";
    auto start = std::chrono::steady_clock::now();
    auto end = start + duration;
    int printCounter = 0;
    auto lastInterrupt = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() < end) {
        while (std::chrono::steady_clock::now() - lastInterrupt < std::chrono::milliseconds(speedInterruptMillis)) {
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
        double leftSpeed = leftWheel.getSpeed();
        lastInterrupt = std::chrono::steady_clock::now();
        printCounter++;
        if (printCounter > 10) {
            std::cout << "Left wheel speed: " << leftSpeed << "rad/s. RPM: " << leftSpeed * 60 / (2 * M_PI) << std::endl;
            printCounter = 0;
        }
    }
}

int main(int argc, char** argv)
{
    // stores the GPIO chip and all pin references
    Config config("config.yaml");
    RobotFactory factory;

    std::cout << "Creating motors\n";
    std::cout << "Creating left motor\n";
    Motor leftMotor = factory.createMotor(config.enableLeft, config.in1Left, config.in2Left);

    std::cout << "Creating encoders\n";
    std::cout << "Creating left encoder\n";
    Encoder leftEncoder = factory.createLeftEncoder(config.encALeft, config.encBLeft, config.ticksPerRev, config.speedInterruptMillis);

    std::cout << "Creating wheels (needs motors and encoders)";
    Wheel leftWheel = factory.createWheel(config.wheelRadius, leftMotor, leftEncoder);

    try {
        std::cout << "Creating left encoder thread\n";
        std::thread leftThread(Loops::encoder_event_handler, std::ref(leftEncoder));

        std::cout << "BOTH tests\n";
        std::cout << "Driving forwards for 5s\n";
        leftMotor.sendCommand(0.5, Direction::FORWARD);
        poll(std::chrono::seconds(5), leftWheel, config.speedInterruptMillis);
        leftMotor.stop();

        std::cout << "Turning left\n";
        leftMotor.sendCommand(0.5, Direction::FORWARD);
        poll(std::chrono::seconds(5), leftWheel, config.speedInterruptMillis);
        leftMotor.stop();

        std::cout << "Turning right\n";
        leftMotor.sendCommand(0.5, Direction::REVERSE);
        poll(std::chrono::seconds(5), leftWheel, config.speedInterruptMillis);
        leftMotor.stop();

        std::cout << "Driving backwards\n";
        leftMotor.sendCommand(0.5, Direction::REVERSE);
        poll(std::chrono::seconds(5), leftWheel, config.speedInterruptMillis);
        leftMotor.stop();
        
        // kill encoder loops
        Loops::stopFlag = true;
        leftThread.join();
    } catch (const std::exception &e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
