#include <chrono>
#include <thread>

#include "robot_factory.h"
#include "config.h"
#include "loops.h"

int main(int argc, char** argv)
{
    // stores the GPIO chip and all pin references
    Config config("config.yaml");
    RobotFactory factory;

    std::cout << "Creating left encoder\n";
    Encoder leftEncoder = factory.createLeftEncoder(config.encALeft, config.encBLeft, config.ticksPerRev, config.speedInterruptMillis);
    std::cout << "Creating right encoder\n";
    Encoder rightEncoder = factory.createRightEncoder(config.encARight, config.encBRight, config.ticksPerRev, config.speedInterruptMillis);

    try {
        std::cout << "Creating left encoder thread\n";
        std::thread leftThread(Loops::encoder_event_handler, std::ref(leftEncoder));

        std::cout << "Creating right encoder thread\n";
        std::thread rightThread(Loops::encoder_event_handler, std::ref(rightEncoder));

        std::cout << "Running main polling loop in main thread\n";
        int printCounter = 0;
        while (true) {
            std::this_thread::sleep_for(std::chrono::milliseconds(config.speedInterruptMillis));
            double leftSpeed = leftEncoder.getSpeed(config.speedInterruptMillis);
            double rightSpeed = rightEncoder.getSpeed(config.speedInterruptMillis);
            printCounter++;
            if (printCounter > 10) {
                std::cout << "Left encoder speed: " << leftSpeed << "rad/s. RPM: " << leftSpeed * 60 / (2 * M_PI) << std::endl;
                std::cout << "Right encoder speed: " << rightSpeed << "rad/s. RPM: " << rightSpeed * 60 / (2 * M_PI) << std::endl;
                printCounter = 0;
            }
        }

        leftThread.join();
        rightThread.join();
    } catch (const std::exception &e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
