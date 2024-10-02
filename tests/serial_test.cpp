#include <iostream>
#include <csignal>
#include <cstdlib>
#include <atomic>
#include <thread>
#include <string.h>

#include "types.h"
#include "serial.h"
#include "loops.h"
#include "robot_factory.h"
#include "config.h"

void catchKeyboardInterrupt(int signum) {
    std::cout << "Got keyboard interrupt, exiting\n";
    Loops::stopFlag = true;
}

void writeToSerial(Serial& serial, std::chrono::milliseconds send_period) {
    double v = 1.555;
    double w = 2.342;
    double x = 1.000;
    double y = 2.000;
    double theta = 3.000;
    while (!Loops::stopFlag.load()) {
        // serial.writeCurrentVelocity(v, w);
        v -= 0.1;
        w += 0.1;
        serial.writeCurrentState(x, y, theta);
        x -= 0.1;
        y += 0.1;
        theta -= 0.1;
        std::this_thread::sleep_for(std::chrono::milliseconds(send_period));
    }
}


int main(int argc, char** argv)
{
    // create and open serial port
    Serial serial = Serial();

    // dummy robot for writing desired velocity
    Config config("config.yaml");
    RobotFactory factory;
    Motor leftMotor = factory.createMotor(config.enableLeft, config.in1Left, config.in2Left);
    Motor rightMotor = factory.createMotor(config.enableRight, config.in1Right, config.in2Right);
    Encoder leftEncoder = factory.createLeftEncoder(config.encALeft, config.encBLeft, config.ticksPerRev, config.speedInterruptMillis);
    Encoder rightEncoder = factory.createRightEncoder(config.encARight, config.encBRight, config.ticksPerRev, config.speedInterruptMillis);
    Wheel leftWheel = factory.createWheel(config.wheelRadius, leftMotor, leftEncoder);
    Wheel rightWheel = factory.createWheel(config.wheelRadius, rightMotor, rightEncoder);
    Controller leftController = factory.createController(config.Kp, config.Ki);
    Controller rightController = factory.createController(config.Kp, config.Ki);
    Robot robot = factory.createRobot(
        config.wheelSeparation, config.wheelSeparationScale, config.wheelRadiusScale, 
        leftWheel, rightWheel, leftController, rightController, config.speedInterruptMillis);
    std::cout << robot.getWheelRadius() << " cm (wheel radius)\n";

    // register keyboard interrupt handler
    signal(SIGINT, catchKeyboardInterrupt);

    // create worker thread to read and write to serial port
    std::thread readThread(Loops::readAndSetDesiredVelocity, std::ref(serial), std::ref(robot));
    std::thread writeThread(writeToSerial, std::ref(serial), std::chrono::milliseconds(2500));

    readThread.join();
    writeThread.join();

    return 0;
}
