#include "robot_factory.h"
#include "config.h"

int main(int argc, char** argv)
{
    // stores the GPIO chip and all pin references
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

    // setup interrupts

    // every SPEED_INTERRUPT_MILLIS, get speed from encoders
    // this will be done via Robot::onSpeedInterrupt()

    // control polling loop
    // read input
    robot.getCurrentVelocity();

    return 0;
}
