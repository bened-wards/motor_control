#include "robot_factory.h"
#include "config.h"
#include "loops.h"

namespace {
    void catchKeyboardInterrupt(int signum) {
        std::cout << "Got keyboard interrupt, exiting\n";
        Loops::stopFlag = true;
    }
}

void poll(std::chrono::seconds duration, Robot& robot, Serial& serial, int speedInterruptMillis, bool debugMode) {
    std::cout << "Running main polling loop in main thread for " << static_cast<long>(duration.count()) << "seconds\n";
    auto start = std::chrono::steady_clock::now();
    auto end = start + duration;
    int printCounter = 0;
    auto lastInterrupt = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() < end && !Loops::stopFlag.load()) {
        while (std::chrono::steady_clock::now() - lastInterrupt < std::chrono::milliseconds(speedInterruptMillis)) {
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
        robot.onSpeedInterrupt();
        lastInterrupt = std::chrono::steady_clock::now();
        // send the current robot state to the main controller
        const State& state = robot.getState();
        serial.writeCurrentState(state.x, state.y, state.theta);

        if (debugMode && ++printCounter > 25) {
	        const VelocityState& vel = robot.getCurrentVelocity();
            const VelocityState& desiredVel = robot.getDesiredVelocity();
            std::cout << "Curr. v=" << vel.v << ", w=" << vel.w << ". Des. v=" << desiredVel.v << ", w=" << desiredVel.w << std::endl;
            std::cout << "State: x=" << state.x << ", y=" << state.y << ", theta=" << state.theta << std::endl;
            printCounter = 0;
        }
    }
}

int main(int argc, char** argv)
{
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <integer> - <timeout>\n";
        return 1;
    }
    int timeout = std::atoi(argv[1]);

    // register keyboard interrupt handler
    signal(SIGINT, catchKeyboardInterrupt);

    // Set global formatting
    std::cout << std::fixed << std::setprecision(2);

    // create and open serial port
    std::cout << "Creating serial port\n";
    Serial serial = Serial();

    // stores the GPIO chip and all pin references
    std::cout << "Loading config\n";
    Config config("config.yaml");
    RobotFactory factory;

    std::cout << "Creating motors\n";
    std::cout << "Creating left motor\n";
    Motor leftMotor = factory.createMotor(config.enableLeft, config.in1Left, config.in2Left);
    std::cout << "Creating right motor\n";
    Motor rightMotor = factory.createMotor(config.enableRight, config.in1Right, config.in2Right);

    std::cout << "Creating encoders\n";
    std::cout << "Creating left encoder\n";
    Encoder leftEncoder = factory.createLeftEncoder(config.encALeft, config.encBLeft, config.ticksPerRev, config.speedInterruptMillis);
    std::cout << "Creating right encoder\n";
    Encoder rightEncoder = factory.createRightEncoder(config.encARight, config.encBRight, config.ticksPerRev, config.speedInterruptMillis);

    std::cout << "Creating wheels (needs motors and encoders)";
    Wheel leftWheel = factory.createWheel(config.wheelRadius, leftMotor, leftEncoder);
    Wheel rightWheel = factory.createWheel(config.wheelRadius, rightMotor, rightEncoder);

    std::cout << "Creating controllers with Kp: " << config.Kp << " and Ki: " << config.Ki << std::endl;
    Controller leftController = factory.createController(config.Kp, config.Ki);
    Controller rightController = factory.createController(config.Kp, config.Ki);

    std::cout << "Creating robot\n";
    Robot robot = factory.createRobot(
        config.wheelSeparation, config.wheelSeparationScale, config.wheelRadiusScale, 
        leftWheel, rightWheel, leftController, rightController, config.speedInterruptMillis);
    std::cout << robot.getWheelRadius() << " cm (wheel radius)\n";

    try {
        std::cout << "Creating left encoder thread\n";
        std::thread leftThread(Loops::encoder_event_handler, std::ref(leftEncoder));
        std::cout << "Creating right encoder thread\n";
        std::thread rightThread(Loops::encoder_event_handler, std::ref(rightEncoder));

        // create worker thread to read from serial port and set robot velocity
        std::thread readThread(Loops::readAndSetVelocityAndState, std::ref(serial), std::ref(robot));

        // main polling loop
        poll(std::chrono::seconds(timeout), robot, serial, config.speedInterruptMillis, config.debugMode);

        // kill all threads (encoder loops, read and write serial threads)
        Loops::stopFlag = true;
        leftThread.join();
        rightThread.join();
        readThread.join();
    } catch (const std::exception &e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
