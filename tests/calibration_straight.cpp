#include "robot_factory.h"
#include "config.h"
#include "loops.h"
#include "precise_sleep.h"

void poll(std::chrono::seconds duration, Robot& robot, int speedInterruptMillis, bool debugMode) {
    std::cout << "Running main polling loop in main thread for " << static_cast<long>(duration.count()) << "seconds\n";
    auto start = std::chrono::high_resolution_clock::now();
    auto end = start + duration;
    int printCounter = 0;
    auto lastInterrupt = std::chrono::high_resolution_clock::now();
    while (std::chrono::high_resolution_clock::now() < end) {
        auto nextClock = std::chrono::high_resolution_clock::now();
        double dTMillis = (nextClock - lastInterrupt).count() / 1e6;
        lastInterrupt = std::chrono::high_resolution_clock::now();
        robot.onSpeedInterrupt(dTMillis);
        const State& state = robot.getState();
        
        if (debugMode && ++printCounter > 25) {
            std::cout << "dT: " << dTMillis << "ms\n";
	        const VelocityState& vel = robot.getCurrentVelocity();
            const VelocityState& desiredVel = robot.getDesiredVelocity();
            const WheelSpeed& wheelSpeed = robot.getCurrentWheelSpeed();
            std::cout << "LWS RPM" << wheelSpeed.left * 60 / (2 * M_PI) << " RWS RPM" << wheelSpeed.right * 60 / (2 * M_PI) << std::endl;
            std::cout << "Curr. v=" << vel.v << ", w=" << vel.w << ". Des. v=" << desiredVel.v << ", w=" << desiredVel.w << std::endl;
            std::cout << "State: x=" << state.x << ", y=" << state.y << ", theta=" << state.theta << std::endl;
            printCounter = 0;
        }

        auto sleepClock = std::chrono::high_resolution_clock::now();
        double sleepMillis = speedInterruptMillis - (lastInterrupt - sleepClock).count() / 1e6;
        if (sleepMillis > 0) {
            preciseSleep((sleepMillis-0.2)/1e3);
        }
    }
}

int main(int argc, char** argv)
{
    // Set global formatting
    std::cout << std::fixed << std::setprecision(5);

    // stores the GPIO chip and all pin references
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
    // use scales of 1 for calibration
    Robot robot = factory.createRobot(config.wheelSeparation, 1, 1, leftWheel, rightWheel, leftController, rightController, config.speedInterruptMillis);
    std::cout << robot.getWheelRadius() << " cm (wheel radius)\n";

    try {
        std::cout << "Creating left encoder thread\n";
        std::thread leftThread(Loops::encoder_event_handler, std::ref(leftEncoder));
        std::cout << "Creating right encoder thread\n";
        std::thread rightThread(Loops::encoder_event_handler, std::ref(rightEncoder));

        double arr[3] = {0.1, 0.2, 0.25};
        for (int i = 0; i < 3; i++) {
            double vel = arr[i];
            std::cout << "Driving forwards for 8s with speed " << vel << std::endl;
            robot.setDesiredVelocity(VelocityState{vel,0});
            poll(std::chrono::seconds(8), robot, config.speedInterruptMillis, config.debugMode);
            leftMotor.stop();
            rightMotor.stop();
            State state = robot.getState();
            std::cout << "Final State: x=" << state.x << ", y=" << state.y << ", theta=" << state.theta << std::endl;
            std::cin.get(); // pause until enter
            robot.setState(State{0,0,0});
        }

        // kill encoder loops
        Loops::stopFlag = true;
        leftThread.join();
        rightThread.join();
    } catch (const std::exception &e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
