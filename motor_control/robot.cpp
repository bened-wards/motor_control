#include "robot.h"

#include "math.h"

// TO BE CALLED EVERY m_speedInterruptMillis
// this function will get the current wheel speeds
// update the robot state based on the current wheel speeds
// and then calculate and send the new wheel commands based on the 
// current wheel speeds and the desired wheel speeds
void Robot::onSpeedInterrupt() {
    double leftSpeed = m_leftWheel.getSpeed();
    double rightSpeed = m_rightWheel.getSpeed();
    setCurrentVelocity(leftSpeed, rightSpeed);
    // std::cout << "\nONSPEEDINTERRUPT\n" << std::endl;

    // TODO should probably just call getSpeed and do the rest in the polling loop

    updatePose(m_speedInterruptMillis);

    WheelSpeed desiredWheelSpeed = getDesiredWheelSpeed();
    Command leftCommand = m_leftController.getCommand(desiredWheelSpeed.left, leftSpeed);
    Command rightCommand = m_rightController.getCommand(desiredWheelSpeed.right, rightSpeed);

    // std::cout << "Curr wheel speeds: " << leftSpeed << ", " << rightSpeed << " vs desired: " << desiredWheelSpeed.left << ", " << desiredWheelSpeed.right << std::endl;
    // std::cout << "Left command: " << ((leftCommand.direction==Direction::REVERSE) ? '-' : '+') <<  leftCommand.dutyCycle << 
    //     ". Right command: " << ((rightCommand.direction==Direction::REVERSE) ? '-' : '+') << rightCommand.dutyCycle << std::endl;

    m_leftWheel.sendCommand(leftCommand);
    m_rightWheel.sendCommand(rightCommand);
}

// update state based on driving at velocity for last dt millis
void Robot::updatePose(const VelocityState& velocity, double dtMillis) {
    std::lock_guard<std::mutex> lock(m_stateMutex);
    // convert dt to seconds
    double dt = dtMillis / 1000.0;
    double v = velocity.v;
    double w = velocity.w;
    if (w == 0) {
        m_state.x += std::cos(m_state.theta) * v * dt;
        m_state.y += std::sin(m_state.theta) * v * dt;
    }
    else {
        m_state.x += v / w * (std::sin(m_state.theta+dt*w) - std::sin(m_state.theta));
        m_state.y += -v / w * (std::cos(m_state.theta+dt*w) - std::cos(m_state.theta));
        m_state.theta += dt * w;
    }
}

VelocityState Robot::wheelSpeedToVelocity(double leftSpeed, double rightSpeed) {
    double wheelRadius = getWheelRadius();
    double linearVel = m_wheelRadiusScale * wheelRadius * (leftSpeed +  rightSpeed) / 2.0;
    double angularVel = m_wheelRadiusScale * wheelRadius * (rightSpeed - leftSpeed) / (m_wheelSeparation * m_wheelSeparationScale);

    return VelocityState{linearVel, angularVel};
}

VelocityState Robot::wheelSpeedToVelocity(const WheelSpeed& wheelSpeed) {
    return wheelSpeedToVelocity(wheelSpeed.left, wheelSpeed.right);
}

WheelSpeed Robot::velocityToWheelSpeed(double linearVelocity, double angularVelocity) {
    double wheelRadius = getWheelRadius();
    double leftSpeed = (linearVelocity - 0.5 * (m_wheelSeparation * m_wheelSeparationScale) * angularVelocity) / (wheelRadius * m_wheelRadiusScale);
    double rightSpeed = (linearVelocity + 0.5 * (m_wheelSeparation * m_wheelSeparationScale) * angularVelocity) / (wheelRadius * m_wheelRadiusScale);
    return WheelSpeed {leftSpeed, rightSpeed};
}

WheelSpeed Robot::velocityToWheelSpeed(const VelocityState& velocity) {
    return velocityToWheelSpeed(velocity.v, velocity.w);
}
