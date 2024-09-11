#pragma once

#include <mutex>

#include "types.h"
#include "wheel.h"
#include "controller.h"

class Robot {
public:
    Robot() = delete;
    Robot(
        double wheelSeparation, double wheelSeparationScale,
        double wheelRadiusScale, 
        IWheel& leftWheel, IWheel& rightWheel,
        IController& leftController, IController& rightController,
        double speedInterruptMillis) : 
        m_wheelSeparation(wheelSeparation), m_wheelSeparationScale(wheelSeparationScale),
        m_wheelRadiusScale(wheelRadiusScale),
        m_leftWheel(leftWheel), m_rightWheel(rightWheel),
        m_leftController(leftController), m_rightController(rightController),
        m_speedInterruptMillis(speedInterruptMillis)
    {
        assert(m_leftWheel.getRadius() == m_rightWheel.getRadius());
    }

    void onSpeedInterrupt();
    void updatePose(const VelocityState& velocity, double dt);
    void updatePose(double dt) { updatePose(m_currentVelocity, dt);}

    VelocityState wheelSpeedToVelocity(double leftSpeed, double rightSpeed);
    VelocityState wheelSpeedToVelocity(const WheelSpeed& wheelSpeed);
    WheelSpeed velocityToWheelSpeed(double linearVelocity, double angularVelocity);
    WheelSpeed velocityToWheelSpeed(const VelocityState& velocity);

    const State& getState() { 
        std::lock_guard<std::mutex> lock(m_stateMutex);
        return m_state; 
    }
    void setState(const State& state) { 
        std::lock_guard<std::mutex> lock(m_stateMutex);
        m_state = state; 
    }
    double getWheelRadius() { return m_leftWheel.getRadius(); }

    const VelocityState& getDesiredVelocity() { 
        std::lock_guard<std::mutex> lock(m_desiredVelocityMutex);
        return m_desiredVelocity; 
    }
    WheelSpeed getDesiredWheelSpeed() { 
        std::lock_guard<std::mutex> lock(m_desiredVelocityMutex);
        return velocityToWheelSpeed(m_desiredVelocity); 
    };
    const VelocityState& getCurrentVelocity() { 
        std::lock_guard<std::mutex> lock(m_currentVelocityMutex);
        return m_currentVelocity; 
    }
    WheelSpeed getCurrentWheelSpeed() { 
        std::lock_guard<std::mutex> lock(m_currentVelocityMutex);
        return velocityToWheelSpeed(m_currentVelocity); 
    }
    void setDesiredVelocity(double leftSpeed, double rightSpeed) { setDesiredVelocity(wheelSpeedToVelocity(leftSpeed, rightSpeed)); }
    void setDesiredVelocity(const WheelSpeed& wheelSpeed) { setDesiredVelocity(wheelSpeedToVelocity(wheelSpeed)); }
    void setDesiredVelocity(const VelocityState& velocity) { 
        std::lock_guard<std::mutex> lock(m_desiredVelocityMutex);
        m_desiredVelocity = velocity; 
    }
    // TODO -> maybe these should be private/not exist?
    void setCurrentVelocity(double leftSpeed, double rightSpeed) { setCurrentVelocity(wheelSpeedToVelocity(leftSpeed, rightSpeed)); }
    void setCurrentVelocity(const WheelSpeed& wheelSpeed) { setCurrentVelocity(wheelSpeedToVelocity(wheelSpeed)); }
    void setCurrentVelocity(const VelocityState& velocity) { 
        std::lock_guard<std::mutex> lock(m_currentVelocityMutex);
        m_currentVelocity = velocity; 
    }

private:
    std::mutex m_stateMutex;
    State m_state;
    std::mutex m_desiredVelocityMutex;
    VelocityState m_desiredVelocity;
    std::mutex m_currentVelocityMutex;
    VelocityState m_currentVelocity;

    const double m_wheelSeparation;
    const double m_wheelSeparationScale; // for adjustment of wheel baseline
    const double m_wheelRadiusScale; // for adjustment of wheel radius based on wheel slip calibration
    IWheel& m_leftWheel;
    IWheel& m_rightWheel;
    IController& m_leftController;
    IController& m_rightController;
    const double m_speedInterruptMillis;
};
