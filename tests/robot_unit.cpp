#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "mock_classes.h"
#include "robot.h"

using namespace ::testing;

class RobotTest : public ::testing::Test {
protected:
    void SetUp() override {
        EXPECT_CALL(leftWheel, getRadius()).WillRepeatedly(Return(wheelRadius));
        EXPECT_CALL(rightWheel, getRadius()).WillRepeatedly(Return(wheelRadius));
        robot = std::make_unique<Robot>(wheelSeparation, wheelSeparationScale, wheelRadius, leftWheel, rightWheel, leftController, rightController, speedInterruptMillis);
    }

    void TearDown() override {
    }

    void setupCommandGotAndSent(
        double lCurr, double rCurr, double lDesired, double rDesired, 
        double dutyCycle=0.0, Direction direction=Direction::FORWARD
    ) {
        EXPECT_CALL(leftWheel, getSpeed()).WillRepeatedly(Return(lCurr));
        EXPECT_CALL(rightWheel, getSpeed()).WillRepeatedly(Return(rCurr));
        EXPECT_CALL(leftController, getCommand(lDesired, lCurr)).WillRepeatedly(Return(Command{dutyCycle, direction}));
        EXPECT_CALL(rightController, getCommand(rDesired, rCurr)).WillRepeatedly(Return(Command{dutyCycle, direction}));
        EXPECT_CALL(leftWheel, sendCommand(Command{dutyCycle, direction})).WillRepeatedly(Return(true));
        EXPECT_CALL(rightWheel, sendCommand(Command{dutyCycle, direction})).WillRepeatedly(Return(true));
    }

    void checkState(State current, State expected) {
        EXPECT_DOUBLE_EQ(current.x, expected.x);
        EXPECT_DOUBLE_EQ(current.y, expected.y);
        EXPECT_DOUBLE_EQ(current.theta, expected.theta);
    }

    void checkVelocityState(VelocityState current, VelocityState expected) {
        EXPECT_DOUBLE_EQ(current.v, expected.v);
        EXPECT_DOUBLE_EQ(current.w, expected.w);
    }

    void checkWheelSpeeds(WheelSpeed current, WheelSpeed expected) {
        EXPECT_DOUBLE_EQ(current.left, expected.left);
        EXPECT_DOUBLE_EQ(current.right, expected.right);
    }

    double wheelRadius = 0.05; // 5cm
    double wheelRadiusScale = 1.0; 
    double wheelSeparation = 0.1; // 10cm
    double wheelSeparationScale = 1.0;
    double speedInterruptMillis = 1000; // 1s to simplify calcs
    MockWheel leftWheel;
    MockWheel rightWheel;
    MockController leftController;
    MockController rightController;
    std::unique_ptr<Robot> robot;
};

TEST_F(RobotTest, wheelSpeedToVelocity_positiveSpeeds_returnsCorrectVelocity) {
    double leftSpeed = 1.0;
    double rightSpeed = 1.0;

    VelocityState velocity = robot->wheelSpeedToVelocity(leftSpeed, rightSpeed);

    // 1.0 * 0.05 * (1.0+1.0) / 2.0 = 0.05
    double linearVel = wheelRadiusScale * wheelRadius * (leftSpeed +  rightSpeed) / 2.0;
    // 1.0 * 0.05 * (1.0-1.0) / 0.1 = 0.0
    double angularVel = wheelRadiusScale * wheelRadius * (rightSpeed - leftSpeed) / (wheelSeparation * wheelSeparationScale);

    EXPECT_DOUBLE_EQ(linearVel, velocity.v);
    EXPECT_DOUBLE_EQ(0.05, velocity.v);
    EXPECT_DOUBLE_EQ(angularVel, velocity.w);
    EXPECT_DOUBLE_EQ(0.0, velocity.w);
}

TEST_F(RobotTest, wheelSpeedToVelocity_positiveSpeedsDiff_returnsCorrectVelocity) {
    double leftSpeed = 1.0;
    double rightSpeed = 2.0;

    VelocityState velocity = robot->wheelSpeedToVelocity(leftSpeed, rightSpeed);

    // 1.0 * 0.05 * (1.0+2.0) / 2.0 = 0.075
    double linearVel = wheelRadiusScale * wheelRadius * (leftSpeed +  rightSpeed) / 2.0;
    // 1.0 * 0.05 * (2.0-1.0) / 0.1 = 0.5
    double angularVel = wheelRadiusScale * wheelRadius * (rightSpeed - leftSpeed) / (wheelSeparation * wheelSeparationScale);

    EXPECT_DOUBLE_EQ(linearVel, velocity.v);
    EXPECT_DOUBLE_EQ(0.075, velocity.v);
    EXPECT_DOUBLE_EQ(angularVel, velocity.w);
    EXPECT_DOUBLE_EQ(0.5, velocity.w);
}

TEST_F(RobotTest, wheelSpeedToVelocity_onSpot_leftPositive_returnsCorrectVelocity) {
    double leftSpeed = 1.0;
    double rightSpeed = -1.0;

    VelocityState velocity = robot->wheelSpeedToVelocity(leftSpeed, rightSpeed);

    // 1.0 * 0.05 * (1.0-1.0) / 2.0 = 0.0
    double linearVel = wheelRadiusScale * wheelRadius * (leftSpeed +  rightSpeed) / 2.0;
    // 1.0 * 0.05 * (-1.0-1.0) / 0.1 = -1.0
    double angularVel = wheelRadiusScale * wheelRadius * (rightSpeed - leftSpeed) / (wheelSeparation * wheelSeparationScale);

    EXPECT_DOUBLE_EQ(linearVel, velocity.v);
    EXPECT_DOUBLE_EQ(0.0, velocity.v);
    EXPECT_DOUBLE_EQ(angularVel, velocity.w);
    EXPECT_DOUBLE_EQ(-1.0, velocity.w);
}

TEST_F(RobotTest, wheelSpeedToVelocity_onSpot_rightPositive_returnsCorrectVelocity) {
    double leftSpeed = -1.0;
    double rightSpeed = 1.0;

    VelocityState velocity = robot->wheelSpeedToVelocity(leftSpeed, rightSpeed);

    // 1.0 * 0.05 * (-1.0+1.0) / 2.0 = 0.0
    double linearVel = wheelRadiusScale * wheelRadius * (leftSpeed +  rightSpeed) / 2.0;
    // 1.0 * 0.05 * (1.0+1.0) / 0.1 = 1.0
    double angularVel = wheelRadiusScale * wheelRadius * (rightSpeed - leftSpeed) / (wheelSeparation * wheelSeparationScale);

    EXPECT_DOUBLE_EQ(linearVel, velocity.v);
    EXPECT_DOUBLE_EQ(0.0, velocity.v);
    EXPECT_DOUBLE_EQ(angularVel, velocity.w);
    EXPECT_DOUBLE_EQ(1.0, velocity.w);
}

TEST_F(RobotTest, velocityToWheelSpeed_positiveLinear_returnsCorrectWheelSpeed) {
    double linearVelocity = wheelRadius;
    double angularVelocity = 0.0;

    WheelSpeed wheelSpeed = robot->velocityToWheelSpeed(linearVelocity, angularVelocity);

    // (0.05 - 0.5 * 0.1 * 0.0) / 0.05 = 1.0
    double leftSpeed = (linearVelocity - 0.5 * (wheelSeparation * wheelSeparationScale) * angularVelocity) / wheelRadius;
    // (0.05 + 0.5 * 0.1 * 0.0) / 0.05 = 1.0
    double rightSpeed = (linearVelocity + 0.5 * (wheelSeparation * wheelSeparationScale) * angularVelocity) / wheelRadius;

    EXPECT_DOUBLE_EQ(leftSpeed, wheelSpeed.left);
    EXPECT_DOUBLE_EQ(1.0, wheelSpeed.left);
    EXPECT_DOUBLE_EQ(rightSpeed, wheelSpeed.right);
    EXPECT_DOUBLE_EQ(1.0, wheelSpeed.right);
}

TEST_F(RobotTest, velocityToWheelSpeed_positiveAngular_returnsCorrectWheelSpeed) {
    double linearVelocity = 0.0;
    double angularVelocity = 1.0;

    WheelSpeed wheelSpeed = robot->velocityToWheelSpeed(linearVelocity, angularVelocity);

    // (0.0 - 0.5 * 0.1 * 1.0) / 0.05 = -1.0
    double leftSpeed = (linearVelocity - 0.5 * (wheelSeparation * wheelSeparationScale) * angularVelocity) / wheelRadius;
    // (0.0 + 0.5 * 0.1 * 1.0) / 0.05 = 1.0
    double rightSpeed = (linearVelocity + 0.5 * (wheelSeparation * wheelSeparationScale) * angularVelocity) / wheelRadius;

    EXPECT_DOUBLE_EQ(leftSpeed, wheelSpeed.left);
    EXPECT_DOUBLE_EQ(-1.0, wheelSpeed.left);
    EXPECT_DOUBLE_EQ(rightSpeed, wheelSpeed.right);
    EXPECT_DOUBLE_EQ(1.0, wheelSpeed.right);
}

TEST_F(RobotTest, velocityToWheelSpeed_negativeAngular_returnsCorrectWheelSpeed) {
    double linearVelocity = 0.0;
    double angularVelocity = -1.0;

    WheelSpeed wheelSpeed = robot->velocityToWheelSpeed(linearVelocity, angularVelocity);

    // (0.0 - 0.5 * 0.1 * 1.0) / 0.05 = 1.0
    double leftSpeed = (linearVelocity - 0.5 * (wheelSeparation * wheelSeparationScale) * angularVelocity) / wheelRadius;
    // (0.0 + 0.5 * 0.1 * 1.0) / 0.05 = -1.0
    double rightSpeed = (linearVelocity + 0.5 * (wheelSeparation * wheelSeparationScale) * angularVelocity) / wheelRadius;

    EXPECT_DOUBLE_EQ(leftSpeed, wheelSpeed.left);
    EXPECT_DOUBLE_EQ(1.0, wheelSpeed.left);
    EXPECT_DOUBLE_EQ(rightSpeed, wheelSpeed.right);
    EXPECT_DOUBLE_EQ(-1.0, wheelSpeed.right);
}

TEST_F(RobotTest, velocityToWheelSpeed_positiveBoth_returnsCorrectWheelSpeed) {
    double linearVelocity = 0.075;
    double angularVelocity = 0.5;

    WheelSpeed wheelSpeed = robot->velocityToWheelSpeed(linearVelocity, angularVelocity);

    // (0.075 - 0.5 * 0.1 * 0.5) / 0.05 = 1.0
    double leftSpeed = (linearVelocity - 0.5 * (wheelSeparation * wheelSeparationScale) * angularVelocity) / wheelRadius;
    // (0.075 + 0.5 * 0.1 * 0.5) / 0.05 = 2.0
    double rightSpeed = (linearVelocity + 0.5 * (wheelSeparation * wheelSeparationScale) * angularVelocity) / wheelRadius;

    EXPECT_DOUBLE_EQ(leftSpeed, wheelSpeed.left);
    EXPECT_DOUBLE_EQ(1.0, wheelSpeed.left);
    EXPECT_DOUBLE_EQ(rightSpeed, wheelSpeed.right);
    EXPECT_DOUBLE_EQ(2.0, wheelSpeed.right);
}

TEST_F(RobotTest, onSpeedInterrupt_noSpeeds_noChange) {
    setupCommandGotAndSent(0.0, 0.0, 0.0, 0.0);

    robot->onSpeedInterrupt();

    checkState(robot->getState(), State{0.0, 0.0, 0.0});
    checkVelocityState(robot->getCurrentVelocity(), VelocityState{0.0, 0.0});
}

TEST_F(RobotTest, onSpeedInterrupt_positiveSpeeds_currentVelocitySet) {
    double lCurr = 2.0;
    double rCurr = 1.0;
    double lDesired = 0.0;
    double rDesired = 0.0;
    setupCommandGotAndSent(lCurr, rCurr, lDesired, rDesired);

    robot->onSpeedInterrupt();

    VelocityState expectedVelocity = robot->wheelSpeedToVelocity(lCurr, rCurr);
    checkVelocityState(robot->getCurrentVelocity(), expectedVelocity);
    checkWheelSpeeds(robot->getCurrentWheelSpeed(), WheelSpeed{lCurr, rCurr});
}

TEST_F(RobotTest, onSpeedInterrupt_negativeSpeeds_currentVelocitySet) {
    double lCurr = -2.0;
    double rCurr = -1.0;
    double lDesired = 0.0;
    double rDesired = 0.0;
    setupCommandGotAndSent(lCurr, rCurr, lDesired, rDesired);

    robot->onSpeedInterrupt();

    VelocityState expectedVelocity = robot->wheelSpeedToVelocity(lCurr, rCurr);
    checkVelocityState(robot->getCurrentVelocity(), expectedVelocity);
    checkWheelSpeeds(robot->getCurrentWheelSpeed(), WheelSpeed{lCurr, rCurr});
}

TEST_F(RobotTest, onSpeedInterrupt_positiveSpeeds_straight_fromZero_stateUpdated) {
    double lCurr = 1.0;
    double rCurr = 1.0;
    double lDesired = 0.0;
    double rDesired = 0.0;
    setupCommandGotAndSent(lCurr, rCurr, lDesired, rDesired);

    robot->onSpeedInterrupt();

    VelocityState expectedVel = robot->wheelSpeedToVelocity(lCurr, rCurr);
    double v = expectedVel.v;
    double w = expectedVel.w;
    EXPECT_DOUBLE_EQ(w, 0.0);
    double dt = speedInterruptMillis / 1000.0;
    State expectedState;
    expectedState.x += std::cos(0.0) * v * dt;
    expectedState.y += std::sin(0.0) * v * dt;
    checkState(robot->getState(), expectedState);
}

TEST_F(RobotTest, onSpeedInterrupt_positiveSpeeds_arcTurn_fromZero_stateUpdated) {
    double lCurr = 2.0;
    double rCurr = 1.0;
    double lDesired = 0.0;
    double rDesired = 0.0;
    setupCommandGotAndSent(lCurr, rCurr, lDesired, rDesired);

    robot->onSpeedInterrupt();

    VelocityState expectedVel = robot->wheelSpeedToVelocity(lCurr, rCurr);
    double v = expectedVel.v;
    double w = expectedVel.w;
    double dt = speedInterruptMillis / 1000.0;
    State expectedState;
    expectedState.x = v / w * (std::sin(0.0+dt*w) - std::sin(0.0));
    expectedState.y = -v / w * (std::cos(0.0+dt*w) - std::cos(0.0));
    expectedState.theta = dt * w;
    checkState(robot->getState(), expectedState);
}

TEST_F(RobotTest, onSpeedInterrupt_positiveLeft_onSpotTurn_fromZero_stateUpdated) {
    double lCurr = 1.0;
    double rCurr = -1.0;
    double lDesired = 0.0;
    double rDesired = 0.0;
    setupCommandGotAndSent(lCurr, rCurr, lDesired, rDesired);

    robot->onSpeedInterrupt();

    VelocityState expectedVel = robot->wheelSpeedToVelocity(lCurr, rCurr);
    double v = expectedVel.v;
    double w = expectedVel.w;
    EXPECT_DOUBLE_EQ(v, 0.0);
    double dt = speedInterruptMillis / 1000.0;
    State expectedState;
    expectedState.theta = dt * w;
    checkState(robot->getState(), expectedState);
    EXPECT_LT(robot->getState().theta, 0.0);
}

TEST_F(RobotTest, onSpeedInterrupt_positiveRight_onSpotTurn_fromZero_stateUpdated) {
    double lCurr = -1.0;
    double rCurr = 1.0;
    double lDesired = 0.0;
    double rDesired = 0.0;
    setupCommandGotAndSent(lCurr, rCurr, lDesired, rDesired);

    robot->onSpeedInterrupt();

    VelocityState expectedVel = robot->wheelSpeedToVelocity(lCurr, rCurr);
    double v = expectedVel.v;
    double w = expectedVel.w;
    EXPECT_DOUBLE_EQ(v, 0.0);
    double dt = speedInterruptMillis / 1000.0;
    State expectedState;
    expectedState.theta = dt * w;
    checkState(robot->getState(), expectedState);
    EXPECT_GT(robot->getState().theta, 0.0);
}

TEST_F(RobotTest, onSpeedInterrupt_negativeSpeeds_straight_fromZero_stateUpdated) {
    double lCurr = -1.0;
    double rCurr = -1.0;
    double lDesired = 0.0;
    double rDesired = 0.0;
    setupCommandGotAndSent(lCurr, rCurr, lDesired, rDesired);

    robot->onSpeedInterrupt();

    VelocityState expectedVel = robot->wheelSpeedToVelocity(lCurr, rCurr);
    double v = expectedVel.v;
    double w = expectedVel.w;
    EXPECT_DOUBLE_EQ(w, 0.0);
    double dt = speedInterruptMillis / 1000.0;
    State expectedState;
    expectedState.x += std::cos(0.0) * v * dt;
    expectedState.y += std::sin(0.0) * v * dt;
    checkState(robot->getState(), expectedState);
}

TEST_F(RobotTest, onSpeedInterrupt_negativeSpeeds_arcTurn_fromZero_stateUpdated) {
    double lCurr = -2.0;
    double rCurr = -1.0;
    double lDesired = 0.0;
    double rDesired = 0.0;
    setupCommandGotAndSent(lCurr, rCurr, lDesired, rDesired);

    robot->onSpeedInterrupt();

    VelocityState expectedVel = robot->wheelSpeedToVelocity(lCurr, rCurr);
    double v = expectedVel.v;
    double w = expectedVel.w;
    double dt = speedInterruptMillis / 1000.0;
    State expectedState;
    expectedState.x = v / w * (std::sin(0.0+dt*w) - std::sin(0.0));
    expectedState.y = -v / w * (std::cos(0.0+dt*w) - std::cos(0.0));
    expectedState.theta = dt * w;
    checkState(robot->getState(), expectedState);
}

TEST_F(RobotTest, onSpeedInterrupt_straight_fromNonZeroState_stateUpdated) {
    double lCurr = 1.0;
    double rCurr = 1.0;
    double lDesired = 0.0;
    double rDesired = 0.0;
    setupCommandGotAndSent(lCurr, rCurr, lDesired, rDesired);

    robot->setState(State{1.0, 1.0, M_PI/4});
    robot->onSpeedInterrupt();

    VelocityState expectedVel = robot->wheelSpeedToVelocity(lCurr, rCurr);
    double v = expectedVel.v;
    double dt = speedInterruptMillis / 1000.0;
    State expectedState;
    expectedState.x = 1.0 + std::cos(M_PI/4) * v * dt;
    expectedState.y = 1.0 + std::sin(M_PI/4) * v * dt;
    expectedState.theta = M_PI/4;
    checkState(robot->getState(), expectedState);
}

TEST_F(RobotTest, onSpeedInterrupt_arcTurn_fromNonZeroState_stateUpdated) {
    double lCurr = 2.0;
    double rCurr = 1.0;
    double lDesired = 0.0;
    double rDesired = 0.0;
    setupCommandGotAndSent(lCurr, rCurr, lDesired, rDesired);

    robot->setState(State{1.0, 1.0, M_PI/4});
    robot->onSpeedInterrupt();

    VelocityState expectedVel = robot->wheelSpeedToVelocity(lCurr, rCurr);
    double v = expectedVel.v;
    double w = expectedVel.w;
    double dt = speedInterruptMillis / 1000.0;
    State expectedState;
    expectedState.x = 1.0 + v / w * (std::sin(M_PI/4+dt*w) - std::sin(M_PI/4));
    expectedState.y = 1.0 - v / w * (std::cos(M_PI/4+dt*w) - std::cos(M_PI/4));
    expectedState.theta = M_PI / 4 + dt * w;
    checkState(robot->getState(), expectedState);
}

TEST_F(RobotTest, onSpeedInterrupt_onSpotTurn_fromNonZeroState_stateUpdated) {
    double lCurr = 1.0;
    double rCurr = -1.0;
    double lDesired = 0.0;
    double rDesired = 0.0;
    setupCommandGotAndSent(lCurr, rCurr, lDesired, rDesired);

    robot->setState(State{1.0, 1.0, M_PI/4});
    robot->onSpeedInterrupt();

    VelocityState expectedVel = robot->wheelSpeedToVelocity(lCurr, rCurr);
    double w = expectedVel.w;
    double dt = speedInterruptMillis / 1000.0;
    State expectedState {1.0, 1.0, M_PI/4};
    expectedState.theta = M_PI/4 + dt * w;
    checkState(robot->getState(), expectedState);
}

TEST_F(RobotTest, onSpeedInterrupt_fromZero_arcTurn_twice_stateUpdated) {
    double lCurr = 2.0;
    double rCurr = 1.0;
    double lDesired = 0.0;
    double rDesired = 0.0;
    setupCommandGotAndSent(lCurr, rCurr, lDesired, rDesired);

    robot->onSpeedInterrupt();
    robot->onSpeedInterrupt();

    VelocityState expectedVel = robot->wheelSpeedToVelocity(lCurr, rCurr);
    double v = expectedVel.v;
    double w = expectedVel.w;
    checkVelocityState(robot->getCurrentVelocity(), expectedVel);
    double dt = speedInterruptMillis / 1000.0;
    State expectedState;
    expectedState.x = v / w * (std::sin(dt*w) - std::sin(0.0));
    expectedState.y = -v / w * (std::cos(dt*w) - std::cos(0.0));
    expectedState.theta = dt * w;
    expectedState.x += v / w * (std::sin(expectedState.theta+dt*w) - std::sin(expectedState.theta));
    expectedState.y += -v / w * (std::cos(expectedState.theta+dt*w) - std::cos(expectedState.theta));
    expectedState.theta += dt * w;
    checkState(robot->getState(), expectedState);
}

TEST_F(RobotTest, onSpeedInterrupt_fromNonZero_arcTurn_twice_stateUpdated) {
    double lCurr = 2.0;
    double rCurr = 3.0;
    double lDesired = 0.0;
    double rDesired = 0.0;
    setupCommandGotAndSent(lCurr, rCurr, lDesired, rDesired);

    robot->setState(State{1.0, 1.0, M_PI/4});
    robot->onSpeedInterrupt();
    robot->onSpeedInterrupt();

    VelocityState expectedVel = robot->wheelSpeedToVelocity(lCurr, rCurr);
    double v = expectedVel.v;
    double w = expectedVel.w;
    checkVelocityState(robot->getCurrentVelocity(), expectedVel);
    double dt = speedInterruptMillis / 1000.0;
    State expectedState {1.0,1.0,M_PI/4};
    expectedState.x += v / w * (std::sin(M_PI/4+dt*w) - std::sin(M_PI/4));
    expectedState.y += -v / w * (std::cos(M_PI/4+dt*w) - std::cos(M_PI/4));
    expectedState.theta += dt * w;
    expectedState.x += v / w * (std::sin(expectedState.theta+dt*w) - std::sin(expectedState.theta));
    expectedState.y += -v / w * (std::cos(expectedState.theta+dt*w) - std::cos(expectedState.theta));
    expectedState.theta += dt * w;
    checkState(robot->getState(), expectedState);
}
