#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "motor.h"
#include "mock_classes.h"

using namespace ::testing;

class MotorTest : public ::testing::Test {
protected:
    void SetUp() override {
        motor = new Motor(pwmPin, in1Pin, in2Pin);
    }

    void TearDown() override {
        EXPECT_CALL(pwmPin, set_duty_cycle(0.0)).WillOnce(Return(true));
        // Clean up resources
        delete motor;
    }

    MockGPIOLine in1Pin;
    MockGPIOLine in2Pin;
    MockPWMPin pwmPin;
    Motor* motor;
};

TEST_F(MotorTest, stop) {
    EXPECT_CALL(pwmPin, set_duty_cycle(0.0)).WillOnce(Return(true));

    EXPECT_TRUE(motor->stop());
}

TEST_F(MotorTest, sendCommand_validDutyCycle_forward_setsCorrectly) {
    double dutyCycle = 0.5;
    Direction dir = Direction::FORWARD;

    EXPECT_CALL(pwmPin, set_duty_cycle(dutyCycle)).WillOnce(Return(true));
    EXPECT_CALL(in1Pin, set_value(1)).WillOnce(Return(true));
    EXPECT_CALL(in2Pin, set_value(0)).WillOnce(Return(true));

    EXPECT_TRUE(motor->sendCommand(dutyCycle, dir));
}

TEST_F(MotorTest, sendCommand_validDutyCycle_reverse_setsCorrectly) {
    double dutyCycle = 0.5;
    Direction dir = Direction::REVERSE;

    EXPECT_CALL(pwmPin, set_duty_cycle(dutyCycle)).WillOnce(Return(true));
    EXPECT_CALL(in1Pin, set_value(0)).WillOnce(Return(true));
    EXPECT_CALL(in2Pin, set_value(1)).WillOnce(Return(true));

    EXPECT_TRUE(motor->sendCommand(dutyCycle, dir));
}

TEST_F(MotorTest, sendCommand_invalidDutyCycle_reverse_fails) {
    double dutyCycle = 1.5;
    Direction dir = Direction::REVERSE;

    EXPECT_FALSE(motor->sendCommand(dutyCycle, dir));
}

TEST_F(MotorTest, sendCommand_negativeDutyCycle_reverse_fails) {
    double dutyCycle = -0.1;
    Direction dir = Direction::REVERSE;

    EXPECT_FALSE(motor->sendCommand(dutyCycle, dir));
}
