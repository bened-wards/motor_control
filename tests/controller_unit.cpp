#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "controller.h"

using namespace ::testing;

class ControllerTest : public ::testing::Test {
protected:
    void SetUp() override {
        controller = new Controller(Kp, Ki);
    }

    void TearDown() override {
        delete controller;
    }

    int Kp = 1;
    int Ki = 0;
    Controller* controller;
};

TEST_F(ControllerTest, getCommand_kp1_ki0_sameVelocity_zeroDutyCycle) {
    Command command = controller->getCommand(5.0, 5.0);

    EXPECT_DOUBLE_EQ(0.0, command.dutyCycle);
    EXPECT_EQ(Direction::FORWARD, command.direction);
}

TEST_F(ControllerTest, getCommand_kp1_ki0_increaseVelocity_positiveDutyCycle) {
    // error 0.5 -> Kp * 0.5 + Ki * 0 = 0.5
    Command command = controller->getCommand(5.5, 5.0);

    EXPECT_DOUBLE_EQ(0.5, command.dutyCycle);
    EXPECT_EQ(Direction::FORWARD, command.direction);
}

TEST_F(ControllerTest, getCommand_kp1_ki0_increaseVelocity_maxDutyCycle) {
    // error 1 -> Kp * 1 + Ki * 0 = 1
    Command command = controller->getCommand(6.0, 5.0);

    EXPECT_DOUBLE_EQ(1.0, command.dutyCycle);
    EXPECT_EQ(Direction::FORWARD, command.direction);
}

TEST_F(ControllerTest, getCommand_kp1_ki0_increaseVelocity_maxDutyCycle_clamped) {
    // error 2 -> Kp * 2 + Ki * 0 = 2 -> clamped to 1
    Command command = controller->getCommand(7.0, 5.0);

    EXPECT_DOUBLE_EQ(1.0, command.dutyCycle);
    EXPECT_EQ(Direction::FORWARD, command.direction);
}

TEST_F(ControllerTest, getCommand_kp1_ki0_decreaseVelocity_negativeDutyCycle) {
    // error -0.5 -> Kp * -0.5 + Ki * 0 = -0.5
    Command command = controller->getCommand(4.5, 5.0);

    EXPECT_DOUBLE_EQ(0.5, command.dutyCycle);
    EXPECT_EQ(Direction::REVERSE, command.direction);
}

TEST_F(ControllerTest, getCommand_kp1_ki0_decreaseVelocity_negMaxDutyCycle) {
    // error -1 -> Kp * -1 + Ki * 0 = -1
    Command command = controller->getCommand(4.0, 5.0);

    EXPECT_DOUBLE_EQ(1.0, command.dutyCycle);
    EXPECT_EQ(Direction::REVERSE, command.direction);
}

TEST_F(ControllerTest, getCommand_kp1_ki0_decreaseVelocity_negMaxDutyCycle_clamped) {
    // error -2 -> Kp * -2 + Ki * 0 = -2 -> clamped to -1
    Command command = controller->getCommand(3.0, 5.0);

    EXPECT_DOUBLE_EQ(1.0, command.dutyCycle);
    EXPECT_EQ(Direction::REVERSE, command.direction);
}

TEST_F(ControllerTest, getCommand_kp1_ki0_increaseVelocity_posDutyCycle_negVelocities) {
    // error 0.5 -> Kp * 0.5 + Ki * 0 = 0.5
    Command command = controller->getCommand(-5.0, -5.5);

    EXPECT_DOUBLE_EQ(0.5, command.dutyCycle);
    EXPECT_EQ(Direction::FORWARD, command.direction);
}

TEST_F(ControllerTest, getCommand_kp1_ki0_increaseVelocity_posMaxDutyCycle_negVelocities) {
    // error 1 -> Kp * 1 + Ki * 0 = 1
    Command command = controller->getCommand(-5.0, -6.0);

    EXPECT_DOUBLE_EQ(1.0, command.dutyCycle);
    EXPECT_EQ(Direction::FORWARD, command.direction);
}

TEST_F(ControllerTest, getCommand_kp1_ki0_decreaseVelocity_negDutyCycle_negVelocities) {
    // error -0.5 -> Kp * -0.5 + Ki * 0 = -0.5
    Command command = controller->getCommand(-5.0, -4.5);

    EXPECT_DOUBLE_EQ(0.5, command.dutyCycle);
    EXPECT_EQ(Direction::REVERSE, command.direction);
}

TEST_F(ControllerTest, getCommand_kp1_ki0_decreaseVelocity_negMaxDutyCycle_negVelocities) {
    // error -1 -> Kp * -1 + Ki * 0 = -1
    Command command = controller->getCommand(-5.0, -4.0);

    EXPECT_DOUBLE_EQ(1.0, command.dutyCycle);
    EXPECT_EQ(Direction::REVERSE, command.direction);
}

TEST_F(ControllerTest, getCommand_kp1_ki0_increaseVelocity_posMaxDutyCycle_posNegVelocities_clamped) {
    // error 11 -> Kp * 11 + Ki * 0 = 11 -> clamped to 1
    Command command = controller->getCommand(5.0, -6.0);

    EXPECT_DOUBLE_EQ(1.0, command.dutyCycle);
    EXPECT_EQ(Direction::FORWARD, command.direction);
}

TEST_F(ControllerTest, getCommand_kp1_ki0_decreaseVelocity_negMaxDutyCycle_posNegVelocities_clamped) {
    // error 11 -> Kp * 11 + Ki * 0 = 11 -> clamped to 1
    Command command = controller->getCommand(-5.0, 6.0);

    EXPECT_DOUBLE_EQ(1.0, command.dutyCycle);
    EXPECT_EQ(Direction::REVERSE, command.direction);
}

TEST_F(ControllerTest, getCommand_kp1_ki0_errorSum_noEffect) {
    // errorSum = 0.5
    Command command = controller->getCommand(5.5, 5.0);

    // errorSum = 1
    command = controller->getCommand(5.5, 5.0);

    EXPECT_DOUBLE_EQ(0.5, command.dutyCycle);
    EXPECT_EQ(Direction::FORWARD, command.direction);
}

TEST_F(ControllerTest, getCommand_kp1_ki0_errorSumNeg_noEffect) {
    // errorSum = -10
    Command command = controller->getCommand(-5.0, 5.0);

    // errorSum = -9.5
    command = controller->getCommand(5.5, 5.0);

    EXPECT_DOUBLE_EQ(0.5, command.dutyCycle);
    EXPECT_EQ(Direction::FORWARD, command.direction);
}

TEST_F(ControllerTest, getCommand_kp1_ki0_errorSumNegPos_noEffect) {
    // errorSum = -10
    Command command = controller->getCommand(-5.0, 5.0);

    // errorSum = 5.5
    command = controller->getCommand(5.5, -10.0);

    // errorSum = 5.0
    command = controller->getCommand(5.5, 5.0);

    EXPECT_DOUBLE_EQ(0.5, command.dutyCycle);
    EXPECT_EQ(Direction::FORWARD, command.direction);
}

TEST_F(ControllerTest, getCommand_kp1_ki1_sameVelocity_zeroDutyCycle) {
    controller = new Controller(1, 1);
    Command command = controller->getCommand(5.0, 5.0);

    EXPECT_DOUBLE_EQ(0.0, command.dutyCycle);
    EXPECT_EQ(Direction::FORWARD, command.direction);
}

TEST_F(ControllerTest, getCommand_kp1_ki1_increaseVelocity_positiveDutyCycle) {
    controller = new Controller(1, 1);
    // error 0.5 -> Kp * 0.5 + Ki * 0.5 = 0.5 + 0.5 = 1.0
    Command command = controller->getCommand(5.5, 5.0);
    command = controller->getCommand(5.5, 5.0);

    EXPECT_DOUBLE_EQ(1.0, command.dutyCycle);
    EXPECT_EQ(Direction::FORWARD, command.direction);
}

TEST_F(ControllerTest, getCommand_kp1_ki1_increaseVelocity_maxDutyCycle) {
    controller = new Controller(1, 1);
    // error 1 -> Kp * 1 + Ki * 1 = 1 + 1 = 2.0 -> clamped to 1.0
    Command command = controller->getCommand(6.0, 5.0);
    command = controller->getCommand(6.0, 5.0);

    EXPECT_DOUBLE_EQ(1.0, command.dutyCycle);
    EXPECT_EQ(Direction::FORWARD, command.direction);
}

TEST_F(ControllerTest, getCommand_kp1_ki1_decreaseVelocity_zeroDutyCycle) {
    controller = new Controller(1, 1);
    // error -0.5 -> Kp * 0.5 + Ki * -0.5 = 0.5 + -0.5 = 0
    Command command = controller->getCommand(4.5, 5.0);
    command = controller->getCommand(5.5, 5.0);

    EXPECT_DOUBLE_EQ(0.0, command.dutyCycle);
    EXPECT_EQ(Direction::FORWARD, command.direction);
}

TEST_F(ControllerTest, getCommand_kp1_ki1_decreaseVelocity_negativeDutyCycle) {
    controller = new Controller(1, 1);
    // error -0.5 -> Kp * -0.5 + Ki * -0.5 = -0.5 + -0.5 = -1.0 -> clamped to -1.0
    Command command = controller->getCommand(4.5, 5.0);
    command = controller->getCommand(4.5, 5.0);

    EXPECT_DOUBLE_EQ(1.0, command.dutyCycle);
    EXPECT_EQ(Direction::REVERSE, command.direction);
}

TEST_F(ControllerTest, getCommand_kp1_ki1_decreaseVelocity_negMaxDutyCycle) {
    controller = new Controller(1, 1);
    // error -1 -> Kp * -1 + Ki * -1 = -1 + -1 = -2.0 -> clamped to -1.0
    Command command = controller->getCommand(4.0, 5.0);
    command = controller->getCommand(4.0, 5.0);

    EXPECT_DOUBLE_EQ(1.0, command.dutyCycle);
    EXPECT_EQ(Direction::REVERSE, command.direction);
}

TEST_F(ControllerTest, getCommand_kp1_ki1_increaseVelocity_zeroDutyCycle_negVelocities) {
    controller = new Controller(1, 1);
    // error 0.5 -> Kp * 0.5 + Ki * 0.5 = -0.5 + 0.5 = 0.0
    Command command = controller->getCommand(-5.0, -5.5);
    command = controller->getCommand(-5.5, -5.0);

    EXPECT_DOUBLE_EQ(0.0, command.dutyCycle);
    EXPECT_EQ(Direction::FORWARD, command.direction);
}

TEST_F(ControllerTest, getCommand_kp1_ki1_increaseVelocity_negDutyCycle_negVelocities) {
    controller = new Controller(1, 1);
    // error 0.5 -> Kp * 0.5 + Ki * 0.5 = -1.0 + 0.5 = -0.5
    Command command = controller->getCommand(-5.0, -5.5);
    command = controller->getCommand(-6.0, -5.0);

    EXPECT_DOUBLE_EQ(0.5, command.dutyCycle);
    EXPECT_EQ(Direction::REVERSE, command.direction);
}

TEST_F(ControllerTest, getCommand_kp1_ki1_errorSum_effect) {
    controller = new Controller(1, 1);
    // errorSum = 0.5
    Command command = controller->getCommand(5.5, 5.0);

    // errorSum = 1.0 -> Kp * 0.5 + Ki * 0.5 = 0.5 + 0.5 = 1.0 -> clamped to 1.0
    command = controller->getCommand(5.5, 5.0);

    EXPECT_DOUBLE_EQ(1.0, command.dutyCycle);
    EXPECT_EQ(Direction::FORWARD, command.direction);
}

TEST_F(ControllerTest, getCommand_kp1_ki1_errorSumNeg_noEffect) {
    controller = new Controller(1, 1);
    // errorSum = -10
    Command command = controller->getCommand(-5.0, 5.0);

    // errorSum = -9.5 -> Kp * 0.5 + Ki * -9.5 = 0.5 - 9.5 = -9.0 -> clamped to -1.0
    command = controller->getCommand(5.5, 5.0);

    EXPECT_DOUBLE_EQ(1.0, command.dutyCycle);
    EXPECT_EQ(Direction::REVERSE, command.direction);
}

TEST_F(ControllerTest, getCommand_kp1_ki1_errorSumNegPos_noEffect) {
    controller = new Controller(1, 1);
    // errorSum = -10
    Command command = controller->getCommand(-5.0, 5.0);

    // errorSum = 5.5 -> Kp * 10.5 + Ki * 5.5 = 10.5 + 5.5 = 16.0 -> clamped to 1.0
    command = controller->getCommand(5.5, -10.0);

    // errorSum = 5.0 -> Kp * 0.5 + Ki * 5.0 = 0.5 + 5.0 = 5.5 -> clamped to 1.0
    command = controller->getCommand(5.5, 5.0);

    EXPECT_DOUBLE_EQ(1.0, command.dutyCycle);
    EXPECT_EQ(Direction::FORWARD, command.direction);
}
