#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "encoder.h"
#include "mock_classes.h"

using namespace ::testing;

class EncoderTest : public ::testing::Test {
protected:
    void SetUp() override {
        encoder = new Encoder(encALine, encBLine, ticksPerRev, deltaTMillis);
    }

    void TearDown() override {
        delete encoder;
    }

    // 1 for forward, -1 for backward
    void PerformTicks(int noTicks, int direction) {
        EXPECT_CALL(encBLine, get_value()).WillRepeatedly(Return(direction == 1 ? 1 : 0));
        for (int i = 0; i < noTicks; i++) {
            encoder->tick();
        }
    }

    int ticksPerRev = 12 * 75;
    int deltaTMillis = 20;
    double radiansPerTick = 2.0 * M_PI / static_cast<double>(ticksPerRev);
    MockGPIOLine encALine;
    MockGPIOLine encBLine;
    Encoder* encoder;
};

TEST_F(EncoderTest, getSpeed_noTicks_returnsZero) {
    EXPECT_DOUBLE_EQ(0.0, encoder->getSpeed());
}   

TEST_F(EncoderTest, getSpeed_oneTickForward_returnsCorrectSpeed) {
    PerformTicks(1, 1);

    EXPECT_DOUBLE_EQ(1.0 * radiansPerTick * 1000.0 / deltaTMillis, encoder->getSpeed());
}

TEST_F(EncoderTest, getSpeed_oneTickBackward_returnsCorrectSpeed) {
    PerformTicks(1, -1);

    EXPECT_DOUBLE_EQ(-1.0 * radiansPerTick * 1000.0 / deltaTMillis, encoder->getSpeed());
}

TEST_F(EncoderTest, getSpeed_multipleTicksForward_returnsCorrectSpeed) {
    PerformTicks(10, 1);

    EXPECT_DOUBLE_EQ(10.0 * radiansPerTick * 1000.0 / deltaTMillis, encoder->getSpeed());
}

TEST_F(EncoderTest, getSpeed_multipleTicksBackward_returnsCorrectSpeed) {
    PerformTicks(10, -1);

    EXPECT_DOUBLE_EQ(-10.0 * radiansPerTick * 1000.0 / deltaTMillis, encoder->getSpeed());
}
