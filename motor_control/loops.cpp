#include "loops.h"

#include <iostream>
#include <thread>

namespace {
    bool DEBUG = false;
    const int readPeriodMillis = 20;
}

namespace Loops {
    // infinitely loop waiting for rising edge on encoder A pin
    // on rising edge ticks the encoder
    void encoder_event_handler(Encoder& encoder) {
        std::shared_ptr<gpiod::line> encALine = encoder.getEncALine();
        std::shared_ptr<gpiod::line> encBLine = encoder.getEncBLine();
        while (!stopFlag.load()) {
            bool eventA = encALine->event_wait(std::chrono::microseconds(10));
            if (eventA) {
                bool rising = encALine->event_read().event_type == gpiod::line_event::RISING_EDGE;
                encoder.tick(true, rising);
            }
            bool eventB = encBLine->event_wait(std::chrono::microseconds(10));
            if (eventB) {
                bool rising = encBLine->event_read().event_type == gpiod::line_event::RISING_EDGE;
                encoder.tick(false, rising);
            }
        }
    }

    // infinitely loop waiting for information from main controller
    void readAndSetVelocityAndState(Serial& serial, Robot& robot) {
        while (!stopFlag.load()) {
            bool bufferReady = serial.read();
            if (bufferReady) {
                std::optional<VelocityState> desiredVelOpt = serial.parseVelocityMsg();
                if (desiredVelOpt.has_value()) {
                    VelocityState desiredVel = desiredVelOpt.value();
                    if (DEBUG) std::cout << "Setting desired velocity from read buffer: linear=" << desiredVel.v << ", angular=" << desiredVel.w << std::endl;
                    robot.setDesiredVelocity(desiredVel);
                    continue;
                }
                else {
                    if (DEBUG) std::cout << "Invalid velocity message, trying state message\n";
                }

                std::optional<State> stateOpt = serial.parseStateMsg();
                if (stateOpt.has_value()) {
                    State state = stateOpt.value();
                    if (DEBUG) std::cout << "Setting state from read buffer: x=" << state.x << ", y=" << state.y << ", theta=" << state.theta << std::endl;
                    robot.setState(state);
                    continue;
                }

                if (DEBUG) std::cerr << "Invalid message received, skipping\n";
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(readPeriodMillis));
        }
    }
}
