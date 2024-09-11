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
    void encoder_event_handler(Encoder& encoder, bool debug=false) {
        std::shared_ptr<gpiod::line> encALine = encoder.getEncALine();
        std::shared_ptr<gpiod::line> encBLine = encoder.getEncBLine();
        while (!stopFlag.load()) {
            bool eventA = encALine->event_wait(std::chrono::microseconds(1));
            if (eventA) {
                bool rising = encALine->event_read().event_type == gpiod::line_event::RISING_EDGE;
                if (debug) std::cout << "Event A. Rising: " << (rising) << std::endl;
                encoder.tick(true, rising);
            }
            bool eventB = encBLine->event_wait(std::chrono::microseconds(1));
            if (eventB) {
                bool rising = encBLine->event_read().event_type == gpiod::line_event::RISING_EDGE;
                if (debug) std::cout << "Event B. Rising: " << (rising) << std::endl;
                encoder.tick(false, rising);
            }
        }
    }

    // infinitely loop waiting for information from main controller
    void readAndSetDesiredVelocity(Serial& serial, Robot& robot) {
        while (!stopFlag.load()) {
            bool bufferReady = serial.read();
            if (bufferReady) {
                VelocityState desiredVel = serial.readBuffer();
                if (std::isnan(desiredVel.v)) {
                    if (DEBUG) std::cout << "Desired velocity is NaN, skipping\n";
                    continue;
                }
                if (DEBUG) std::cout << "Setting desired velocity from read buffer: linear=" << desiredVel.v << ", angular=" << desiredVel.w << std::endl;
                robot.setDesiredVelocity(desiredVel);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(readPeriodMillis));
        }
    }
}
