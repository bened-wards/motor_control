#pragma once

#include "encoder.h"
#include "serial.h"
#include "robot.h"

namespace Loops {
    inline std::atomic<bool> stopFlag {false};

    // infinitely loop waiting for rising edge on encoder A pin
    // on rising edge ticks the encoder
    void encoder_event_handler(Encoder& encoder);

    void readAndSetVelocityAndState(Serial& serial, Robot& robot);
}

