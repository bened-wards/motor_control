#pragma once

#include <iostream>
#include <optional>
#include <csignal>
#include <cstdlib>
#include <string.h>

// Linux headers
// file controls
#include <fcntl.h>
#include <errno.h> 
// POSIX terminal control defs
#include <termios.h>
// write(), read(), close()
#include <unistd.h> 

#include "types.h"

class Serial {
public:
    // automatically open port on creation and destroy on destruction
    Serial() : m_serialPort(-1), m_deviceStr("/dev/ttyS0"), m_baudRate(B115200), m_waitTimeSeconds(1) {
        open();
        setTermiosSettings();
    }
    ~Serial() {
        close();
    }

    // read in non-blocking manner. Returns true if buffer is full and ready to be read
    bool read();
    // parse message received from main controller
    std::optional<VelocityState> parseVelocityMsg();
    std::optional<State> parseStateMsg();

    // send current velocity to main controller
    void writeCurrentVelocity(double v, double w);
    void writeCurrentState(double x, double y, double theta);

private:
    void open();
    void setTermiosSettings();
    void close();
    bool flushInputStream(int noBytes);

    

    int m_serialPort;
    std::string m_deviceStr;
    int m_baudRate;
    int m_waitTimeSeconds;

    // v+x.xxxw-x.xxx\n -> 15 bytes
    // + or - after v/w indicates sign
    static const int VELOCITY_MSG_LENGTH = 15;
    static const int BUFFER_SIZE = 256;
    char c;
    char m_buffer[BUFFER_SIZE]; // make buffer large enough
    int m_totalBytes = 0;

    // x+*.***y+*.***t-*.***\n -> 22 bytes
    static const int STATE_MSG_LENGTH = 22;
};
