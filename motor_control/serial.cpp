#include "serial.h"

#include <limits>

namespace {
    bool DEBUG = true;
};

void Serial::open() {
    m_serialPort = ::open(m_deviceStr.c_str(), O_RDWR);
    if (m_serialPort < 0) {
        std::cerr << "Error " << errno << " from open: " << strerror(errno) << std::endl;
    }
    else if (DEBUG) std::cout << "Opened serial port\n";
}

void Serial::setTermiosSettings() {
    struct termios tty;
    // read existing settings and handle errors from them
    if (tcgetattr(m_serialPort, &tty) != 0) {
        std::cerr << "Error " << errno << " from tcgetattr: " << strerror(errno) << std::endl;
        return;
    }

    // control modes
    tty.c_cflag &= ~PARENB; // clear parity bit
    tty.c_cflag &= ~CSTOPB; // clear stop field, one stop bit 
    tty.c_cflag &= ~CSIZE; // clear all size bit so it defaults to 8 bits
    tty.c_cflag |= CS8; // 8 bits per byte
    tty.c_cflag &= ~CRTSCTS; // disable hardware flow control
    tty.c_cflag |= CREAD | CLOCAL; // turn on read and ignore ctrl lines
    // local modes
    tty.c_lflag &= ~ICANON; // disable canonical mode
    tty.c_lflag &= ~ECHO; // disable echo
    tty.c_lflag &= ~ECHOE; // disable erasure
    tty.c_lflag &= ~ECHONL; // disable new-line echo
    tty.c_lflag &= ~ISIG; // disable signal handling
    // input modes
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // turn off software flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // disable any handling of received bytes
    // output modes
    tty.c_oflag &= ~OPOST; // prevent special interpretation of output bytes
    tty.c_oflag &= ~ONLCR; // prevent conversion of newline to carriage return/line feed
    // control characters
    // will return when any data received or after 1s
    tty.c_cc[VTIME] = m_waitTimeSeconds * 10; // max wait in deciseconds
    tty.c_cc[VMIN] = 0; // return when MSG_LENGTH bytes have been received

    // set baud rate
    cfsetospeed(&tty, m_baudRate);
    cfsetispeed(&tty, m_baudRate);

    // Save tty settings
    if (tcsetattr(m_serialPort, TCSANOW, &tty) != 0) {
        std::cerr << "Error " << errno << " from tcsetattr: " << strerror(errno) << std::endl;
        return;
    }
}

void Serial::close() {
    if (m_serialPort != -1) {
        ::close(m_serialPort);
        if (DEBUG) std::cout << "Closed serial port\n";
    }
}

bool Serial::flushInputStream(int noBytes) {
    std::cout << "Trying to flush " << noBytes << " bytes\n";
    static char tempBuffer[50];
    int totalBytesRead = 0;
    while (totalBytesRead < noBytes) {
        int noBytesRead = ::read(m_serialPort, tempBuffer, noBytes - totalBytesRead);
        if (noBytesRead == -1) {
            std::cerr << "Error reading from serial port: " << errno << " : " << strerror(errno) << std::endl;
            return false;
        }
        else if (noBytesRead == 0) return false;
        totalBytesRead += noBytesRead;
    }
    tempBuffer[totalBytesRead] = '\0'; // Null-terminate the string
    std::cout << "Flushed " << totalBytesRead << " bytes: " << tempBuffer << std::endl;
    return true;
}

bool Serial::read() {
    while (m_totalBytes < BUFFER_SIZE - 2) {
        int noBytes = ::read(m_serialPort, &c, 1);
        if (noBytes != -1) {
            if (c == '\n') {
                m_buffer[m_totalBytes] = '\0'; // Null-terminate the string
                std::cout << "Received message: " << m_buffer << std::endl;
                m_totalBytes = 0;
                return true;
            }
            else if (m_totalBytes < BUFFER_SIZE - 1) {
                m_buffer[m_totalBytes++] = c;
            }
            else {
                std::cerr << "Buffer full, flushing\n";
                m_totalBytes = 0;
                return false;
            }
        }
        else {
            std::cerr << "Error reading from serial port: " << errno << " : " << strerror(errno) << std::endl;
            std::cerr << "Total bytes: " << m_totalBytes << std::endl;
            return false;
        }
    }
    return false;
}

VelocityState Serial::readBuffer() {
    VelocityState desiredVel = parseVelocityMsg(m_buffer);
    if (DEBUG) std::cout << "Desired velocity: linear=" << desiredVel.v << ", angular=" << desiredVel.w << std::endl;
    // m_totalBytes = 0;
    return desiredVel;
}

VelocityState Serial::parseVelocityMsg(const char* msg) {
    if (msg[0] != 'v') {
        std::cerr << "Invalid message received: " << msg << std::endl;
        return VelocityState{std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN()};
    }

    double v = atof(msg+2);
    if (msg[1] == '-') {
        v *= -1;
    }
    double w = atof(msg+9);
    if (msg[8] == '-') {
        w *= -1;
    }
    return VelocityState{v, w};
}

void Serial::writeCurrentVelocity(double v, double w) {
    static char msg[VELOCITY_MSG_LENGTH+1]; 
    std::sprintf(msg, "v%c%.3fw%c%.3f\n", 
        (v > 0 ? '+' : '-'), std::abs(v), 
        (w > 0 ? '+' : '-'), std::abs(w));
    int noBytes = write(m_serialPort, msg, strlen(msg));
    if (DEBUG) std::cout << "Sent current velocity " << msg << " back to navigation with " << noBytes << " written\n";
}

void Serial::writeCurrentState(double x, double y, double theta) {
    static char msg[STATE_MSG_LENGTH+1]; 
    std::sprintf(msg, "x%c%.3fy%c%.3ft%c%.3f\n", 
        (x > 0 ? '+' : '-'), std::abs(x),
        (y > 0 ? '+' : '-'), std::abs(y),
        (theta > 0 ? '+' : '-'), std::abs(theta));
    [[maybe_unused]] int noBytes = write(m_serialPort, msg, strlen(msg));
    // if (DEBUG) std::cout << "Sent current state " << msg << " back to navigation with " << noBytes << " written\n";
}
