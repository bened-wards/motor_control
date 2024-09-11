#include <iostream>
#include <csignal>
#include <cstdlib>
#include <atomic>
#include <thread>
#include <vector>
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

namespace {
    std::atomic<bool> stopFlag(false);

    // v+x.xxxw-x.xxx\n -> 15 bytes
    // + or - after v/w indicates sign
    const int MSG_LENGTH = 15;
    char buffer[MSG_LENGTH+1];
    int totalBytes = 0;
}

void printVelocity(double v, double w) {
    std::cout << "Desired velocity: linear=" << v << ", angular=" << w << std::endl;
}

void sendDesiredVelocity(int serial_port, double v, double w) {
    char msg[MSG_LENGTH+1]; 
    std::sprintf(msg, "v%c%.3fw%c%.3f\n", 
        (v > 0 ? '+' : '-'), std::abs(v), 
        (w > 0 ? '+' : '-'), std::abs(w));
    // std::cout << "Sending: " << msg;
    int noBytes = write(serial_port, msg, sizeof(msg));
    std::cout << "Sent current velocity " << msg << " back to navigation with " << noBytes << " written\n";
}

VelocityState parseMsg(const char* msg) {
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

void catchKeyboardInterrupt(int signum) {
    std::cout << "Got keyboard interrupt, exiting\n";
    stopFlag = true;
}

void readFromSerial(int serial_port) {
    while (!stopFlag.load()) {
        int noBytes = read(serial_port, buffer + totalBytes, MSG_LENGTH+1);
        if (noBytes != -1) {
            totalBytes += noBytes;
            std::cout << "Read " << noBytes << " bytes\n";
        }
        else {
            std::cerr << "Error reading from serial port\n";
            break;
        }

        if (totalBytes == MSG_LENGTH) {
            buffer[MSG_LENGTH] = '\0'; // Null-terminate the string
            std::cout << "Got new message " << buffer;
            VelocityState desiredVel = parseMsg(buffer);
            printVelocity(desiredVel.v, desiredVel.w);
            totalBytes = 0;
        }
    }
}

void writeToSerial(int serial_port, std::chrono::seconds send_period) {
    while (!stopFlag.load()) {
        sendDesiredVelocity(serial_port, 1.555, 2.342);
        std::this_thread::sleep_for(std::chrono::milliseconds(send_period));
    }
}


int main(int argc, char** argv)
{
    int serial_port = open("/dev/ttyS0", O_RDWR);
    if (serial_port < 0) {
        std::cerr << "Error " << errno << " from open: " << strerror(errno) << std::endl;
        return 1;
    }

    struct termios tty;
    // read existing settings and handle errors from them
    if (tcgetattr(serial_port, &tty) != 0) {
        std::cerr << "Error " << errno << " from tcgetattr: " << strerror(errno) << std::endl;
        return 1;
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
    tty.c_cc[VTIME] = 10; // max wait 1s
    tty.c_cc[VMIN] = 0; // return when MSG_LENGTH bytes have been received

    // set baud rate
    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    // Save tty settings
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        std::cerr << "Error " << errno << " from tcsetattr: " << strerror(errno) << std::endl;
        return 1;
    }

    // register keyboard interrupt handler
    signal(SIGINT, catchKeyboardInterrupt);

    // create worker thread to read and write to serial port
    std::thread readThread(readFromSerial, serial_port);
    std::thread writeThread(writeToSerial, serial_port, std::chrono::seconds(1));

    readThread.join();
    writeThread.join();

    close(serial_port);
    return 0;
}
