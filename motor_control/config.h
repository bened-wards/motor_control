#pragma once

#include <string>

struct Config {
    Config(const std::string& configFileName) {
        loadConfig(configFileName);
    }

    bool debugMode;

    int ticksPerRev;
    int speedInterruptMillis;

    double wheelRadiusScale;
    double wheelRadius;
    double wheelSeparationScale;
    double wheelSeparation;

    double Kp;
    double Ki;

    int enableLeft;
    int in1Left;
    int in2Left;
    int enableRight;
    int in1Right;
    int in2Right;
    int encALeft;
    int encBLeft;
    int encARight;
    int encBRight;

private:
    void loadConfig(const std::string& configFileName);
};
