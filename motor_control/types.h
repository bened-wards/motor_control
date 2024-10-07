#pragma once

enum Direction {
    FORWARD,
    REVERSE
};

struct Command {
    Command() : dutyCycle(0), direction(Direction::FORWARD) {}
    Command(double dutyCycle, Direction direction) : 
        dutyCycle(dutyCycle), direction(direction) 
    {}

    double dutyCycle;
    Direction direction;

    bool operator==(const Command& other) const {
        return dutyCycle == other.dutyCycle && direction == other.direction;
    }
};

struct State {
    State() : x(0), y(0), theta(0) {}
    State(double x, double y, double theta) : x(x), y(y), theta(theta) {}

    double x, y, theta;
};

// track the velocity of differential drive robot
// v - linear velocity. measured in m/s
// w - angular velocity. measured in rad/s
struct VelocityState {
    VelocityState() : v(0), w(0) {}
    VelocityState(double v, double w) : v(v), w(w) {}

    double v, w;
};

// track the wheel speed of differential drive robot
// left - left wheel speed. measured in rad/s
// right - right wheel speed. measured in rad/s
struct WheelSpeed {
    WheelSpeed() : left(0), right(0) {}
    WheelSpeed(double left, double right) : left(left), right(right) {}

    double left, right; // rad/s
};
