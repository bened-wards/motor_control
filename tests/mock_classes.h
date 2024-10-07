#pragma once

#include <gmock/gmock.h>
#include "pwm_pin.h"
#include "pin_interfaces.h"
#include "wheel.h"
#include "controller.h"

class MockWheel : public IWheel {
public:
	MOCK_METHOD(bool, sendCommand, (Command), (const, override));
	MOCK_METHOD(bool, sendCommand, (double, Direction), (const, override));
	MOCK_METHOD(double, getSpeed, (), (const, override));
	MOCK_METHOD(double, getRadius, (), (const, override));
};

class MockController : public IController {
public:
	MOCK_METHOD(Command, getCommand, (double, double), (override));
};

class MockGPIOLine : public GPIOLine {
public:
    MOCK_METHOD(unsigned int, offset, (), (const));
	MOCK_METHOD(std::string, name, (), (const));
	MOCK_METHOD(int, direction, (), (const, noexcept));
	MOCK_METHOD(bool, is_used, (), (const));
	MOCK_METHOD(void, request, (const gpiod::line_request&, int), (const));
	MOCK_METHOD(void, release, (),  (const));
	MOCK_METHOD(bool, is_requested, (),  (const));
	MOCK_METHOD(int, get_value, (),  (const));
	MOCK_METHOD(bool, set_value, (int), (const));
	MOCK_METHOD(bool, event_wait, (const std::chrono::nanoseconds&), (const));
	MOCK_METHOD(gpiod::line_event, event_read, (), (const));
	MOCK_METHOD(int, event_get_fd, (),  (const));
	MOCK_METHOD(const gpiod::chip&, get_chip, (), (const));
	MOCK_METHOD(void, reset, (), ());
};

class MockPWMPin : public IPWMPin {
public:
    MOCK_METHOD(bool, set_period, (unsigned int nanoseconds), (override));
    MOCK_METHOD(unsigned int, get_period, (), (override));
    MOCK_METHOD(bool, set_duty_cycle, (double percentage), (override));
    MOCK_METHOD(unsigned int, get_duty_cycle, (), (override));
    MOCK_METHOD(bool, get_status, (), (override));
    MOCK_METHOD(bool, enable, (), (override));
    MOCK_METHOD(bool, disable, (), (override));
};
