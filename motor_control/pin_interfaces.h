#pragma once

#include <gpiod.hpp>

class IPWMPin {
public:
    virtual ~IPWMPin() = default;

    virtual bool set_period(unsigned int nanoseconds) = 0;
    virtual unsigned int get_period() = 0;
    virtual bool set_duty_cycle(double percentage) = 0;
    virtual unsigned int get_duty_cycle() = 0;
    virtual bool get_status() = 0;
    virtual bool enable() = 0;
    virtual bool disable() = 0;
};


// wrapper around gpiod::line to allow for mocking
class GPIOLine {
public:
    GPIOLine() {};
    GPIOLine(gpiod::line line) : m_line(std::move(line)) {}

	virtual unsigned int offset() const {
        return m_line.offset();
    }
	virtual std::string name() const {
        return m_line.name();
    }
	virtual int direction() const noexcept {
        return m_line.direction();
    }
	virtual bool is_used() const {
        return m_line.is_used();
    }
	virtual void request(const gpiod::line_request& config, int default_val = 0) const {
        m_line.request(config, default_val);
    }
	void release() const {
        m_line.release();
    }
	virtual bool is_requested() const {
        return m_line.is_requested();
    }
	virtual int get_value() const {
        return m_line.get_value();
    }
	virtual bool set_value(int val) const {
        m_line.set_value(val);
        return m_line.get_value() == val;
    }
	virtual bool event_wait(const ::std::chrono::nanoseconds& timeout) const {
        return m_line.event_wait(timeout);
    }
	virtual gpiod::line_event event_read() const {
        return m_line.event_read();
    }
	virtual int event_get_fd() const {
        return m_line.event_get_fd();
    }
	virtual const gpiod::chip& get_chip() const {
        return m_line.get_chip();
    }
	virtual void reset() {
        m_line.reset();
    }

private:
    gpiod::line m_line;
};
