#pragma once

#include <atomic>
#include <math.h>
#include <gpiod.hpp>

#include "pin_interfaces.h"

class Encoder {
public:
    Encoder() = delete;
    Encoder(gpiod::line_bulk& encLines, int ticksPerRev, int deltaTMillis) : 
        m_lines(encLines), 
        m_ticksPerRev(static_cast<double>(ticksPerRev)), m_deltaTMillis(static_cast<double>(deltaTMillis)),
        m_radiansPerTick(2.0 * M_PI / static_cast<double>(ticksPerRev))
    { 
        m_encALine = std::make_shared<gpiod::line>(m_lines.get(0));
        m_encBLine = std::make_shared<gpiod::line>(m_lines.get(1));
        m_count = 0;
        m_prevCount = 0;
        m_newCount = 0;
    }

    // interrupt methods
    // getSpeed called at 50Hz by controller via Robot
    double getSpeed(double dtMillis);
    // tick called every time a tick occurs in encoder
    void tick();
    void tick(bool encAEvent, bool risingEdge);
    
    gpiod::line_bulk& getLines() { return m_lines; }
    std::shared_ptr<gpiod::line> getEncALine() { return m_encALine; }
    std::shared_ptr<gpiod::line> getEncBLine() { return m_encBLine; }

private:
    gpiod::line_bulk& m_lines;
    std::shared_ptr<gpiod::line> m_encALine;
    std::shared_ptr<gpiod::line> m_encBLine;
    // store as doubles to use for speed calculations fast
    const double m_ticksPerRev; // specific to each encoder
    // time period for each interrupt 
    // -> this means that getSpeed needs to be called every m_deltaTMillis
    //    otherwise the speed of the motor will be inaccurate
    const double m_deltaTMillis; 
    const double m_radiansPerTick;
    std::atomic<long> m_count;
    std::atomic<long> m_prevCount;
    std::atomic<long> m_newCount;
};
