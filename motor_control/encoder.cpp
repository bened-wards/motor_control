#include "encoder.h"

#include <iostream>

double Encoder::getSpeed(double dtMillis) {
    m_prevCount = m_newCount.load();
    m_newCount = m_count.load();
    // number of ticks since prev deltaT
    double diff = m_newCount - m_prevCount;

    double radiansPerSecond = diff * m_radiansPerTick * 1000.0 / dtMillis;
    return radiansPerSecond;
}

// called on edges of both encoder pins
void Encoder::tick() {
    // returns values of encA (values[0]) and encB (values[1]) pins
    std::vector<int> values {std::move(m_lines.get_values())};
    if (values[1] == 1) {
        values[0] ? m_count++ : m_count--;
    }
    else {
        values[0] ? m_count-- : m_count++;
    }
}

void Encoder::tick(bool encAEvent, bool risingEdge) {
    if (encAEvent) {
        if (m_encBLine->get_value() == 1) {
            risingEdge ? m_count++ : m_count--;
        }
        else {
            risingEdge ? m_count-- : m_count++;
        }
    }
    else {
        if (m_encALine->get_value() == 1) {
            risingEdge ? m_count-- : m_count++;
        }
        else {
            risingEdge ? m_count++ : m_count--;
        }
    }
}

