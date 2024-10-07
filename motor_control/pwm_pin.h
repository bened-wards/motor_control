#pragma once

#include <iostream>
#include <fstream>
#include <string>
#include <cassert>
#include <limits>

#include "pin_interfaces.h"

#if defined(__GNUC__) && __GNUC__ > 7
#include <filesystem>
namespace filesystem = std::filesystem;
#else
#include <experimental/filesystem>
namespace filesystem = std::experimental::filesystem;
#endif

class PWMPin : public IPWMPin {
public:

    // Constructor that takes the chip index and pwm channel index
    PWMPin(unsigned int chip_idx, unsigned int channel_idx) :
        m_fs_path{"/sys/class/pwm/pwmchip" + std::to_string(chip_idx)},
        m_channel_index{channel_idx},
        m_pwm_pth{m_fs_path + "/pwm" + std::to_string(channel_idx)}
    {
        if (filesystem::exists(m_pwm_pth)) {
            throw std::runtime_error(m_pwm_pth + " already enabled, someone else may already be controlling it");
        }
        std::string export_pth = m_fs_path + "/export";
        std::ofstream write_file(export_pth);
        write_file << m_channel_index;
        write_file.close();
        if (write_file.bad() || !filesystem::exists(m_pwm_pth)) {
            throw std::runtime_error("Error exporting " + m_pwm_pth);
        }
        this->disable(); // Start with PWM disabled
        m_duty_ns = this->get_duty_cycle();
        m_period_ns = this->get_period();
        m_duty_percent = static_cast<double>(m_duty_ns) / static_cast<double>(m_period_ns);
        std::cout << "PWMPin constructed\n";
    }

    ~PWMPin()
    {
        if (!filesystem::exists(m_pwm_pth)) {
            std::cerr << m_pwm_pth  << " isn't enabled during pwmchip destruction";
        }
        this->disable(); // Disable PWM output before unexport
        std::string unexport_pth = m_fs_path + "/unexport";
        std::ofstream write_file(unexport_pth);
        write_file << m_channel_index;
        write_file.close();
        if (write_file.bad() || filesystem::exists(m_pwm_pth)) {
            std::cerr << "Error unexporting " << m_pwm_pth;
        }
        std::cout << "PWMPin destructed\n";
    }

    // There should only be one instance, don't want to be exporting/unexporting by accident.
    PWMPin(const PWMPin& other) = delete; // No copy constructor
    PWMPin& operator=(const PWMPin& other) = delete; // No copy assignment

    // PWMPin(PWMPin&& other) noexcept
    //     : m_fs_path(std::move(other.m_fs_path)), 
    //       m_channel_index(other.m_channel_index), 
    //       m_pwm_pth(std::move(other.m_pwm_pth)),
    //       m_period_ns(other.m_period_ns),
    //       m_duty_ns(other.m_duty_ns),
    //       m_duty_percent(other.m_duty_percent) 
    // {
    //     std::cout << "PWMPin moved\n";
    // }
    PWMPin(PWMPin&& other) = delete;
    PWMPin& operator=(PWMPin&& other) = delete;

    // Set the period of the duty cycle in nanoseconds
    bool set_period(unsigned int nanoseconds) override 
    {
        m_period_ns = nanoseconds;
        m_duty_ns = static_cast<unsigned int>(m_duty_percent * static_cast<double>(m_period_ns));

        // period must be larger than current duty cycle on time
        if (this->get_duty_cycle() > m_period_ns) {
            bool result1 = this->write_duty_cycle();
            bool result2 = this->write_period();
            return result1 && result2;
        } else {
            bool result1 = this->write_period();
            bool result2 = this->write_duty_cycle();
            return result1 && result2;
        }
    }

    // Read period currently stored in sysfs
    [[nodiscard]] unsigned int get_period() override 
    {
        const std::string ctrl_pth = m_pwm_pth + "/period";
        std::ifstream read_file(ctrl_pth);
        unsigned int f_period;
        read_file >> f_period;
        read_file.close();
        return f_period;
    }

    // Duty cycle is the requred percentage as a fraction between 0 and 1
    bool set_duty_cycle(double percentage) override 
    {
        assert((percentage >= 0.f && percentage <= 1.f) && "duty cycle should be in range [0, 1]");
        m_duty_percent = percentage;
        m_duty_ns = static_cast<unsigned int>(m_duty_percent * static_cast<double>(m_period_ns));
        return this->write_duty_cycle();
    }

    // Read duty cycle currently stored in sysfs
    [[nodiscard]] unsigned int get_duty_cycle() override 
    {
        const std::string ctrl_pth = m_pwm_pth + "/duty_cycle";
        std::ifstream read_file(ctrl_pth);
        unsigned int f_duty;
        read_file >> f_duty;
        read_file.close();
        return f_duty;
    }

    // Read enable status currently stored in sysfs
    [[nodiscard]] bool get_status() override 
    {
        const std::string ctrl_pth = m_pwm_pth + "/enable";
        std::ifstream read_file(ctrl_pth);
        bool status;
        read_file >> status;
        read_file.close();
        return status;
    }

    // Enable the PWM Signal
    bool enable() override 
    {
        const std::string ctrl_pth = m_pwm_pth + "/enable";
        std::ofstream write_file(ctrl_pth);
        write_file << 1;
        write_file.close();
        if (!this->get_status()) {
            std::cerr << "Error Enabling PWM " << ctrl_pth << "\n";
            return false;
        }
        return true;
    }

    // Disable the PWM Signal
    bool disable() override 
    {
        const std::string ctrl_pth = m_pwm_pth + "/enable";
        std::ofstream write_file(ctrl_pth);
        write_file << 0;
        write_file.close();
        if (this->get_status()) {
            std::cerr << "Error Disabling PWM " << ctrl_pth << "\n";
            return false;
        }
        return true;
    }

private:
    const std::string m_fs_path;
    const unsigned int m_channel_index;
    const std::string m_pwm_pth;
    unsigned int m_period_ns;
    unsigned int m_duty_ns;
    double m_duty_percent;

    // Write the target duty cycle to the file
    bool write_duty_cycle() 
    {
        const std::string ctrl_pth = m_pwm_pth + "/duty_cycle";
        std::ofstream write_file(ctrl_pth);
        write_file << m_duty_ns;
        write_file.close();
        if (this->get_duty_cycle() != m_duty_ns) {
            std::cerr << "Error setting duty cycle " << m_duty_ns << ": " << ctrl_pth << "\n";
            return false;
        }
        return true;
    }

    // Write the target period to the file
    bool write_period() 
    {
        const std::string ctrl_pth = m_pwm_pth + "/period";
        std::ofstream write_file(ctrl_pth, std::ios::binary);
        write_file << m_period_ns;
        write_file.close();
        auto new_period = this->get_period();
        if (new_period != m_period_ns) {
            std::cerr << "Error setting period to " << m_period_ns << ", remained " << new_period << ": "<< ctrl_pth << "\n";
            return false;
        }
        return true;
    }

};
