// Example on how to use sysfs interface for hardware pwm
#include <chrono>
#include <thread>
#include <iostream>

#include "pwm_pin.h"

void test_pwm(PWMPin& pwm)
{
    using namespace std::chrono_literals;
    
    pwm.set_period(2000000); // 2ms period
    pwm.set_duty_cycle(0.5); // 50% duty cycle

    pwm.enable();
    std::cout << "Enabling PWM for 5s" << std::endl;
    std::cout << "Period is: " << pwm.get_period() << std::endl;
    std::this_thread::sleep_for(5s);

    pwm.disable();
    std::cout << "Disabling PWM for 3s" << std::endl;
    std::this_thread::sleep_for(3s);

    pwm.set_period(10000000); // 10ms period

    pwm.enable();
    std::cout << "Enabling PWM for 5s" << std::endl;
    std::cout << "Period is: " << pwm.get_period() << std::endl;
    std::this_thread::sleep_for(5s);

    pwm.disable();
    std::cout << "PWM Test Complete" << std::endl;
}

int main(int argc, char** argv)
{
    using namespace std::chrono_literals;

    std::cout << "Maximum unsigned int: " << std::numeric_limits<unsigned int>::max() << std::endl;
    
    std::cout << "Setting up PWM0\n";
    PWMPin pwm_0 {0, 0};
    std::cout << "Setting up PWM1\n";
    PWMPin pwm_1 {0, 1};
    std::this_thread::sleep_for(2s);

    std::cout << "Testing PWM0\n";
    test_pwm(pwm_0);
    std::cout << "Testing PWM1\n";
    test_pwm(pwm_1);

    return 0;
}
