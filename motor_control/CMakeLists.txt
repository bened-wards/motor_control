add_library(motor_control STATIC
    controller.cpp
    encoder.cpp
    motor.cpp
    robot.cpp
    wheel.cpp
    loops.cpp
    serial.cpp
    config.cpp)

target_include_directories(motor_control PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# Link against the gpiod and pwm based sysfs library
target_link_libraries(motor_control PUBLIC stdc++fs gpiodcxx)
target_link_libraries(motor_control PUBLIC yaml-cpp::yaml-cpp)
