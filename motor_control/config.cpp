#include "config.h"

#include <iostream>

#include <yaml-cpp/yaml.h>

void Config::loadConfig(const std::string& configFileName) {
    YAML::Node config = YAML::LoadFile(configFileName);

    std::cout << "Reading in config file: " << configFileName << std::endl;
    debugMode = config["debug_mode"].as<bool>();
    std::cout << "Debug mode: " << debugMode << std::endl;

    ticksPerRev = config["ticks_per_rev"].as<int>();
    speedInterruptMillis = config["speed_interrupt_millis"].as<int>();
    std::cout << "Ticks per revolution: " << ticksPerRev << std::endl;
    std::cout << "Speed interrupt millis: " << speedInterruptMillis << std::endl;

    wheelRadiusScale = config["wheel_radius_scale"].as<double>();
    wheelRadius = config["wheel_radius"].as<double>();
    wheelSeparationScale = config["wheel_separation_scale"].as<double>();
    wheelSeparation = config["wheel_separation"].as<double>();
    std::cout << "Wheel radius scale: " << wheelRadiusScale << std::endl;
    std::cout << "Wheel radius: " << wheelRadius << std::endl;
    std::cout << "Wheel separation scale: " << wheelSeparationScale << std::endl;
    std::cout << "Wheel separation: " << wheelSeparation << std::endl;

    Kp = config["kp"].as<double>();
    Ki = config["ki"].as<double>();
    std::cout << "Kp: " << Kp << std::endl;
    std::cout << "Ki: " << Ki << std::endl;

    enableLeft = config["enable_left"].as<int>();
    in1Left = config["in1_left"].as<int>();
    in2Left = config["in2_left"].as<int>();
    enableRight = config["enable_right"].as<int>();
    in1Right = config["in1_right"].as<int>();
    in2Right = config["in2_right"].as<int>();
    std::cout << "ENLeft: " << enableLeft << ", IN1Left: " << in1Left << ", IN2Left: " << in2Left << std::endl;
    std::cout << "ENRight: " << enableRight << ", IN1Right: " << in1Right << ", IN2Right: " << in2Right << std::endl;

    encALeft = config["encA_left"].as<int>();
    encBLeft = config["encB_left"].as<int>();
    encARight = config["encA_right"].as<int>();
    encBRight = config["encB_right"].as<int>();
    std::cout << "EncALeft: " << encALeft << ", EncBLeft: " << encBLeft << std::endl;
    std::cout << "EncARight: " << encARight << ", EncBRight: " << encBRight << std::endl;
}
