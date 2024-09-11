cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
project(motor_control LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS "-march=native -O3 -Wall -Wfatal-errors")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(CMakePrintHelpers)
cmake_print_variables(CMAKE_SYSTEM_PROCESSOR)

include(FetchContent)
FetchContent_Declare(
  yaml-cpp
  GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
  GIT_TAG yaml-cpp-0.7.0
)
FetchContent_MakeAvailable(yaml-cpp)

if(NOT CMAKE_SYSTEM_PROCESSOR STREQUAL "armv6l") 
  include(FetchContent)
  FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
  )
  # For Windows: Prevent overriding the parent project's compiler/linker settings
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)
endif()

set(CONFIG_FILE "${CMAKE_SOURCE_DIR}/config.yaml")
configure_file(${CONFIG_FILE} ${CMAKE_BINARY_DIR}/config.yaml COPYONLY)

add_subdirectory(motor_control)
add_subdirectory(apps)
add_subdirectory(tests)


