# Compiling the motor Test Program
add_executable(main main.cpp)
add_executable(setup setup.cpp)
add_executable(serial_reader_test serial_reader_test.cpp)

# link motor control library
target_link_libraries(main PRIVATE motor_control pthread)
target_link_libraries(setup PRIVATE motor_control)
target_link_libraries(serial_reader_test PRIVATE motor_control pthread)
