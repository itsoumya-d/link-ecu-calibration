cmake_minimum_required(VERSION 3.20)
project(link_ecu_calibration LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(ecu_core src/ecu.cpp)
target_include_directories(ecu_core PUBLIC include)
target_compile_options(ecu_core PRIVATE -Wall -Wextra -Wpedantic)

add_executable(link_ecu_calibration src/main.cpp)
target_link_libraries(link_ecu_calibration PRIVATE ecu_core)

enable_testing()
add_executable(ecu_tests tests/test_ecu.cpp)
target_link_libraries(ecu_tests PRIVATE ecu_core)
add_test(NAME ecu_tests COMMAND ecu_tests)
