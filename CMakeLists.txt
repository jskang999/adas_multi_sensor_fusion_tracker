cmake_minimum_required(VERSION 3.10)
project(multi_sensor_fusion_tracker LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(BUILD_EXAMPLES "Build example applications" ON)

# Eigen3 필요 (Ubuntu 기준: sudo apt-get install libeigen3-dev)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_library(msft
    src/kalman_filter.cpp
    src/sensor_models.cpp
    src/data_association.cpp
    src/tracker.cpp
)

target_include_directories(msft
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(msft
    PUBLIC
        Eigen3::Eigen
)

target_compile_features(msft PUBLIC cxx_std_17)
target_compile_options(msft PRIVATE -Wall -Wextra -Wpedantic)

if (BUILD_EXAMPLES)
    add_executable(run_simulation
        apps/run_simulation.cpp
        sim/highway_scenario.cpp
        sim/sensor_simulator.cpp
    )
    target_include_directories(run_simulation
        PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/include
            ${CMAKE_CURRENT_SOURCE_DIR}/sim
    )
    target_link_libraries(run_simulation PRIVATE msft)
endif()
