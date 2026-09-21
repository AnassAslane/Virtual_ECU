cmake_minimum_required(VERSION 3.16)

project(Virtual_ECU
    VERSION 0.1.0
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_executable(vecu
    src/main.cpp
    src/can.cpp
    src/scheduler.cpp
    src/diagnostics.cpp
    src/safety.cpp
    src/watchdog.cpp
)

target_include_directories(vecu
    PRIVATE
        ${PROJECT_SOURCE_DIR}/include
)

target_compile_options(vecu
    PRIVATE
        -Wall
        -Wextra
        -Wpedantic
        -Wconversion
        -Wsign-conversion
)

target_link_libraries(vecu
    PRIVATE
        pthread
)
add_executable(vehicle_simulator
    simulator/vehicle_simulator.cpp
)

target_compile_options(vehicle_simulator
    PRIVATE
        -Wall
        -Wextra
        -Wpedantic
        -Wconversion
        -Wsign-conversion
)