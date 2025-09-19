cmake_minimum_required(VERSION 3.20)
project(swarmgrid VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Set policy for modern Boost
cmake_policy(SET CMP0167 NEW)

# Find required packages using modern approach
find_package(PkgConfig REQUIRED)
find_package(Boost 1.75 REQUIRED COMPONENTS program_options)
find_package(Threads REQUIRED)

# Optional GUI dependencies
option(BUILD_GUI "Build GUI application" ON)
if(BUILD_GUI)
    find_package(SDL2 QUIET)
    find_package(OpenGL QUIET)
endif()

# Fetch external dependencies
include(FetchContent)

# Catch2 for testing
FetchContent_Declare(
    Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG v3.4.0
)

# spdlog for logging
FetchContent_Declare(
    spdlog
    GIT_REPOSITORY https://github.com/gabime/spdlog.git
    GIT_TAG v1.12.0
)

FetchContent_MakeAvailable(Catch2 spdlog)

# Core library (pure, no dependencies on adapters)
add_library(swarmgrid_core STATIC
    src/core/types.cpp
    src/core/world.cpp
    src/core/planner.cpp
    src/core/metrics.cpp
)

target_include_directories(swarmgrid_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(swarmgrid_core PUBLIC
    Boost::boost
)

# Adapters library (I/O, networking)
add_library(swarmgrid_adapters STATIC
    src/adapters/net_sim_asio.cpp
    src/adapters/map_loader_file.cpp
    src/adapters/metrics_writer.cpp
)

target_include_directories(swarmgrid_adapters PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(swarmgrid_adapters PUBLIC
    swarmgrid_core
    Boost::boost
    spdlog::spdlog
    Threads::Threads
)

# Main executable
add_executable(swarmgrid_app
    src/main.cpp
    src/simulation.cpp
)

target_link_libraries(swarmgrid_app PRIVATE
    swarmgrid_core
    swarmgrid_adapters
    Boost::boost
    Boost::program_options
    spdlog::spdlog
)

# Tests
enable_testing()

add_executable(tests
    tests/test_main.cpp
    tests/test_types.cpp
    tests/test_world.cpp
    tests/test_planner.cpp
    tests/test_reservations.cpp
    tests/test_metrics.cpp
    tests/test_net_sim.cpp
    tests/test_map_loader.cpp
    tests/test_integration.cpp
    tests/test_properties.cpp
    src/simulation.cpp
)

target_link_libraries(tests PRIVATE
    swarmgrid_core
    swarmgrid_adapters
    Catch2::Catch2WithMain
)

include(CTest)
include(Catch)
catch_discover_tests(tests)

# Installation
install(TARGETS swarmgrid_core swarmgrid_adapters swarmgrid_app
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)