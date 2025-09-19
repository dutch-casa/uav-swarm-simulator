cmake_minimum_required(VERSION 3.31.5)
set(CMAKE_MODULE_PATH "/Users/dutchcaz/Projects/uav-swarm-coordinator/build/_deps/catch2-src/extras;/Users/dutchcaz/Projects/uav-swarm-coordinator/build/_deps/sdl2-src/cmake")
project(CheckOBJC OBJC)
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/result.cmake"
  "set(CMAKE_OBJC_COMPILER \"${CMAKE_OBJC_COMPILER}\")\n"
  
  )