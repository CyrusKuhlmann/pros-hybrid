cmake_minimum_required(VERSION 3.20)
project(robot-simulator LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add the pros-sim library
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../pros-sim pros-sim)

# Shared robot code
set(ROBOT_SHARED_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../shared")
file(GLOB_RECURSE ROBOT_SOURCES "${ROBOT_SHARED_DIR}/src/*.cpp")

# Create executable
add_executable(robot-sim 
    main_entry.cpp
    ${ROBOT_SOURCES}
)

# Include directories
target_include_directories(robot-sim PRIVATE
    "${ROBOT_SHARED_DIR}/include"
)

# Link against pros-sim
target_link_libraries(robot-sim PRIVATE pros-sim)

# Set startup code to auto-connect to simulator
target_compile_definitions(robot-sim PRIVATE
    SIM_HOST="127.0.0.1"
    SIM_PORT=9090
)

# On Windows, ensure we don't get a console window popup (optional)
if(WIN32)
    # Keep console for now to see output
    # set_target_properties(robot-sim PROPERTIES WIN32_EXECUTABLE TRUE)
endif()
