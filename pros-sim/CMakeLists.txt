cmake_minimum_required(VERSION 3.20)
project(pros-sim VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Source files
file(GLOB_RECURSE PROS_SIM_SOURCES 
    "src/api/*.cpp"
    "src/rtos/*.cpp"
    "src/sim/*.cpp"
)

# Create library
add_library(pros-sim STATIC ${PROS_SIM_SOURCES})

# Include directories
target_include_directories(pros-sim PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Windows socket library for TCP transport
if(WIN32)
    target_link_libraries(pros-sim PRIVATE ws2_32)
endif()

# Compiler warnings
if(MSVC)
    target_compile_options(pros-sim PRIVATE /W4)
else()
    target_compile_options(pros-sim PRIVATE -Wall -Wextra -Wpedantic)
endif()

# Install targets
install(TARGETS pros-sim
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)

install(DIRECTORY include/
    DESTINATION include
    FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)
