cmake_minimum_required(VERSION 3.16)
project(pros-sim LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)  # Enable GNU extensions for __attribute__ support

# Gather all library source files
file(GLOB PROS_SIM_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/pros/*.cpp
)

# Build the PROS simulation library as an OBJECT library to avoid static lib weak-symbol issues
add_library(pros-sim-lib OBJECT ${PROS_SIM_SOURCES})

target_include_directories(pros-sim-lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Suppress deprecation warnings from legacy typedefs and deprecated namespace
target_compile_options(pros-sim-lib PRIVATE
    -Wno-deprecated
    -Wno-attributes
)

# Build the hello world executable
add_executable(hello-world ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp)

target_link_libraries(hello-world PRIVATE pros-sim-lib)

target_compile_options(hello-world PRIVATE
    -Wno-deprecated
    -Wno-attributes
)
