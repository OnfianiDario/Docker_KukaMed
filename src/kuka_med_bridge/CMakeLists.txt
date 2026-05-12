cmake_minimum_required(VERSION 3.10)
project(KukaBridge)

# Standard C++ 17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Header directory
include_directories(include)

# Source files
add_executable(KukaBridge
    src/main.cpp
    src/TcpServer.cpp
    src/Controller.cpp
)

# Just for windows: connect socket library
if (WIN32)
    target_link_libraries(KukaBridge ws2_32)
endif()
