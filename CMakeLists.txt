cmake_minimum_required(VERSION 3.2)
project(pid)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(SOURCE_FILES
    pid_stabilizer.cpp
    pid_stabilizer.h
    Engine.cpp
    Engine.h
    sliding_buffer.cpp
    sliding_buffer.h
    )

add_executable(pid ${SOURCE_FILES})