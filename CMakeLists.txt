cmake_minimum_required(VERSION 3.2)
project(pid)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(SOURCE_FILES
    pid.cpp
    pid.h)

add_executable(pid ${SOURCE_FILES})