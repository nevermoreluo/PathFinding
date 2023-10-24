cmake_minimum_required(VERSION 3.20)
project(pathfinding_main)

add_subdirectory(src)

if(ENABLE_GTEST)
    add_subdirectory(test)
endif ()






