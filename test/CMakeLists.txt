cmake_minimum_required(VERSION 3.20)
project(Google_Tests_run)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MD")

include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})
include_directories(../src)
include_directories(..)

set(CMAKE_CXX_STANDARD 14)

file(GLOB_RECURSE Tests_Sources ./*.h ./*.cpp ./*.c)

# 'Google_Tests_run' is the target name
# 'test1.cpp tests2.cpp' are source files with tests
add_executable(Google_Tests_run ${Tests_Sources})

target_link_libraries(Google_Tests_run pathfinding gtest gtest_main )

if (MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MDd")
    set_property(TARGET Google_Tests_run PROPERTY
            MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")

endif()


enable_testing()

add_test(gtest ${PROJECT_NAME})