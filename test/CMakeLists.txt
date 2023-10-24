cmake_minimum_required(VERSION 3.26)
project(Google_Tests_run)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MD")

include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})
include_directories(../src)
include_directories(..)

set(CMAKE_CXX_STANDARD 14)

file(GLOB_RECURSE Tests_Sources ./*.h ./*.cpp ./*.c)
# 要排除的特定文件
set(Excluded_Files
        main.cpp
)

# 使用list(FILTER)命令过滤并排除特定文件
list(FILTER Tests_Sources EXCLUDE REGEX "(${Excluded_Files})")


set(-DENABLE_GTEST=ON)
include(FetchContent)
FetchContent_Declare(
        googletest
        # Specify the commit you depend on and update it regularly.
        URL https://github.com/google/googletest/archive/5376968f6948923e2411081fd9372e71a59d8e77.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_GetProperties(googletest)
if(NOT googletest_POPULATED)
    FetchContent_MakeAvailable(googletest)
endif()


#add_executable(test main.cpp )
target_link_libraries(test pathfinding)

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