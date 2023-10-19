cmake_minimum_required(VERSION 3.26)
project(pathfinding)

include_directories(src/)
aux_source_directory(./src/core SRC_CORES)
aux_source_directory(./src/finders SRC_FINDERS)


set(ENABLE_GTEST ON CACHE BOOL "Enable GTest")
set(CMAKE_CXX_STANDARD 11)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MD")

add_library(pathfinding SHARED ${SRC_CORES} ${SRC_FINDERS})
#add_executable(test src/main.cpp )
#target_link_libraries(test pathfinding)

if (MSVC)
    # 或者设置运行时库选项为静态链接
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MDd")
    set_property(TARGET ${PROJECT_NAME} PROPERTY
            MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
endif()

if(ENABLE_GTEST)

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

    add_subdirectory(test)

endif ()






