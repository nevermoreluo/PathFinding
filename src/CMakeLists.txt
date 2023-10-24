cmake_minimum_required(VERSION 3.20)
project(pathfinding)

include_directories(.)
aux_source_directory(./core SRC_CORES)
aux_source_directory(./finders SRC_FINDERS)


set(CMAKE_CXX_STANDARD 11)



add_library(${PROJECT_NAME} ${SRC_CORES} ${SRC_FINDERS})


if (MSVC)
    # 或者设置运行时库选项为静态链接
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MDd")
    set_property(TARGET ${PROJECT_NAME} PROPERTY
            MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
endif()


