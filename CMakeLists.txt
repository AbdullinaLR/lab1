# cmake_minimum_required(VERSION <specify CMake version here>)
project(1)

set(CMAKE_CXX_STANDARD 14)


add_executable(lb1
        astar.cpp
        astar.h
        dijkstra.cpp
        dijkstra.h
        input.cpp
        input.h
        main.cpp
        structures.h)
