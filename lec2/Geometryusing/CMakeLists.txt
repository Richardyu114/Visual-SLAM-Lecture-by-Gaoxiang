cmake_minimum_required(VERSION 2.8)
set (CMAKE_BUILD_TYPE "Release")
set (CMAKE_CXX_FLAGS "-O3")

project(Geometryusing)
include_directories("/usr/include/eigen3")

add_executable(Geometryusing Geometryusing.cpp)
#eigen3都是头文件，不需要target_link_libraries
