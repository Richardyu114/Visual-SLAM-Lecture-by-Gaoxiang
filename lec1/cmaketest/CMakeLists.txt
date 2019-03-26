project(sayHello)

cmake_minimum_required(VERSION 2.8)

#添加子目录
add_subdirectory(src) 
add_subdirectory(libhello)

set(CMAKE_BUILD_TYPE "Release")
