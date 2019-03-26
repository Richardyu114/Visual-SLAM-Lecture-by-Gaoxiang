cmake_minimum_required(VERSION 2.8)

project(QR_cholesky)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-O3") #Debug版会使用参数-g；Release版使用-O3 –DNDEBUG

include_directories("/usr/include/eigen3")

add_executable(QR_cholesky QR_cholesky.cpp)
#eigen3都是头文件，不需要target_link_libraries
