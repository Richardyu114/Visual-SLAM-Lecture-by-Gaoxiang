cmake_minimum_required(VERSION 2.8)

#头文件目录
include_directories(${PROJECT_SOURCE_DIR}/libhello)

set(APP_SRC useHello.cpp)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)

add_executable(sayHello ${APP_SRC})

target_link_libraries(sayHello libhello)
