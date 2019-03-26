cmake_minimum_required(VERSION 2.8)

set(LIB_SRC hello.cpp)

add_definitions("-DLIBHELLO_BUILD")
add_library(libhello SHARED ${LIB_SRC})
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

set_target_properties(libhello PROPERTIES OUTPUT_NAME "sayhello")
