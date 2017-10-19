cmake_minimum_required(VERSION 2.8)
set(PROJECT_NAME_STR test)
project(${PROJECT_NAME_STR} CXX C)

# Set compiling flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -std=c++11")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -std=c99  -fPIC")

add_executable(RANSAC_test
               plane_ransac.cpp)
