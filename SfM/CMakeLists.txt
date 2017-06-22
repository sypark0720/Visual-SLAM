cmake_minimum_required( VERSION 2.8 )
project ( SfM )

set( CMAKE_CXX_COMPILER "g++" )
set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -march=native -O3" )

# OpenCV
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )


add_executable( SfM sfm.cpp )
# 链接OpenCV库
target_link_libraries( SfM ${OpenCV_LIBS} )

