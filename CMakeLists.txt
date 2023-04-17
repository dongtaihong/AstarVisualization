cmake_minimum_required(VERSION 2.6)
project(astar)
# c++ 11
set(CMAKE_CXX_FLAGS "-std=c++11")
# 调试程序
set(CMAKE_BUILD_TYPE "Debug")
 
find_package(OpenCV 4.7.0 REQUIRED) 	# find opencv4.7 installed path
include_directories(OpenCV_INCLUDE_DIRS)	# load opencv header files
include_directories(include)
add_executable(astar src/main.cpp src/astar.cpp)	# create exe file
target_link_libraries(astar ${OpenCV_LIBS})	# link llib files to exe
