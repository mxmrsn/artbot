cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
project(artbot_cpp_test)

# find_package (Python COMPONENTS Interpreter Development)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(EIGEN3_INCLUDE_DIR /usr/local/include/eigen3)
#set(MATLAB_LIB_DIR /home/lungrobotplanner/Matlab2020b/extern/bin/glnxa64)

include_directories(
	include
	${EIGEN3_INCLUDE_DIR}
)

add_executable(artrobot_cpp_test src/main.cpp)
