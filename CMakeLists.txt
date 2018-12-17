cmake_minimum_required( VERSION 2.8 )
PROJECT(MSCNAV)

SET(CMAKE_CXX_COMPILE "g++")
SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -march=native -O3 -pthread -lglog" )
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
LINK_DIRECTORIES(${PROJECT_SOURCE_DIR}/lib)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include)
INCLUDE_DIRECTORIES(/usr/include/)
include_directories(/usr/include/eigen3)

ADD_SUBDIRECTORY(${PROJECT_SOURCE_DIR}/src/util)

ADD_EXECUTABLE(NavTime ./example/NavTime.cc)
target_link_libraries( NavTime Util)
ADD_EXECUTABLE(NavLog ./example/NavLog.cc)
target_link_libraries( NavLog Util glog)