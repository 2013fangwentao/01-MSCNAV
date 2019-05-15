cmake_minimum_required( VERSION 2.8 )
PROJECT(MSCNAV)

find_package(glog REQUIRED)

SET(CMAKE_CXX_COMPILE "g++")
SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -march=native -O0 -pthread" )
SET( CMAKE_BUILD_TYPE "Debug")
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
LINK_DIRECTORIES(${PROJECT_SOURCE_DIR}/lib)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include)
INCLUDE_DIRECTORIES(/opt/thirdparty/install/include)
INCLUDE_DIRECTORIES(/opt/thirdparty/install/include/eigen3)


ADD_SUBDIRECTORY(${PROJECT_SOURCE_DIR}/src/util)
ADD_SUBDIRECTORY(${PROJECT_SOURCE_DIR}/src/filter)

ADD_EXECUTABLE(NavTime ${PROJECT_SOURCE_DIR}/example/NavTime.cc)
target_link_libraries( NavTime Util glog::glog)
ADD_EXECUTABLE(NavLog ${PROJECT_SOURCE_DIR}/example/NavLog.cc)
target_link_libraries( NavLog Util glog::glog)
ADD_EXECUTABLE(NavFilter ${PROJECT_SOURCE_DIR}/example/NavFilter.cc)
target_link_libraries( NavFilter Filter Util glog::glog)