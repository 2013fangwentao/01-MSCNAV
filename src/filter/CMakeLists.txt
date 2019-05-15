cmake_minimum_required( VERSION 2.8)
PROJECT(Filter)
AUX_SOURCE_DIRECTORY(./ CCFILES)
add_library(${PROJECT_NAME} SHARED ${CCFILES})