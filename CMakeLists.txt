cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
include_directories(include)

set(EXTRA_COMPONENT_DIRS components/lsm6dso components/ft813 include)

project(hello_world)