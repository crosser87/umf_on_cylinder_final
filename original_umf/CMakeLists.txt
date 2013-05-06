#-------------------------------------------------------------------------------
# Project name: Configuration file for CMake
# File: CMakeLists.txt
# File encoding: UTF-8
# File created by: Radim Kriz (xkrizr03@stud.fit.vutbr.cz)
#
# Description: Cmake configuration file
#-------------------------------------------------------------------------------

cmake_minimum_required(VERSION 2.6)

if (CMAKE_COMPILER_IS_GNUCXX)
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -ggdb -I. -std=c++0x -lm")
endif ()

find_package(OpenCV REQUIRED)
find_package(Eigen2 REQUIRED)

set(HDRS_ORIGINAL
area.h
cross_ratio.h
defines.h
detect_directions.h
detector_global.h
detect_umf.h
find_vanish.cpp
find_vanish.h
fit_hyperplane.cpp
fit_hyperplane.h
structures.h
pixel_counter.h
epnp.h
)

set(SRCS_ORIGINAL
area.cpp
cross_ratio.cpp
detect_directions.cpp
detect_umf.cpp
epnp.cpp
find_vanish.cpp
fit_hyperplane.cpp
structures.cpp
epnp.cpp
)

add_library(umfOriginal SHARED ${HDRS_ORIGINAL} ${SRCS_ORIGINAL})
target_link_libraries(umfOriginal ${OpenCV_LIBS} ${Eigen2_INCLUDE_DIR})
