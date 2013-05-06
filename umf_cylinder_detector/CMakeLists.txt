#-------------------------------------------------------------------------------
# Project name: Configuration file for CMake
# File: CMakeLists.txt
# File encoding: UTF-8
# File created by: Radim Kriz (xkrizr03@stud.fit.vutbr.cz)
#
# Description: Cmake configuration file
#-------------------------------------------------------------------------------

cmake_minimum_required(VERSION 2.6)

find_package(OpenCV REQUIRED)

if (CMAKE_COMPILER_IS_GNUCXX)
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -pedantic -Werror -Wall")
endif ()

include_directories (${umf_cylinder_detector_SOURCE_DIR}/original_umf)

link_directories (${umf_cylinder_detector_BINARY_DIR}/original_umf)

set(SRCS
edges.cpp
edgels.cpp
ellipses_ransac.cpp
geometry_fundamentals.cpp
umf_wrapper.cpp
lines_clustering.cpp
parabolas_fitting.cpp
parabolas_ransac.cpp
lines_fitting.cpp
ellipses_clustering.cpp
ellipses_fitting.cpp
ellipses_ransac.cpp
parabolas_clustering.cpp
supplement.cpp
find_grid.cpp
detector.cpp
)

set(HDRS
edges.h
edgels.h
ellipses_ransac.h
ransac.h
geometry_fundamentals.h
umf_wrapper.h
lines_clustering.h
standart_deviation.h
parabolas_fitting.h
parabolas_ransac.h
lines_fitting.h
ellipses_clustering.h
ellipses_fitting.h
ellipses_ransac.h
parabolas_clustering.h
supplement.h
find_grid.h
detector.h
)

add_library(cylinderDetector SHARED ${SRCS} ${HDRS})
target_link_libraries(cylinderDetector umfOriginal ${OpenCV_LIBS})

# Ellipse version
#add_executable(cylinder_detector main.cpp ${SRCS} ${HDRS})
#target_link_libraries(cylinder_detector umfOriginal ${OpenCV_LIBS})

# Parabola testing
#add_executable(parabola_test parabola_fitting_test.cpp ${SRCS} ${HDRS})
#target_link_libraries(parabola_test umfOriginal ${OpenCV_LIBS})

# Parabola version
#add_executable(parabola_version main2.cpp ${SRCS} ${HDRS})
#target_link_libraries(parabola_version umfOriginal ${OpenCV_LIBS})

# Ellipse testing
#add_executable(ransac_test main_ransac_test.cpp ${SRCS} ${HDRS})
#target_link_libraries(ransac_test ${OpenCV_LIBS})
