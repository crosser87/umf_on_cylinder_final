#-------------------------------------------------------------------------------
# Projekt: Detektor UMF na válci 
# Soubor: CMakeLists.txt
# Kódování: UTF-8
# Autor: Radim Kříž (xkrizr03@stud.fit.vutbr.cz)
#
# Popis: Konfigurační soubor programu CMake pro celý projekt
#-------------------------------------------------------------------------------

cmake_minimum_required(VERSION 2.6)

project(umf_cylinder_detector)

add_subdirectory (original_umf)
add_subdirectory (umf_cylinder_detector)

find_package(OpenCV REQUIRED)

if (CMAKE_COMPILER_IS_GNUCXX)
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -pedantic -Werror -Wall")
endif ()

include_directories (${umf_cylinder_detector_SOURCE_DIR}/umf_cylinder_detector)
link_directories (${umf_cylinder_detector_BINARY_DIR}/umf_cylinder_detector)

include_directories (${umf_cylinder_detector_SOURCE_DIR}/original_umf)
link_directories (${umf_cylinder_detector_BINARY_DIR}/original_umf)

add_executable(umfCylinderDetectorTest umf_cylinder_detector_test.cpp)
target_link_libraries(umfCylinderDetectorTest cylinderDetector umfOriginal ${OpenCV_LIBS})
