cmake_minimum_required(VERSION 2.6)
project(manipulability_app)

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin/")

set(MANIP_APP_ROOT "${PROJECT_SOURCE_DIR}")
set(MANIP_CORE "${PROJECT_SOURCE_DIR}/src/manipulability_core/")
set(MANIP_APP_INCLUDE "${PROJECT_SOURCE_DIR}/include/")


###############################################


find_package(Eigen3 REQUIRED)
find_package(Xerces REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(DrawStuff REQUIRED)

set(DS_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/ode/include/")
set(XEUMEULEU_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/xeumeuleu/xeumeuleu-1.6.0/")
set(DS_LIBRARY_DIR "${PROJECT_SOURCE_DIR}/ode/lib/")

add_subdirectory (src/manipulability_core)
add_subdirectory (src/manip_app)

