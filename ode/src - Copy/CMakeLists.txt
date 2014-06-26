cmake_minimum_required(VERSION 2.6)
PROJECT(drawstuff)

include_directories("./")

	
set(SOURCES
    CMakeLists.txt   internal.h   x11.cpp
    osx.cpp      resources.rc
    drawstuff.cpp   resource.h   windows.cpp
)

add_library(drawstuff ${SOURCES})

SET_TARGET_PROPERTIES(manipulability_core PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
SET_TARGET_PROPERTIES(manipulability_core PROPERTIES ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")

