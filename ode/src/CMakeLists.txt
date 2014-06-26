cmake_minimum_required(VERSION 2.6)
PROJECT(drawstuff)

include_directories("./")

	
set(SOURCES
    CMakeLists.txt   internal.h   x11.cpp
    osx.cpp      resources.rc
    drawstuff.cpp   resource.h   windows.cpp
    drawstuff.h
)

add_library(drawstuff ${SOURCES})


