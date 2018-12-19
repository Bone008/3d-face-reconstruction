cmake_minimum_required(VERSION 2.8)
project(face_reconstruction)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake ${CMAKE_MODULE_PATH})
set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS}")
add_definitions(-DPROJECT_DIR="${PROJECT_SOURCE_DIR}")

# Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# PCL
find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

if (MSVC)
    # For precompiled header.
    # Set 
    # "Precompiled Header" to "Use (/Yu)"
    # "Precompiled Header File" to "stdafx.h"
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /Yustdafx.h /FIstdafx.h")
endif()

# Set files to be compiled
set(HEADER_FILES
		CoarseAlignment.h
		FeaturePointExtractor.h
		ProcrustesAligner.h
		VirtualSensor.h
		vtk.h
		FaceModel.h
		Optimizer.h
		Sensor.h)
set(SOURCE_FILES
		CoarseAlignment.cpp
		FaceModel.cpp
		Optimizer.cpp
		main.cpp
		vtk.cpp
)

add_executable(face_reconstruction ${HEADER_FILES} ${SOURCE_FILES})
target_link_libraries(face_reconstruction ${PCL_LIBRARIES})
