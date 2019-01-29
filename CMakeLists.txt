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

# Ceres
if(MSVC)
    set(Ceres_DIR "Ceres_DIR-NOTFOUND" CACHE PATH "Directory of the precompiled Ceres package we received from the lecturers.")
    if(NOT EXISTS ${Ceres_DIR})
        message(FATAL_ERROR "Ceres not found. Please configure Ceres_DIR.")
    endif()
    set(CERES_INCLUDE_DIRS
        "${Ceres_DIR}/ceres-solver/include"
        "${Ceres_DIR}/glog/src/windows"
        "${Ceres_DIR}/win/include"
    )
    set(CERES_LIBRARIES
        debug "${Ceres_DIR}/x64/libglog_staticd.lib"
        debug "${Ceres_DIR}/x64/ceres_staticd.lib"
        optimized "${Ceres_DIR}/x64/libglog_static.lib"
        optimized "${Ceres_DIR}/x64/ceres_static.lib"
    )
    add_compile_definitions(CERES_USING_STATIC_LIBRARY CERES_MSVC_USE_UNDERSCORE_PREFIXED_BESSEL_FUNCTIONS)
else()
    find_package(Ceres REQUIRED)
endif()
include_directories(${CERES_INCLUDE_DIRS})

# Set files to be compiled
set(HEADER_FILES
        cxxopts.hpp
        Settings.h
		CoarseAlignment.h
		FeaturePointExtractor.h
		ProcrustesAligner.h
		VirtualSensor.h
		Mesh.h
		FaceModel.h
		Optimizer.h
		OptimizerInit.h
        Rasterizer.h
		Sensor.h
		stdafx.h
		utils.h
		SwitchControl.h
        Visualizer.h
		OpenNI2Sensor.h)
set(SOURCE_FILES
		ProcrustesAligner.cpp
		CoarseAlignment.cpp
		FaceModel.cpp
		Optimizer.cpp
		OptimizerInit.cpp
        Rasterizer.cpp
		main.cpp
		utils.cpp
        SwitchControl.cpp
        Visualizer.cpp
		OpenNI2Sensor.cpp
)

if (MSVC)
    # For precompiled header.
    # Set 
    # "Precompiled Header" to "Use (/Yu)"
    # "Precompiled Header File" to "stdafx.h"
    set (SOURCE_FILES ${SOURCE_FILES} stdafx.cpp)

    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /Yustdafx.h /FIstdafx.h")
	set_source_files_properties(stdafx.cpp
	    PROPERTIES
	    COMPILE_FLAGS "/Ycstdafx.h"
	)

    # Enable link time code generation, as Ceres binaries require it.
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /LTCG:INCREMENTAL")
endif()

add_executable(face_reconstruction ${HEADER_FILES} ${SOURCE_FILES})
target_link_libraries(face_reconstruction
    ${PCL_LIBRARIES}
    ${CERES_LIBRARIES}
)
