# ------------------------------------------------------------------------------
#        A Modular Optimization framework for Localization and mApping
#                               (MOLA)
#
# Copyright (C) 2018-2021, Jose Luis Blanco-Claraco, contributors (AUTHORS.md)
# All rights reserved.
# Released under GNU GPL v3. See LICENSE file
# ------------------------------------------------------------------------------

# Minimum CMake vesion: limited by CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS
cmake_minimum_required(VERSION 3.4)

# Tell CMake we'll use C++ for use in its tests/flags
project(mola-lidar-segmentation LANGUAGES CXX)

# MOLA CMake scripts: "mola_xxx()"
find_package(mola-common REQUIRED)

# find dependencies:
find_package(mrpt-maps)
find_package(mrpt-maps)
find_package(mrpt-obs)
find_package(mrpt-gui)
find_package(mrpt-tclap)
find_package(mrpt-containers)

# Find the ICP C++ library:
find_mola_package(mp2p_icp)
find_mola_package(mola-kernel)

# -----------------------
# define lib:
file(GLOB_RECURSE LIB_SRCS src/*.cpp src/*.h)
file(GLOB_RECURSE LIB_PUBLIC_HDRS include/*.h)

mola_add_library(
	TARGET ${PROJECT_NAME}
	SOURCES ${LIB_SRCS} ${LIB_PUBLIC_HDRS}
	PUBLIC_LINK_LIBRARIES
		mp2p_icp
		mola-kernel
		mrpt::maps
		mrpt::obs
	CMAKE_DEPENDENCIES
		mola-kernel
		mp2p_icp
		mrpt-maps
)

# ----------------------
# define app targets:
mola_add_executable(
	TARGET  mola-app-lidar-segment-scan
	SOURCES apps/mola-app-lidar-segment-scan.cpp
	LINK_LIBRARIES
		mp2p_icp
		mrpt::tclap
		mrpt::gui
		mrpt::containers
		${PROJECT_NAME}
)
