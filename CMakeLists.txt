cmake_minimum_required(VERSION 3.1)
project(bvh11 CXX)
set(CMAKE_CXX_STANDARD 11)
file(GLOB headers ${CMAKE_SOURCE_DIR}/bvh11/*.hpp)
install(FILES ${headers} DESTINATION include/bvh11)

option(BVH11_BUILD_DEMOS "Build demos" OFF)

if(BVH11_BUILD_DEMOS)
	find_package(Eigen3 REQUIRED)
	include_directories(${EIGEN3_INCLUDE_DIR})

	add_subdirectory(demos/simple_demo)
endif()
