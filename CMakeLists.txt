cmake_minimum_required(VERSION 3.1)
project(bvh11 CXX)
set(CMAKE_CXX_STANDARD 11)

file(GLOB HEADERS ${CMAKE_SOURCE_DIR}/include/bvh11/*.hpp)

add_library(bvh11 INTERFACE)
target_sources(bvh11 INTERFACE ${HEADERS})
target_include_directories(bvh11 INTERFACE ${CMAKE_SOURCE_DIR}/include)

install(FILES ${HEADERS} DESTINATION include/bvh11)

option(BVH11_BUILD_DEMOS "Build demos" OFF)

if(BVH11_BUILD_DEMOS)
	find_package(Eigen3 REQUIRED)
	include_directories(${EIGEN3_INCLUDE_DIR})

	set(RESOURCE_FILES ${CMAKE_SOURCE_DIR}/resources/sample.bvh)
	file(COPY ${RESOURCE_FILES} DESTINATION ${CMAKE_BINARY_DIR})

	set(THREEDIMUTIL_BUILD_DEMOS OFF CACHE INTERNAL "" FORCE)
	add_subdirectory(external/three-dim-util)

	add_subdirectory(demos/simple_demo)
	add_subdirectory(demos/visual_demo)
endif()
