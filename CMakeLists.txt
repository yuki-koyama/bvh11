cmake_minimum_required(VERSION 3.1)
project(bvh11 CXX)
set(CMAKE_CXX_STANDARD 11)

find_package(Eigen3)

file(GLOB HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/include/bvh11/*.hpp)

add_library(bvh11 INTERFACE)
target_sources(bvh11 INTERFACE ${HEADERS})
target_include_directories(bvh11 INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(bvh11 INTERFACE Eigen3::Eigen)

install(FILES ${HEADERS} DESTINATION include/bvh11)

option(BVH11_BUILD_DEMOS "Build demos" OFF)

if(BVH11_BUILD_DEMOS)
	file(GLOB RESOURCE_FILES ${CMAKE_CURRENT_SOURCE_DIR}/resources/*.bvh)

	set(THREEDIMUTIL_BUILD_DEMOS OFF CACHE INTERNAL "" FORCE)
	add_subdirectory(external/three-dim-util)

	add_subdirectory(demos/simple_demo)
	add_subdirectory(demos/visual_demo)
endif()
