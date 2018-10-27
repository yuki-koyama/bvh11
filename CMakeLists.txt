cmake_minimum_required(VERSION 3.1)
project(bvh11 CXX)
set(CMAKE_CXX_STANDARD 11)

find_package(Eigen3)

file(GLOB HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/include/bvh11.hpp)
file(GLOB SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/bvh11.cpp)

add_library(bvh11 STATIC ${HEADERS} ${SOURCES})
target_link_libraries(bvh11 Eigen3::Eigen)
target_include_directories(bvh11 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

install(FILES ${HEADERS} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/)
install(TARGETS bvh11 ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)

option(BVH11_BUILD_DEMOS "Build demos" OFF)

if(BVH11_BUILD_DEMOS)
	file(GLOB RESOURCE_FILES ${CMAKE_CURRENT_SOURCE_DIR}/resources/*.bvh)

	set(THREEDIMUTIL_BUILD_DEMOS OFF CACHE INTERNAL "" FORCE)
	add_subdirectory(external/three-dim-util)

	add_subdirectory(demos/simple_demo)
	add_subdirectory(demos/visual_demo)
endif()
