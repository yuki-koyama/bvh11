include_directories(${CMAKE_SOURCE_DIR})
file(GLOB files main.cpp)
add_executable(simple_demo ${files})
