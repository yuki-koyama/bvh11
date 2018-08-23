find_package(Qt5 COMPONENTS Widgets Gui REQUIRED)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

include_directories(${CMAKE_SOURCE_DIR}/external/libigl/include)

add_executable(visual_demo Widget.hpp Widget.cpp main.cpp)
target_link_libraries(visual_demo bvh11 three-dim-util)
