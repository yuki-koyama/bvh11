add_executable(simple_demo main.cpp)
target_link_libraries(simple_demo bvh11)

add_custom_command(TARGET simple_demo POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy ${RESOURCE_FILES} $<TARGET_FILE_DIR:simple_demo>)
