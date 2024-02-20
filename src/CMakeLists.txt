set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
add_executable(visualizer visualizer.cpp)
target_link_libraries(visualizer ${SUBSTATION_COMMON_LIBRARIES})

add_executable(simviz simviz.cpp)
target_link_libraries(simviz ${SUBSTATION_COMMON_LIBRARIES})

add_executable(controller controller.cpp)
target_link_libraries(controller ${SUBSTATION_COMMON_LIBRARIES})

add_executable(robot_controller robot_controller.cpp)
target_link_libraries(robot_controller ${SUBSTATION_COMMON_LIBRARIES})