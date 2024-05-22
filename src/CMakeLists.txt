set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

add_executable(visualizer visualizer.cpp)
target_link_libraries(visualizer ${SUBSTATION_COMMON_LIBRARIES})

add_executable(simviz simviz.cpp)
target_link_libraries(simviz ${SUBSTATION_COMMON_LIBRARIES})

add_executable(controller controller.cpp)
target_link_libraries(controller ${SUBSTATION_COMMON_LIBRARIES})

add_executable(robot_controller robot_controller.cpp)
target_link_libraries(robot_controller ${SUBSTATION_COMMON_LIBRARIES})

add_executable(robot_controller_v2 robot_controller_v2.cpp)
target_link_libraries(robot_controller_v2 ${SUBSTATION_COMMON_LIBRARIES})

add_executable(save_pose save_pose.cpp)
target_link_libraries(save_pose ${SUBSTATION_COMMON_LIBRARIES})

add_executable(getTransform getTransform.cpp)
target_link_libraries(getTransform ${SUBSTATION_COMMON_LIBRARIES})

add_executable(camera_calibration_test camera_calibration_test.cpp)
target_link_libraries(camera_calibration_test ${SUBSTATION_COMMON_LIBRARIES})

add_executable(simulation simulation.cpp)
target_link_libraries(simulation ${SUBSTATION_COMMON_LIBRARIES})

add_executable(sim_controller sim_controller.cpp)
target_link_libraries(sim_controller ${SUBSTATION_COMMON_LIBRARIES})

# for visual servoing teleoperation application.

add_executable(vs_controller vs_controller.cpp)
target_link_libraries(vs_controller ${SUBSTATION_COMMON_LIBRARIES})


add_subdirectory("dual_proxy_motion_haptic")
add_subdirectory("dual_proxy_motion_robot")