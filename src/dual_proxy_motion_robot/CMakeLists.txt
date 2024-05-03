# create an executable
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${DEWA_BINARY_DIR}/01-dual_proxy_motion_robot)
ADD_EXECUTABLE (control_vs_robot robot_controller.cpp ${DEWA_COMMON_SOURCE})
ADD_EXECUTABLE (simviz_robot simviz.cpp ${DEWA_COMMON_SOURCE})
ADD_EXECUTABLE (visualizer_robot visualizer.cpp ${DEWA_COMMON_SOURCE})

# and link the library against the executable
TARGET_LINK_LIBRARIES (control_vs_robot ${DEWA_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (simviz_robot ${DEWA_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (visualizer_robot ${DEWA_COMMON_LIBRARIES})

# export resources such as model files.
# NOTE: this requires an install build

#SET(APP_RESOURCE_DIR ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/resources)
#FILE(MAKE_DIRECTORY ${APP_RESOURCE_DIR})
#FILE(COPY world.urdf DESTINATION ${APP_RESOURCE_DIR})
#FILE(COPY panda_arm.urdf DESTINATION ${APP_RESOURCE_DIR})
#FILE(COPY iiwa14.urdf DESTINATION ${APP_RESOURCE_DIR})
