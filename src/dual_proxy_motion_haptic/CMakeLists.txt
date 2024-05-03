# create an executable
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${DEWA_BINARY_DIR}/01-dual_proxy_motion_haptic)
ADD_EXECUTABLE (control_vs_haptic haptic_controller.cpp ${DEWA_COMMON_SOURCE})

# and link the library against the executable
TARGET_LINK_LIBRARIES (control_vs_haptic ${DEWA_COMMON_LIBRARIES})
