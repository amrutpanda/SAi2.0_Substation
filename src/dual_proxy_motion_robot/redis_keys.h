/**
 * redis keys 
*/

#pragma once 

// robot local control loop
string JOINT_ANGLES_KEY = "sai2::HapticApplications::01::simviz::sensors::q";
string JOINT_VELOCITIES_KEY = "sai2::HapticApplications::01::simviz::sensors::dq";
string ROBOT_COMMAND_TORQUES_KEY = "sai2::HapticApplications::01::simviz::actuators::tau_cmd";

string ROBOT_SENSED_FORCE_KEY = "sai2::HapticApplications::01::simviz::sensors::sensed_force";

string MASSMATRIX_KEY;
string CORIOLIS_KEY;

string ROBOT_STATE_KEY = "sai2::HapticApplications::01::robot_state";
string ROBOT_STATE_TRANSITION_KEY = "sai2::HapticApplications::01::robot_state_transition";
string ROBOT_STATE_REQUESTED_KEY = "sai2::HapticApplications::01::robot_state_requested";

string ROBOT_POS_KEY = "sai2::HapticApplications::01::robot_pos";
string ROBOT_ROT_KEY = "sai2::HapticApplications::01::robot_rot";

// dual proxy
string ROBOT_PROXY_KEY = "sai2::HapticApplications::01::dual_proxy::robot_proxy";
string ROBOT_PROXY_ROT_KEY = "sai2::HapticApplications::01::dual_proxy::robot_proxy_rot";
string HAPTIC_PROXY_KEY = "sai2::HapticApplications::01::dual_proxy::haptic_proxy";
string FORCE_SPACE_DIMENSION_KEY = "sai2::HapticApplications::01::dual_proxy::force_space_dimension";
string SIGMA_FORCE_KEY = "sai2::HapticApplications::01::dual_proxy::sigma_force";
string ROBOT_DEFAULT_ROT_KEY = "sai2::HapticApplications::01::dual_proxy::robot_default_rot";
string ROBOT_DEFAULT_POS_KEY = "sai2::HapticApplications::01::dual_proxy::robot_default_pos";
string HAPTIC_DEVICE_READY_KEY = "sai2::HapticApplications::01::dual_proxy::haptic_device_ready";

// cleaning utilities 
string ROBOT_CLEANING_DIRECTION_KEY = "sai2::HapticApplications::01::cleaning_direction";
string HAPTIC_RESET_CENTER_KEY = "sai2::HapticApplications::01::haptic_reset_center";
string ROBOT_CLEANING_HAPTIC_ENABLE_KEY = "sai2::HapticApplications::01::haptic_override";

// state keys 
string CONTROLLER_RUNNING_KEY = "sai2::HapticApplications::01::dual_proxy::controller_running";
string ROBOT_EE_FORCE_KEY = "sai2::HapticApplications::01::dual_proxy::ee_force";
string ROBOT_READY_STATE_KEY = "sai2::HapticApplications::01::robot_ready";