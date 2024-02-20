/**
 * @file redis_keys.h
 * @brief Includes redis key names for the simulation and control.
 * 
 */

// Simulation keys 
string JOINT_ANGLES_KEY = "sai2::sim::power::panda::sensors::q";
string JOINT_VELOCITIES_KEY = "sai2::sim::power::panda::sensors::dq";
string JOINT_TORQUES_COMMANDED_KEY = "sai2::sim::power::panda::actuators::fgc";
string BASE_TORQUES_COMMANDED_KEY = "sai2::sim::power::base::actuators::fgc";
string CONTROLLER_RUNNING_KEY = "sai2::sim::power::panda::controller";
string ROBOT_SENSED_FORCE_KEY = "sai2::sim::power::sensors::sensed_force";
string BASE_HEADING_KEY = "sai2::sim::power::base::heading";
string MASSMATRIX_KEY;
string CORIOLIS_KEY;
string BASE_POS_KEY = "sai2::sim::power::base_pos";

// Keyboard keys 
string FORWARD_KEY = "sai2::sim::power::forward";
string BACKWARD_KEY = "sai2::sim::power::backward";
string LEFT_KEY = "sai2::sim::power::left";
string RIGHT_KEY = "sai2::sim::power::right";
string UP_KEY = "sai2::sim::power::up";
string DOWN_KEY = "sai2::sim::power::down";
string CW_KEY = "sai2::sim::power::cw";
string CCW_KEY = "sai2::sim::power::ccw";

// Controller
string ROBOT_COMMAND_TORQUES_KEY = "sai2::sim::power::panda::actuators::fgc";  // alias for joint torques commanded key
string ROBOT_PROXY_KEY = "sai2::sim::power::dual_proxy::robot_proxy";
string ROBOT_PROXY_ROT_KEY = "sai2::sim::power::dual_proxy::robot_proxy_rot";
string HAPTIC_PROXY_KEY = "sai2::sim::power::dual_proxy::haptic_proxy";
string FORCE_SPACE_DIMENSION_KEY = "sai2::sim::power::dual_proxy::force_space_dimension";
string SIGMA_FORCE_KEY = "sai2::sim::power::dual_proxy::sigma_force";
string ROBOT_DEFAULT_ROT_KEY = "sai2::sim::power::dual_proxy::robot_default_rot";
string ROBOT_DEFAULT_POS_KEY = "sai2::sim::power::dual_proxy::robot_default_pos";
string HAPTIC_DEVICE_READY_KEY = "sai2::sim::power::dual_proxy::haptic_device_ready";

// Haptics 
vector<string> DEVICE_MAX_STIFFNESS_KEYS = {
	"sai2::ChaiHapticDevice::device0::specifications::max_stiffness",
	"sai2::ChaiHapticDevice::device1::specifications::max_stiffness",
	};
vector<string> DEVICE_MAX_DAMPING_KEYS = {
	"sai2::ChaiHapticDevice::device0::specifications::max_damping",
	"sai2::ChaiHapticDevice::device1::specifications::max_damping",
	};
vector<string> DEVICE_MAX_FORCE_KEYS = {
	"sai2::ChaiHapticDevice::device0::specifications::max_force",
	"sai2::ChaiHapticDevice::device1::specifications::max_force",
	};
// Set force and torque feedback of the haptic device
vector<string> DEVICE_COMMANDED_FORCE_KEYS = {
	"sai2::ChaiHapticDevice::device0::actuators::commanded_force",
	"sai2::ChaiHapticDevice::device1::actuators::commanded_force",
	};
vector<string> DEVICE_COMMANDED_TORQUE_KEYS = {
	"sai2::ChaiHapticDevice::device0::actuators::commanded_torque",
	"sai2::ChaiHapticDevice::device1::actuators::commanded_torque",
	};
vector<string> DEVICE_COMMANDED_GRIPPER_FORCE_KEYS = {
	"sai2::ChaiHapticDevice::device0::actuators::commanded_force_gripper",
	"sai2::ChaiHapticDevice::device1::actuators::commanded_force_gripper",
	};
// Haptic device current position and rotation
vector<string> DEVICE_POSITION_KEYS = {
	"sai2::ChaiHapticDevice::device0::sensors::current_position",
	"sai2::ChaiHapticDevice::device1::sensors::current_position",
	};
vector<string> DEVICE_ROTATION_KEYS = {
	"sai2::ChaiHapticDevice::device0::sensors::current_rotation",
	"sai2::ChaiHapticDevice::device1::sensors::current_rotation",
	};
vector<string> DEVICE_GRIPPER_POSITION_KEYS = {
	"sai2::ChaiHapticDevice::device0::sensors::current_position_gripper",
	"sai2::ChaiHapticDevice::device1::sensors::current_position_gripper",
	};
// Haptic device current velocity
vector<string> DEVICE_TRANS_VELOCITY_KEYS = {
	"sai2::ChaiHapticDevice::device0::sensors::current_trans_velocity",
	"sai2::ChaiHapticDevice::device1::sensors::current_trans_velocity",
	};
vector<string> DEVICE_ROT_VELOCITY_KEYS = {
	"sai2::ChaiHapticDevice::device0::sensors::current_rot_velocity",
	"sai2::ChaiHapticDevice::device1::sensors::current_rot_velocity",
	};
vector<string> DEVICE_GRIPPER_VELOCITY_KEYS = {
	"sai2::ChaiHapticDevice::device0::sensors::current_gripper_velocity",
	"sai2::ChaiHapticDevice::device1::sensors::current_gripper_velocity",
	};
vector<string> DEVICE_SENSED_FORCE_KEYS = {
	"sai2::ChaiHapticDevice::device0::sensors::sensed_force",
	"sai2::ChaiHapticDevice::device1::sensors::sensed_force",
	};
vector<string> DEVICE_SENSED_TORQUE_KEYS = {
	"sai2::ChaiHapticDevice::device0::sensors::sensed_torque",
	"sai2::ChaiHapticDevice::device1::sensors::sensed_torque",
	};

