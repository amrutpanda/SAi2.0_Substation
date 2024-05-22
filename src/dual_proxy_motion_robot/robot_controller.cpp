/**
 * @file controller.cpp
 * @author William Chong (wmchong@stanford.edu)
 * @brief Robot control 
 * @version 0.1
 * @date 2023-08-10
 * 
 * @copyright Copyright (c) 2023
 * 
 */

/*
	The controller performs different control modes for the vertical pin insulator:
	- Free: free-space motion with force feedback
	- Orientation: fixed position, but free to rotate orientation
	- Cleaning: fixed position and orientation, send rotation commands through rotary encoder to clean 
	- Recenter: recenters the haptic workspace based on the robot's current position 
*/
#define USING_OTG 1
#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "tasks/JointTask.h"
#include "tasks/PosOriTask.h"
#include "filters/ButterworthFilter.h"
#include "../Logger.h"
#include "perception/ForceSpaceParticleFilter.h"
#include "redis_keys.h"
#include <iostream>
#include <string>
#include <random>
#include <queue>

enum State {
	INIT = 0,
	FREE,
	ORIENTATION,
	CLEANING,
	CLEANING_EXIT_TRANSITION,
	RECENTER,
	VS,
	IDLE,
	TOTAL_STATES
};
const vector<string> state_names {"Init", "Free", "Orientation", "Cleaning", "Cleaning Exit", "Recentering","vs","idle"};

#include <signal.h>
bool runloop = false;
void sighandler(int){runloop = false;}

using namespace std;
using namespace Eigen;

const string robot_file = "./resources/panda_arm.urdf";
// const string robot_file = "./resources/iiwa14.urdf";

RedisClient redis_client;

/*
	Particle filter parameters
*/
const int n_particles = 1000;
MatrixXd particle_positions_to_redis = MatrixXd::Zero(3, n_particles);
int force_space_dimension = 0;
int prev_force_space_dimension = 0;
Matrix3d sigma_force = Matrix3d::Zero();
Matrix3d sigma_motion = Matrix3d::Identity();

Vector3d motion_control_pfilter;
Vector3d force_control_pfilter;
Vector3d measured_velocity_pfilter;
Vector3d measured_force_pfilter;

queue<Vector3d> pfilter_motion_control_buffer;
queue<Vector3d> pfilter_force_control_buffer;
queue<Vector3d> pfilter_sensed_force_buffer;
queue<Vector3d> pfilter_sensed_velocity_buffer;

const double control_loop_freq = 1000.0;
const double pfilter_freq = 50.0;
const double freq_ratio_filter_control = pfilter_freq / control_loop_freq;

/*
	Control and sense points 
*/
const double tool_length = 0.21;
// const double tool_length = 0.195;
const string control_link_name = "link7";
// const Vector3d control_pos_in_link = Vector3d(0, 0, 0);
// const Vector3d control_pos_in_link = Vector3d(0.180 + 0.015, 0, 0.11);
// const Vector3d control_pos_in_link = Vector3d(0.180 + 0.015, 0, 0.09);
const Vector3d control_pos_in_link = Vector3d(tool_length * sin(22.5 * M_PI / 180), tool_length * cos(22.5 * M_PI / 180), 0.09);
// const Vector3d control_pos_in_link = Vector3d(0.185, 0, 0.09);
// const Vector3d control_pos_in_link = Vector3d(0.21, 0, 0.09);
const string sensor_link_name = "link7";
const Vector3d sensor_pos_in_link = Vector3d(0, 0, 0.040);
Vector3d ee_origin_point, ee_control_point;
// Vector3d ee_control_pos_in_link = Vector3d(0.180 + 0.015, 0, 0.11);
Vector3d ee_control_pos_in_link = Vector3d(0.180 + 0.015, 0, 0.09);
// Vector3d ee_control_pos_in_link = Vector3d(0.185, 0, 0.09);
// Vector3d ee_control_pos_in_link = Vector3d(0.21, 0, 0.09);

// particle filter loop
void particle_filter();

const bool flag_simulation = false;
// const bool flag_simulation = true;

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

int main() {

	if (!flag_simulation) {
		ROBOT_COMMAND_TORQUES_KEY = "sai2::FrankaPanda::Clyde::actuators::fgc";
		JOINT_ANGLES_KEY  = "sai2::FrankaPanda::Clyde::sensors::q";
		JOINT_VELOCITIES_KEY = "sai2::FrankaPanda::Clyde::sensors::dq";
		MASSMATRIX_KEY = "sai2::FrankaPanda::Clyde::sensors::model::massmatrix";
		CORIOLIS_KEY = "sai2::FrankaPanda::Clyde::sensors::model::coriolis";
		ROBOT_SENSED_FORCE_KEY = "sai2::ATIGamma_Sensor::Clyde::force_torque";

		// ROBOT_COMMAND_TORQUES_KEY = "sai2::iiwa14::actuators::fgc";
		// JOINT_ANGLES_KEY  = "sai2::iiwa14::sensors::q";
		// JOINT_VELOCITIES_KEY = "sai2::iiwa14::sensors::dq";
		// MASSMATRIX_KEY = "sai2::iiwa14::model::massmatrix";
		// CORIOLIS_KEY = "sai2::iiwa14::model::coriolis";
		// // ROBOT_SENSED_FORCE_KEY = "sai2::optoforceSensor::6Dsensor::force";
		// ROBOT_SENSED_FORCE_KEY = "sai2::ATIGamma_Sensor::iiwa14::force_torque";
	}

	// start redis client local
	redis_client = RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load robots
	Affine3d T_world_robot = Affine3d::Identity();
	T_world_robot.translation() = Vector3d(0, 0, 0);
	auto robot = new Sai2Model::Sai2Model(robot_file, false, T_world_robot);

	robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
	robot->updateModel();

	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);
	VectorXd coriolis = VectorXd::Zero(dof);
	MatrixXd N_prec = MatrixXd::Identity(dof,dof);

	// state handling 
	int state = INIT;
	int requested_state = state;
	int prev_state = state;
	int state_transition_flag = 0;  
	redis_client.set(ROBOT_STATE_KEY, to_string(state));
	redis_client.set(ROBOT_STATE_TRANSITION_KEY, to_string(state_transition_flag));
	redis_client.set(ROBOT_STATE_REQUESTED_KEY, to_string(requested_state));

	// joint task
	auto joint_task = new Sai2Primitives::JointTask(robot);
	VectorXd joint_task_torques = VectorXd::Zero(dof);
	joint_task->_use_interpolation_flag = false;
	joint_task->_use_velocity_saturation_flag = true;
	joint_task->_saturation_velocity.array() = 35 * (M_PI / 180);
	// joint_task->_saturation_velocity << 35*(M_PI/180);

	joint_task->_kp = 300.0;
	joint_task->_kv = 15.0;
	joint_task->_ki = 0.0;
	int integrate_flag = 1;

	VectorXd q_init(dof);
	// q_init << 0, -30, 0, -130, 0, 100, 0;
	// q_init << 0, 18, 0, -70, 0, 90, -71 * 0;  // changed last joint from 0 to -90
	// q_init *= M_PI / 180.0;
	// q_init << 0,-0.448781,-0.0481126,-2.12039,0.019258,1.66288,-0.0841794;
	// q_init << 0.0509167,-0.683943,-0.178354,-2.33097,-0.153555,1.76067,0;
	// q_init << 0,-0.895205,-0.188536,-2.25996,-0.0967708,1.34953,0;
	// q_init << -2.05761,-1.04975,1.35334,-2.34941,-2.11049,1.35565,0;
	q_init << 1.63374,1.10276,-2.06883,-2.59503,-2.254,1.36884,0; // SET BY WILLIAM.
	q_init << -0.390553,-1.34995,0.758122,-2.49509,-2.29837,1.86602,-1.06781; // SET BY AMRUT.
	joint_task->_desired_position = q_init;

	// posori task
	auto posori_task = new Sai2Primitives::PosOriTask(robot, control_link_name, control_pos_in_link);
	Vector3d x_init = posori_task->_current_position;
	Matrix3d R_init = posori_task->_current_orientation;
	redis_client.setEigenMatrixJSON(ROBOT_PROXY_KEY, x_init);
	redis_client.setEigenMatrixJSON(ROBOT_PROXY_ROT_KEY, R_init);
	redis_client.setEigenMatrixJSON(HAPTIC_PROXY_KEY, x_init);

	// compute expected default rotation to send to haptic
	robot->_q = q_init; 
	robot->updateModel();
	Matrix3d R_default = Matrix3d::Identity();
	Vector3d pos_default = Vector3d::Zero();
	robot->rotation(R_default, control_link_name);
	robot->position(pos_default, control_link_name, control_pos_in_link);
	redis_client.setEigenMatrixJSON(ROBOT_DEFAULT_ROT_KEY, R_default);
	redis_client.setEigenMatrixJSON(ROBOT_DEFAULT_POS_KEY, pos_default);

	Vector3d robot_pos = Vector3d::Zero();
	Matrix3d robot_rot = Matrix3d::Identity();
	Vector3d ee_lin_vel = Vector3d::Zero();
	Vector3d ee_ang_vel = Vector3d::Zero();

	VectorXd posori_task_torques = VectorXd::Zero(dof);
	posori_task->_use_interpolation_flag = false;
	posori_task->_use_velocity_saturation_flag = false;

	posori_task->_kp_pos = 400.0;
	posori_task->_kv_pos = 15.0;
	posori_task->_kp_ori = 400.0;
	posori_task->_kv_ori = 15.0;

	posori_task->_e_max = 3e-2; 
	posori_task->_e_min = 3e-3;
	// posori_task->_e_max = 1e-2;
	// posori_task->_e_min = 1e-3;

	// create a new posori_task object for visual servoing purpose.
	auto posori2_task = new Sai2Primitives::PosOriTask(robot,"eef_2",Vector3d(0,0,0));
	posori2_task->_use_interpolation_flag = true;
	posori2_task->_use_velocity_saturation_flag = true;
	posori2_task->_linear_saturation_velocity = 0.2;

	posori2_task->_kp_pos = 400.0;
	posori2_task->_kv_pos = 15.0;
	posori2_task->_kp_ori = 400.0;
	posori2_task->_kv_ori = 15.0;

	posori2_task->_e_max = 3e-2;
	posori2_task->_e_min = 3e-3;

	VectorXd posori2_task_torques = VectorXd::Zero(dof);

	posori2_task->updateTaskModel((MatrixXd::Identity(dof,dof)));

	// set visual servoing variables.
    Affine3d T;
    bool positon_reached = false;
    bool first_time_set = true;
    bool angle_reached = false;
	bool first_vs_loop = true;
	Matrix3d M; // mapping matrix.
	M << 0.707, 0, 0.707,
			-0.707, 0, 0.707,
			0,  -1, 0;

	// rotation inputs from the keyboard 
	int rot_direction = 0;
	int prev_rot_direction = 0;
	Vector3d cleaning_rotation_axis = Vector3d(0, 0, 1);
	redis_client.set(ROBOT_CLEANING_DIRECTION_KEY, "0");

	// force sensing
	Affine3d sensor_transform_in_link;
	Vector3d sensor_pos_in_link = Vector3d(0, 0, 0);
	Matrix3d R_link_sensor = Matrix3d::Identity();
	// R_link_sensor << -1, 0, 0, 0, -1, 0, 0, 0, 1;
	// R_link_sensor = AngleAxisd(M_PI, Vector3d(0, 0, 1)).toRotationMatrix();
	sensor_transform_in_link.translation() = sensor_pos_in_link;
	sensor_transform_in_link.linear() = R_link_sensor;
	posori_task->setForceSensorFrame(sensor_link_name, sensor_transform_in_link);

	VectorXd sensed_force_moment_local_frame = VectorXd::Zero(6);
	VectorXd sensed_force_moment_world_frame = VectorXd::Zero(6);
	VectorXd force_bias = VectorXd::Zero(6);
	double tool_mass = 0;
	Vector3d tool_com = Vector3d::Zero();

	Vector3d init_force = Vector3d::Zero();
	bool first_loop = true;
	double tol = 1e-1;

	if (!flag_simulation) {
		// force_bias << -2.87094, -4.22234, -234.362, 0.382103, 2.09627, -0.0524391;
		force_bias << 0, 0, 0, 0, 0, 0;
		// tool_mass = 0.1;
		// tool_com = Vector3d(0.0100291, 0.0122058, 0.0216975);
		// tool_mass = 1.2;
		tool_mass = 0.65 - 0.145;
		tool_com = Vector3d(0.10, 0, 0.025);
		// force_bias *= 0;
		// tool_mass *= 0;	
	}

	// dual proxy parameters and variables
	double k_vir = 250.0;
	double max_force_diff = 0.1;
	double max_force = 10.0;

	int haptic_ready = 0;
	redis_client.set(HAPTIC_DEVICE_READY_KEY, "0");

	Vector3d robot_proxy = Vector3d::Zero();
	Matrix3d robot_proxy_rot = Matrix3d::Identity();
	Vector3d haptic_proxy = Vector3d::Zero();
	Vector3d prev_desired_force = Vector3d::Zero();

	// containers 
	Matrix3d R_accumulator = Matrix3d::Identity();  // tracks the amount of rotation difference when exiting the cleaning state (virtual rot to actual rot (R_{virtual}^{actual}))
	bool first_cleaning_loop = true;
	double max_amp = 0; 
	double transition_time = 0;
	Matrix3d starting_ori = Matrix3d::Identity();
	double zero_angle = 0;
	double des_delta_angle = 0;
	double prev_angle = 0;
	int robot_ready = 0;
	redis_client.set(ROBOT_READY_STATE_KEY, to_string(robot_ready));

	// setup redis keys to be updated with the callback
	redis_client.createReadCallback(0);
	redis_client.createWriteCallback(0);

	// Objects to read from redis
    redis_client.addEigenToReadCallback(0, JOINT_ANGLES_KEY, robot->_q);
    redis_client.addEigenToReadCallback(0, JOINT_VELOCITIES_KEY, robot->_dq);

    MatrixXd mass_from_robot = MatrixXd::Identity(dof,dof);
    VectorXd coriolis_from_robot = VectorXd::Zero(dof);
	if (!flag_simulation) {
		redis_client.addEigenToReadCallback(0, MASSMATRIX_KEY, mass_from_robot);
		redis_client.addEigenToReadCallback(0, CORIOLIS_KEY, coriolis_from_robot);
	}

	redis_client.addEigenToReadCallback(0, ROBOT_PROXY_KEY, robot_proxy);
	redis_client.addEigenToReadCallback(0, ROBOT_PROXY_ROT_KEY, robot_proxy_rot);
	redis_client.addIntToReadCallback(0, HAPTIC_DEVICE_READY_KEY, haptic_ready);

    redis_client.addEigenToReadCallback(0, ROBOT_SENSED_FORCE_KEY, sensed_force_moment_local_frame);
	redis_client.addEigenToWriteCallback(0, ROBOT_EE_FORCE_KEY, posori_task->_sensed_force);

	redis_client.addIntToReadCallback(0, ROBOT_STATE_REQUESTED_KEY, requested_state);
	redis_client.addIntToReadCallback(0, ROBOT_STATE_TRANSITION_KEY, state_transition_flag);

	redis_client.addIntToReadCallback(0, ROBOT_CLEANING_DIRECTION_KEY, rot_direction);
	// redis_client.addIntToReadCallback(0, ROBOT_CLEANING_DELTA_X_POS_KEY, delta_x_cleaning);
	// redis_client.addIntToReadCallback(0, ROBOT_CLEANING_DELTA_Y_POS_KEY, delta_y_cleaning);
	// redis_client.addIntToReadCallback(0, ROBOT_CLEANING_DELTA_Z_POS_KEY, delta_z_cleaning);

	// Objects to write to redis
	redis_client.addEigenToWriteCallback(0, ROBOT_COMMAND_TORQUES_KEY, command_torques);
	redis_client.addIntToWriteCallback(0, ROBOT_STATE_KEY, state);
	redis_client.addEigenToWriteCallback(0, ROBOT_POS_KEY, robot_pos);
	redis_client.addEigenToWriteCallback(0, ROBOT_ROT_KEY, robot_rot);

	redis_client.addEigenToWriteCallback(0, HAPTIC_PROXY_KEY, haptic_proxy);
	redis_client.addEigenToWriteCallback(0, SIGMA_FORCE_KEY, sigma_force);
	redis_client.addIntToWriteCallback(0, FORCE_SPACE_DIMENSION_KEY, force_space_dimension);
	redis_client.addIntToWriteCallback(0, ROBOT_READY_STATE_KEY, robot_ready);

	// setup data logging
    string folder = "../../04-dual_proxy_motion_robot/data_logging/data/";
	string filename = "data";
    auto logger = new Logging::Logger(100000, folder + filename);
	
	Vector3d log_robot_ee_position = x_init;
	Vector3d log_robot_ee_velocity = Vector3d::Zero();
	Vector3d log_robot_proxy_position = robot_proxy;
	VectorXd log_joint_angles = robot->_q;
	VectorXd log_joint_velocities = robot->_dq;
	VectorXd log_joint_command_torques = command_torques;
	VectorXd log_sensed_force_moments = VectorXd::Zero(6);
    Vector3d log_desired_force = Vector3d::Zero();

	logger->addVectorToLog(&log_robot_ee_position, "robot_ee_position");
	logger->addVectorToLog(&log_robot_ee_velocity, "robot_ee_velocity");
	logger->addVectorToLog(&log_robot_proxy_position, "robot_proxy_position");
	logger->addVectorToLog(&log_joint_angles, "joint_angles");
	logger->addVectorToLog(&log_joint_velocities, "joint_velocities");
	logger->addVectorToLog(&log_joint_command_torques, "joint_command_torques");
	logger->addVectorToLog(&log_sensed_force_moments, "sensed_forces_moments");
    logger->addVectorToLog(&log_desired_force, "desired_force");

	logger->start();

	// start particle filter thread
	runloop = true;
	redis_client.set(CONTROLLER_RUNNING_KEY,"1");
	thread particle_filter_thread(particle_filter);

	// create a timer
	unsigned long long controller_counter = 0;
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(control_loop_freq); 
	double current_time = 0;
	double prev_time = 0;
	bool fTimerDidSleep = true;
	double start_time = timer.elapsedTime(); //secs

	while (runloop) {
		// wait for next scheduled loop
		timer.waitForNextLoop();
		current_time = timer.elapsedTime() - start_time;
		prev_state = state;
		prev_rot_direction = rot_direction;

		// some calculation.
		// Matrix3d m_rot;
		// m_rot = AngleAxisd(0,Vector3d::UnitX())* AngleAxisd(0,Vector3d::UnitY()) * AngleAxisd(0.7,Vector3d::UnitZ());
		// Affine3d T_ee,T_cam, T_eef_2;
		// robot->transform(T_eef_2,"eef_2",Vector3d(0,1,0),M*m_rot);
		// // robot->transform(T_ee,"end_effector");
		// robot->transform(T_cam,"camera_link",Vector3d(2,0,0),m_rot);
		// // cout << "T_eef_2_to_cam: \n" << (T_eef_2.rotation().transpose() * T_cam.rotation())<< "\n";
		// cout << "R_ee : \n" << T_eef_2.rotation() << endl << "R_cam : \n" << T_cam.rotation() << endl;
		// cout << "Angles: = \n" << (M).eulerAngles(2,1,0)*(180/M_PI) << "\n";
		// exit(0);

		// read haptic state and robot state
		redis_client.executeReadCallback(0);
		if (flag_simulation) {
			robot->updateModel();
			robot->coriolisForce(coriolis);
		} else {
			robot->updateKinematics();
			MatrixXd Jv;
			robot->Jv(Jv, sensor_link_name, tool_com);
			robot->_M = mass_from_robot + (tool_mass + 0 * 0.145) * Jv.transpose() * Jv;
			robot->_M(4, 4) += 0.5;
			robot->_M(5, 5) += 0.5;
			robot->_M(6, 6) += 0.5;
			robot->updateInverseInertia();
			coriolis = coriolis_from_robot;
		}

		robot->position(robot_pos, control_link_name, control_pos_in_link);
		robot->rotation(robot_rot, control_link_name);
		robot->linearVelocity(ee_lin_vel, control_link_name, control_pos_in_link);
		robot->angularVelocity(ee_ang_vel, control_link_name);

		robot->position(ee_control_point, control_link_name, ee_control_pos_in_link);
		robot->position(ee_origin_point, control_link_name, Vector3d(0, 0, 0));
		// CO_vec = ee_origin_point - ee_control_point;

		if (state_transition_flag) {
			if (requested_state < TOTAL_STATES) {
				state = requested_state;			
			} else {
				std::cout << "Invalid state\n";
			}
			redis_client.set(ROBOT_STATE_TRANSITION_KEY, "0");
		}

		// state transition handling
		if (state != prev_state) {
			std::cout << "From " << state_names[prev_state] << " to " << state_names[state] << "\n";
			posori_task->reInitializeTask();
			joint_task->reInitializeTask();

			if (state == CLEANING) {
				cleaning_rotation_axis = robot_rot.col(2);
				std::cout << "Cleaning state with rotation axis: " << cleaning_rotation_axis.transpose() << "\n";
			}

			if (prev_state == CLEANING) {
				first_cleaning_loop = true;
				if (state != CLEANING_EXIT_TRANSITION) {
					std::cout << "Invalid state selected exiting cleaning - changing to exit transition\n";
					state = CLEANING_EXIT_TRANSITION;
				}
			}
		}

		// update task hieararchy
		N_prec.setIdentity(dof, dof);
		posori_task->updateTaskModel(N_prec);
		N_prec = posori_task->_N;
		joint_task->updateTaskModel(N_prec);

		// remove load effect from force sensor 
		sensed_force_moment_local_frame -= force_bias;
		Matrix3d R_world_sensor;
		robot->rotation(R_world_sensor, control_link_name);
		R_world_sensor = R_world_sensor * R_link_sensor;

		// remove load forces
		Vector3d p_tool_local_frame = tool_mass * R_world_sensor.transpose() * Vector3d(0, 0, -9.81);
		sensed_force_moment_local_frame.head(3) += p_tool_local_frame;
		sensed_force_moment_local_frame.tail(3) += tool_com.cross(p_tool_local_frame);  

		// remove load dynamic forces (should be zero at the start)
		VectorXd xdd;
		robot->acceleration6d(xdd, sensor_link_name, sensor_pos_in_link);
		Vector3d w_tool = R_world_sensor.transpose() * ee_ang_vel;
		Vector3d alpha_tool = xdd.tail(3);
		sensed_force_moment_local_frame.head(3) -= tool_mass * xdd.head(3) + alpha_tool.cross(tool_mass * tool_com) + w_tool.cross(w_tool.cross(tool_mass * tool_com));

		// remove initial base at first counter (robot is stationary)
		if (first_loop) {
			init_force = sensed_force_moment_local_frame.head(3);
			first_loop = false;
		}
		sensed_force_moment_local_frame.head(3) -= init_force;

		// update forces for posori task
		if (state != CLEANING && state != CLEANING_EXIT_TRANSITION) {
			posori_task->updateSensedForceAndMoment(sensed_force_moment_local_frame.head(3), sensed_force_moment_local_frame.tail(3));
			sensed_force_moment_world_frame.head(3) = R_world_sensor * sensed_force_moment_local_frame.head(3);
			sensed_force_moment_world_frame.tail(3) = R_world_sensor * sensed_force_moment_local_frame.tail(3);
		}

		if (state == INIT) {
			robot_ready = 0;
			joint_task->_desired_position = q_init;
			joint_task->updateTaskModel(MatrixXd::Identity(dof, dof));
			joint_task->computeTorques(joint_task_torques);
			command_torques = joint_task_torques + coriolis;

			if (integrate_flag && (joint_task->_desired_position - joint_task->_current_position).norm() < 2 * tol) {
				joint_task->_ki = 150;
				joint_task->_integrated_position_error.setZero();
				integrate_flag = 0;
			}

			if ((joint_task->_desired_position - joint_task->_current_position).norm() < tol) {
				robot_ready = 1;
				if (haptic_ready) {
					posori_task->reInitializeTask();
					joint_task->reInitializeTask();

					joint_task->_ki = 0.0;
					joint_task->_use_velocity_saturation_flag = false;

					// state = FREE;
					state = FREE;
				}
			}
		} else if (state == FREE) {
			// // dual proxy
			// posori_task->_sigma_force = sigma_force;
			// posori_task->_sigma_position = sigma_motion;

			Vector3d robot_position = posori_task->_current_position;
			Vector3d motion_proxy = robot_position + sigma_motion * (robot_proxy - robot_position);

			Vector3d desired_force = k_vir * sigma_force * (robot_proxy - robot_position);
			Vector3d desired_force_diff = desired_force - prev_desired_force;
			if (desired_force_diff.norm() > max_force_diff) {
				desired_force = prev_desired_force + desired_force_diff*max_force_diff/desired_force_diff.norm();
			}
			if (desired_force.norm() > max_force) {
				desired_force *= max_force/desired_force.norm();
			}

			// control
			posori_task->_desired_position = robot_proxy;
            posori_task->_desired_orientation = robot_proxy_rot;  
			// posori_task->_desired_force = desired_force;
            // posori_task->_desired_orientation = robot_proxy_rot * R_accumulator.transpose();  
			// std::cout << robot_proxy_rot << "\n\n";

			try	{
				posori_task->computeTorques(posori_task_torques);
			}
			catch (exception e) {
				cout << "control cycle: " << controller_counter << endl;
				cout << "error in the torque computation of posori_task:" << endl;
				cerr << e.what() << endl;
				cout << "setting torques to zero for this control cycle" << endl;
				cout << endl;
				// posori_task_torques.setZero(); // set task torques to zero, TODO: test this
			}
			joint_task->computeTorques(joint_task_torques);

			command_torques = posori_task_torques + joint_task_torques + coriolis;

			// push forward
			prev_desired_force = desired_force;
			prev_force_space_dimension = force_space_dimension;

		} else if (state == ORIENTATION) {
			posori_task->_desired_position = robot_proxy;
			posori_task->_desired_orientation = robot_proxy_rot;

			try	{
				posori_task->computeTorques(posori_task_torques);
			}
			catch (exception e) {
				cout << "control cycle: " << controller_counter << endl;
				cout << "error in the torque computation of posori_task:" << endl;
				cerr << e.what() << endl;
				cout << "setting torques to zero for this control cycle" << endl;
				cout << endl;
				// posori_task_torques.setZero(); // set task torques to zero, TODO: test this
			}
			joint_task->computeTorques(joint_task_torques);
			command_torques = posori_task_torques + joint_task_torques + coriolis;

			// push forward 
			// prev_desired_force = desired_force;  // not doing force control in the cleaning task for now 
			prev_force_space_dimension = force_space_dimension;

		} else if (state == CLEANING) {
			/*
				The position is nomi4nally held constant, and the tool rotates about the last orientation from rotary knob encoder.
				If the position needs to change, then the macropad input will trigger relative position motion from the axis.
			*/
			double max_angle = 90;
			double upper_joint_limit = 125 * M_PI / 180;
			double lower_joint_limit = - 125 * M_PI / 180;
			double cleaning_sf = 0.5;

			if (first_cleaning_loop) {
				zero_angle = robot->_q(6);  // starting angle from last joint 
				des_delta_angle = 0;
				prev_angle = zero_angle;
				double pos_dist = abs(max_angle - zero_angle);
				double neg_dist = abs(zero_angle - (-max_angle));
				if (pos_dist < neg_dist) {
					max_amp = pos_dist;
				} else {
					max_amp = neg_dist;
				}
				starting_ori = robot_rot;
				redis_client.set(ROBOT_CLEANING_DIRECTION_KEY, "0");
				rot_direction = 0;
				first_cleaning_loop = false;
			}
			
			if (rot_direction != -1 && rot_direction != 0 && rot_direction != 1) {
				std::cout << "Invalid rotation direction - setting to 0\n";
				rot_direction = 0;
			}

			if (rot_direction != 0) {
				des_delta_angle = prev_angle + rot_direction * (M_PI / 180) * 1;  // 5 degree difference
				if (des_delta_angle + zero_angle > upper_joint_limit || des_delta_angle + zero_angle < lower_joint_limit) {
					des_delta_angle = prev_angle;
				}
				redis_client.set(ROBOT_CLEANING_DIRECTION_KEY, "0");
			}

			AngleAxisd rot_increment = AngleAxisd(des_delta_angle, Vector3d(0, 0, 1));  // rotate about the local z frame 
			posori_task->_desired_position = robot_proxy;
			posori_task->_desired_orientation = starting_ori * rot_increment.toRotationMatrix();  // euler frame rotation 

			// posori_task->computeTorques(posori_task_torques);
			try	{
				posori_task->computeTorques(posori_task_torques);
			}
			catch (exception e) {
				cout << "control cycle: " << controller_counter << endl;
				cout << "error in the torque computation of posori_task:" << endl;
				cerr << e.what() << endl;
				cout << "setting torques to zero for this control cycle" << endl;
				cout << endl;
				// posori_task_torques.setZero(); // set task torques to zero, TODO: test this
			}
			joint_task->computeTorques(joint_task_torques);
			command_torques = posori_task_torques + joint_task_torques + coriolis;

			// push forward 
			// prev_desired_force = desired_force;  // not doing force control in the cleaning task for now 
			prev_force_space_dimension = force_space_dimension;
			prev_angle = des_delta_angle;

		} else if (state == CLEANING_EXIT_TRANSITION) {
			/*
				Reversal of angle back to the starting angle, and then transitions to free control
			*/
			rot_direction = - sgn(des_delta_angle);
			des_delta_angle = prev_angle + rot_direction * (M_PI / 180) * 0.05;  // slower return 

			AngleAxisd rot_increment = AngleAxisd(des_delta_angle, Vector3d(0, 0, 1));  // rotate about the local z frame 
			posori_task->_desired_position = robot_proxy;
			posori_task->_desired_orientation = starting_ori * rot_increment.toRotationMatrix();  // euler frame rotation 

			// posori_task->computeTorques(posori_task_torques);
			try	{
				posori_task->computeTorques(posori_task_torques);
			}
			catch (exception e) {
				cout << "control cycle: " << controller_counter << endl;
				cout << "error in the torque computation of posori_task:" << endl;
				cerr << e.what() << endl;
				cout << "setting torques to zero for this control cycle" << endl;
				cout << endl;
				// posori_task_torques.setZero(); // set task torques to zero, TODO: test this
			}
			joint_task->computeTorques(joint_task_torques);
			command_torques = posori_task_torques + joint_task_torques + coriolis;

			// push forward 
			// prev_desired_force = desired_force;  // not doing force control in the cleaning task for now 
			prev_force_space_dimension = force_space_dimension;
			prev_angle = des_delta_angle;

			// exit conditions
			if (abs(des_delta_angle) < 1e-10) {
				std::cout << "From cleaning exit to free\n";
				state = FREE;
				redis_client.set(ROBOT_STATE_KEY, to_string(state));
			}
		} else if (state == RECENTER) {
			/*
				Robot stays in place while haptics device is recentered
			*/
			try	{
				posori_task->computeTorques(posori_task_torques);
			}
			catch (exception e) {
				cout << "control cycle: " << controller_counter << endl;
				cout << "error in the torque computation of posori_task:" << endl;
				cerr << e.what() << endl;
				cout << "setting torques to zero for this control cycle" << endl;
				cout << endl;
				// posori_task_torques.setZero(); // set task torques to zero, TODO: test this
			}
			joint_task->computeTorques(joint_task_torques);
			command_torques = posori_task_torques + joint_task_torques + coriolis;
		}

		else if (state == VS) {	
			if (first_vs_loop) {
				posori2_task->reInitializeTask();
				first_vs_loop = false;
			}
			 // get Rotation between end_effector frame and camera frame.
            double angle,step_size = 0.1;
            Affine3d T_EE;
            Vector3d x_new = Vector3d(0,0,0);
            Vector3d dir_vector,unnormalized_dir_vector;
            int object_in_frame = -1;
			if (redis_client.get(CAMERA_TRIGGER_KEY) == "1") {
                unnormalized_dir_vector = redis_client.getEigenMatrixJSON(DIRECTION_KEY);
                dir_vector = unnormalized_dir_vector.normalized();
                angle = stod(redis_client.get(ANGLE_KEY));
                object_in_frame = stoi(redis_client.get(OBJECT_IN_FRAME_KEY));
                redis_client.set(CAMERA_TRIGGER_KEY,"0" );
                if (object_in_frame == 0) {
                    cout << "Object out of frame. Switching to IDLE state.\n";
                    joint_task->reInitializeTask();
					// set robot is ready.
					robot_ready = 1;
					state = IDLE;
                    continue;
                }

                x_new = step_size*M*dir_vector;
                robot->transform(T_EE,"eef_2",x_new);
                posori2_task->_desired_position = T_EE.translation();
                // check has whether the position is reached or not.
                if (dir_vector.norm() <= 0.001){
                    positon_reached = true;
                }
            }

			if (positon_reached && !angle_reached) {
                if (abs(angle) <= 0.1) {
                    joint_task->reInitializeTask();

                    // cout << "Angle is aligned.\n";
                    joint_task->updateTaskModel((MatrixXd::Identity(dof,dof)));
                    joint_task->computeTorques(joint_task_torques);
                    command_torques = joint_task_torques + coriolis;
                    angle_reached = true;
                    
                }
                else {
					Matrix3d R_cam;
					R_cam = AngleAxisd(0,Vector3d::UnitX())* AngleAxisd(0,Vector3d::UnitY()) * AngleAxisd(0,Vector3d::UnitZ());
					robot->transform(T_EE,"eef_2",Vector3d(0,0,0),M*R_cam);
					// posori2_task->_desired_position = x_new;
					posori2_task->_desired_orientation = T_EE.rotation();
					joint_task->updateTaskModel(posori2_task->_N);
					joint_task->computeTorques(joint_task_torques);
					posori2_task->computeTorques(posori2_task_torques);
					command_torques = posori2_task_torques + joint_task_torques + coriolis;


                    // joint_task->reInitializeTask();
                    // joint_task->_use_velocity_saturation_flag = true;
                    // joint_task->_saturation_velocity << M_PI/6,M_PI/6,M_PI/6,M_PI/4,M_PI/3,M_PI/3,M_PI/8;

                    // VectorXd joint_position = robot->_q;
                    // joint_position(6) = joint_position(6) + angle;
                    // joint_task->updateTaskModel((MatrixXd::Identity(dof,dof)));
                    // joint_task->_desired_position = joint_position;
                    // joint_task->computeTorques(joint_task_torques);
                    // command_torques = joint_task_torques + coriolis;
                }
                // joint_task->reInitializeTask();
                // redis_client.set(STATE_TRANSITION_KEY,"1");
                // cout << "Switching to IDLE state as angle is aligned.\n";
                // continue;
            }
            else {
                joint_task->updateTaskModel(posori2_task->_N);
                joint_task->computeTorques(joint_task_torques);
                posori2_task->computeTorques(posori2_task_torques);
                command_torques = posori2_task_torques + coriolis;
            }

			// joint_task->updateTaskModel((MatrixXd::Identity(dof,dof)));
            // joint_task->computeTorques(joint_task_torques);
            // command_torques = joint_task_torques + coriolis;

			if (angle_reached) {
                if (first_time_set) {    
                    Vector3d pos = redis_client.getEigenMatrixJSON(POSE_FROM_CAMERA_KEY);
                    robot->transform(T,"eef_2",pos);
                    first_time_set = false;
                    cout << "Target set\n";
                }
                posori2_task->_desired_position = T.translation();
                posori2_task->_desired_orientation = T.rotation();

                if ((posori2_task->_desired_position - posori2_task->_current_position).norm() < 0.01) {
					state = IDLE;
					redis_client.set(ROBOT_STATE_KEY, to_string(state));
                    cout << "Reached position. Switching to FREE state.\n";
                    // reset the flags to original.
                    positon_reached = false;
                    angle_reached = false;
                    first_time_set = true;
					first_vs_loop = true;
					robot_ready = 1;
					// set the control torque.
					// joint_task->updateTaskModel(posori2_task->_N);
					// joint_task->computeTorques(joint_task_torques);
					// posori2_task->computeTorques(posori2_task_torques);
					// command_torques = posori2_task_torques + joint_task_torques + coriolis;
					// reInitialize the controllers.
					joint_task->reInitializeTask();
                    continue;
                }
                joint_task->updateTaskModel(posori2_task->_N);
                joint_task->computeTorques(joint_task_torques);
                posori2_task->computeTorques(posori2_task_torques);
                command_torques = posori2_task_torques + joint_task_torques + coriolis;

                // joint_task->updateTaskModel((MatrixXd::Identity(dof,dof)));
                // joint_task->computeTorques(joint_task_torques);
                // command_torques = joint_task_torques + coriolis;
            } 

		} else if (state == IDLE) {
				// joint_task->_use_velocity_saturation_flag = true;
                // joint_task->_saturation_velocity << M_PI/6,M_PI/6,M_PI/6,M_PI/4,M_PI/4,M_PI/4,M_PI/8;
				// joint_task->_desired_position << -0.390553+0.02,-1.34995+0.02,0.758122-0.02,-2.49509,-2.29837,1.86602,-1.06781;
				joint_task->updateTaskModel((MatrixXd::Identity(dof,dof)));
				joint_task->computeTorques(joint_task_torques);
				command_torques = joint_task_torques + coriolis;
		}

		// write control torques and dual proxy variables
		robot->position(haptic_proxy, control_link_name, control_pos_in_link);
		redis_client.executeWriteCallback(0);

		// particle filter
		pfilter_motion_control_buffer.push(sigma_motion * (robot_proxy - posori_task->_current_position) * freq_ratio_filter_control);
		pfilter_force_control_buffer.push(sigma_force * (robot_proxy - posori_task->_current_position) * freq_ratio_filter_control);
		// pfilter_motion_control_buffer.push(sigma_motion * posori_task->_Lambda_modified.block<3,3>(0,0) * posori_task->_linear_motion_control * freq_ratio_filter_control);
		// pfilter_force_control_buffer.push(sigma_force * posori_task->_linear_force_control * freq_ratio_filter_control);

		pfilter_sensed_velocity_buffer.push(posori_task->_current_velocity * freq_ratio_filter_control);
		pfilter_sensed_force_buffer.push(sensed_force_moment_world_frame.head(3) * freq_ratio_filter_control);

		motion_control_pfilter += pfilter_motion_control_buffer.back();
		force_control_pfilter += pfilter_force_control_buffer.back();
		measured_velocity_pfilter += pfilter_sensed_velocity_buffer.back();
		measured_force_pfilter += pfilter_sensed_force_buffer.back();

		if (pfilter_motion_control_buffer.size() > 1/freq_ratio_filter_control) {
			motion_control_pfilter -= pfilter_motion_control_buffer.front();
			force_control_pfilter -= pfilter_force_control_buffer.front();
			measured_velocity_pfilter -= pfilter_sensed_velocity_buffer.front();
			measured_force_pfilter -= pfilter_sensed_force_buffer.front();

			pfilter_motion_control_buffer.pop();
			pfilter_force_control_buffer.pop();
			pfilter_sensed_velocity_buffer.pop();
			pfilter_sensed_force_buffer.pop();			
		}

		// update logger values
		Vector3d ee_vel = Vector3d::Zero();
		robot->linearVelocity(ee_vel, control_link_name, control_pos_in_link);
		
		log_robot_ee_position = haptic_proxy;
		log_robot_ee_velocity = ee_vel;
		log_robot_proxy_position = robot_proxy;
		log_joint_angles = robot->_q;
		log_joint_velocities = robot->_dq;
		log_joint_command_torques = command_torques;
		log_sensed_force_moments = sensed_force_moment_world_frame;
        log_desired_force = posori_task->_desired_force;
	
		controller_counter++;
	}

	// wait for particle filter thread
	particle_filter_thread.join();

	// stop logger
	logger->stop();

	//// Send zero force/torque to robot ////
	command_torques.setZero();
	redis_client.setEigenMatrixJSON(ROBOT_COMMAND_TORQUES_KEY, command_torques);
	redis_client.set(CONTROLLER_RUNNING_KEY, "0");

	double end_time = timer.elapsedTime();
	std::cout << "\n";
	std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
	std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";
}



void particle_filter() {
	// start redis client for particles
	auto redis_client_particles = RedisClient();
	redis_client_particles.connect();

	unsigned long long pf_counter = 0;

	// create particle filter
	auto pfilter = new Sai2Primitives::ForceSpaceParticleFilter(n_particles);

	pfilter->_mean_scatter = 0.0;
	pfilter->_std_scatter = 0.025;

	pfilter->_alpha_add = 0.3;
	pfilter->_alpha_remove = 0.05;

	pfilter->_F_low = 2.0;
	pfilter->_F_high = 6.0;
	pfilter->_v_low = 0.01;
	pfilter->_v_high = 0.07;

	pfilter->_F_low_add = 5.0;
	pfilter->_F_high_add = 10.0;
	pfilter->_v_low_add = 0.02;
	pfilter->_v_high_add = 0.1;

	Vector3d evals = Vector3d::Zero();
	Matrix3d evecs = Matrix3d::Identity();

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(pfilter_freq); //Compiler en mode release
	double current_time = 0;
	double prev_time = 0;
	// double dt = 0;
	bool fTimerDidSleep = true;
	double start_time = timer.elapsedTime(); //secs

	while(runloop) {
		timer.waitForNextLoop();

		pfilter->update(motion_control_pfilter, force_control_pfilter, measured_velocity_pfilter, measured_force_pfilter);
//		sigma_force = pfilter->getSigmaForce(); // disable to only control motion
		sigma_motion = Matrix3d::Identity() - sigma_force;
		force_space_dimension = pfilter->_force_space_dimension;

		// for(int i=0 ; i<n_particles ; i++)
		// {
		// 	particle_positions_to_redis.col(i) = pfilter->_particles[i];
		// }
		// redis_client_particles.setEigenMatrixJSON(PARTICLE_POSITIONS_KEY, particle_positions_to_redis);

		pf_counter++;
	}

	double end_time = timer.elapsedTime();
	std::cout << "\n";
	std::cout << "Particle Filter Loop run time  : " << end_time << " seconds\n";
	std::cout << "Particle Filter Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Particle Filter Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";
}
