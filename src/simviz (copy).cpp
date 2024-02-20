#include <GL/glew.h>
#include "Sai2Model.h"
#include "Sai2Graphics.h"
#include "Sai2Simulation.h"
#include <dynamics3d.h>
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include <GLFW/glfw3.h>  // must be loaded after loading opengl/glew
#include <signal.h>
#include "force_sensor/ForceSensorSim.h"

#include "tasks/JointTask.h"
#include "tasks/PosOriTask.h"

bool fSimulationRunning = false;
void sighandler(int){fSimulationRunning = false;}

using namespace std;
using namespace Eigen;

// specify urdf and robots 
const string world_file = "../model/panda/world.urdf";
const string robot_file = "../model/panda/panda_base.urdf";
const string panda_robot_file = "../model/panda/panda_arm.urdf";
const string robot_name = "panda_base";
const string camera_name = "camera_fixed";
const string base_link_name = "link0";
const string ee_link_name = "link7";

// redis keys:
#include "redis_keys.h"

RedisClient redis_client; 

// simulation thread
void simulation(Sai2Model::Sai2Model* robot, Sai2Model::Sai2Model* panda_robot, Simulation::Sai2Simulation* sim, ForceSensorSim* force_sensor);

// callback to print glfw errors
void glfwError(int error, const char* description);

// callback to print glew errors
bool glewInitialize();

// callback when a key is pressed
void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods);

// callback when a mouse button is pressed
void mouseClick(GLFWwindow* window, int button, int action, int mods);

// flags for scene camera movement
bool fTransXp = false;
bool fTransXn = false;
bool fTransYp = false;
bool fTransYn = false;
bool fTransZp = false;
bool fTransZn = false;
bool fRotPanTilt = false;
bool fRobotLinkSelect = false;

int main() {
	cout << "Loading URDF world model file: " << world_file << endl;

	// start redis client
	redis_client = RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load graphics scene
	auto graphics = new Sai2Graphics::Sai2Graphics(world_file, true);
	Eigen::Vector3d camera_pos, camera_lookat, camera_vertical;
	graphics->getCameraPose(camera_name, camera_pos, camera_vertical, camera_lookat);
	graphics->_world->setBackgroundColor(197.0/255, 221.0/255, 237.0/255);
	// graphics->_world->setBackgroundColor(66.0/255, 135.0/255, 245.0/255);  // set blue background 	
	// graphics->_world->setBackgroundColor(1, 1, 1);  // set blue background 	
	// graphics->showLinkFrame(true, robot_name, ee_link_name, 0.15);  // can add frames for different links
	// graphics->showLinkFrame(true, robot_name, base_link_name, 0.25);  // can add frames for different links
	graphics->getCamera(camera_name)->setClippingPlanes(0.1, 50);
	// graphics->_world->enableLightSourceRendering(true);
	// graphics->_world->setUseShadowCasting(true);

	// load robots
	auto robot = new Sai2Model::Sai2Model(robot_file, false);
	// first 3 are the cherry picker joints 
	// robot->_q(0) = -50 * M_PI / 180;  
	// robot->_q(1) = -100 * M_PI / 180;
	// robot->_q(2) = 100 * M_PI / 180;
	robot->_q(0) = -130 * M_PI / 180;  
	robot->_q(1) = -280 * M_PI / 180;
	robot->_q(2) = 280 * M_PI / 180;
	// robot->_q(3) = 50 * M_PI / 180;
	VectorXd q_init(7);
	q_init << 80, -45, 0, -125, 0, 80, 0;
	q_init *= M_PI / 180;
	robot->_q.tail(7) = q_init;
	robot->_dq.setZero();
	robot->updateModel();

	auto panda_robot = new Sai2Model::Sai2Model(panda_robot_file, false);
	panda_robot->_q = q_init;
	panda_robot->_dq.setZero();
	panda_robot->updateModel();

	// load simulation world
	auto sim = new Simulation::Sai2Simulation(world_file, false);
	sim->setJointPositions(robot_name, robot->_q);
	sim->setJointVelocities(robot_name, robot->_dq);

    // set co-efficient of restition to zero for force control
    sim->setCollisionRestitution(0.0);

    // set co-efficient of friction also to zero for now as this causes jitter
    sim->setCoeffFrictionStatic(0.0);
    sim->setCoeffFrictionDynamic(0.0);

	// Add force sensor to the end-effector
	Affine3d sensor_transform_in_link = Affine3d::Identity();
	const Vector3d sensor_pos_in_link = Eigen::Vector3d(0.0,0.0,0.0);
	const Vector3d pos_in_link = Vector3d(0.0,0.0,0.0);
	sensor_transform_in_link.translation() = sensor_pos_in_link;
	auto force_sensor = new ForceSensorSim(robot_name, ee_link_name, sensor_transform_in_link, robot);
	// force_sensor->enableFilter(0.01);

	/*------- Set up visualization -------*/
	// set up error callback
	glfwSetErrorCallback(glfwError);

	// initialize GLFW
	glfwInit();

	// retrieve resolution of computer display and position window accordingly
	GLFWmonitor* primary = glfwGetPrimaryMonitor();
	const GLFWvidmode* mode = glfwGetVideoMode(primary);

	// information about computer screen and GLUT display window
	int screenW = mode->width;
	int screenH = mode->height;
	int windowW = 0.8 * screenW;
	int windowH = 0.8 * screenH;
	int windowPosY = (screenH - windowH) / 2;
	int windowPosX = windowPosY;

	// create window and make it current
	glfwWindowHint(GLFW_VISIBLE, 0);
	GLFWwindow* window = glfwCreateWindow(windowW, windowH, "DEWA Demo", NULL, NULL);
	glfwSetWindowPos(window, windowPosX, windowPosY);
	glfwShowWindow(window);
	glfwMakeContextCurrent(window);
	glfwSwapInterval(1);

	// set callbacks
	glfwSetKeyCallback(window, keySelect);
	glfwSetMouseButtonCallback(window, mouseClick);

	// cache variables
	double last_cursorx, last_cursory;

	// init redis client values 
	redis_client.set(CONTROLLER_RUNNING_KEY, "0");  
	redis_client.setEigenMatrixJSON(JOINT_ANGLES_KEY, panda_robot->_q); 
	redis_client.setEigenMatrixJSON(JOINT_VELOCITIES_KEY, panda_robot->_dq); 
	redis_client.setEigenMatrixJSON(ROBOT_SENSED_FORCE_KEY, VectorXd::Zero(6));
	redis_client.set(FORWARD_KEY, "0");
	redis_client.set(BACKWARD_KEY, "0");
	redis_client.set(LEFT_KEY, "0");
	redis_client.set(RIGHT_KEY, "0");
	redis_client.set(UP_KEY, "0");
	redis_client.set(DOWN_KEY, "0");
	redis_client.set(CW_KEY, "0");
	redis_client.set(CCW_KEY, "0");

	// start simulation thread
	thread sim_thread(simulation, robot, panda_robot, sim, force_sensor);

	// initialize glew
	glewInitialize();

	// while window is open:
	while (!glfwWindowShouldClose(window) && fSimulationRunning)
	{
		// update graphics. this automatically waits for the correct amount of time
		int width, height;
		glfwGetFramebufferSize(window, &width, &height);
		graphics->updateGraphics(robot_name, robot);
		graphics->render(camera_name, width, height);

		// swap buffers
		glfwSwapBuffers(window);

		// wait until all GL commands are completed
		glFinish();

		// check for any OpenGL errors
		GLenum err;
		err = glGetError();
		assert(err == GL_NO_ERROR);

		// poll for events
		glfwPollEvents();

		// move scene camera as required
		// graphics->getCameraPose(camera_name, camera_pos, camera_vertical, camera_lookat);
		Eigen::Vector3d cam_depth_axis;
		cam_depth_axis = camera_lookat - camera_pos;
		cam_depth_axis.normalize();
		Eigen::Vector3d cam_up_axis;
		// cam_up_axis = camera_vertical;
		// cam_up_axis.normalize();
		cam_up_axis << 0.0, 0.0, 1.0; //TODO: there might be a better way to do this
		Eigen::Vector3d cam_roll_axis = (camera_lookat - camera_pos).cross(cam_up_axis);
		cam_roll_axis.normalize();
		Eigen::Vector3d cam_lookat_axis = camera_lookat;
		cam_lookat_axis.normalize();

		if (fTransXp) {
			camera_pos = camera_pos + 0.05*cam_roll_axis;
			camera_lookat = camera_lookat + 0.05*cam_roll_axis;
		}
		if (fTransXn) {
			camera_pos = camera_pos - 0.05*cam_roll_axis;
			camera_lookat = camera_lookat - 0.05*cam_roll_axis;
		}
		if (fTransYp) {
			// camera_pos = camera_pos + 0.05*cam_lookat_axis;
			camera_pos = camera_pos + 0.05*cam_up_axis;
			camera_lookat = camera_lookat + 0.05*cam_up_axis;
		}
		if (fTransYn) {
			// camera_pos = camera_pos - 0.05*cam_lookat_axis;
			camera_pos = camera_pos - 0.05*cam_up_axis;
			camera_lookat = camera_lookat - 0.05*cam_up_axis;
		}
		if (fTransZp) {
			camera_pos = camera_pos + 0.1*cam_depth_axis;
			camera_lookat = camera_lookat + 0.1*cam_depth_axis;
		}	    
		if (fTransZn) {
			camera_pos = camera_pos - 0.1*cam_depth_axis;
			camera_lookat = camera_lookat - 0.1*cam_depth_axis;
		}
		if (fRotPanTilt) {
			// get current cursor position
			double cursorx, cursory;
			glfwGetCursorPos(window, &cursorx, &cursory);
			//TODO: might need to re-scale from screen units to physical units
			double compass = 0.006*(cursorx - last_cursorx);
			double azimuth = 0.006*(cursory - last_cursory);
			double radius = (camera_pos - camera_lookat).norm();
			Eigen::Matrix3d m_tilt; m_tilt = Eigen::AngleAxisd(azimuth, -cam_roll_axis);
			camera_pos = camera_lookat + m_tilt*(camera_pos - camera_lookat);
			Eigen::Matrix3d m_pan; m_pan = Eigen::AngleAxisd(compass, -cam_up_axis);
			camera_pos = camera_lookat + m_pan*(camera_pos - camera_lookat);
		}
		graphics->setCameraPose(camera_name, camera_pos, cam_up_axis, camera_lookat);
		glfwGetCursorPos(window, &last_cursorx, &last_cursory);
		
	}

	// wait for simulation to finish
	fSimulationRunning = false;
	sim_thread.join();

	// destroy context
	glfwDestroyWindow(window);

	// terminate
	glfwTerminate();

	return 0;
}

//------------------------------------------------------------------------------

void simulation(Sai2Model::Sai2Model* robot, Sai2Model::Sai2Model* panda_robot, Simulation::Sai2Simulation* sim, ForceSensorSim* force_sensor)
{
	// prepare simulation
	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);
	VectorXd base_command_torques = VectorXd::Zero(4);
	VectorXd panda_command_torques = VectorXd::Zero(7);
	redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, panda_command_torques);
	redis_client.setEigenMatrixJSON(BASE_TORQUES_COMMANDED_KEY, base_command_torques);
	redis_client.setEigenMatrixJSON(ROBOT_SENSED_FORCE_KEY, VectorXd::Zero(6));
	VectorXd g = VectorXd::Zero(dof);
	string controller_status = "0";
	string prev_controller_status = "0";
	redis_client.set(CONTROLLER_RUNNING_KEY, controller_status);
	VectorXd arm_q(7);
	VectorXd arm_dq(7);
	double base_heading = 0;
	double world_offset = 0;

	// base motion parameters
	double linear_step = 0.005; 
	double angular_step = (2.) * (M_PI / 180);
	int base_forward = 0;
	int base_backward = 0;
	int base_left = 0;
	int base_right = 0;
	int base_up = 0;
	int base_down = 0;
	Vector3d base_pos_desired;
	Matrix3d base_ori_desired;
	string link_name = "link0";
	Vector3d pos_in_link = Vector3d(0, 0, 0);
	robot->position(base_pos_desired, link_name, pos_in_link);
	robot->rotation(base_ori_desired, link_name);

	// base motion primitive 
	auto posori_task = new Sai2Primitives::PosOriTask(robot, link_name, pos_in_link);
	// posori_task->setDynamicDecouplingFull();
	posori_task->_kp_pos = 1e5;
	posori_task->_kv_pos = 200;
	posori_task->_kp_ori = 1e5;
	posori_task->_kv_ori = 200;
	posori_task->_desired_position = base_pos_desired;
	posori_task->_desired_orientation = base_ori_desired;
	VectorXd base_torques(dof);

	auto joint_task = new Sai2Primitives::JointTask(robot);
	joint_task->setDynamicDecouplingFull();
	joint_task->_kp = 200;
	joint_task->_kv = 20;
	joint_task->_desired_position = robot->_q;
	VectorXd base_posture_torques(dof);

	// panda arm posture holding if controller is off
	auto panda_joint_task = new Sai2Primitives::JointTask(panda_robot);
	panda_joint_task->setDynamicDecouplingFull();
	panda_joint_task->_kp = 200;
	panda_joint_task->_kv = 20;
	panda_joint_task->_desired_position = panda_robot->_q;
	VectorXd panda_posture_torques(7);

	// force sensor
	Vector3d sensed_force;
	Vector3d sensed_moment;
	VectorXd sensed_force_moment(6);
    
	// Callbacks
	redis_client.createReadCallback(0);
	redis_client.createWriteCallback(0);

	// Read
	redis_client.addEigenToReadCallback(0, JOINT_TORQUES_COMMANDED_KEY, panda_command_torques);
	redis_client.addIntToReadCallback(0, FORWARD_KEY, base_forward);
	redis_client.addIntToReadCallback(0, BACKWARD_KEY, base_backward);
	redis_client.addIntToReadCallback(0, LEFT_KEY, base_left);
	redis_client.addIntToReadCallback(0, RIGHT_KEY, base_right);
	redis_client.addIntToReadCallback(0, UP_KEY, base_up);
	redis_client.addIntToReadCallback(0, DOWN_KEY, base_down);
	// redis_client.addIntToReadCallback(0, CW_KEY, base_cw);
	// redis_client.addIntToReadCallback(0, CCW_KEY, base_ccw);
	redis_client.addStringToReadCallback(0, CONTROLLER_RUNNING_KEY, controller_status);

	// Write
	redis_client.addEigenToWriteCallback(0, JOINT_ANGLES_KEY, arm_q);
	redis_client.addEigenToWriteCallback(0, JOINT_VELOCITIES_KEY, arm_dq);
	redis_client.addEigenToWriteCallback(0, ROBOT_SENSED_FORCE_KEY, sensed_force_moment);
	redis_client.addDoubleToWriteCallback(0, BASE_HEADING_KEY, base_heading);
	redis_client.addEigenToWriteCallback(0, BASE_POS_KEY, posori_task->_current_position);

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(2000); 
	bool fTimerDidSleep = true;
	double start_time = timer.elapsedTime();
	double last_time = start_time;
	bool first_loop = true;

	// start simulation 
	fSimulationRunning = true;	
	while (fSimulationRunning) {
		fTimerDidSleep = timer.waitForNextLoop();

		// reset command torques 
		command_torques.setZero();

		// read 
		redis_client.executeReadCallback(0);

		// base_command_torques = redis_client.getEigenMatrixJSON(BASE_TORQUES_COMMANDED_KEY);
		// command_torques.head(4) = base_command_torques;

		// get controller status
		// controller_status = redis_client.get(CONTROLLER_RUNNING_KEY);

		// read arm torques from redis and apply to simulated robot
		if (controller_status == "1") {
			command_torques.tail(7) = panda_command_torques;
		}

		// apply gravity compensation and joint-space damping if command torques are zero
		robot->gravityVector(g);

		// update base heading 
		base_heading = robot->_q(3) + world_offset;

		// control base from keys (absolute coordinates)
		if (base_forward) {
			base_pos_desired(1) += linear_step;
			redis_client.set(FORWARD_KEY, "0");
		} else if (base_backward) {
			base_pos_desired(1) -= linear_step;
			redis_client.set(BACKWARD_KEY, "0");
		} else if (base_left) {
			base_pos_desired(0) -= linear_step;
			redis_client.set(LEFT_KEY, "0");
		} else if (base_right) {
			base_pos_desired(0) += linear_step;
			redis_client.set(RIGHT_KEY, "0");
		} else if (base_up) {
			base_pos_desired(2) += linear_step;
			redis_client.set(UP_KEY, "0");
		} else if (base_down) {
			base_pos_desired(2) -= linear_step;
			redis_client.set(DOWN_KEY, "0");
		} 

		// compute base control
		posori_task->_desired_position = base_pos_desired;
		posori_task->updateTaskModel(MatrixXd::Identity(dof, dof));
		posori_task->computeTorques(base_torques);
		joint_task->updateTaskModel(posori_task->_N);
		joint_task->computeTorques(base_posture_torques);
		base_posture_torques.tail(7).setZero();  // remove posture associated with robot arm 

		// compute panda torques if control is off 
		if (controller_status == "1" && prev_controller_status == "0" && first_loop == false) {
			panda_joint_task->reInitializeTask();
		}
		panda_joint_task->updateTaskModel(MatrixXd::Identity(7, 7));
		panda_joint_task->computeTorques(panda_posture_torques);

		// set joint torques
		if (controller_status == "1") {
			sim->setJointTorques(robot_name, g + command_torques + base_torques + base_posture_torques);
		} else {
			g.tail(7) += panda_posture_torques;
			sim->setJointTorques(robot_name, g + base_torques + base_posture_torques - 0 * robot->_M * (2 * robot->_dq));
		}

		// integrate forward
		double curr_time = timer.elapsedTime();
		double loop_dt = curr_time - last_time; 
		sim->integrate(loop_dt);

		// read joint positions, velocities, update model
		sim->getJointPositions(robot_name, robot->_q);
		sim->getJointVelocities(robot_name, robot->_dq);
		robot->updateModel();
		arm_q = robot->_q.tail(7);
		arm_dq = robot->_dq.tail(7);

		panda_robot->_q = arm_q;
		panda_robot->_dq = arm_dq;
		panda_robot->updateModel();

		base_heading -= world_offset;  // remove world offset to send to haptics device since haptics already has offset 

		// read end-effector task forces from the force sensor simulation
		force_sensor->update(sim);
		force_sensor->getForceLocalFrame(sensed_force);
		force_sensor->getMomentLocalFrame(sensed_moment);
		sensed_force_moment << -sensed_force, -sensed_moment * 0;

		redis_client.executeWriteCallback(0);

		prev_controller_status = controller_status;
		first_loop = false;

		// update last time
		last_time = curr_time;
	}

	double end_time = timer.elapsedTime();
	std::cout << "\n";
	std::cout << "Simulation Loop run time  : " << end_time << " seconds\n";
	std::cout << "Simulation Loop updates   : " << timer.elapsedCycles() << "\n";
	std::cout << "Simulation Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

}


//------------------------------------------------------------------------------

void glfwError(int error, const char* description) {
	cerr << "GLFW Error: " << description << endl;
	exit(1);
}

//------------------------------------------------------------------------------

bool glewInitialize() {
	bool ret = false;
	#ifdef GLEW_VERSION
	if (glewInit() != GLEW_OK) {
		cout << "Failed to initialize GLEW library" << endl;
		cout << glewGetErrorString(ret) << endl;
		glfwTerminate();
	} else {
		ret = true;
	}
	#endif
	return ret;
}

//------------------------------------------------------------------------------

void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	bool set = (action != GLFW_RELEASE);
	switch(key) {
		case GLFW_KEY_ESCAPE:
			// exit application
			fSimulationRunning = false;
			glfwSetWindowShouldClose(window, GL_TRUE);
			break;
		case GLFW_KEY_RIGHT:
			fTransXp = set;
			break;
		case GLFW_KEY_LEFT:
			fTransXn = set;
			break;
		case GLFW_KEY_UP:
			fTransYp = set;
			break;
		case GLFW_KEY_DOWN:
			fTransYn = set;
			break;
		case GLFW_KEY_Z:
			fTransZp = set;
			break;
		case GLFW_KEY_X:
			fTransZn = set;
			break;
		default:
			break;
	}
}

//------------------------------------------------------------------------------

void mouseClick(GLFWwindow* window, int button, int action, int mods) {
	bool set = (action != GLFW_RELEASE);
	//TODO: mouse interaction with robot
	switch (button) {
		// left click pans and tilts
		case GLFW_MOUSE_BUTTON_LEFT:
			fRotPanTilt = set;
			// NOTE: the code below is recommended but doesn't work well
			// if (fRotPanTilt) {
			// 	// lock cursor
			// 	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
			// } else {
			// 	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
			// }
			break;
		// if right click: don't handle. this is for menu selection
		case GLFW_MOUSE_BUTTON_RIGHT:
			fRobotLinkSelect = set;
			break;
		// if middle click: don't handle. doesn't work well on laptops
		case GLFW_MOUSE_BUTTON_MIDDLE:
			break;
		default:
			break;
	}
}