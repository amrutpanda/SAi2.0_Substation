#include <iostream>
#include <Sai2Model.h>
#include <tasks/JointTask.h>
#include <tasks/PositionTask.h>
#include <tasks/PosOriTask.h>
#include <redis/RedisClient.h>
#include <timer/LoopTimer.h>
#include <signal.h>
#include <string>
#include <vector>


enum State {
    INIT = 0, // tool change position. not needed for this application.
    LOOK,
    POSE,
    MOVE, // move towards the desired position and orientation.
    IDLE 
};

std::string CONTROLLER_RUNNING_KEY;
std::string ROBOT_COMMAND_TORQUES_KEY;
std::string JOINT_ANGLES_KEY;
std::string JOINT_VELOCITIES_KEY;
std::string MASSMATRIX_KEY;
std::string CORIOLIS_KEY;
std::string ROBOT_SENSED_FORCE_KEY;
std::string ROBOT_END_EFFECTOR_POSE;
std::string ROBOT_END_EFFECTOR_ROTATION_MATRIX;
std::string STATE_TRANSITION_KEY;
std::string POSE_FROM_CAMERA_KEY;

bool runloop = false;
void sighandler(int) {runloop = false;}

const string robot_file = "../model/panda/panda_arm.urdf";
unsigned long long controller_counter = 0;
double tol = 1e-2;
double error = 1e-2;

const string control_link_name = "end_effector";
const VectorXd control_pos_in_link = Vector3d(0,0,0);
const Vector3d pos_in_ee_link = Vector3d(0, 0, 0);

RedisClient redis_client;

const bool flag_simulation = false;

int main(int argc, char const *argv[])
{
    if (!flag_simulation) {
		// ROBOT_COMMAND_TORQUES_KEY = "sai2::FrankaPanda::Clyde::actuators::fgc";
		// JOINT_ANGLES_KEY  = "sai2::FrankaPanda::Clyde::sensors::q";
		// JOINT_VELOCITIES_KEY = "sai2::FrankaPanda::Clyde::sensors::dq";
		// MASSMATRIX_KEY = "sai2::FrankaPanda::Clyde::sensors::model::massmatrix";
		// CORIOLIS_KEY = "sai2::FrankaPanda::Clyde::sensors::model::coriolis";
		// ROBOT_SENSED_FORCE_KEY = "sai2::ATIGamma_Sensor::Clyde::force_torque";

        ROBOT_COMMAND_TORQUES_KEY = "sai2::FrankaPanda::Bonnie::actuators::fgc";
		JOINT_ANGLES_KEY  = "sai2::FrankaPanda::Bonnie::sensors::q";
		JOINT_VELOCITIES_KEY = "sai2::FrankaPanda::Bonnie::sensors::dq";
		MASSMATRIX_KEY = "sai2::FrankaPanda::Bonnie::sensors::model::massmatrix";
		CORIOLIS_KEY = "sai2::FrankaPanda::Bonnie::sensors::model::coriolis";
		ROBOT_SENSED_FORCE_KEY = "sai2::ATIGamma_Sensor::Bonnie::force_torque";
        ROBOT_END_EFFECTOR_POSE = "sai2::FrankaPanda::Bonnie::end_effector::pose";
        ROBOT_END_EFFECTOR_ROTATION_MATRIX = "sai2::Frankapanda::Bonnie::end_effector::rotation_matrix";
        STATE_TRANSITION_KEY = "sai2::substation::state_transition";
        POSE_FROM_CAMERA_KEY = "sai2::substation::camera::pose";

	}
    signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

    // start redis client local
	redis_client = RedisClient();
	redis_client.connect();
    redis_client.set(CONTROLLER_RUNNING_KEY,"0");

    Affine3d T_world_robot = Affine3d::Identity();
	T_world_robot.translation() = Vector3d(0, 0, 0);
	auto robot = new Sai2Model::Sai2Model(robot_file, false, T_world_robot);

	robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
	robot->updateModel();

    int dof = robot->dof();
    VectorXd command_torques = VectorXd::Zero(dof);
    VectorXd coriolis = VectorXd::Zero(dof);
    MatrixXd mass_from_robot = MatrixXd::Zero(dof,dof);
    Vector3d ee_pose  = Vector3d::Zero();
    Matrix3d R = Matrix3d::Identity();

     R << -0.726272,0.686320,0.038636,0.687345,0.724304,0.054231,0.009236,0.065943,-0.997781;

    auto joint_task = new Sai2Primitives::JointTask(robot);
    VectorXd joint_task_torques = VectorXd::Zero(dof);
    joint_task->_use_velocity_saturation_flag = true;
    joint_task->_saturation_velocity = (M_PI/10)*VectorXd::Ones(dof);

    joint_task->_kp = 50.0;
    joint_task->_kv = 15.0;
    joint_task->_ki = 0.0;
    
    VectorXd q_init (dof);
    VectorXd q_look (dof);
    // q_init << 0.622035,0.202844,-0.748116,-2.34133,0.161845,2.42087,0.440028;
    // q_init << 0.162313,-1.09008,-0.153488,-2.61457,-0.133069,1.54065,0.778681; // -ve x direction pose U shape pose.
    // q_init << 0.104063,-0.766375,0.0456267,-2.19672,0.00163179,1.43858,-2.22505; // -ve x direction 180 rotated end-effector
    q_init << -1.62959,-0.288922,0.152178,-1.47316,-0.0436687,1.1186,0.845332;
    // q_look << 0.0368602,-1.47652,0.0553265,-2.53061,-0.0289527,1.65088,0.845973;
    q_look << -1.62959,-0.288922,0.152178,-1.47316,-0.0436687,1.1186,0.845332;
    joint_task->_desired_position = q_init;
    int integrate_flag = 1;

    Vector3d robot_pos(0.45,-0.268, 0.46);
    robot_pos << 0.478792,-0.230273,0.506473; // later change in position for testing.
    Matrix3d robot_rot = Matrix3d::Identity();

    // position orientation task.
    auto posori_task = new Sai2Primitives::PosOriTask(robot,control_link_name,control_pos_in_link);
    Vector3d x_init = posori_task->_current_position;
    Matrix3d R_init = posori_task->_current_orientation;
    R_init = R;
    R_init << -0.726272,0.686320,0.038636,0.687345,0.724304,0.054231,0.009236,0.065943,-0.997781;

    VectorXd posori_task_torques = VectorXd::Zero(dof);
    posori_task->_use_velocity_saturation_flag = true;

    MatrixXd N_prec = MatrixXd::Identity(dof,dof);

    posori_task->_kp_pos = 200.0;
    posori_task->_kv_pos = 28.0;
    posori_task->_ki_pos = 10.0;
    posori_task->_kp_ori = 50.0;
    posori_task->_kv_ori = 10.8;
    posori_task->_ki_ori = 10.0;

    posori_task->_e_max = 3e-2;
    posori_task->_e_min = 3e-3;
    posori_task->_linear_saturation_velocity = 0.1;

    // state handling.
    int state = IDLE;

    // define camera pose and transformation variable.
    VectorXd pose_from_camera(3);
    MatrixXd rot_from_camera(3,3);

    // setup redis keys to be updated with the callback
	redis_client.createReadCallback(0);
	redis_client.createWriteCallback(0);

    // setup read and write callbacks;
    // Objects to read from redis
    redis_client.addEigenToReadCallback(0, JOINT_ANGLES_KEY, robot->_q);
    redis_client.addEigenToReadCallback(0, JOINT_VELOCITIES_KEY, robot->_dq);

    if(!flag_simulation)
    {
        redis_client.addEigenToReadCallback(0,CORIOLIS_KEY,coriolis);
        redis_client.addEigenToReadCallback(0,MASSMATRIX_KEY,mass_from_robot);
    }

    redis_client.addEigenToWriteCallback(0, ROBOT_COMMAND_TORQUES_KEY, command_torques);
    redis_client.addEigenToWriteCallback(0,ROBOT_END_EFFECTOR_POSE,ee_pose);
    redis_client.addEigenToWriteCallback(0,ROBOT_END_EFFECTOR_ROTATION_MATRIX,R);
    redis_client.addIntToReadCallback(0,STATE_TRANSITION_KEY,state);
    redis_client.addIntToWriteCallback(0,STATE_TRANSITION_KEY,state);

    // In the beginning change the state transition key to IDLE;
    // redis_client.set(STATE_TRANSITION_KEY,"4");
    runloop = true;
	redis_client.set(CONTROLLER_RUNNING_KEY,"1");

     // initialize timer and controller counter.
    unsigned long long controller_counter = 0;
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); 
	double current_time = 0;
	double prev_time = 0;
	bool fTimerDidSleep = true;
	double start_time = timer.elapsedTime(); //secs

    while (runloop)
    {
       // set the timer loop;
        timer.waitForNextLoop();
        current_time = timer.elapsedTime() - start_time;
        redis_client.executeReadCallback(0);
        // update kinematics after reading joint states;
        // robot->updateKinematics();
        robot->_M = mass_from_robot;
        robot->updateModel();
        // write the position of the robot end-effector.
        robot->position(ee_pose,"end_effector",pos_in_ee_link);
        robot->rotation(R,control_link_name);

        if (state == INIT) // not neede for this application.
        {
            joint_task->_desired_position = q_init;
            joint_task->updateTaskModel((MatrixXd::Identity(dof,dof)));
            joint_task->computeTorques(joint_task_torques);
            command_torques = joint_task_torques + coriolis;

            if (integrate_flag && (joint_task->_desired_position - joint_task->_current_position).norm() < 2 * tol) {
                    joint_task->_ki = 250;
                    joint_task->_integrated_position_error.setZero();
                    integrate_flag = 0;
                }
        }
        else if (state == LOOK)
        {
            joint_task->_desired_position = q_look;
            joint_task->updateTaskModel((MatrixXd::Identity(dof,dof)));
            joint_task->computeTorques(joint_task_torques);
            command_torques = joint_task_torques + coriolis;

            if (integrate_flag && (joint_task->_desired_position - joint_task->_current_position).norm() < 2 * tol) {
                    joint_task->_ki = 250;
                    joint_task->_integrated_position_error.setZero();
                    integrate_flag = 0;
                }
            if ((joint_task->_desired_position - joint_task->_current_position).norm() < error/100)
                {
                    state = IDLE; // changing state to IDLE after reaching within the error limit.
                    cout << "\n Changing to LOOK state to IDLE\n";
                }
        
        }
        else if (state == MOVE)
        {
            posori_task->_desired_position = robot_pos;
            posori_task->_desired_orientation = R_init;
            posori_task->updateTaskModel((MatrixXd::Identity(dof,dof)));
            posori_task->computeTorques(posori_task_torques);
            // set some _ki value for accurate tracking.
            if (integrate_flag && (posori_task->_desired_position - posori_task->_current_position).norm() < 20* tol)
            {
                posori_task->_integrated_position_error.setZero();
                posori_task->_ki_pos = 250;
                integrate_flag = 0;
            }
            if ((posori_task->_desired_position - posori_task->_current_position).norm() < error)
            {
            //     command_torques = coriolis;
                state = IDLE; // change state to IDLE when the robot reaches within the error limit.
                cout << "\nchanging MOVE state to IDLE\n";
            }
            command_torques = posori_task_torques + coriolis;
        }
        else if (state == IDLE)
        {
            // cout << "In IDLE state\n";
            command_torques = coriolis;
            integrate_flag = 1;
        }   
        else if (state == POSE)
        {
            // robot_pos = (get pose from redis.)
            pose_from_camera = redis_client.getEigenMatrix(POSE_FROM_CAMERA_KEY);
            robot->position(robot_pos,"camera_link",pose_from_camera);
            cout<< "Received position from camera: "<< robot_pos<< "\n";
            // print the pose to verify the format.
            command_torques = coriolis;
            state = IDLE;
        }

        redis_client.executeWriteCallback(0);
        controller_counter++;

    }

    double end_time = timer.elapsedTime();
    command_torques.setZero();
    redis_client.set(CONTROLLER_RUNNING_KEY,"0");
    redis_client.set(STATE_TRANSITION_KEY,"4");
    cout << "Controller elapsed time : " << end_time - start_time << "\n";
    cout << "controller elapsed cycle: " << timer.elapsedCycles() << "\n";

    return 0;
}