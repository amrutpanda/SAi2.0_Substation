#define USING_OTG 1
#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "tasks/JointTask.h"
#include "tasks/PosOriTask.h"
#include "filters/ButterworthFilter.h"
#include "perception/ForceSpaceParticleFilter.h"
#include <iostream>
#include <string>
#include <random>
#include <queue>

// enum construct for states.
enum State{
    IDLE = 0,
    INIT
};

bool runloop = false;
void sighandler(int) {runloop = false;}

const string robot_file = "./dual_proxy_motion_robot/resources/panda_arm.urdf";

const string control_link_name = "end_effector";
const VectorXd control_pos_in_link = Vector3d(0,0,0);
const Vector3d pos_in_ee_link = Vector3d(0, 0, 0);

RedisClient redis_client;

// redis keys:
string CONTROLLER_RUNNING_KEY = "sai2::substation::simviz::controller_running_key";
string JOINT_ANGLES_KEY = "sai2::substation::simviz::sensors::q";
string JOINT_VELOCITIES_KEY = "sai2::substation::simviz::sensors::dq";
string ROBOT_COMMAND_TORQUES_KEY = "sai2::substation::simviz::actuators::tau_cmd";
string ROBOT_STATE_TRANSITION_KEY = "sai2::substation::simviz::transition::state";
string ROBOT_SENSED_FORCE_KEY = "sai2::substation::simviz::sensors::sensed_force";

const bool flag_simulation = false;




int main(int argc, char const *argv[])
{
    signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

    // start redis client local
	redis_client = RedisClient();
	redis_client.connect();
    redis_client.set(CONTROLLER_RUNNING_KEY,"0");

    // define state variable.
    int state = IDLE;

    Affine3d T_world_robot = Affine3d::Identity();
	T_world_robot.translation() = Vector3d(0, 0, 0);
	auto robot = new Sai2Model::Sai2Model(robot_file, false, T_world_robot);

	robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
	robot->updateModel();

    int dof = robot->dof();
    VectorXd command_torques = VectorXd::Zero(dof);
    // intialize variable for storing coriolis torque and mass matrix.
    VectorXd coriolis = VectorXd::Zero(dof);
    MatrixXd mass_from_robot = MatrixXd::Zero(dof,dof);
    Vector3d ee_pose  = Vector3d::Zero();
    Matrix3d R = Matrix3d::Identity();

    R << -0.998378,-0.054084,0.017773,-0.054754,0.997710,-0.039698,-0.015586,-0.040607,-0.999054;

    // Define joint task object.
    auto joint_task = new Sai2Primitives::JointTask(robot);
    VectorXd joint_task_torques = VectorXd::Zero(dof);
    // enable joint saturation flag and set saturation angular velocity.
    joint_task->_use_velocity_saturation_flag = true;
    // joint_task->_saturation_velocity.array() = (M_PI/8)*VectorXd::Ones(dof);
    joint_task->_saturation_velocity << M_PI/6,M_PI/6,M_PI/6,M_PI/4,M_PI/3,M_PI/3,M_PI/3;

    joint_task->_kp = 300.0;
    joint_task->_kv = 15.0;
    joint_task->_ki = 0.0;
    joint_task->_use_interpolation_flag = false;

    // set integration flag. initial value set as false.
    int integrate_flag = 1;

    VectorXd q_init (dof);
    q_init << -1.46734,-0.356668,-0.0455042,-1.65865,-0.0728375,1.23512,0.855649;

    // position orientation task.
    auto posori_task = new Sai2Primitives::PosOriTask(robot,control_link_name,control_pos_in_link);
    VectorXd posori_task_torques = VectorXd::Zero(dof);
    MatrixXd x_cnt = posori_task->_current_position;
    MatrixXd R_cnt = posori_task->_current_orientation;
    // set N_prec matrix.
    MatrixXd N_prec = MatrixXd::Identity(dof,dof);

    posori_task->_kp_pos = 400.0;
    posori_task->_kv_pos = 15.0;
    posori_task->_ki_pos = 0.0;
    posori_task->_kp_ori = 100.0;
    posori_task->_kv_ori = 15.0;
    posori_task->_ki_ori = 0.0;

    posori_task->_e_max = 3e-2;
    posori_task->_e_min = 3e-3;

    posori_task->_use_velocity_saturation_flag = true;
    posori_task->_linear_saturation_velocity = 0.2;
    posori_task->_use_interpolation_flag = true;

    // update task hierarchy.
    N_prec.setIdentity(dof,dof);
    posori_task->updateTaskModel(N_prec);
    joint_task->updateTaskModel(posori_task->_N);

    // create handle for redis read and write callbacks.
    redis_client.createReadCallback(0);
	redis_client.createWriteCallback(0);

    // set read and write callbacks.
    redis_client.addEigenToReadCallback(0, JOINT_ANGLES_KEY, robot->_q);
    redis_client.addEigenToReadCallback(0, JOINT_VELOCITIES_KEY, robot->_dq);
    redis_client.addIntToReadCallback(0,ROBOT_STATE_TRANSITION_KEY,state);

    redis_client.addEigenToWriteCallback(0, ROBOT_COMMAND_TORQUES_KEY, command_torques);

    // initialize timer and controller counter.
    unsigned long long controller_counter = 0;
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); 
	double current_time = 0;
	double prev_time = 0;
	bool fTimerDidSleep = true;
	double start_time = timer.elapsedTime(); //secs
    // set the intial state to be idle.
    redis_client.set(ROBOT_STATE_TRANSITION_KEY,"0");
    // set runloop as true;
    runloop = true;
    while (runloop)
    {
           // set the timer loop;
        timer.waitForNextLoop();
        current_time = timer.elapsedTime() - start_time;
        redis_client.executeReadCallback(0);
        // update robot model after reading robot states from redis.
        robot->_M = mass_from_robot;
        robot->updateModel();

        // write the position of the robot end-effector.
        robot->position(ee_pose,"end_effector",pos_in_ee_link);
        robot->rotation(R,control_link_name);

        if (state == INIT) {
            joint_task->_desired_position = q_init;
            joint_task->updateTaskModel((MatrixXd::Identity(dof,dof)));
            joint_task->computeTorques(joint_task_torques);
            command_torques = joint_task_torques + coriolis;

            if (integrate_flag && (joint_task->_desired_position - joint_task->_current_position).norm() < 0.01) {
                    joint_task->_ki = 150;
                    joint_task->_integrated_position_error.setZero();
                    integrate_flag = 0;
                    cout << "Activating Integral part.\n";
                }
            // change state to IDLE;
            if ((joint_task->_desired_position - joint_task->_current_position).norm() < 0.005) {
                cout << "\nReached INIT Pose\n";
                // set ki to be zero.
                joint_task->reInitializeTask();
                posori_task->reInitializeTask();
                joint_task->_ki = 0;
                joint_task->_use_velocity_saturation_flag = true;
                // change state to IDLE.
                redis_client.set(ROBOT_STATE_TRANSITION_KEY,"0");
            }
        } else if (state == IDLE) {
            // cout << "In IDLE state.\n";
            integrate_flag = 1;
            joint_task->updateTaskModel((MatrixXd::Identity(dof,dof)));
            joint_task->computeTorques(joint_task_torques);
            command_torques = joint_task_torques + coriolis;
        }

    redis_client.executeWriteCallback(0);
    controller_counter++;
        
    }
    

    return 0;
}
