#include <iostream>
#include <chrono>
#include <signal.h>
#include <vector>
#include <unistd.h>
// SAI2 header files.
#include <Sai2Model.h>
#include <tasks/JointTask.h>
#include <tasks/PosOriTask.h>
#include <redis/RedisClient.h>
#include <timer/LoopTimer.h>

// enum construct for states.
enum State{
    INIT =0,
    IDLE,
    VS
};

// redis key string.
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
std::string ROTATION_FROM_CAMERA_KEY;
std::string CAMERA_TRIGGER_KEY;
std::string DIRECTION_KEY;
std::string ANGLE_KEY;
std::string OBJECT_IN_FRAME_KEY;

bool runloop = false;
void sighandler(int) {runloop = false;}

const string robot_file = "../model/panda/panda_arm.urdf";
unsigned long long controller_counter = 0;
double tol = 1e-1;
double error = 1e-2;

const string control_link_name = "end_effector";
const VectorXd control_pos_in_link = Vector3d(0,0,0);
const Vector3d pos_in_ee_link = Vector3d(0, 0, 0);

RedisClient redis_client;
const bool flag_simulation = false;

int main(int argc, char const *argv[])
{
    if (!flag_simulation)
    {
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
        ROTATION_FROM_CAMERA_KEY = "sai2::substation::camera::rotation";
        CAMERA_TRIGGER_KEY = "sai2::substation::camera::trigger";
        DIRECTION_KEY = "sai2::substation::camera::direction";
        ANGLE_KEY = "sai2::substation::camera::angle";
        OBJECT_IN_FRAME_KEY = "sai2::substation::camera::object_in_frame";
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
    joint_task->_saturation_velocity << M_PI/6,M_PI/6,M_PI/6,M_PI/4,M_PI/3,M_PI/3,M_PI/4;

    joint_task->_kp = 300.0;
    joint_task->_kv = 15.0;
    joint_task->_ki = 0.0;

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
    posori_task->_kp_ori = 400.0;
    posori_task->_kv_ori = 15.0;
    posori_task->_ki_ori = 0.0;

    posori_task->_e_max = 3e-2;
    posori_task->_e_min = 3e-3;

    posori_task->_use_velocity_saturation_flag = true;
    posori_task->_linear_saturation_velocity = 0.02;

    // update task hierarchy.
    N_prec.setIdentity(dof,dof);
    posori_task->updateTaskModel(N_prec);
    N_prec = posori_task->_N;
    joint_task->updateTaskModel(N_prec);

    // set initial state
    int state = IDLE;

    // set visual servoing variables.
    // VectorXd dir_vector = VectorXd::Zero(3);
    // VectorXd x_new = VectorXd::Zero(3);
    // MatrixXd Rotation = MatrixXd::Zero(3,3);
    // double angle_new;
    // double angle;
    bool first_iteration = true;
    // create handle for redis read and write callbacks.
    redis_client.createReadCallback(0);
	redis_client.createWriteCallback(0);

    // set read and write callbacks.
    redis_client.addEigenToReadCallback(0, JOINT_ANGLES_KEY, robot->_q);
    redis_client.addEigenToReadCallback(0, JOINT_VELOCITIES_KEY, robot->_dq);

    redis_client.addEigenToReadCallback(0,CORIOLIS_KEY,coriolis);
    redis_client.addEigenToReadCallback(0,MASSMATRIX_KEY,mass_from_robot);

    redis_client.addEigenToWriteCallback(0, ROBOT_COMMAND_TORQUES_KEY, command_torques);
    redis_client.addEigenToWriteCallback(0,ROBOT_END_EFFECTOR_POSE,ee_pose);
    redis_client.addEigenToWriteCallback(0,ROBOT_END_EFFECTOR_ROTATION_MATRIX,R);
    redis_client.addIntToReadCallback(0,STATE_TRANSITION_KEY,state);

    // set initial redis keys to start the controller and fix the initial states.
    redis_client.set(STATE_TRANSITION_KEY,"1"); // IDLE state 
    runloop = true;
	redis_client.set(CONTROLLER_RUNNING_KEY,"1");
    redis_client.set(CAMERA_TRIGGER_KEY,"0");


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
        // update robot model after reading robot states from redis.
        robot->_M = mass_from_robot;
        robot->updateModel();

        // write the position of the robot end-effector.
        robot->position(ee_pose,"end_effector",pos_in_ee_link);
        robot->rotation(R,control_link_name);

        if (state == INIT)
        {
            joint_task->_desired_position = q_init;
            joint_task->updateTaskModel((MatrixXd::Identity(dof,dof)));
            joint_task->computeTorques(joint_task_torques);
            command_torques = joint_task_torques + coriolis;

            if (integrate_flag && (joint_task->_desired_position - joint_task->_current_position).norm() < 2 * tol) {
                    joint_task->_ki = 150;
                    joint_task->_integrated_position_error.setZero();
                    integrate_flag = 0;
                    cout << "Activating Integral part.\n";
                }
            // change state to IDLE;
            if ((joint_task->_desired_position - joint_task->_current_position).norm() < tol/2)
            {
                cout << "\nReached INIT Pose\n";
                // set ki to be zero.
                joint_task->reInitializeTask();
                posori_task->reInitializeTask();
                joint_task->_ki = 0;
                joint_task->_use_velocity_saturation_flag = false;
                // change state to IDLE.
                redis_client.set(STATE_TRANSITION_KEY,"1");
            }
        }

        else if (state == IDLE)
        {
            // cout << "In IDLE state.\n";
            integrate_flag = 1;
            joint_task->updateTaskModel((MatrixXd::Identity(dof,dof)));
            joint_task->computeTorques(joint_task_torques);
            command_torques = joint_task_torques + coriolis;
        }

        else if (state == VS)
        {
            // get Rotation between end_effector frame and camera frame.
            double angle,step_size = 0.05;
            Affine3d T_EE;
            Vector3d x_new = Vector3d(0,0,0);
            Vector3d dir_vector,unnormalized_dir_vector;
            int object_in_frame = -1;
            
            // x_new as the current position.

            if (redis_client.get(CAMERA_TRIGGER_KEY) == "1")
            {
                unnormalized_dir_vector = redis_client.getEigenMatrixJSON(DIRECTION_KEY);
                dir_vector = unnormalized_dir_vector.normalized();
                angle = stod(redis_client.get(ANGLE_KEY));
                object_in_frame = stoi(redis_client.get(OBJECT_IN_FRAME_KEY));
                redis_client.set(CAMERA_TRIGGER_KEY,"0");

                
            }

            if (object_in_frame == 0)
            {
                cout << "Object out of frame. Switching to IDLE state.\n";
                redis_client.set(STATE_TRANSITION_KEY,"1");
                continue;
            }
            
            // set end-effector target.

            if (dir_vector(1) <= 0){
                x_new = x_new + Vector3d(step_size*abs(dir_vector(1)),0,0);
            }
            else {
                x_new = x_new + Vector3d(-step_size*abs(dir_vector(1)),0,0);
            }

            if (dir_vector(0) <= 0){
                x_new = x_new + Vector3d(0,-step_size*abs(dir_vector(0)),0);
            }
            else {
                x_new = x_new + Vector3d(0,step_size*abs(dir_vector(0)),0);
            }
            // tranform x_new in base frame.
            robot->transform(T_EE,control_link_name,pos_in_ee_link);
            cout << "direction vector : " << unnormalized_dir_vector << "\n";
            cout << "x_new without transformation : " << x_new <<"\n";
            cout << "x_curr: " << posori_task->_current_position <<"\n";
            robot->transform(T_EE,control_link_name,x_new);
            x_new = T_EE.translation();
            cout << "x_new: " << x_new <<"\n";
            cout << "===========================\n";
            // set controller parameter and compute torques.
            
            if (unnormalized_dir_vector.norm() >= 30)
            {
                cout << "norm more than 30\n";
                posori_task->_desired_position = x_new;
                posori_task->computeTorques(posori_task_torques);
                joint_task->updateTaskModel(posori_task->_N);
                joint_task->computeTorques(joint_task_torques);
                command_torques = posori_task_torques + joint_task_torques + coriolis;
            }
            else 
            {   
                cout << "norm less than 30\n";
                posori_task->_desired_position = posori_task->_current_position;
                joint_task->updateTaskModel((MatrixXd::Identity(dof,dof)));
                joint_task->computeTorques(joint_task_torques);
                posori_task->computeTorques(posori_task_torques);
                command_torques = posori_task_torques + joint_task_torques + coriolis;
            }
          
            // joint_task->updateTaskModel((MatrixXd::Identity(dof,dof)));
            // joint_task->computeTorques(joint_task_torques);
            // command_torques = joint_task_torques + coriolis;
        }


        redis_client.executeWriteCallback(0);
        controller_counter++;
    }
    double end_time = timer.elapsedTime();
    command_torques.setZero();
    redis_client.set(CONTROLLER_RUNNING_KEY,"0");
    redis_client.set(STATE_TRANSITION_KEY,"3");
    cout << "Controller elapsed time : " << end_time - start_time << "\n";
    cout << "controller elapsed cycle: " << timer.elapsedCycles() << "\n";

    return 0;
}
