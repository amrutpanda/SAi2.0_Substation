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

int signnum(double x) {
  if (x >= 0.0) return 1;
  if (x < 0.0) return -1;
  return 1;
}

void generate_path(std::vector<Vector3d>& path, double ybox, double zbox, Vector3d init_pos);


enum State {
    INIT = 0,
    GRIP,
    TRAJ
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

bool runloop = false;
void sighandler(int) {runloop = false;}

const string robot_file = "../model/panda/panda_arm.urdf";
unsigned long long controller_counter = 0;
double tol = 1e-2;

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
    // [[0.999829,0.018033,-0.004094],[-0.018039,0.999836,-0.001361],[0.004069,0.001435,0.999991]]
    R << 0.999829,0.018033,-0.004094,-0.018039,0.999836,-0.001361,0.004069,0.001435,0.999991;

    // joint task0.999829,0.018033,-0.004094],[-0.018039,0.999836,-0.001361],[0.004069,0.001435,0.999991]
    auto joint_task = new Sai2Primitives::JointTask(robot);
    VectorXd joint_task_torques = VectorXd::Zero(dof);
    joint_task->_use_velocity_saturation_flag = false;
    // joint_task->_saturation_velocity.array() = 35 * (M_PI / 180);

    joint_task->_kp = 50.0;
    joint_task->_kv = 15.0;
    joint_task->_ki = 0.0;
    
    VectorXd q_init (dof);
    q_init << 0.622035,0.202844,-0.748116,-2.34133,0.161845,2.42087,0.440028;
    joint_task->_desired_position = q_init;
    int integrate_flag = 1;

    // set position and orientation task.

    Vector3d robot_pos(0.5,-0.068, 0.29);
    Matrix3d robot_rot = Matrix3d::Identity();
    // [[0.999829,0.018033,-0.004094],[-0.018039,0.999836,-0.001361],[0.004069,0.001435,0.999991]]

    // create a rectangular trajectory path from one given corner point.
    std::vector<Vector3d> path;
    generate_path(path,0.25,0.15,robot_pos); 
    int path_size = path.size();
    // initialize traj_point_count;
    int traj_point_count = 0;

    // position orientation task.
    auto posori_task = new Sai2Primitives::PosOriTask(robot,control_link_name,control_pos_in_link);
    Vector3d x_init = posori_task->_current_position;
    Matrix3d R_init = posori_task->_current_orientation;

    VectorXd posori_task_torques = VectorXd::Zero(dof);
    posori_task->_use_velocity_saturation_flag = false;

    MatrixXd N_prec = MatrixXd::Identity(dof,dof);

    posori_task->_kp_pos = 400.0;
    posori_task->_kv_pos = 40.0;
    posori_task->_ki_pos = 250.0;
    posori_task->_kp_ori = 50.0;
    posori_task->_kv_ori = 10.8;
    posori_task->_ki_ori = 100.0;

    posori_task->_e_max = 3e-2;
    posori_task->_e_min = 3e-3;

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

    runloop = true;
	redis_client.set(CONTROLLER_RUNNING_KEY,"1");

    // state handling.
    int state = TRAJ;

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
                }
        }
        else if (state == GRIP)
        {
            joint_task->_desired_position = q_init;
            joint_task->updateTaskModel((MatrixXd::Identity(dof,dof)));
            joint_task->computeTorques(joint_task_torques);
            command_torques = joint_task_torques + coriolis;

            if (integrate_flag && (joint_task->_desired_position - joint_task->_current_position).norm() < 2 * tol) {
                    joint_task->_ki = 150;
                    joint_task->_integrated_position_error.setZero();
                    integrate_flag = 0;
                }
            
        }
        else if (state == TRAJ)
        {
            posori_task->_desired_position = robot_pos;
            posori_task->_desired_orientation = R;
            posori_task->computeTorques(posori_task_torques);

            if (integrate_flag && (posori_task->_desired_position - posori_task->_current_position).norm() < 2* tol)
            {
                posori_task->_ki_pos = 250;
                posori_task->_integrated_position_error.setZero();
                integrate_flag = 0;
            }
            else if ((posori_task->_desired_position - posori_task->_current_position).norm() < 2* tol)
            {
                integrate_flag = 1;
                if (traj_point_count >= path_size){
                    traj_point_count = 0;
                    cout << "traj point: " << path_size << "\n";
                    // cout << "moving to INIT pose.\n";
                    state = INIT;
    
                }
                else{
                    robot_pos = path[traj_point_count];
                    cout<< "============\n";
                    cout << "robot->_desired position: " << posori_task->_desired_position << "\n";
                    cout << "robot->_current position: " << posori_task->_current_position << "\n";
                    cout << "************\n";
                    posori_task->_integrated_position_error.setZero();
                    traj_point_count++;
                }
                
            }
            else
            {
                posori_task->_ki_pos = 20.0;
            }
            
            command_torques = posori_task_torques + coriolis;
        }
        
        
        // redis_client.setEigenMatrix(ROBOT_COMMAND_TORQUES_KEY,command_torques);

        redis_client.executeWriteCallback(0);
        controller_counter++;
    }

    double end_time = timer.elapsedTime();
    command_torques.setZero();
    redis_client.set(CONTROLLER_RUNNING_KEY,"0");
    cout << "controller start time: " << start_time << "\n";
    cout << "controller end time: " << end_time << "\n";
    cout << "controller elapsed cycle: " << timer.elapsedCycles() << "\n";

    return 0;
}

void generate_path(std::vector<Vector3d>& path, double ybox, double zbox, Vector3d init_pos)
{
    // make the container empty.
    path.clear();
    // sign change checking variable.
    int ys1,ys2,zs1,zs2;
    ys1 = signnum(init_pos(1,0));
    zs1 = signnum(init_pos(2,0));
    int n = 10;
    // create a rectangular path.
    Vector3d pivot = init_pos;
    std::vector<VectorXd> corner_points;
    corner_points.push_back(pivot);
    pivot = pivot - Vector3d(0,signnum(init_pos(1,0))*ybox,0);
    ys2 = signnum(pivot(1,0));
    corner_points.push_back(pivot);
    pivot = pivot - Vector3d(0,0,signnum(init_pos(2,0))*zbox);
    zs2 = signnum(pivot(2,0));
    corner_points.push_back(pivot);
    
    // Y coordinates.
    if (signnum(ys1) > 0)
        {
            pivot = pivot + Vector3d(0,ybox,0);
            corner_points.push_back(pivot);
        }
    else
        {
            pivot = pivot - Vector3d(0,ybox,0);
            corner_points.push_back(pivot);
        } 
    // pivot = pivot + Vector3d(0,ybox,0);
    // corner_points.push_back(pivot);

    // z coordinates.
    if (signnum(zs1) > 0)
        {
            pivot = pivot + Vector3d(0,0,zbox);
            corner_points.push_back(pivot);
        }
    else
        {
            pivot = pivot - Vector3d(0,0,zbox);
            corner_points.push_back(pivot);
        } 

    assert(corner_points.size() == 5);
    // generate intermediate points.

    for (int i = 0; i < 4; i++)
    {
        Vector3d x1 = corner_points[i];
        Vector3d x2 = corner_points[i+1];

        Vector3d diff = x2 - x1;

        for (int j = 0; j < n ; j++)
        {
            // std::cout << "increment: \n" << (double)j/(double)n << "\n";
            path.push_back(corner_points[i] + (double)j/(double)n* diff );
        }
        
    }
    path.push_back(corner_points[corner_points.size() - 1]);
    // print the trajectory.
    // cout << "printing the trajectory\n";
    // for (int i = 0; i < path.size(); i++)
    // {
    //     cout << "\ni : " << i << "\n" << path[i] << "\n"; 
    // }
    
}

