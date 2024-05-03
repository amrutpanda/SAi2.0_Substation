#include <iostream>
#include <Sai2Model.h>
#include <tasks/JointTask.h>
#include <tasks/PositionTask.h>
#include <tasks/PosOriTask.h>
#include <redis/RedisClient.h>
#include <timer/LoopTimer.h>
#include <signal.h>
#include <unistd.h>
#include <string>
#include <vector>

int signnum(double x) {
  if (x >= 0.0) return 1;
  if (x < 0.0) return -1;
  return 1;
}

void generate_path(std::vector<Vector3d>& path,double l, double w, Affine3d T);

enum State {
    INIT = 0,
    LOOK,
    TRAJ,
    IDLE,
    CAM
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
std::string ROTATION_FROM_CAMERA_KEY;
std::string CAMERA_TRIGGER_KEY;

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
        ROTATION_FROM_CAMERA_KEY = "sai2::substation::camera::rotation";
        CAMERA_TRIGGER_KEY = "sai2::substation::camera::trigger";

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
    // R << 0.999829,0.018033,-0.004094,-0.018039,0.999836,-0.001361,0.004069,0.001435,0.999991;
    // R << 0.772916,-0.631131,-0.065375,-0.633495,-0.773396,-0.023316,-0.035846,0.059436,-0.997588;
    // R << -0.726272,0.686320,0.038636,0.687345,0.724304,0.054231,0.009236,0.065943,-0.997781;
    R << -0.998378,-0.054084,0.017773,-0.054754,0.997710,-0.039698,-0.015586,-0.040607,-0.999054;

    // joint task0.999829,0.018033,-0.004094],[-0.018039,0.999836,-0.001361],[0.004069,0.001435,0.999991]
    auto joint_task = new Sai2Primitives::JointTask(robot);
    VectorXd joint_task_torques = VectorXd::Zero(dof);
    joint_task->_use_velocity_saturation_flag = true;
    joint_task->_saturation_velocity.array() = (M_PI/8)*VectorXd::Ones(dof);

    joint_task->_kp = 300.0;
    joint_task->_kv = 15.0;
    joint_task->_ki = 0.0;
    
    VectorXd q_init (dof);
    VectorXd q_look (dof);
    // q_init << 0.162313,-1.09008,-0.153488,-2.61457,-0.133069,1.54065,0.778681; // -ve x direction pose U shape pose.
    // q_init << -1.78969,-0.71409,0.1784,-2.12389,0.132822,1.78369,0.730236;; // -ve x direction 180 rotated end-effector
    // angles when box in put on plane

    q_init << -1.76471,-1.00067,0.0906794,-2.41891,0.218476,2.53667,0.543382;
    q_look << -1.78969,-0.71409,0.1784,-2.12389,0.132822,1.78369,0.730236;

    q_init << -1.62959,-0.288922,0.152178,-1.47316,-0.0436687,1.1186,0.845332;
    q_init << -1.46734,-0.356668,-0.0455042,-1.65865,-0.0728375,1.23512,0.855649;

    // angles when box is not put on plane.
    // q_init << -0.018177,-1.10597,-0.0369388,-2.37861,-0.142355,1.73228,-2.33937;
    // q_look << -0.109834,-1.45582,-0.0775309,-2.52178,-0.122086,1.7794,0.693609;

    joint_task->_desired_position = q_init;
    int integrate_flag = 1;

    // set position and orientation task.

    Vector3d robot_pos(0.45,-0.268, 0.46);
    robot_pos << 0.478792,-0.230273,0.506473;
    Matrix3d robot_rot = Matrix3d::Identity();
    // [[0.999829,0.018033,-0.004094],[-0.018039,0.999836,-0.001361],[0.004069,0.001435,0.999991]]

    // create a rectangular trajectory path from one given corner point.
    std::vector<Vector3d> path;
    int path_size = path.size();
    // initialize traj_point_count;
    int traj_point_count = 0;

    // position orientation task.
    auto posori_task = new Sai2Primitives::PosOriTask(robot,control_link_name,control_pos_in_link);
    Vector3d x_init = posori_task->_current_position;
    Matrix3d R_init = posori_task->_current_orientation;
    R_init = R;
    R_init << -0.726272,0.686320,0.038636,0.687345,0.724304,0.054231,0.009236,0.065943,-0.997781;

    VectorXd posori_task_torques = VectorXd::Zero(dof);
    posori_task->_use_velocity_saturation_flag = true;

    MatrixXd N_prec = MatrixXd::Identity(dof,dof);
    

    posori_task->_kp_pos = 400.0;
    posori_task->_kv_pos = 15.0;
    posori_task->_ki_pos = 0.0;
    posori_task->_kp_ori = 50.0;
    posori_task->_kv_ori = 10.8;
    posori_task->_ki_ori = 0.0;

    posori_task->_e_max = 3e-2;
    posori_task->_e_min = 3e-3;
    posori_task->_linear_saturation_velocity = 0.2;

    // state handling.
    int state = INIT;

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
    
    // In the beginning change the state transition key to IDLE;
    redis_client.set(STATE_TRANSITION_KEY,"3");
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
                    joint_task->_ki = 250;
                    joint_task->_integrated_position_error.setZero();
                    integrate_flag = 0;
                }
            // change state to IDLE;
            if ((joint_task->_desired_position - joint_task->_current_position).norm() < error/10)
            {
                cout << "\nReached INIT Pose\n";
                joint_task->_ki = 0;              
                joint_task->reInitializeTask();
                posori_task->reInitializeTask();
                redis_client.set(STATE_TRANSITION_KEY,"3");
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
            // change state to IDLE;
            if ((joint_task->_desired_position - joint_task->_current_position).norm() < error)
            {
                cout << "\nReached LOOK Pose\n";
                joint_task->reInitializeTask();
                posori_task->reInitializeTask();
                redis_client.set(STATE_TRANSITION_KEY,"3");
                // joint_task->reInitializeTask();
            }
            
        }
        else if (state == TRAJ)
        {
            posori_task->_desired_position = robot_pos;
            posori_task->_desired_orientation = R_init;
            posori_task->updateTaskModel((MatrixXd::Identity(dof,dof)));
            posori_task->computeTorques(posori_task_torques);
            // set some _ki value for accurate tracking.
            if (integrate_flag && (posori_task->_desired_position - posori_task->_current_position).norm() < 2* tol)
            {
                posori_task->_integrated_position_error.setZero();
                posori_task->_ki_pos = 250;
                integrate_flag = 0;
                cout << "Activating integral component in the controller.\n";    
            }

            if ((posori_task->_desired_position - posori_task->_current_position).norm() < error)
            {
                if (traj_point_count >= path.size()){
                    integrate_flag = 1;
                    traj_point_count = 0;
                    cout << "traj point: " << path_size << "\n";
                    cout << "moving to INIT pose.\n";
                    // state = INIT;
                    posori_task->_ki_pos = 0.0;
                    posori_task->_integrated_position_error.setZero();
                    posori_task->_integrated_orientation_error.setZero();
                    joint_task->reInitializeTask();
                    // posori_task->reInitializeTask();
                    redis_client.set(STATE_TRANSITION_KEY,"3");
    
                }
                else{
                    integrate_flag = 1;
                    robot_pos = path[traj_point_count];
                    cout << "============\n";
                    cout << "trajectory point: " << traj_point_count << "\n";
                    cout << "robot->_desired position: " << posori_task->_desired_position << "\n";
                    cout << "robot->_current position: " << posori_task->_current_position << "\n";
                    cout << "************\n";
                    posori_task->_ki_pos = 250.0;
                    posori_task->_integrated_position_error.setZero();
                    traj_point_count++;
                }
                
            }
            
            // else
            // {
            //     // posori_task->_ki_pos = 0.0;
            // }
            joint_task->computeTorques(joint_task_torques);
            
            command_torques = posori_task_torques + coriolis ;
            // command_torques = posori_task_torques + joint_task_torques + coriolis ;
        }
        else if(state == CAM)
        {
            // set the camera trigger key to 1.
            redis_client.set(CAMERA_TRIGGER_KEY,"1");
            usleep(1000000);
            Vector3d pose_from_camera = redis_client.getEigenMatrixJSON(POSE_FROM_CAMERA_KEY);
            MatrixXd rot_from_camera = redis_client.getEigenMatrixJSON(ROTATION_FROM_CAMERA_KEY);
            cout << "Pose from camera: " << pose_from_camera<< "\n";
            // find transformation to base frame.
            Affine3d T;
            // robot->transform(T,"camera_link",pose_from_camera,rot_from_camera);
            robot->transform(T,"camera_link",pose_from_camera,rot_from_camera);
            cout << "Rotation Matrix: " << T.rotation() << "\n";
            MatrixXd rot_mat;
             
            // compute trajectory.
            generate_path(path,0.2,0.2,T);
            robot_pos = path[0]; // set the robot_pos to the start of the trajectory.
            // // for testing.
            // robot_pos = T.translation() + Vector3d(0,0,0.02);
            R_init = T.rotation();

            // R_init = rot_from_camera;
            // set the state transition key to IDLE;
            redis_client.set(STATE_TRANSITION_KEY,"3");
            // redis_client.set(CAMERA_TRIGGER_KEY,"0");
            command_torques = coriolis;
        }
        else if (state == IDLE)
        {
            redis_client.set(CAMERA_TRIGGER_KEY,"0");
            // posori_task->reInitializeTask();
            // joint_task->reInitializeTask();
            integrate_flag = 1;
            // joint_task->_ki = 0;
            // joint_task->_integrated_position_error.setZero();
            joint_task->computeTorques(joint_task_torques);
            command_torques = joint_task_torques + coriolis;
            // command_torques = coriolis;
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


void generate_path(std::vector<Vector3d>& path,double l, double w, Affine3d T)
{
    // clear the container.
    path.clear();
    // make the rectangle in local coordinate system starting from top left corner.
    double offset = -0.05; // z axis offset for safety.
    MatrixXd corner_points(3,5);
    corner_points.col(0) = Vector3d(-l/2,w/2,offset);
    corner_points.col(1) = Vector3d(l/2,w/2,offset);
    corner_points.col(2) = Vector3d(l/2,-w/2,offset);
    corner_points.col(3) = Vector3d(-l/2,-w/2,offset);
    // last point same as first point.
    corner_points.col(4) = Vector3d(-l/2,w/2,offset);

    int n = 10; // no. intermediate points.
    for (int i = 0; i < corner_points.cols()-1; i++)
    {
        Vector3d xi = corner_points.col(i);
        Vector3d xe = corner_points.col(i+1);
        Vector3d diff = xe - xi;
        // cout<< "T: " << T.matrix() << "\n";
        for (int j = 0; j < n; j++)
        {
            Vector3d temp_var = xi + ((double)j/(double)n)* diff;
            // transform and save to container.
            path.push_back(T.rotation()*temp_var + T.translation());
            // path.push_back(temp_var);
        }
        
    }
    path.push_back(path[0]);

    // print all elements of vector.
    // cout << "printing the trajectory\n";
    // for (int i = 0; i < path.size(); i++)
    // {
    //     cout << "\ni : " << i << "\n" << path[i] << "\n"; 
    // }
}
