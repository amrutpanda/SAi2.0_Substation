#include <iostream>
#include <Sai2Model.h>
#include <redis/RedisClient.h>
#include <timer/LoopTimer.h>
#include <signal.h>
#include <string>

bool runloop = false;
void sighandler(int) {runloop = false;}

const string robot_file = "../model/panda/panda_arm.urdf";
unsigned long long controller_counter = 0;

RedisClient redis_client;

// redis keys.
// ----- write.----
const string ROBOT_COMMAND_TORQUES_KEY = "sai2::substation::simviz::actuators::tau_cmd";
const string CONTROLLER_RUNNING_KEY = "sai2::substation::simviz::controller_running_key";
// ----- read ----
const string JOINT_ANGLES_KEY = "sai2::substation::simviz::sensors::q";
const string JOINT_VELOCITIES_KEY = "sai2::substation::simviz::sensors::dq";
const string ROBOT_SENSED_FORCE_KEY = "sai2::substation::simviz::sensors::sensed_force";

int main(int argc, char const *argv[])
{
    RedisClient redis_client;
    // connect to redis client.
    redis_client.connect();
    // setup signal handler
    signal(SIGTERM,sighandler);
    signal(SIGABRT,sighandler);
    signal(SIGINT,sighandler);

    // load the robots.
    Eigen::Affine3d T_world_robot = Eigen::Affine3d::Identity();
    T_world_robot.translation() << 0,0,0;
    auto robot = new Sai2Model::Sai2Model(robot_file,false);
    robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
    // do not forget to update the model.
    robot->updateModel();

    // // prepare controller.
    int dof = robot->dof();
    Eigen::VectorXd computed_torque(7);

    // create timer.
    LoopTimer timer;
    timer.initializeTimer();
    timer.setLoopFrequency(1000);
    double start_time = timer.elapsedTime();
    redis_client.set(CONTROLLER_RUNNING_KEY,"1");
    // set runloop to be true.
    runloop = true;
    while (runloop)
    {
        // wait for the next scheduled loop.
        timer.waitForNextLoop();
        double time = timer.elapsedTime() - start_time;
        // read robot q and dq;
        robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
        robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);
        robot->updateModel();
        // computed commanded torques.
        computed_torque << -11.0,11.0,11.0,11.0,11.0,11.0,11.0;
        // send to redis.
        redis_client.setEigenMatrixJSON(ROBOT_COMMAND_TORQUES_KEY,computed_torque);
        controller_counter++;

    }
    
    computed_torque.setZero();
    redis_client.set(CONTROLLER_RUNNING_KEY,"0");
    std::cout << "\n";
    std::cout << "start time : " << start_time << "\n";
    std::cout << "Elapsed time: " << time << "\n";
    std::cout << "No. of loops : " << controller_counter << "\n";

    return 0;
}
