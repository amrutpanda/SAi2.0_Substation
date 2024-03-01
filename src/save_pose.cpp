#include <iostream>
#include <Sai2Model.h>
#include <redis/RedisClient.h>
#include <string>
#include <fstream>
#include <ostream>
#include <signal.h>


bool runloop = false;

void sighandler(int) {runloop = false;}
std::string JOINT_ANGLES_KEY;
std::string ROBOT_POSE_KEY;
std::string FILE_SAVE_KEY;
bool flag_simulation = false;

// set robot urdf file path and counter
const std::string robot_file = "../model/panda/panda_arm.urdf";
unsigned long long controller_counter = 0;

// set the redis client.
RedisClient redis_client;

const std::string link_name = "end_effector";
const Vector3d pos_in_link = Vector3d(0,0,0);

int main(int argc, char const *argv[])
{
    signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

    if (!flag_simulation)
        {
            JOINT_ANGLES_KEY = "sai2::FrankaPanda::Bonnie::sensors::q";
            ROBOT_POSE_KEY = "sai2::FrankaPanda::Bonnie::pose";
            FILE_SAVE_KEY = "sai2::file::save";
        }
    else
        {
            JOINT_ANGLES_KEY = "sai2::substation::simviz::sensors::q";
            ROBOT_POSE_KEY = "sai2::substation::Bonnie::pose";
            FILE_SAVE_KEY = "sai2::file::save";
        }

    
    // start redis client local
	redis_client = RedisClient();
	redis_client.connect();

    // set file save key to be false;
    redis_client.set(FILE_SAVE_KEY,"0");

    // load robot model.
    Affine3d T_world_robot = Affine3d::Identity();
	T_world_robot.translation() = Vector3d(0, 0, 0);
    auto robot = new Sai2Model::Sai2Model(robot_file,false,T_world_robot);

    robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
	robot->updateModel();

    // define pose vector and rotation matrix.
    int dof = robot->dof();
    Vector3d pos = Vector3d::Zero();
    Matrix3d R = Matrix3d::Identity();

    // open file handle.
    std::ofstream file;
    file.open("points_v1.1.txt",std::ios::out | std::ios::trunc);
    // set a input variable.
    std::string filesave;
    // set runloop to true
    runloop = true;
    while (runloop)
    {
        robot->_q = redis_client.getEigenMatrix(JOINT_ANGLES_KEY);
        robot->updateKinematics();

        robot->position(pos,link_name,pos_in_link);
        redis_client.setEigenMatrix(ROBOT_POSE_KEY,pos);

        filesave = redis_client.get(FILE_SAVE_KEY);
        
        if (filesave == "1")
        {
            // print the pose to be saved.
            cout << "Pose: " << pos << "\n";
            cout << "Saving to file.\n" ;
            // save to file.
            file << pos(0) << "," << pos(1) << "," << pos(2) << "\n";
            
            redis_client.set(FILE_SAVE_KEY,"0");
        }
    
        
    }
    
    file.close();
    cout << "file saved" << "\n";
    return 0;
}


