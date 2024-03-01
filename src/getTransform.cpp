#include <iostream>
#include <Sai2Model.h>
#include <redis/RedisClient.h>
#include <string>
#include <fstream>
#include <ostream>

MatrixXd readFromFileToEigen(std::string filepath);
void saveEigenToFile(std::string filename,MatrixXd mat);
void inverse_transformation_matrix(MatrixXd &mat1,MatrixXd &mat2);
void get_position_from_transformation(MatrixXd& T, MatrixXd& p_in, MatrixXd& p_out);

std::string JOINT_ANGLES_KEY;
std::string ROBOT_POSE_KEY;
std::string FILE_SAVE_KEY;
bool flag_simulation = false;

// set robot urdf file path and counter
const std::string robot_file = "../model/panda/panda_arm.urdf";
unsigned long long controller_counter = 0;

// set the redis client.
RedisClient redis_client;

const std::string link_name = "link7";
const Vector3d pos_in_link = Vector3d(0,0,0);

int main(int argc, char const *argv[])
{
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

    MatrixXd poslist = readFromFileToEigen("points_v1.1.txt");
    Affine3d T;
    MatrixXd T1, T2;
    
    MatrixXd transformed_poslist;
    transformed_poslist.resize(poslist.rows(),3);
    
    int count = 0;
    while (count < poslist.rows())
    {
        robot->_q = redis_client.getEigenMatrix(JOINT_ANGLES_KEY);
        robot->updateKinematics();
        // robot->position(pos,link_name,pos_in_link);
        // redis_client.setEigenMatrix(ROBOT_POSE_KEY,pos);

        robot->transform(T,link_name,pos_in_link);
        // cout << T1.matrix() << "\n";
        // cout << "++++++++++++++++" << "\n";
        MatrixXd temp = T.matrix();
        T2 = T.matrix().inverse();
        // cout << "T2 : " << T2 << "\n";
        // inverse_transformation_matrix(temp,T2);
        MatrixXd p_in, p_out;
        p_in = poslist.row(count).transpose();
        get_position_from_transformation(T2,p_in,p_out);

        // cout << "p_out : \n" << p_out << "\n";
        transformed_poslist(count,0) = p_out(0,0);
        transformed_poslist(count,1) = p_out(1,0);
        transformed_poslist(count,2) = p_out(2,0);

        count++;
    }
    saveEigenToFile("newfilev1.txt",transformed_poslist);
    
    return 0;
}


MatrixXd readFromFileToEigen(std::string filepath)
{
    /* 
     * the Eigen Matrix "mat" must have rows and columns defined beforehand.
    */

    int rows = 0;
    int cols = 0;
    int prev_cols = 0;
    std::vector<double> container;
    // open file handle.
    ifstream file(filepath.c_str());
    
    while (file)
    {
        // read one line.
        std::string line;
        getline(file,line,'\n');
        // string stream for fields.
        istringstream iss(line);
        std::string element;

        double value;
        // set col to zero.
        cols = 0;
        // read numbers from line to element.
        while (iss.good())
        {   
            getline(iss,element,',');
            if (element.size() == 0)
            {
                break;
            }
            else
            {
                value = stod(element);
                container.push_back(value);
                cols++;
                prev_cols = cols;
            }
            // std::cout << stod(element) << " ";
        }
        
        rows++;
    }
    cols = prev_cols;
    // decrementing rows as intial was 0.
    rows = rows -1;    
    // create a eigen matrix;
    MatrixXd mat;
    mat.resize(rows,cols);
    int ele_count = 0;
    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < cols; j++)
        {
            mat(i,j) = container[ele_count];
            ele_count++;
        }
        
    }
    return mat;
}

void saveEigenToFile(std::string filename,MatrixXd mat)
{
    // create a file handle.
    ofstream file;
    // open file.
    file.open(filename.c_str(),std::ios::out | std::ios::trunc);
    int rows = mat.rows();
    int cols = mat.cols();
    // write to comma separated
    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < cols; j++)
        {
            file << mat(i,j);
            if (j != rows -1)
                file << ", ";
        }
        file << "\n";
    }
    
    file.close();
}

void inverse_transformation_matrix(MatrixXd &mat1,MatrixXd &mat2)
{
    mat2.resize(4,4);
    MatrixXd R;
    MatrixXd t = mat1.block<3,1>(0,3);
    
    R = mat1.topLeftCorner(3,3);
    mat2.topLeftCorner(3,3) = R.transpose(); 
    mat2.block<3,1>(0,3) = -R.transpose()*t;
    mat2.row(3) = mat1.row(3);
}

void get_position_from_transformation(MatrixXd& T, MatrixXd& p_in, MatrixXd& p_out)
{
    // p_in and p_out are single column matrix like column vector.
    MatrixXd p_hat_in, p_hat_out;
    p_hat_in.resize(4,1);
    p_hat_in(3,0) = 1;
    p_hat_out.resize(4,1);
    p_hat_out(3,0) = 1;

    p_hat_in.block<3,1>(0,0) = p_in;
    // cout << "p_hat_in = \n" << p_hat_in << "\n";
    p_hat_out = T*p_hat_in;

    p_out = p_hat_out.block<3,1>(0,0);

    // cout << "p_hat_out = \n" << p_hat_out << "\n";
}