#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <Eigen/Dense>

#include <towr/optjump.h>
#include <towr/terrain/terrain_data.h>
#include <towr/models/robot_model.h>
#include <towr/variables/variable_names.h>

using namespace Eigen;
using namespace towr;

Eigen::VectorXd parseCSV(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open file: " + filename);
    }

    std::vector<double> data;

    std::string line;
    std::getline(file, line); // Read the entire line
    std::stringstream ss(line);
    std::string cell;
    while (std::getline(ss, cell, ',')) {
        data.push_back(std::stod(cell));
    }

    Eigen::VectorXd vector(data.size());

    for (int i = 0; i < data.size(); ++i) {
        vector(i) = data[i];
    }

    return vector;
}

void ParseTerrainData(TerrainData& terrain_data){
  std::string folder_path = "/home/bolivar/Olympus-ws/towr_ros_catkin/src/tojr/towr/terrain_data";
  std::string x_file = folder_path + "/x.csv";
  std::string y_file = folder_path + "/y.csv";
  std::string z_file = folder_path + "/z.csv";
  Eigen::VectorXd x = parseCSV(x_file);
  Eigen::VectorXd y = parseCSV(y_file);
  Eigen::VectorXd z = parseCSV(z_file);
  
  const int nx = 3499;
  const int ny = 6981;
  double x_min = -3521.690825824481;
  double y_min = -2830992.8435943313;
  double scale = 1.998122454494478;





  double x0 = 0.0;
  double y0 = 0.0;
  double length = 10.0;
  double width = 10.0;

  x.resize(nx, ny);
  y.resize(nx, ny);
  z.resize(nx, ny);
  x*= 0.05;
  y*= 0.05;  
  z*= 0.05;
  scale*= 0.05;


  int i_min = floor((x0-x_min)/scale);
  int j_min = floor((y0-y_min)/scale);
  int i_max = floor((x0+length-x_min)/scale);
  int j_max = floor((y0+width-y_min)/scale);

  terrain_data.mu = 0.8;
  Eigen::MatrixXd x_block = x.block(i_min, j_min, i_max-i_min, j_max-j_min);
  x_block.resize(-1,1);
  terrain_data.x = x_block;
  Eigen::MatrixXd y_block = y.block(i_min, j_min, i_max-i_min, j_max-j_min);
  y_block.resize(-1,1);
  terrain_data.y = y_block;
  Eigen::MatrixXd z_block = z.block(i_min, j_min, i_max-i_min, j_max-j_min);
  z_block.resize(-1,1);
  terrain_data.z = z_block;
}

int main() {

double num_ee = 4;
double z_ground = 0.0;
  
TerrainData terrain_data;
ParseTerrainData(terrain_data);
towr::RobotModel model = towr::RobotModel(towr::RobotModel::Olympus);
auto nominal_stance_B = model.kinematic_model_->GetNominalStanceInBase();
double nominal_height = -nominal_stance_B.front().z();

towr::OptJump::Vector3d base_pos(0.0, 0.0, z_ground + nominal_height);
towr::OptJump::Vector3d base_ang(0.0, 0.0, 0.0);
towr::OptJump::EEpos initial_ee_pos;
for (int ee = 0; ee < num_ee; ++ee) {
    initial_ee_pos.push_back(base_pos + nominal_stance_B.at(ee));
}

towr::OptJump optjump(model, terrain_data);
optjump.SetInitialBaseState(base_pos, base_ang);  
optjump.SetInitialEEState(initial_ee_pos);
optjump.SetJumpLength(4.0);
optjump.Solve();
  
  double dt = 0.1;
  OptJump::BaseTrajectory base_traj = optjump.GetBaseTrajectory(dt);
  OptJump::EETrajectory ee_traj = optjump.GetEETrajectory(dt);

using namespace std;
std::cout.precision(2);
std::cout << fixed;
std::cout << "\n====================\nQuadruped trajectory:\n====================\n"; 
double t = 0.0;
for (int i = 0; i < base_traj.size(); i++) {
    std::cout << "t=" << t << "\n";
    std::cout << "Base linear position x,y,z:   \t";
    std::cout << base_traj.at(i).block<3,1>(0,0).transpose() << "\t[m]" << endl; 
    std::cout << "Base linear vel x,y,z:  \t";
    std::cout << base_traj.at(i).block<3,1>(6,0).transpose() << "\t[m]" << endl; 
    std::cout << "Base Euler roll, pitch, yaw:  \t";
    Eigen::Vector3d rad = base_traj.at(i).block<3,1>(3,0).transpose();
    std::cout << (rad/M_PI*180).transpose() << "\t[deg]" << endl; 
    std::cout << "Base angular vel roll, pitch, yaw:  \t";
    rad = base_traj.at(i).block<3,1>(9,0).transpose();
    std::cout << (rad/M_PI*180).transpose() << "\t[deg/s]" << endl;
    std::cout << endl; 
    t += dt;
}

  return 0;
}