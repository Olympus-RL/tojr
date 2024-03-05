#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <Eigen/Dense>

#include <towr/optjump.h>
#include <towr/terrain/terrain_data.h>
#include <towr/terrain/terrain_parser.h>
#include <towr/models/robot_model.h>
#include <towr/variables/variable_names.h>

using namespace Eigen;
using namespace towr;


int main() {

double num_ee = 4;
double z_ground = 0.0;
  
TerrainData terrain_data;
ParseTerrainData(terrain_data);
towr::RobotModel model = towr::RobotModel(towr::RobotModel::Olympus);
towr::OptJump optjump(model, terrain_data);

auto nominal_stance_B = model.kinematic_model_->GetNominalStanceInBase();
double nominal_height = -nominal_stance_B.front().z();
z_ground = optjump.GetTerrain() -> GetHeight(0.0, 0.0);
towr::OptJump::Vector3d base_pos(0.0, 0.0, z_ground + nominal_height);
towr::OptJump::Vector3d base_ang(0.0, 0.0, 0.0);
towr::OptJump::EEpos initial_ee_pos;
for (int ee = 0; ee < num_ee; ++ee) {
    Eigen::Vector3d p = base_pos + nominal_stance_B.at(ee);
    p(2) = z_ground;
    initial_ee_pos.push_back(p);
}

optjump.SetInitialBaseState(base_pos, base_ang);  
optjump.SetInitialEEState(initial_ee_pos);
optjump.SetJumpLength(2.0);
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