#include <towr/optjump.h>
#include <towr/terrain/examples/height_map_examples.h>
#include <towr/models/robot_model.h>
#include <towr/variables/variable_names.h>
#include <iostream>


using namespace Eigen;
using namespace towr;

int main() {

double num_ee = 4;
double z_ground = 0.0;
  
auto terrain = std::make_shared<towr::FlatGround>(z_ground);
towr::RobotModel model = towr::RobotModel(towr::RobotModel::Olympus);
auto nominal_stance_B = model.kinematic_model_->GetNominalStanceInBase();
double nominal_height = -nominal_stance_B.front().z();

towr::OptJump::Vector3d base_pos(0.0, 0.0, z_ground + nominal_height);
towr::OptJump::Vector3d base_ang(0.0, 0.0, 0.0);

towr::OptJump::EEpos initial_ee_pos;
for (int ee = 0; ee < num_ee; ++ee) {
    initial_ee_pos.push_back(base_pos + nominal_stance_B.at(ee));
}

towr::OptJump optjump;
optjump.SetTerrain(terrain);
optjump.SetModel(model);
  optjump.SetTerrain(terrain);
  optjump.SetModel(model);
  optjump.SetInitialBaseState(base_pos, base_ang);  
  optjump.SetInitialEEState(initial_ee_pos);
  optjump.SetJumpLenght(4.0);
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