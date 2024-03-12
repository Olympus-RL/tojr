#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <chrono>


#include <towr/vec_optjump.h>
#include <towr/terrain/terrain_data.h>
#include <towr/terrain/terrain_parser.h>
#include <towr/models/robot_model.h>
#include <towr/variables/variable_names.h>

using namespace Eigen;
using namespace towr;


int main() {
const int num_envs = 28;
double num_ee = 4;
double z_ground = 0.0;
  
TerrainData terrain_data;
ParseTerrainData(terrain_data);
towr::RobotModel model = towr::RobotModel(towr::RobotModel::Olympus);
towr::VecOptJump optjump(model, terrain_data,num_envs);

auto nominal_stance_B = model.kinematic_model_->GetNominalStanceInBase();
double nominal_height = -nominal_stance_B.front().z();
z_ground = optjump.GetTerrain() -> GetHeight(0.0, 0.0);


std::vector<towr::VecOptJump::Vector3d> vec_pos;
std::vector<towr::VecOptJump::Vector3d> vec_ang;
std::vector<towr::VecOptJump::EEpos> vec_EE_pos;
std::vector<double> vec_jump_lengths;

for (int i=0; i<num_envs; ++i) {
    towr::VecOptJump::Vector3d pos(0.0, 0.0, z_ground + nominal_height);
    towr::VecOptJump::Vector3d ang(0.0, 0.0, 0.0);
    towr::VecOptJump::EEpos initial_ee_pos;
    for (int ee = 0; ee < num_ee; ++ee) {
        Eigen::Vector3d p = pos + nominal_stance_B.at(ee);
        p(2) = z_ground;
        initial_ee_pos.push_back(p);
    }
    vec_pos.push_back(pos);
    vec_ang.push_back(ang);
    vec_EE_pos.push_back(initial_ee_pos);
    vec_jump_lengths.push_back(2.0);
}

optjump.SetInitialBaseStates(vec_pos, vec_ang);
optjump.SetInitialEEStates(vec_EE_pos);
optjump.SetJumpLengths(vec_jump_lengths);

// time to solve the problem
// Start the timer
  auto start = std::chrono::steady_clock::now();

  // Call the Solve() method
  optjump.Solve();

  // Stop the timer
  auto end = std::chrono::steady_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

  // Output the execution time
  std::cout << "Execution time: " << duration.count() << " microseconds" << std::endl;


}