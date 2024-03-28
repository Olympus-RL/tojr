#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <chrono>


#include <towr/optjump.h>
#include <towr/terrain/terrain_data.h>
#include <towr/terrain/examples/height_map_examples.h>
#include <towr/terrain/terrain_parser.h>
#include <towr/models/robot_model.h>
#include <towr/variables/variable_names.h>

using namespace Eigen;
using namespace towr;


int main() {

double num_ee = 4;
double z_ground = 0.0;
  
TerrainData terrain_data;
//ParseTerrainData(terrain_data);
towr::RobotModel model = towr::RobotModel(towr::RobotModel::Olympus);
towr::HeightMap::Ptr terrain = towr::HeightMap::MakeTerrain(towr::HeightMap::FlatID);

auto nominal_stance_B = model.kinematic_model_->GetNominalStanceInBase();
double nominal_height = -nominal_stance_B.front().z();
z_ground = terrain -> GetHeight(0.0, 0.0);
towr::OptJump::Vector3d base_pos(0.0, 0.0, z_ground + nominal_height);
towr::OptJump::Vector3d base_ang(0.0, 0.0, 0.0);
towr::OptJump::EEpos initial_ee_pos;
for (int ee = 0; ee < num_ee; ++ee) {
    Eigen::Vector3d p = base_pos + nominal_stance_B.at(ee);
    p(2) = z_ground;
    initial_ee_pos.push_back(p);
}

const int num_runs = 100;
for (const auto& solver_type : {OptJump::SolverType::IPOPT}) {
  OptJump optjump(model, terrain, solver_type);
  optjump.SetInitialBaseState(base_pos, base_ang);  
  optjump.SetInitialEEState(initial_ee_pos);
  optjump.SetJumpLength(2.0);
  optjump.SetSolverOption("print_level", 0);

  for (std::string lin_solver : {"ma27","ma57","ma77","ma86","ma97"}){
  optjump.SetSolverOption("linear_solver", lin_solver);
  //optjump.SetSolverOption("print_timing_statistics", "yes");}


  auto start_time = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < num_runs; i++) {
       
      optjump.Solve();
  }
  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
  std::cout << lin_solver << " average solve time: "  << duration/static_cast<float>(num_runs) << " milliseconds" << std::endl;
}
}

  return 0;
}