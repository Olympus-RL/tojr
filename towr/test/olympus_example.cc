/******************************************************************************
Copyright (c) 2018, Alexander W. Winkler. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <cmath>
#include <iostream>

#include <towr/terrain/examples/height_map_examples.h>
#include <towr/nlp_formulation.h>
#include <ifopt/ipopt_solver.h>


using namespace towr;

// A minimal example how to build a trajectory optimization problem using TOWR.
//
// The more advanced example that includes ROS integration, GUI, rviz
// visualization and plotting can be found here:
// towr_ros/src/towr_ros_app.cc
int main()
{
  NlpFormulation formulation;
  double num_ee = 4;
  double total_duration = 1.0;
  double z_ground = 0.0;
  
  	
  towr::RobotModel model = towr::RobotModel();
  auto slope = std::make_shared<Slope>();
  formulation.model_ = RobotModel(RobotModel::Olympus);
  formulation.terrain_ = slope;
  formulation.jump_length_ = 4.0;
  
  double pitch = -atan2(0.7, 1.0);
  pitch = 0.0;
  double x_0_ground = 0.0;
  double y_0_ground = 0.0;
  z_ground = slope -> GetHeight(x_0_ground, y_0_ground);
  HeightMap::Vector3d normal = slope -> GetNormalizedBasis(HeightMap::Direction::Normal, x_0_ground, y_0_ground);
  HeightMap::Vector3d x_tangent = slope -> GetNormalizedBasis(HeightMap::Direction::Tangent1, x_0_ground, y_0_ground);
  HeightMap::Vector3d y_tangent = slope -> GetNormalizedBasis(HeightMap::Direction::Tangent2, x_0_ground, y_0_ground);

  towr::DynamicModel::Matrix3d W_R_B;
  W_R_B.col(0) = x_tangent;
  W_R_B.col(1) = y_tangent;
  W_R_B.col(2) = normal;

  
  auto nominal_stance_B = formulation.model_.kinematic_model_->GetNominalStanceInBase();
  double nominal_height = -nominal_stance_B.front().z();
  std::cout << "nominal_height: " << nominal_height << std::endl;
  
 
  formulation.initial_base_.lin.at(kPos) = Eigen::Vector3d(x_0_ground, y_0_ground, z_ground) + normal*nominal_height;
  formulation.initial_base_.ang.at(kPos) = Eigen::Vector3d(0.0, pitch, 0.0);
  for (int ee=0; ee<num_ee; ++ee) {
    formulation.initial_ee_W_.push_back(formulation.initial_base_.lin.at(kPos)+ W_R_B*nominal_stance_B.at(ee));
  }

  

  // Parameters that define the motion. See c'tor for default values or
  // other values that can be modified.
  // First we define the initial phase durations, that can however be changed
  // by the optimizer. The number of swing and stance phases however is fixed.
  // alternating stance and swing:     ____-----_____-----_____-----_____
	
  for (int ee=0; ee<num_ee; ++ee) {
      formulation.params_.ee_phase_durations_.push_back({total_duration});
      formulation.params_.ee_in_contact_at_start_.push_back(true);
  }

  //formulation.params_.costs_.push_back(std::pair<Parameters::CostName, double>(Parameters::ForcesCostID, 1.0));

  
	//formulation.params_.OptimizePhaseDurations();
  
	// Initialize the nonlinear-programming problem with the variables,
  // constraints and costs.
  ifopt::Problem nlp;
  SplineHolder solution;
  for (auto c : formulation.GetVariableSets(solution))
    nlp.AddVariableSet(c);
  for (auto c : formulation.GetConstraints(solution))
    nlp.AddConstraintSet(c);
  for (auto c : formulation.GetCosts())
    nlp.AddCostSet(c);

  // You can add your own elements to the nlp as well, simply by calling:
  // nlp.AddVariablesSet(your_custom_variables);
  // nlp.AddConstraintSet(your_custom_constraints);

  // Choose ifopt solver (IPOPT or SNOPT), set some parameters and solve.
  // solver->SetOption("derivative_test", "first-order");
  auto solver = std::make_shared<ifopt::IpoptSolver>();
  solver->SetOption("jacobian_approximation", "exact"); // "finite difference-values"
  solver->SetOption("max_cpu_time", 20.0);
  solver->Solve(nlp);

  // Can directly view the optimization variables through:
  // Eigen::VectorXd x = nlp.GetVariableValues()
  // However, it's more convenient to access the splines constructed from these
  // variables and query their values at specific times:
  using namespace std;
  cout.precision(2);
  nlp.PrintCurrent(); // view variable-set, constraint violations, indices,...
  cout << fixed;
  cout << "\n====================\nQuadruped trajectory:\n====================\n";

  double t = 0.0;
  while (t<=solution.base_linear_->GetTotalTime() + 1e-5) {
    cout << "t=" << t << "\n";
    cout << "Base linear position x,y,z:   \t";
    cout << solution.base_linear_->GetPoint(t).p().transpose() << "\t[m]" << endl;

    cout << "Base linear vel x,y,z:  \t";
    cout << solution.base_linear_->GetPoint(t).v().transpose() << "\t[m]" << endl;

    cout << "Base Euler roll, pitch, yaw:  \t";
    Eigen::Vector3d rad = solution.base_angular_->GetPoint(t).p();
    cout << (rad/M_PI*180).transpose() << "\t[deg]" << endl;

		for (int ee=0; ee<num_ee; ++ee) {
			bool contact = solution.phase_durations_.at(ee)->IsContactPhase(t);
			// print which foot is in contact or not
			std::string foot_in_contact = contact? "yes" : "no";
			std::cout << "End Effector " << ee << " in contact: " << foot_in_contact << std::endl;
		}


    cout << endl;

    t += 0.1;
  }
}