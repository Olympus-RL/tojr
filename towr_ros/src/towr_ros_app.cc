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

#include <towr/initialization/gait_generator.h>
#include <towr_ros/towr_ros_interface.h>


namespace towr {

/**
 * @brief An example application of using TOWR together with ROS.
 *
 * Build your own application with your own formulation using the building
 * blocks provided in TOWR and following the example below.
 */
class TowrRosApp : public TowrRosInterface {
public:
  /**
   * @brief Sets the feet to nominal position on flat ground and base above.
   */
  void SetTowrInitialState() override
  {
  double pitch = 0.0;
  double x_0_ground = 0.0;
  double y_0_ground = 0.0;
  double z_ground = formulation_.terrain_-> GetHeight(x_0_ground, y_0_ground);
  HeightMap::Vector3d normal = formulation_.terrain_ -> GetNormalizedBasis(HeightMap::Direction::Normal, x_0_ground, y_0_ground);
  HeightMap::Vector3d x_tangent = formulation_.terrain_ -> GetNormalizedBasis(HeightMap::Direction::Tangent1, x_0_ground, y_0_ground);
  HeightMap::Vector3d y_tangent = formulation_.terrain_ -> GetNormalizedBasis(HeightMap::Direction::Tangent2, x_0_ground, y_0_ground);

  towr::DynamicModel::Matrix3d W_R_B;
  W_R_B.col(0) = x_tangent;
  W_R_B.col(1) = y_tangent;
  W_R_B.col(2) = normal;

  
  auto nominal_stance_B = formulation_.model_.kinematic_model_->GetNominalStanceInBase();
  double nominal_height = -nominal_stance_B.front().z();
  
  formulation_.initial_base_.lin.at(kPos) = Eigen::Vector3d(x_0_ground, y_0_ground, z_ground) + normal*nominal_height;
  formulation_.initial_base_.ang.at(kPos) = Eigen::Vector3d(0.0, pitch, 0.0);
  int n_ee = nominal_stance_B.size();
  formulation_.initial_ee_W_.clear();
  std::cout << "n_ee: " << n_ee << std::endl;
  for (int ee=0; ee<n_ee; ++ee) {
    formulation_.initial_ee_W_.push_back(formulation_.initial_base_.lin.at(kPos)+ W_R_B*nominal_stance_B.at(ee));
  }

  }

  /**
   * @brief Sets the parameters required to formulate the TOWR problem.
   */
  Parameters GetTowrParameters(int n_ee, const TowrCommandMsg& msg) const override
  {
    Parameters params;

    for (int ee=0; ee<n_ee; ++ee) {
      params.ee_phase_durations_.push_back({msg.total_duration});
      params.ee_in_contact_at_start_.push_back(true);
   }

    // Here you can also add other constraints or change parameters
    // params.constraints_.push_back(Parameters::BaseRom);

    // increases optimization time, but sometimes helps find a solution for
    // more difficult terrain.
    //if (msg.optimize_phase_durations)
    //  params.OptimizePhaseDurations();

    return params;
  }

  /**
   * @brief Sets the paramters for IPOPT.
   */
  void SetIpoptParameters(const TowrCommandMsg& msg) override
  {
    // the HA-L solvers are alot faster, so consider installing and using
    solver_->SetOption("linear_solver", "mumps"); // ma27, ma57

    // Analytically defining the derivatives in IFOPT as we do it, makes the
    // problem a lot faster. However, if this becomes too difficult, we can also
    // tell IPOPT to just approximate them using finite differences. However,
    // this uses numerical derivatives for ALL constraints, there doesn't yet
    // exist an option to turn on numerical derivatives for only some constraint
    // sets.
    solver_->SetOption("jacobian_approximation", "exact"); // finite difference-values

    // This is a great to test if the analytical derivatives implemented in are
    // correct. Some derivatives that are correct are still flagged, showing a
    // deviation of 10e-4, which is fine. What to watch out for is deviations > 10e-2.
    // solver_->SetOption("derivative_test", "first-order");

    solver_->SetOption("max_cpu_time", 40.0);
    solver_->SetOption("print_level", 5);

    if (msg.play_initialization)
      solver_->SetOption("max_iter", 0);
    else
      solver_->SetOption("max_iter", 3000);
  }
};

} // namespace towr


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "my_towr_ros_app");
  towr::TowrRosApp towr_app;
  ros::spin();

  return 1;
}
