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

#include <towr_ros/towr_ros_interface.h>
#include <towr_ros/terrain_parser.h>

#include <std_msgs/Int32.h>

#include <xpp_states/convert.h>
#include <xpp_msgs/topic_names.h>
#include <xpp_msgs/TerrainInfo.h>

#include <towr/terrain/height_map.h>
#include <towr/terrain/terrain_data.h>
#include <towr/variables/euler_converter.h>
#include <towr_ros/topic_names.h>
#include <towr_ros/towr_xpp_ee_map.h>

#include <towr/variables/spline_holder.h>





namespace towr {


TowrRosInterface::TowrRosInterface ()
{

  std::cout << "TowrRosInterface_start" << std::endl;
  ::ros::NodeHandle n;

  user_command_sub_ = n.subscribe(towr_msgs::user_command, 1,
                                  &TowrRosInterface::UserCommandCallback, this);

  initial_state_pub_  = n.advertise<xpp_msgs::RobotStateCartesian>
                                          (xpp_msgs::robot_state_desired, 1);

  robot_parameters_pub_  = n.advertise<xpp_msgs::RobotParameters>
                                    (xpp_msgs::robot_parameters, 1);
  
  TerrainData terrain_data;
  std::cout << "TowrRosInterface_pre_terrain" << std::endl;
  ParseTerrainData(terrain_data);
  std::cout << "TowrRosInterface_terrain" << std::endl;

  towr::RobotModel model = towr::RobotModel(towr::RobotModel::Olympus);
  std::cout << "TowrRosInterface_model" << std::endl;
  optjump_ = std::make_shared<OptJump>(model, terrain_data);
  std::cout << "TowrRosInterface_optjump" << std::endl;
  terrain_ = optjump_->GetTerrain();


  model_ = model;
  visualization_dt_ = 0.01;
  std::cout << "TowrRosInterface_end" << std::endl;
}

BaseState
TowrRosInterface::GetGoalState(const TowrCommandMsg& msg) const
{
  BaseState goal;
  goal.lin.at(kPos) = xpp::Convert::ToXpp(msg.goal_lin.pos);
  goal.lin.at(kVel) = xpp::Convert::ToXpp(msg.goal_lin.vel);
  goal.ang.at(kPos) = xpp::Convert::ToXpp(msg.goal_ang.pos);
  goal.ang.at(kVel) = xpp::Convert::ToXpp(msg.goal_ang.vel);

  return goal;
}

void
TowrRosInterface::UserCommandCallback(const TowrCommandMsg& msg)
{
  // robot model
  auto robot_params_msg = BuildRobotParametersMsg(model_);
  robot_parameters_pub_.publish(robot_params_msg);

  int n_ee = model_.kinematic_model_->GetNumberOfEndeffectors();

  SetTowrInitialState(msg.x_0,msg.y_0);
  optjump_ -> SetJumpLength(msg.x_land - msg.x_0);

  // solver parameters
  SetIpoptParameters(msg);
  // visualization
  PublishInitialState();

  // Defaults to /home/user/.ros/
  std::string bag_file = "towr_trajectory.bag";
  if (msg.optimize || msg.play_initialization) {
    std::cout << "Optimizing trajectory..." << std::endl;
    optjump_ -> Solve();
    std::cout << "Optimization finished." << std::endl;
    SaveOptimizationAsRosbag(bag_file, robot_params_msg, msg, false);
  }

  // playback using terminal commands
  if (msg.replay_trajectory || msg.play_initialization || msg.optimize) {
    std::cout << "Replaying trajectory..." << std::endl;
    int success = system(("rosbag play --topics "
        + xpp_msgs::robot_state_desired + " "
        + xpp_msgs::terrain_info
        + " -r " + std::to_string(msg.replay_speed)
        + " --quiet " + bag_file).c_str());
    std::cout << "Replay finished." << std::endl;
  }

  if (msg.plot_trajectory) {
    int success = system(("killall rqt_bag; rqt_bag " + bag_file + "&").c_str());
  }

  // to publish entire trajectory (e.g. to send to controller)
  // xpp_msgs::RobotStateCartesianTrajectory xpp_msg = xpp::Convert::ToRos(GetTrajectory());
}

void
TowrRosInterface::PublishInitialState()
{
  std::cout << "Publish initial state" << std::endl;
  int n_ee = 4;
  xpp::RobotStateCartesian xpp(n_ee);
  xpp.base_.lin.p_ = initial_base_pos_;
  xpp.base_.ang.q  = EulerConverter::GetQuaternionBaseToWorld(initial_base_ang_);

  for (int ee_towr=0; ee_towr<n_ee; ++ee_towr) {
    int ee_xpp = ToXppEndeffector(n_ee, ee_towr).first;
    xpp.ee_contact_.at(ee_xpp)   = true;
    xpp.ee_motion_.at(ee_xpp).p_ = initial_ee_pos_.at(ee_towr);
    xpp.ee_forces_.at(ee_xpp).setZero(); // zero for visualization
  }

  initial_state_pub_.publish(xpp::Convert::ToRos(xpp));
}

std::vector<TowrRosInterface::XppVec>
TowrRosInterface::GetIntermediateSolutions ()
{
  std::vector<XppVec> trajectories;
  ifopt::Problem nlp_ = optjump_ -> GetNlp();

  for (int iter=0; iter<nlp_.GetIterationCount(); ++iter) {
    nlp_.SetOptVariables(iter);
    trajectories.push_back(GetTrajectory());
  }

  return trajectories;
}

TowrRosInterface::XppVec
TowrRosInterface::GetTrajectory () const
{
  std::cout << "GetTrajectory" << std::endl;
  XppVec trajectory;
  SplineHolder solution = optjump_ -> GetSolution();
  double t = 0.0;
  double T = solution.base_linear_->GetTotalTime();
  double T_jump = optjump_ -> GetJumpDuration();
  EulerConverter base_angular(solution.base_angular_);
  int n_ee = solution.ee_motion_.size();
  while (t<=T+1e-5) {
    xpp::RobotStateCartesian state(n_ee);

    state.base_.lin = ToXpp(solution.base_linear_->GetPoint(t));

    state.base_.ang.q  = base_angular.GetQuaternionBaseToWorld(t);
    state.base_.ang.w  = base_angular.GetAngularVelocityInWorld(t);
    state.base_.ang.wd = base_angular.GetAngularAccelerationInWorld(t);

    for (int ee_towr=0; ee_towr<n_ee; ++ee_towr) {
      int ee_xpp = ToXppEndeffector(n_ee, ee_towr).first;

      state.ee_contact_.at(ee_xpp) = solution.phase_durations_.at(ee_towr)->IsContactPhase(t);
      state.ee_motion_.at(ee_xpp)  = ToXpp(solution.ee_motion_.at(ee_towr)->GetPoint(t));
      state.ee_forces_ .at(ee_xpp) = solution.ee_force_.at(ee_towr)->GetPoint(t).p();
    }

    state.t_global_ = t;
    trajectory.push_back(state);
    t += visualization_dt_;
  }
  towr::State::VectorXd takeoff_pos = solution.base_linear_->GetPoint(solution.base_linear_->GetTotalTime()).p();
  towr::State::VectorXd takeoff_vel = solution.base_linear_->GetPoint(solution.base_linear_->GetTotalTime()).v();
  towr::State::VectorXd takeoff_ang = solution.base_angular_->GetPoint(solution.base_angular_->GetTotalTime()).p();
  
  xpp::RobotStateCartesian takeoff_state = trajectory.back();
  
  t = 0;
  double g = model_.dynamic_model_->g();
  while (t<=T_jump+1e-5) {
    xpp::RobotStateCartesian state = takeoff_state;

    int n_ee = solution.ee_motion_.size();
    double dx = takeoff_vel.x()*t;
    double dy = takeoff_vel.y()*t;
    double dz = takeoff_vel.z()*t - 0.5*g*std::pow(t,2);

    state.base_.lin.p_.x() += dx;
    state.base_.lin.p_.y() += dy;
    state.base_.lin.p_.z() += dz;

    for (int ee_towr=0; ee_towr<n_ee; ++ee_towr) {
      int ee_xpp = ToXppEndeffector(n_ee, ee_towr).first;
      state.ee_contact_.at(ee_xpp) = false;
      state.ee_motion_.at(ee_xpp).p_.x() += dx;
      state.ee_motion_.at(ee_xpp).p_.y() += dy;
      state.ee_motion_.at(ee_xpp).p_.z() += dz;
      state.ee_forces_ .at(ee_xpp) = Vector3d(0,0,0);
    }

    state.t_global_ = t + T;
    trajectory.push_back(state);
    t += visualization_dt_;



    
  }

  return trajectory;
}

xpp_msgs::RobotParameters
TowrRosInterface::BuildRobotParametersMsg(const RobotModel& model) const
{
  std::cout << "BuildRobotParametersMsg" << std::endl;
  xpp_msgs::RobotParameters params_msg;
  auto max_dev_xyz = model.kinematic_model_->GetMaximumDeviationFromNominal();
  params_msg.ee_max_dev = xpp::Convert::ToRos<geometry_msgs::Vector3>(max_dev_xyz);

  auto nominal_B = model.kinematic_model_->GetNominalStanceInBase();
  int n_ee = nominal_B.size();
  for (int ee_towr=0; ee_towr<n_ee; ++ee_towr) {
    Vector3d pos = nominal_B.at(ee_towr);
    params_msg.nominal_ee_pos.push_back(xpp::Convert::ToRos<geometry_msgs::Point>(pos));
    params_msg.ee_names.push_back(ToXppEndeffector(n_ee, ee_towr).second);
  }

  params_msg.base_mass = model.dynamic_model_->m();

  return params_msg;
}

void
TowrRosInterface::SaveOptimizationAsRosbag (const std::string& bag_name,
                                   const xpp_msgs::RobotParameters& robot_params,
                                   const TowrCommandMsg user_command_msg,
                                   bool include_iterations)
{
  std::cout << "SaveOptimizationAsRosbag" << std::endl;
  rosbag::Bag bag;
  bag.open(bag_name, rosbag::bagmode::Write);
  ::ros::Time t0(1e-6); // t=0.0 throws ROS exception

  // save the a-priori fixed optimization variables
  bag.write(xpp_msgs::robot_parameters, t0, robot_params);
  bag.write(towr_msgs::user_command+"_saved", t0, user_command_msg);

  // save the trajectory of each iteration
  if (include_iterations) {
    auto trajectories = GetIntermediateSolutions();
    int n_iterations = trajectories.size();
    for (int i=0; i<n_iterations; ++i)
      SaveTrajectoryInRosbag(bag, trajectories.at(i), towr_msgs::nlp_iterations_name + std::to_string(i));

    // save number of iterations the optimizer took
    std_msgs::Int32 m;
    m.data = n_iterations;
    bag.write(towr_msgs::nlp_iterations_count, t0, m);
  }

  // save the final trajectory
  auto final_trajectory = GetTrajectory();
  SaveTrajectoryInRosbag(bag, final_trajectory, xpp_msgs::robot_state_desired);

  bag.close();
}

void
TowrRosInterface::SaveTrajectoryInRosbag (rosbag::Bag& bag,
                                 const XppVec& traj,
                                 const std::string& topic) const
{
  std::cout << "SaveTrajectoryInRosbag" << std::endl;
  for (const auto state : traj) {
    auto timestamp = ::ros::Time(state.t_global_ + 1e-6); // t=0.0 throws ROS exception

    xpp_msgs::RobotStateCartesian msg;
    msg = xpp::Convert::ToRos(state);
    bag.write(topic, timestamp, msg);

    xpp_msgs::TerrainInfo terrain_msg;
    for (auto ee : state.ee_motion_.ToImpl()) {
      Vector3d n = terrain_->GetNormalizedBasis(HeightMap::Normal, ee.p_.x(), ee.p_.y());
      terrain_msg.surface_normals.push_back(xpp::Convert::ToRos<geometry_msgs::Vector3>(n));
      terrain_msg.friction_coeff = terrain_->GetFrictionCoeff();
    }

    bag.write(xpp_msgs::terrain_info, timestamp, terrain_msg);
  }
}

} /* namespace towr */

