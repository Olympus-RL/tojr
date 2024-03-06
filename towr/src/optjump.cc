#include <towr/optjump.h>
#include <towr/terrain/terrain_data.h>
#include <towr/terrain/examples/height_map_examples.h>
#include <towr/variables/jump_duration.h>

namespace towr {

OptJump::OptJump(const RobotModel& model, const TerrainData& terrain_data){
    formulation_.model_ = model;
    terrain_ = std::make_shared<TerrainFromData>(terrain_data);
    //terrain_ = HeightMap::MakeTerrain(HeightMap::FlatID);
    formulation_.terrain_ = terrain_;
    //formulation_.params_.costs_.push_back(std::pair<Parameters::CostName, double>(Parameters::ForcesCostID, 1.0));
    solver_ = std::make_shared<ifopt::IpoptSolver>();
    
}
void OptJump::SetSolverOption(const std::string& name, const std::string& value) {
    solver_->SetOption(name, value);
}
void OptJump::SetSolverOption(const std::string& name, int value) {
    solver_->SetOption(name, value);
}
void OptJump::SetSolverOption(const std::string& name, double value) {
    solver_->SetOption(name, value);
}
void OptJump::SetInitialBaseState(const Vector3d pos, const Vector3d ang) {
    initial_base_.lin.at(kPos) = pos;
    initial_base_.ang.at(kPos) = ang;
}
void OptJump::SetInitialEEState(const EEpos initial_ee_pos) {
    initial_ee_pos_ = initial_ee_pos;
}
void OptJump::SetJumpLength(const double length) {
    jump_length_ = length;
}
void OptJump::Solve() {
    std::cout << "Solving jump" << std::endl;
    formulation_.jump_length_ = jump_length_;
    formulation_.initial_base_ = initial_base_;
    formulation_.initial_ee_W_ = initial_ee_pos_;
	int num_ee = initial_ee_pos_.size();
  formulation_.params_.ee_phase_durations_.clear();
  formulation_.params_.ee_in_contact_at_start_.clear();
    for (int ee=0; ee<num_ee; ++ee) {
      formulation_.params_.ee_phase_durations_.push_back({1.0}); // this should be able to modify
      formulation_.params_.ee_in_contact_at_start_.push_back(true);
    }
  nlp_ = ifopt::Problem();
  for (auto c : formulation_.GetVariableSets(solution_))
    nlp_.AddVariableSet(c);
  for (auto c : formulation_.GetConstraints(solution_))
    nlp_.AddConstraintSet(c);
  for (auto c : formulation_.GetCosts())
    nlp_.AddCostSet(c);


  solver_->Solve(nlp_);
}

OptJump::BaseTrajectory OptJump::GetBaseTrajectory(double dt) const {
  BaseTrajectory traj;
  for (double t=0.0; t<=solution_.base_linear_->GetTotalTime(); t+=dt){
    Vector3d pos = solution_.base_linear_->GetPoint(t).p();
    Vector3d vel = solution_.base_linear_->GetPoint(t).v();
    Vector3d ang = solution_.base_angular_->GetPoint(t).p();
    Vector3d ang_vel = solution_.base_angular_->GetPoint(t).v();
    Eigen::Matrix<double,12,1> traj_t;
    traj_t << pos,ang,vel,ang_vel;
    traj.push_back(traj_t);
 }
    return traj;
 }

OptJump::EETrajectory OptJump::GetEETrajectory(double dt) const {
  EETrajectory traj;
  for (double t=0.0; t<=solution_.ee_motion_.front()->GetTotalTime(); t+=dt){
    EEpos traj_t;
    for (int ee=0; ee<solution_.ee_motion_.size(); ++ee) {
      traj_t.push_back(solution_.ee_motion_.at(ee)->GetPoint(t).p());
    }
    traj.push_back(traj_t);
  }
  return traj;
}

double OptJump::GetTakeoffTime() const {
  return solution_.base_linear_->GetTotalTime();
}

double OptJump::GetJumpDuration() const {
  return nlp_.GetOptVariables() -> GetComponent<towr::JumpDuration>("jump_duration") -> GetValues()(0);
}
} /* namespace towr */