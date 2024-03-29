#include <towr/optjump.h>
#include <towr/terrain/terrain_data.h>
#include <towr/terrain/examples/height_map_examples.h>
#include <towr/variables/jump_duration.h>

namespace towr {

OptJump::OptJump() {
  OptJump(RobotModel(RobotModel::Olympus),HeightMap::MakeTerrain(HeightMap::FlatID),SolverType::IPOPT);
}

OptJump::OptJump(const RobotModel& model, HeightMap::Ptr terrain, SolverType solver_type) {
  formulation_.model_ = model;
  terrain_ = terrain;
  formulation_.terrain_ = terrain_;
  solver_type_ = solver_type;
  
  switch (solver_type_)
  {
  case SolverType::IPOPT:
    InitIpopt();
    break;
  case SolverType::SNOPT:
    InitSnopt();
    break;
  default:
    assert(false); // unknown solver type
  }

}

    
void OptJump::SetSolverOption(const std::string& name, const std::string& value) {
    switch (solver_type_)
    {
    case SolverType::IPOPT:
      ipopt_solver_->SetOption(name, value);
      break;
    case SolverType::SNOPT:
      snopt_solver_->SetOption(name, value);
      break;
    default:
      break;
    }
    
}
void OptJump::SetSolverOption(const std::string& name, int value) {
    switch (solver_type_)
    {
    case SolverType::IPOPT:
      ipopt_solver_->SetOption(name, value);
      break;
    case SolverType::SNOPT:
      snopt_solver_->SetOption(name, value);
      break;
    default:
      break;
    }
}
void OptJump::SetSolverOption(const std::string& name, double value) {
    switch (solver_type_)
    {
    case SolverType::IPOPT:
      ipopt_solver_->SetOption(name, value);
      break;
    case SolverType::SNOPT:
      snopt_solver_->SetOption(name, value);
      break;
    default:
      break;
    }
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


  switch (solver_type_)
  {
  case SolverType::IPOPT:
    ipopt_solver_->Solve(nlp_);
    break;
  case SolverType::SNOPT:
    snopt_solver_->Solve(nlp_);
    InitSnopt(); //for some reson you have to do this
    break;
  default:
    break;
  }
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

void OptJump::InitIpopt() {
  ipopt_solver_ = std::make_shared<ifopt::IpoptSolver>();
  ipopt_solver_->SetOption("linear_solver", "ma57");
}

void OptJump::InitSnopt() {
  snopt_solver_ = std::make_shared<ifopt::SnoptSolver>();
}
} /* namespace towr */