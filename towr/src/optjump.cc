#include <towr/optjump.h>
#include <towr/terrain/terrain_data.h>
#include <towr/terrain/examples/height_map_examples.h>
#include <towr/variables/jump_duration.h>

namespace towr {

OptJump::OptJump() : OptJump(RobotModel(RobotModel::Olympus), HeightMap::MakeTerrain(HeightMap::FlatID), SolverType::IPOPT) {
    // Delegate to the other constructor
}

OptJump::OptJump(const RobotModel& model, HeightMap::Ptr terrain, SolverType solver_type) {
  takeoff_duration_ = 1.0;
  formulation_.model_ = model;
  terrain_ = terrain;
  double h = terrain_->GetHeight(0.0,0.0);
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

void 
OptJump::SetInitialBaseState(const Vector3d pos, const Vector3d ang) {
    initial_base_.lin.at(kPos) = pos;
    initial_base_.ang.at(kPos) = ang;
}
void 
OptJump::SetInitialEEState(const EEpos initial_ee_pos) {
    initial_ee_pos_ = initial_ee_pos;
}
void 
OptJump::SetJumpLength(const double length) {
    jump_length_ = length;
}

bool
OptJump::Solve() {
  formulation_.jump_length_ = jump_length_;
  formulation_.initial_base_ = initial_base_;
  formulation_.initial_ee_W_ = initial_ee_pos_;
	int num_ee = initial_ee_pos_.size();
  formulation_.params_.ee_phase_durations_.clear();
  formulation_.params_.ee_in_contact_at_start_.clear();
    for (int ee=0; ee<num_ee; ++ee) {
      formulation_.params_.ee_phase_durations_.push_back({takeoff_duration_}); // this should be able to modify
      formulation_.params_.ee_in_contact_at_start_.push_back(true);
    }
  nlp_ = ifopt::Problem();
  for (auto c : formulation_.GetVariableSets(solution_))
    nlp_.AddVariableSet(c);
  for (auto c : formulation_.GetConstraints(solution_))
    nlp_.AddConstraintSet(c);
  for (auto c : formulation_.GetCosts())
    nlp_.AddCostSet(c);

  int solver_status;
  switch (solver_type_)
  {
  case SolverType::IPOPT:
    ipopt_solver_->Solve(nlp_);
    solver_status = ipopt_solver_->GetReturnStatus();
    
    break;
  case SolverType::SNOPT:
    snopt_solver_->Solve(nlp_);
    InitSnopt(); //for some reson you have to do this
    solver_status = snopt_solver_->GetReturnStatus();
    break;
  default:
    break;
  }
  return solver_status == 0;
}


OptJump::StateVector 
OptJump::GetBaseState(double t) const {
  StateVector state;
  if (t < solution_.base_linear_->GetTotalTime()) {
    towr::State base_lin = solution_.base_linear_->GetPoint(t);
    towr::State base_ang = solution_.base_angular_->GetPoint(t);
    state << base_lin.p(), base_ang.p(), base_lin.v(), base_ang.v();
    return state;
  }
  towr::State takeoff_lin = solution_.base_linear_->GetPoint(solution_.base_linear_->GetTotalTime());
  towr::State takeoff_ang = solution_.base_angular_->GetPoint(solution_.base_linear_->GetTotalTime());
  double dt = std::min<double>(t - solution_.base_linear_->GetTotalTime(),GetJumpDuration());
  double dx = takeoff_lin.v().x() * dt;
  double dy = takeoff_lin.v().y() * dt;
  double dz = -0.5 * formulation_.model_.dynamic_model_->g() * std::pow(dt, 2) + takeoff_lin.v().z() * dt;
  Vector3d dp(dx, dy, dz);
  Vector3d dv(0.0, 0.0, -formulation_.model_.dynamic_model_->g() * dt);

  state << takeoff_lin.p() + dp, takeoff_ang.p(), takeoff_lin.v() + dv, takeoff_ang.v();
  return state;
}

OptJump::EEpos 
OptJump::GetEEPos(double t) const {
  EEpos ee_pos;
  double takeoff_time = solution_.base_linear_->GetTotalTime();

  if (t < takeoff_time) {
    for (int ee=0; ee<solution_.ee_motion_.size(); ++ee) {
      ee_pos.push_back(solution_.ee_motion_.at(ee)->GetPoint(t).p());
    }
    return ee_pos;
  }

  towr::State takeoff_lin = solution_.base_linear_->GetPoint(takeoff_time);
  double dt = std::min<double>(t - takeoff_time,GetJumpDuration());
  double dx = takeoff_lin.v().x() * dt;
  double dy = takeoff_lin.v().y() * dt;
  double dz = -0.5 * formulation_.model_.dynamic_model_->g() * std::pow(dt, 2) + takeoff_lin.v().z() * dt;
  Vector3d dp(dx, dy, dz);

  for (int ee=0; ee<solution_.ee_motion_.size(); ++ee) {
    ee_pos.push_back(solution_.ee_motion_.at(ee)->GetPoint(takeoff_time).p()+dp);
  }
  return ee_pos;
}


OptJump::BaseTrajectory OptJump::GetBaseTrajectory(double dt) const {
  BaseTrajectory traj;
  for (double t=0.0; t<=solution_.base_linear_->GetTotalTime(); t+=dt){
    traj.push_back(GetBaseState(t));
  }
    return traj;
  }


OptJump::EETrajectory OptJump::GetEETrajectory(double dt) const {
  EETrajectory traj;
  for (double t=0.0; t<=solution_.ee_motion_.front()->GetTotalTime(); t+=dt){
    traj.push_back(GetEEPos(t));
  }
  return traj;
}

double 
OptJump::GetTakeoffTime() const {
  return solution_.base_linear_->GetTotalTime();
}

double 
OptJump::GetJumpDuration() const {
  return nlp_.GetOptVariables() -> GetComponent<towr::JumpDuration>("jump_duration") -> GetValues()(0);
}

void 
OptJump::SetTakeoffDuration(const double duration) {
  takeoff_duration_ = duration;
}

void OptJump::InitIpopt() {
  ipopt_solver_ = std::make_shared<ifopt::IpoptSolver>();
  ipopt_solver_->SetOption("linear_solver", "ma57");
}

void OptJump::InitSnopt() {
  snopt_solver_ = std::make_shared<ifopt::SnoptSolver>();
}
} /* namespace towr */