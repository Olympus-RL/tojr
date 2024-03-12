#include <future>
#include <towr/vec_optjump.h>
#include <towr/terrain/terrain_data.h>
#include <towr/terrain/examples/height_map_examples.h>
#include <towr/variables/jump_duration.h>

namespace towr {

VecOptJump::VecOptJump(const RobotModel& model, const TerrainData& terrain_data,int num_envs){
    num_envs_ = num_envs;
    terrain_ = std::make_shared<TerrainFromData>(terrain_data);
    for (int i=0; i<num_envs; ++i) {
      formulations_.push_back(NlpFormulation());
      formulations_.back().model_ = model;
      formulations_.back().terrain_ = terrain_;
      solvers_.push_back(std::make_shared<ifopt::IpoptSolver>());
      initial_base_.push_back(BaseState());
      solutions_.push_back(SplineHolder());
    }
}
//void VecOptJump::SetSolverOption(const std::string& name, const std::string& value) {
//    std::for_each(solvers_.begin(), solvers_.end(), [&](Solver& s){s->SetOption(name, value);});
//}
//void VecOptJump::SetSolverOption(const std::string& name, int value) {
//    std::for_each(solvers_.begin(), solvers_.end(), [&](Solver& s){s->SetOption(name, value);});
//}
//void VecOptJump::SetSolverOption(const std::string& name, double value) {
//    std::for_each(solvers_.begin(), solvers_.end(), [&](Solver& s){s->SetOption(name, value);});
//}
void VecOptJump::SetInitialBaseStates(std::vector<Vector3d> pos, std::vector<Vector3d> ang){
    for (int i=0; i<num_envs_; ++i) {
      initial_base_.at(i).lin.at(kPos) = pos.at(i);
      initial_base_.at(i).ang.at(kPos) = ang.at(i);
    }
};
void VecOptJump::SetInitialEEStates(std::vector<EEpos>  initial_ee_pos)
{
  initial_ee_pos_ = initial_ee_pos;
};
void VecOptJump::SetJumpLengths(const std::vector<double>& lengths){
    jump_lengths_ = lengths;

};

void VecOptJump::Solve() {
    auto solveSingleEnv = [this](int i){
      formulations_.at(i).jump_length_ = jump_lengths_.at(i);
      formulations_.at(i).initial_base_ = initial_base_.at(i);
      formulations_.at(i).initial_ee_W_ = initial_ee_pos_.at(i);
      int num_ee = initial_ee_pos_.at(i).size();
      formulations_.at(i).params_.ee_phase_durations_.clear();
      formulations_.at(i).params_.ee_in_contact_at_start_.clear();
      for (int ee=0; ee<num_ee; ++ee) {
        formulations_.at(i).params_.ee_phase_durations_.push_back({1.0}); // this should be able to modify
        formulations_.at(i).params_.ee_in_contact_at_start_.push_back(true);
      }
      ifopt::Problem nlp;
      for (auto c : formulations_.at(i).GetVariableSets(solutions_.at(i)))
        nlp.AddVariableSet(c);
      for (auto c : formulations_.at(i).GetConstraints(solutions_.at(i)))
        nlp.AddConstraintSet(c);
      for (auto c : formulations_.at(i).GetCosts())
        nlp.AddCostSet(c);
      solvers_.at(i)->Solve(nlp);
    };

    //std::vector<std::future<void>> futures(num_envs_);
    //for (int i = 0; i < num_envs_; ++i) {
    //    futures[i] = std::async(std::launch::async, solveSingleEnv, i);
    //}
//
    //// Wait for all futures to finish
    //for (auto& future : futures) {
    //    future.wait();
    //}

    for (int i = 0; i < num_envs_; ++i) {
        solveSingleEnv(i);
    }

    std::cout << "Solutions: " << solutions_.size() << std::endl;

    std::for_each(solutions_.begin(), solutions_.end(), [](SplineHolder& s){
      using namespace std;
      cout.precision(2);
      cout << fixed;

      double t = 0.0;
      while (t<=s.base_linear_->GetTotalTime() + 1e-5) {
        cout << "t=" << t << "\n";
        cout << "Base linear position x,y,z:   \t";
        cout << s.base_linear_->GetPoint(t).p().transpose() << "\t[m]" << endl;

        cout << "Base Euler roll, pitch, yaw:  \t";
        Eigen::Vector3d rad = s.base_angular_->GetPoint(t).p();
        cout << (rad/M_PI*180).transpose() << "\t[deg]" << endl;

        cout << endl;

        t += 0.2;
      }
    });
    
}


} /* namespace towr */