#ifndef OPT_JUMP_H_
#define OPT_JUMP_H_

#include <ifopt/variable_set.h>
#include <ifopt/constraint_set.h>
#include <ifopt/cost_term.h>
#include <ifopt/ipopt_solver.h>

#include <towr/variables/spline_holder.h>
#include <towr/models/robot_model.h>
#include <towr/terrain/height_map.h>
#include <towr/nlp_formulation.h>
#include <towr/parameters.h>
#include <towr/variables/jump_duration.h>

namespace towr {

class OptJump {
  public:
    using Vector3d = Eigen::Vector3d;
    using EEpos = std::vector<Vector3d>;
    using BaseTrajectory = std::vector<Eigen::Matrix<double,12,1>>;
    using EETrajectory = std::vector<EEpos>;
    using Solver = ifopt::IpoptSolver::Ptr;
    OptJump();
    virtual ~OptJump() = default;
    void SetTerrain(const HeightMap::Ptr terrain);
    void SetModel(const RobotModel& model); 
    void SetInitialBaseState(Vector3d pos, Vector3d ang);
    void SetInitialEEState(EEpos initial_ee_pos);
    void SetJumpLenght(const double lenght);
    void Solve();
    BaseTrajectory GetBaseTrajectory(double dt) const;
    EETrajectory GetEETrajectory(double dt) const;
    
  private: 
    NlpFormulation formulation_;
    HeightMap::Ptr terrain_;
    RobotModel model_;  
    BaseState initial_base_;
    EEpos initial_ee_pos_;
    SplineHolder solution_;
    double jump_length_;
    ifopt::Problem nlp_;
    Solver solver_;

};

} /* namespace towr */

#endif /* OPT_JUMP_H_ */

