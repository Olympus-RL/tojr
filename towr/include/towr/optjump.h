#ifndef OPT_JUMP_H_
#define OPT_JUMP_H_

#include <ifopt/variable_set.h>
#include <ifopt/constraint_set.h>
#include <ifopt/cost_term.h>
#include <ifopt/ipopt_solver.h>

#include <towr/variables/spline_holder.h>
#include <towr/models/robot_model.h>
#include <towr/terrain/height_map.h>
#include <towr/terrain/terrain_data.h>
#include <towr/nlp_formulation.h>
#include <towr/parameters.h>
#include <towr/variables/jump_duration.h>

namespace towr {

class OptJump {
  public:
    using Matrix = Eigen::MatrixXd;
    using VectorXd = Eigen::VectorXd;
    using Vector3d = Eigen::Vector3d;
    using Inertia = Eigen::Matrix3d;
    using EEpos = std::vector<Vector3d>;
    using BaseTrajectory = std::vector<Eigen::Matrix<double,12,1>>;
    using EETrajectory = std::vector<EEpos>;
    using Solver = ifopt::IpoptSolver::Ptr;
    using Ptr = std::shared_ptr<OptJump>;

    OptJump(const RobotModel& model, const TerrainData& terrain_data);
    OptJump(const double mass, const Inertia& inertia, const TerrainData& terrain_data);
    OptJump(const double mass, const Inertia& inertia, const VectorXd& terrain_x, const VectorXd& terrain_y, const VectorXd& terrain_z);
    
    virtual ~OptJump() = default;
    void SetInitialBaseState(Vector3d pos, Vector3d ang);
    void SetInitialEEState(EEpos initial_ee_pos);
    void SetJumpLength(const double length);
    void Solve();
    BaseTrajectory GetBaseTrajectory(double dt) const;
    EETrajectory GetEETrajectory(double dt) const;
    double GetTakeoffTime() const;
    double GetJumpDuration() const;
    const SplineHolder& GetSolution() const{return solution_;};
    HeightMap::Ptr GetTerrain() const{return terrain_;};
    ifopt::Problem& GetNlp() {return nlp_;};
   
  private:
    NlpFormulation formulation_;
    RobotModel model_;
    BaseState initial_base_;
    EEpos initial_ee_pos_;
    SplineHolder solution_;
    double jump_length_;
    ifopt::Problem nlp_;
    Solver solver_;
    HeightMap::Ptr terrain_;
};

} /* namespace towr */

#endif /* OPT_JUMP_H_ */
   