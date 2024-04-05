#ifndef OPT_JUMP_H_
#define OPT_JUMP_H_

#include <ifopt/variable_set.h>
#include <ifopt/constraint_set.h>
#include <ifopt/cost_term.h>
#include <ifopt/ipopt_solver.h>
#include <ifopt/snopt_solver.h>

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
    using StateVector = Eigen::Matrix<double,12,1>;
    using BaseTrajectory = std::vector<StateVector>;
    using EETrajectory = std::vector<EEpos>;
    using IpoptSolver = ifopt::IpoptSolver::Ptr;
    using SnoptSolver = ifopt::SnoptSolver::Ptr;
    using Ptr = std::shared_ptr<OptJump>;
    
    enum SolverType {
      IPOPT,
      SNOPT
    };
    

    OptJump();
    OptJump(const RobotModel& model, HeightMap::Ptr terrain, SolverType solver_type);
    
    virtual ~OptJump() = default;
    
    bool Solve();
    StateVector GetBaseState(double t) const;
    EEpos GetEEPos(double t) const;
    BaseTrajectory GetBaseTrajectory(double dt) const;
    EETrajectory GetEETrajectory(double dt) const;
    double GetTakeoffTime() const;
    double GetJumpDuration() const;
    const SplineHolder& GetSolution() const{return solution_;};
    HeightMap::Ptr GetTerrain() const{return terrain_;};
    ifopt::Problem& GetNlp() {return nlp_;};
    

    void SetInitialBaseState(Vector3d pos, Vector3d ang);
    void SetInitialEEState(EEpos initial_ee_pos);
    void SetJumpLength(const double length);
    void SetTakeoffDuration(const double duration);
    void SetSolverOption(const std::string& name, const std::string& value);
    void SetSolverOption(const std::string& name, int value);
    void SetSolverOption(const std::string& name, double value);
   
  private:
    NlpFormulation formulation_;
    RobotModel model_;
    BaseState initial_base_;
    EEpos initial_ee_pos_;
    SplineHolder solution_;
    double jump_length_;
    double takeoff_duration_;
    ifopt::Problem nlp_;
    IpoptSolver ipopt_solver_;
    SnoptSolver snopt_solver_;
    HeightMap::Ptr terrain_;
    SolverType solver_type_;

    void InitIpopt();
    void InitSnopt();
};

} /* namespace towr */

#endif /* OPT_JUMP_H_ */
   