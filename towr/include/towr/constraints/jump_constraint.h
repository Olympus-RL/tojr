
#ifndef CONSTRAINTS_JUMP_CONSTRAINT_H_
#define CONSTRAINTS_JUMP_CONSTRAINT_H_

#include <towr/variables/spline_holder.h>
#include <towr/variables/spline.h>
#include <towr/terrain/height_map.h>
#include <ifopt/constraint_set.h>



namespace towr {

class JumpConstraint : public ifopt::ConstraintSet {
public:

  JumpConstraint(double g, double land_x, const SplineHolder& spline_holder,const HeightMap::Ptr& terrain, double ground_clearance, double num_eval_points);
  ~JumpConstraint() = default;


  VectorXd GetValues() const override;
  VecBound GetBounds() const override;
  void FillJacobianBlock (std::string var_set, Jacobian&) const override;
  void InitVariableDependedQuantities(const VariablesPtr& x) override;

private:
  NodeSpline::Ptr base_linear_;
  HeightMap::Ptr terrain_;    ///< the height map of the current terrain.
  double ground_clearance_;
  double num_eval_points_;
  double g_;
  double land_x_;
  ifopt::VariableSet::Ptr jump_duration;
};

} // namespace towr

#endif /* CONSTRAINTS_JUMP_CONSTRAINT_H_ */