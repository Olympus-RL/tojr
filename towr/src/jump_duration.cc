#include <towr/variables/jump_duration.h>

namespace towr {


JumpDuration::JumpDuration (const Duration duration)
    // -1 since last phase-duration is not optimized over, but comes from total time
    :VariableSet(1, "jump_duration")
{
  duration_ = duration;
  jump_duration_bounds_ = ifopt::BoundGreaterZero;
}

Eigen::VectorXd
JumpDuration::GetValues () const
{
  VectorXd x(GetRows());
  x(0) = duration_;
  return x;
}

void
JumpDuration::SetVariables (const VectorXd& x)
{

  duration_ = x(0);

}

JumpDuration::VecBound
JumpDuration::GetBounds () const
{
  VecBound bounds;
  bounds.push_back(jump_duration_bounds_);
  return bounds;
}
} /* namespace towr */