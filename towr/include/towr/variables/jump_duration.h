#ifndef VARIABLES_JUMP_DURATIONS_H_
#define VARIABLES_JUMP_DURATIONS_H_

#include <ifopt/variable_set.h>


namespace towr {

/**
 * @brief A variable set composed of the phase durations of an endeffector.
 *
 * This class holds the current variables determining the alternating swing-
 * and stance durations of one foot. These durations are then used in the
 * Spline together with NodeVariables to construct foot motions and forces.
 *
 * See this explanation: https://youtu.be/KhWuLvb934g?t=788
 *
 * @ingroup Variables
 */
class JumpDuration : public ifopt::VariableSet {
public:
  using Ptr = std::shared_ptr<JumpDuration>;
  using Duration  = double;
 


  /**
   * @brief Constructs a variable set for a specific endeffector
   * @param initial_duration  Initial values for the optimization variables.
   */
  JumpDuration (const Duration initial_duration);
  virtual ~JumpDuration () = default;


  /**
   * @returns The optimization variable (jump duration [s]).
   */
  VectorXd GetValues() const override;

  /**
   * @brief  Sets the phase durations from pure Eigen optimization variables.
   */
  void SetVariables(const VectorXd& x) override;

  /**
   * @returns The maximum and minimum time each phase is allowed to take.
   */
  VecBound GetBounds () const override;



private:
  Duration duration_;
  ifopt::Bounds jump_duration_bounds_;

};

} /* namespace towr */

#endif /* TOWR_VARIABLES_CONTACT_SCHEDULE_H_ */