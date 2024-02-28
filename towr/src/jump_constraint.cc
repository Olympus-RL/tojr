#include <towr/constraints/jump_constraint.h>
#include <towr/variables/jump_duration.h>

#include <towr/variables/variable_names.h>
#include <towr/variables/cartesian_dimensions.h>
#include <towr/variables/spline_holder.h>

#include <iostream>

namespace towr {

JumpConstraint::JumpConstraint(double g, double land_x, const SplineHolder& spline_holder,const HeightMap::Ptr& terrain, double ground_clearance, double num_eval_points)
    : ifopt::ConstraintSet(kSpecifyLater, "jump_constraint")
{
  base_linear_  = spline_holder.base_linear_;
  terrain_ = terrain;
  ground_clearance_ = ground_clearance;
  num_eval_points_ = num_eval_points;
  g_ = g;
  land_x_ = land_x;

  SetRows(num_eval_points_+2);
}

void
JumpConstraint::InitVariableDependedQuantities(const VariablesPtr& x)
{
  jump_duration = x->GetComponent<JumpDuration>("jump_duration");
}

Eigen::VectorXd 
JumpConstraint::GetValues() const
{
  VectorXd g(GetRows());
  towr::State::VectorXd takeoff_pos = base_linear_->GetPoint(base_linear_->GetTotalTime()).p();
  towr::State::VectorXd takeoff_vel = base_linear_->GetPoint(base_linear_->GetTotalTime()).v();

  double current_jump_height = takeoff_pos.z() + std::pow(takeoff_vel.z(), 2) / (2 * g_);
  double delta_x = land_x_ - takeoff_pos.x();

  //v_y = 0
  //v_x*jump_duration = delta_x
  // h - h_terreain > ground_clearance for all 0 < t < jump_duration, // h - h_terrain = ground_clearance for t = jump_duration
  
  g(0) = takeoff_vel.y(); // v_y = 0
  g(1) = takeoff_vel.x() * jump_duration->GetValues()(0) - delta_x; // v_x*jump_duration = delta_x
  double t = 0;
  double dt =jump_duration->GetValues()(0) / num_eval_points_;
  for (int i = 2; i < num_eval_points_+2; i++) {
    t += dt;
    double x = takeoff_pos.x() + takeoff_vel.x() * t;
    double z = takeoff_pos.z() + takeoff_vel.z() * t - 0.5*g_*std::pow(t,2);
    double ground_height = terrain_->GetHeight(x, takeoff_pos.y());
    g(i) = z - ground_height - ground_clearance_;
  }
  return g;
}

JumpConstraint::VecBound
JumpConstraint::GetBounds() const
{
  VecBound bounds;
  bounds.push_back(ifopt::BoundZero); // v_y = 0
  bounds.push_back(ifopt::BoundZero); // v_x*jump_duration - delta_x = 0
  for (int i = 2; i < num_eval_points_+1 ; i++) {
    bounds.push_back(ifopt::BoundGreaterZero); 
  }
  bounds.push_back(ifopt::Bounds(-0.05,0.05)); // h - h_terrain = ground_clearance for t = jump_duration
  return bounds;
}

void
JumpConstraint::FillJacobianBlock (std::string var_set, Jacobian& jac) const
{
  int n = jac.cols();
  if (var_set == id::base_lin_nodes) {
    towr::State::VectorXd takeoff_pos = base_linear_->GetPoint(base_linear_->GetTotalTime()).p();
    towr::State::VectorXd takeoff_vel = base_linear_->GetPoint(base_linear_->GetTotalTime()).v();
    Jacobian jac_pos_params = base_linear_->GetJacobianWrtNodes(base_linear_->GetTotalTime(),kPos);
    Jacobian jac_vel_params= base_linear_->GetJacobianWrtNodes(base_linear_->GetTotalTime(),kVel);
    
    // constraint jacobian with respect to base_pos
    Jacobian jac_con_pos(GetRows(),3);
    Jacobian jac_con_vel(GetRows(),3);
    // v_y = 0 => J_con_pos = [0, 1, 0]
    jac_con_vel.insert(0,1) = 1.0; 
    // v_x*jump_duration - (land_x - takeoff_x) = 0 => J_con_pos = [1, 0, 0], J_con_vel = [jump_duration, 0,0]
    jac_con_pos.insert(1,0) = 1.0; 
    jac_con_vel.insert(1,0) = jump_duration->GetValues()(0);
    

    // (z + v_z*t - 0.5*g*t^2 - h_terrain(x + v_x*t,y)) > ground_clerance => J_con_pos = [-h_x,-h_y, 1], J_con_vel = [t*hx, 0, t]
    double t = 0;
    double dt =jump_duration->GetValues()(0) / num_eval_points_;
    for (int i = 2; i < num_eval_points_+2; i++) {
      t += dt;
      double x = takeoff_pos.x() + takeoff_vel.x() *t;
      double hx = terrain_->GetDerivativeOfHeightWrt(Dim2D::X_, x, takeoff_pos.y());
      double hy = terrain_->GetDerivativeOfHeightWrt(Dim2D::Y_, x, takeoff_pos.y());
      jac_con_pos.insert(i,0) = -hx;
      jac_con_pos.insert(i,1) = -hy;
      jac_con_pos.insert(i,2) = 1;
      jac_con_vel.insert(i,0) = t*hx;
      
    }

    // sum up to get full constraint jacobian
    Jacobian constraint_jac(GetRows(),n);
    constraint_jac = jac_con_pos * jac_pos_params + jac_con_vel * jac_vel_params;
    jac.middleRows(0,GetRows()) = constraint_jac;
  }

  if (var_set == jump_duration->GetName()) {
    towr::State::VectorXd takeoff_pos = base_linear_->GetPoint(base_linear_->GetTotalTime()).p();
    towr::State::VectorXd takeoff_vel = base_linear_->GetPoint(base_linear_->GetTotalTime()).v();
    jac.insert(1,0) = takeoff_vel.x(); // v_x*jump_duration - (land_x - takeoff_x) = 0 => J_con_T = [v_x, 0,0]
    // h - h_terrain = z + v_z*t - 0.5*g*t^2 - h_terrain(x + v_x*t,y), t = (i-2)*T/n => J_con_t = [v_z - g*t - h_x*v_x], J_t_T = (i-2)/n
    double t = 0;
    double dt =jump_duration->GetValues()(0) / num_eval_points_;
    for (int i = 2; i < num_eval_points_+2; i++) {
      t += dt;
      double x = takeoff_pos.x() + takeoff_vel.x() *t;
      double hx = terrain_->GetDerivativeOfHeightWrt(Dim2D::X_, x, takeoff_pos.y());
      double J_con_t = takeoff_vel.z() - g_*t - hx*takeoff_vel.x();
      double J_t_T = (i-2)/num_eval_points_;

      jac.insert(i,0) = J_con_t*J_t_T;
    }
  }
}
} /* namespace towr */