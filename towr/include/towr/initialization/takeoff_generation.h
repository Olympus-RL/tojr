#ifndef TOWR_TAKEOFF_GEN_H_
#define TOWR_TAKEOFF_GEN_H_

#include <towr/variables/nodes_variables_all.h>
#include <towr/variables/state.h>
#include <towr/parameters.h>
#include <towr/models/robot_model.h>
#include <towr/variables/polynomial.h>
#include <towr/variables/jump_duration.h>
#include <numeric>
#include <iostream>



namespace towr {

using Vector3d = Eigen::Vector3d;

inline Eigen::Vector3d CalculateTakeoffVel(const double& dx, const double& dz, const double& g) {
    assert(dx > 0);
    Eigen::Vector3d vel;
    //assume 45 degree takeoff angle
    double under_root = 2.0/g*(dx+dz);
    if (under_root > 0){
        double v =dx/sqrt(under_root);
        vel << v, 0, v;
        return vel;
    }
    //if 45 degree takeoffangle is to low we recalculates
    double alpha = -dz/dx + 0.1;
    under_root = 2.0/g*(alpha*dx+dz);
    double v = dx/sqrt(under_root);
    vel << v, 0, alpha*v;
    return vel;
};

inline void GenerateTakeoffTrajectory(
    NodesVariablesAll::Ptr nodes_lin, 
    NodesVariablesAll::Ptr nodes_ang, 
    JumpDuration::Ptr jump_duration,
    const towr::BaseState& initial_base,
    const Parameters::VecTimes& poly_duarations,
    const double& takeoff_height,
    const double& land_height,
    const double& jump_lenght,
    const double& gravity
) {

    double total_duration = std::accumulate(poly_duarations.begin(), poly_duarations.end(),0.0);

    towr::BaseState takeoff_state;
    Vector3d takeoff_pos = initial_base.lin.at(kPos);
    Vector3d takeoff_vel = CalculateTakeoffVel(jump_lenght, land_height - takeoff_height, gravity);
    double t_flight = jump_lenght/takeoff_vel(0);
    takeoff_pos(2) = takeoff_height;
    takeoff_state.lin.at(kPos) = takeoff_pos;
    takeoff_state.lin.at(kVel) = takeoff_vel;
    takeoff_state.ang.at(kPos) = Vector3d(0,0,0);
    takeoff_state.ang.at(kVel) = Vector3d(0,0,0);
    
    CubicHermitePolynomial base(3);
    CubicHermitePolynomial ang(3);
    base.SetDuration(total_duration);
    base.SetNodes(initial_base.lin, takeoff_state.lin);
    base.UpdateCoeff(); 
    ang.SetDuration(total_duration);
    ang.SetNodes(initial_base.ang, takeoff_state.ang);
    ang.UpdateCoeff();

    nodes_lin -> FitToPolynomial(base, poly_duarations);
    nodes_ang -> FitToPolynomial(ang, poly_duarations);
    Eigen::VectorXd duration_vars(1);
    duration_vars << t_flight;
    jump_duration -> SetVariables(duration_vars);
};


} /* namespace towr */


#endif  // TOWR_TAKEOFF_GEN_H_