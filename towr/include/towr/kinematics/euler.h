#ifndef TOWR_KINEMATICS_EULER_H_
#define TOWR_KINEMATICS_EULER_H_

#include <Eigen/Dense>
#include <Eigen/Sparse>

using Matrix3d = Eigen::Matrix3d;
using Vector3d = Eigen::Vector3d;
using EulerAngels = Vector3d;

namespace towr {
Matrix3d EulerZYXToRotMat (const EulerAngels& e);
Matrix3d EulerZYXToAngVel(const EulerAngels& e, const EulerAngels& e_dot);
Matrix3d EulerZYXToAngAcc(const EulerAngels& e, const EulerAngels& e_dot, const EulerAngels& e_ddot);
Matrix3d GetJacobian


} /* namespace towr */
#endif /* TOWR_KINEMATICS_EULER_H_ */