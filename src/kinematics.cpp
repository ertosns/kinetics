#include "../include/kinematics.hpp"

Eigen::MatrixXd
Kinematics::ForwardKin(const Eigen::VectorXd& thetaList) {
  Eigen::MatrixXd T;
  if (space_frame)
    T = FKinSpace(thetaList);
  else
    T = FKinBody(thetaList);
  return T;
}

Eigen::VectorXd
Kinematics::InverseKin(const Eigen::MatrixXd& T,
                       Eigen::VectorXd thetaList) {
  if (space_frame)
    thetaList=IKinSpace(T, thetaList);
  else
    thetaList=IKinBody(T, thetaList);
  return thetaList;
}

Eigen::MatrixXd
Kinematics::Jacobian(const Eigen::MatrixXd& thetaList) {
  Eigen::MatrixXd Jb;
  if (space_frame)
    Jb = JacobianSpace(thetaList);
  else
    Jb = JacobianBody(thetaList);
  return Jb;
}
