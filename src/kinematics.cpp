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

bool
Kinematics::InverseKin(const Eigen::MatrixXd& T,
                       Eigen::VectorXd& thetaList) {
  bool err;
  if (space_frame)
    err=IKinSpace(T, thetaList);
  else
    err=IKinBody(T, thetaList);
  return err;
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
