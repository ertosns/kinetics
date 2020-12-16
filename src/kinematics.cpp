#include "../include/kinematics.hpp"


Eigen::MatrixXd
Kinematics::ForwardKin(const Eigen::MatrixXd &M,
                       const Eigen::MatrixXd& Slist,
                       const Eigen::VectorXd& thetaList) {
  Eigen::MatrixXd T;
  if (space_state)
    T = FKinSpace(M, Slist, thetaList);
  else
    T = FKinBody(M, Slist, thetaList);
  return T;
}

bool
Kinematics::InverseKin(const Eigen::MatrixXd& Slist,
                       const Eigen::MatrixXd& M,
                       const Eigen::MatrixXd& T,
                       Eigen::VectorXd& thetalist,
                       double eomg, double ev) {
  bool err;
  if (space_state)
    err=IKinSpace(Slist, M, T, thetalist, eomg, ev);
  else
    err=IKinBody(Slist, M, T, thetalist, eomg, ev);
  return err;
}



Eigen::MatrixXd
Kinematics::Jacobian(const Eigen::MatrixXd& Slist,
                     const Eigen::MatrixXd& thetaList) {
  Eigen::MatrixXd Jb;
  if (space_state)
    Jb = JacobianSpace(Slist, thetaList);
  else
    Jb = JacobianBody(Slist, thetaList);
  return Jb;
}
