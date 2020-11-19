#include "../include/kinematics.hpp"


Kinematics::Kinematics(bool state=true) {
  space_state=state;
}


/* Function: Compute end effector frame
 * Inputs: M, Home configuration of end-effector
 *         Slist joint screw axis in (space_state by default, 
 *         unless space_state is false for body state),
 *          when the manipulator is at home position.
 *         thetaList A list of joint coordinates.
 * Returns: transformation matrix representing the end-effector 
 *          frame when the joints are at the specified coordinates
 */
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


/* Function Compute joints-theta given the end-effector configuration
 * Inputs: Slist screw axis (space_state by default, 
 *         unless space_state is false for body state),
 *         M is home configuration of end-effector
 *         T is the target configuration
 *         thetalist: the the desired  joint angles to calculate
 *         eomg, ev are error checkers
 * Returns: boolean success flag
 */
bool
Kinematics::InverseKin(const Eigen::MatrixXd& Slist,
                       const Eigen::MatrixXd& M,
                       const Eigen::MatrixXd& T,
                       Eigen::VectorXd& thetalist,
                       double eomg, double ev) {
  bool err;
  if (space_state)
    err=IkinSpace(Slist, M, T, thealist, eomg, ev);
  else
    err=IkinBody(Slist, M, T, thealist, eomg, ev);
  return err;
}


/* Function: Gives the  Jacobian
 * Inputs: Screw axis, joint configuration
 * Returns: 6xn Spatial Jacobian
 */
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
