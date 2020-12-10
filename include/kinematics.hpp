#include <Eigen/Dense>
#include <iostream>
#include "algebra.hpp"

//TODO add eomg, ev to auto generated configuration file
class Kinematics {
  
public:
  //TODO move this to a general configuration file.
  // true for space state, false for body state
  bool space_state;
  
  Kinematics(bool state=true);
  
  /** Compute end effector frame
   *
   * @param M, Home configuration of end-effector
   * @param Slist joint screw axis in (space_state by default, unless space_state is false for body state), when the manipulator is at home position.
   * @param thetaList A list of joint coordinates.
   * @return T transformation matrix representing the end-effector frame when the joints are at the specified coordinates
  */
  Eigen::MatrixXd ForwardKin(const Eigen::MatrixXd &M,
                             const Eigen::MatrixXd& Slist,
                             const Eigen::VectorXd& thetaList);
  
  /** Compute joints thetas given the end-effector configuration
   *
   * @param Slist screw axis (space_state by default, 
   *              unless space_state is false for body state),
   * @param M is home configuration of end-effector
   * @param T is the target configuration
   * @param thetalist: the the desired  joint angles to calculate
   * @param eomg, ev are error checkers
   * @return boolean success flag
   */
  bool InverseKin(const Eigen::MatrixXd& Slist,
                  const Eigen::MatrixXd& M,
                  const Eigen::MatrixXd& T,
                  Eigen::VectorXd& thetalist,
                  double eomg, double ev);
  
  /** Gives the Jacobian
   *
   * @param Slist Screw axis
   * @param thetaList joint configuration
   * @return 6xn Spatial Jacobian
   */
  Eigen::MatrixXd Jacobian(const Eigen::MatrixXd& Slist,
                           const Eigen::MatrixXd& thetaList);
  
private:
  /** 
   * Compute end effector frame (used for current  spatial position calculation)
   * 
   @param M Home configuration (position and orientation) of end-effector The joint screw axes in the space frame when the manipulator is at the home position.
   @param Slist of screw axis vectors representing velocity.
   @param thetaList A list of joint coordinates.
   @return T Transfomation matrix representing the end-effector frame when the joints are at the specified coordinates
   */
  inline Eigen::MatrixXd
  FKinSpace(const Eigen::MatrixXd& M,
            const Eigen::MatrixXd& Slist,
            const Eigen::VectorXd& thetaList) const {
    Eigen::MatrixXd T = M;
    for (int i = (thetaList.size() - 1); i > -1; i--) {
      T = Algebra::MatrixExp6(Algebra::VecTose3(Slist.col(i)*thetaList(i))) * T;
    }
    return T;
  }
  
  /** Compute end effector frame (used for current  spatial position calculation)
   *
   @param M Home configuration (position and orientation) of end-effector The joint screw axes in the body frame when the manipulator is at the home position.
   @param Blist list of screw axis vectors representing velocity.
   @param thetaList A list of joint coordinates.
   @return T Transfomation matrix representing the end-effector frame when the joints are at the specified coordinates
   */
  inline Eigen::MatrixXd
  FKinBody(const Eigen::MatrixXd& M,
           const Eigen::MatrixXd& Blist,
           const Eigen::VectorXd& thetaList) const {
    Eigen::MatrixXd T = M;
    for (int i = 0; i < thetaList.size(); i++) {
      T = T * Algebra::MatrixExp6(Algebra::VecTose3(Blist.col(i)*thetaList(i)));
    }
    return T;
  }
  
  /** Inverse Kinematics in body frame, given the mechanism configurations, IKinSpace derives the joints angles.
   *
   * @param Blist body frame when the manipulator is at the home position
   * @param M Home configuration (position and orientation) of end-effector The joint screw axes in the 
   * @param thetaList A list of joint coordinates.
   * @param T Transfomation matrix representing the end-effector frame when the joints are at the specified coordinates
   * @param eomg allowable difference between home configuration, and specified coordinates, as long as the angular difference is greater than this value then iterate.
   * @param ev allowable difference to iterate through between the current configuration, and the specified configuration.
   * @return err boolean True if the iteration is ended without reaching the allowable limits determined by eomg, and ev.
   */
  inline bool IKinBody(const Eigen::MatrixXd& Blist,
                       const Eigen::MatrixXd& M,
                       const Eigen::MatrixXd& T,
                       Eigen::VectorXd& thetalist,
                       double eomg, double ev) const {
    int i = 0;
    //add this to the auto-generated configuration file
    int maxiterations = 20;
    Eigen::MatrixXd Tfk = FKinBody(M, Blist, thetalist);
    Eigen::MatrixXd Tdiff = Algebra::TransInv(Tfk)*T;
    Eigen::VectorXd Vb = Algebra::se3ToVec(Algebra::MatrixLog6(Tdiff));
    Eigen::Vector3d angular(Vb(0), Vb(1), Vb(2));
    Eigen::Vector3d linear(Vb(3), Vb(4), Vb(5));
    
    bool err = (angular.norm() > eomg || linear.norm() > ev);
    Eigen::MatrixXd Jb;
    while (err && i < maxiterations) {
      Jb = JacobianBody(Blist, thetalist);
      thetalist += Jb.bdcSvd(Eigen::ComputeThinU |
                             Eigen::ComputeThinV).solve(Vb);
      i += 1;
      // iterate
      Tfk = FKinBody(M, Blist, thetalist);
      Tdiff = Algebra::TransInv(Tfk)*T;
      Vb = Algebra::se3ToVec(Algebra::MatrixLog6(Tdiff));
      angular = Eigen::Vector3d(Vb(0), Vb(1), Vb(2));
      linear = Eigen::Vector3d(Vb(3), Vb(4), Vb(5));
      err = (angular.norm() > eomg || linear.norm() > ev);
    }
    return !err;
  }

  /** Inverse Kinematics in Space frame, given the mechanism configurations, IKinSpace derives the joints angles.
   *
   * @param Slist space frame when the manipulator is at the home position
   * @param M Home configuration (position and orientation) of end-effector The joint screw axes in the 
   * @param thetaList A list of joint coordinates.
   * @param T Transfomation matrix representing the end-effector frame when the joints are at the specified coordinates
   * @param eomg allowable difference between home configuration, and specified coordinates, as long as the angular difference is greater than this value then iterate.
   * @param ev allowable difference to iterate through between the current configuration, and the specified configuration.
   * @return err boolean True if the iteration is ended without reaching the allowable limits determined by eomg, and ev.
   */
  inline bool IKinSpace(const Eigen::MatrixXd& Slist,
                        const Eigen::MatrixXd& M,
                        const Eigen::MatrixXd& T,
                        Eigen::VectorXd& thetalist,
                        double eomg, double ev) const {
    int i = 0;
    //TODO add this to a config file
    int maxiterations = 20;
    Eigen::MatrixXd Tfk = FKinSpace(M, Slist, thetalist);
    Eigen::MatrixXd Tdiff = Algebra::TransInv(Tfk)*T;
    Eigen::VectorXd Vs = Algebra::Adjoint(Tfk)*Algebra::se3ToVec(Algebra::MatrixLog6(Tdiff));
    Eigen::Vector3d angular(Vs(0), Vs(1), Vs(2));
    Eigen::Vector3d linear(Vs(3), Vs(4), Vs(5));
    
    bool err = (angular.norm() > eomg || linear.norm() > ev);
    Eigen::MatrixXd Js;
    while (err && i < maxiterations) {
      Js = JacobianSpace(Slist, thetalist);
      thetalist += Js.bdcSvd(Eigen::ComputeThinU |
                             Eigen::ComputeThinV).solve(Vs);
      i += 1;
      // iterate
      Tfk = FKinSpace(M, Slist, thetalist);
      Tdiff = Algebra::TransInv(Tfk)*T;
      Vs = Algebra::Adjoint(Tfk)*Algebra::se3ToVec(Algebra::MatrixLog6(Tdiff));
      angular = Eigen::Vector3d(Vs(0), Vs(1), Vs(2));
      linear = Eigen::Vector3d(Vs(3), Vs(4), Vs(5));
      err = (angular.norm() > eomg || linear.norm() > ev);
    }
    return !err;
  }

  /** Gives the Jacobian in space frame
   *
   * @param Slist Screw axis in space frame
   * @param thetaList joint configuration
   * @return 6xn Spatial Jacobian
   */
  inline Eigen::MatrixXd
  JacobianSpace(const Eigen::MatrixXd& Slist,
                const Eigen::MatrixXd& thetaList) const {
    Eigen::MatrixXd Js = Slist;
    Eigen::MatrixXd T = Eigen::MatrixXd::Identity(4, 4);
    Eigen::VectorXd sListTemp(Slist.col(0).size());
    for (int i = 1; i < thetaList.size(); i++) {
      sListTemp << Slist.col(i - 1) * thetaList(i - 1);
      T = T * Algebra::MatrixExp6(Algebra::VecTose3(sListTemp));
      // std::cout << "array: " << sListTemp << std::endl;
      Js.col(i) = Algebra::Adjoint(T) * Slist.col(i);
    }
    return Js;
  }
  
  /** Gives the Jacobian in body frame
   *
   * @param Blist Screw axis in body frame
   * @param thetaList joint configuration
   * @return 6xn Spatial Jacobian
   */
  inline Eigen::MatrixXd
  JacobianBody(const Eigen::MatrixXd& Blist,
               const Eigen::MatrixXd& thetaList) const {
    Eigen::MatrixXd Jb = Blist;
    Eigen::MatrixXd T = Eigen::MatrixXd::Identity(4, 4);
    Eigen::VectorXd bListTemp(Blist.col(0).size());
    for (int i = thetaList.size() - 2; i >= 0; i--) {
      bListTemp << Blist.col(i + 1) * thetaList(i + 1);
      T = T * Algebra::MatrixExp6(VecTose3(-1 * bListTemp));
      // std::cout << "array: " << sListTemp << std::endl;
      Jb.col(i) = Algebra::Adjoint(T) * Blist.col(i);
    }
    return Jb;
  }  
};
