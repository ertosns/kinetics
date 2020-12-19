#include <Eigen/Dense>
#include <iostream>
#include "algebra.hpp"
#include <cassert>

//TODO add eomg, ev to auto generated configuration file
class Kinematics {
  
public:
  //TODO move this to a general configuration file.
  // true for space state, false for body state
  bool space_frame;
  // eomg, ev are error checkers
  /*
    eomg allowable difference between home configuration, and specified coordinates, as long as the angular difference is greater than this value then iterate.
    ev allowable difference to iterate through between the current configuration, and the specified configuration.
  */
  double eomg;
  double ev;
  Eigen::MatrixXd M;
  /*
    note those must be resized at runtime,
    and make sure assertion isn't on.
  */
  Eigen::MatrixXd Slist;
  Eigen::MatrixXd Blist;
  Kinematics(Eigen::MatrixXd _S, Eigen::MatrixXd _B,
             Eigen::MatrixXd _M, double _eomg=0.01,
             double _ev=0.001) : M(_M), eomg(_eomg), ev(_ev) {
    
    space_frame= (_S.rows()>0) ? true : false;
    //TODO (res) redundancy! actually it can work without the follwoing as long as Slist, Blist isn't initialized yet, you reassign those values directly.
    int nrow,ncol;
    if (space_frame) {
      nrow=_S.rows();
      ncol=_S.cols();
      Slist.resize(nrow,ncol);
      Slist=_S;
    } else {
      nrow=_B.rows();
      ncol=_B.cols();
      Blist.resize(nrow,ncol);
      Blist=_B;
    }
    if (space_frame)
      //"Slist is 6 rows of corresponding (w,v)" 
      assert((Slist.rows()==6));
    else
      //"Blist is 6 rows of corresponding (w,v)" 
      assert((Blist.rows()==6));
  }
  
  /** Compute end effector frame
   * @param thetaList A list of joint coordinates.
   * @return T transformation matrix representing the end-effector frame when the joints are at the specified coordinates
   */
  Eigen::MatrixXd ForwardKin(const Eigen::VectorXd& thetaList);
  
  /** Compute joints thetas given the end-effector configuration
   *
   * @param T is the target configuration
   * @param thetalist: the the desired  joint angles to calculate
   * @return boolean success flag
   */
  bool InverseKin(const Eigen::MatrixXd& T,
                  Eigen::VectorXd& thetalist);
  
  /** Gives the Jacobian
   *
   * @param thetaList joint configuration
   * @return 6xn Spatial Jacobian
   */
  Eigen::MatrixXd Jacobian(const Eigen::MatrixXd& thetaList);
  
private:
  /** 
   * Compute end effector frame (used for current  spatial position calculation)
   * 
   * @param thetaList A list of joint coordinates.
   * @return T Transfomation matrix representing the end-effector frame when the joints are at the specified coordinates
   */
  inline Eigen::MatrixXd
  FKinSpace(const Eigen::VectorXd& thetaList) const {
    Eigen::MatrixXd T = M;
    for (int i = (thetaList.size() - 1); i > -1; i--) {
      T = Algebra::MatrixExp6(Algebra::VecTose3(Slist.col(i)*thetaList(i))) * T;
    }
    return T;
  }
  
  /** Compute end effector frame (used for current  spatial position calculation)
   *
   * @param thetaList A list of joint coordinates.
   * @return T Transfomation matrix representing the end-effector frame when the joints are at the specified coordinates
   */
  inline Eigen::MatrixXd
  FKinBody(const Eigen::VectorXd& thetaList) const {
    Eigen::MatrixXd T = M;
    for (int i = 0; i < thetaList.size(); i++) {
      T = T * Algebra::MatrixExp6(Algebra::VecTose3(Blist.col(i)*thetaList(i)));
    }
    return T;
  }
  
  /** Inverse Kinematics in body frame, given the mechanism configurations, IKinSpace derives the joints angles.
   *
   * @param thetaList A list of joint coordinates.
   * @param T Transfomation matrix representing the end-effector frame when the joints are at the specified coordinates
   * @return err boolean True if the iteration is ended without reaching the allowable limits determined by eomg, and ev.
   */
  inline bool IKinBody(const Eigen::MatrixXd& T,
                       Eigen::VectorXd& thetalist) const {
    int i = 0;
    //add this to the auto-generated configuration file
    int maxiterations = 20;
    Eigen::MatrixXd Tfk = FKinBody(thetalist);
    Eigen::MatrixXd Tdiff = Algebra::TransInv(Tfk)*T;
    Eigen::VectorXd Vb = Algebra::se3ToVec(Algebra::MatrixLog6(Tdiff));
    Eigen::Vector3d angular(Vb(0), Vb(1), Vb(2));
    Eigen::Vector3d linear(Vb(3), Vb(4), Vb(5));
    
    bool err = (angular.norm() > eomg || linear.norm() > ev);
    Eigen::MatrixXd Jb;
    while (err && i < maxiterations) {
      Jb = JacobianBody(thetalist);
      thetalist += Jb.bdcSvd(Eigen::ComputeThinU |
                             Eigen::ComputeThinV).solve(Vb);
      i += 1;
      // iterate
      Tfk = FKinBody(thetalist);
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
   * @param T output Transfomation matrix representing the end-effector frame when the joints are at the specified coordinates
   * @return err boolean True if the iteration is ended without reaching the allowable limits determined by eomg, and ev.
   */
  inline bool IKinSpace(const Eigen::MatrixXd& T,
                        Eigen::VectorXd& thetalist) const {
    int i = 0;
    //TODO add this to a config file
    int maxiterations = 20;
    Eigen::MatrixXd Tfk = FKinSpace(thetalist);
    Eigen::MatrixXd Tdiff = Algebra::TransInv(Tfk)*T;
    Eigen::VectorXd Vs = Algebra::Adjoint(Tfk)*Algebra::se3ToVec(Algebra::MatrixLog6(Tdiff));
    Eigen::Vector3d angular(Vs(0), Vs(1), Vs(2));
    Eigen::Vector3d linear(Vs(3), Vs(4), Vs(5));
    
    bool err = (angular.norm() > eomg || linear.norm() > ev);
    Eigen::MatrixXd Js;
    while (err && i < maxiterations) {
      Js = JacobianSpace(thetalist);
      thetalist += Js.bdcSvd(Eigen::ComputeThinU |
                             Eigen::ComputeThinV).solve(Vs);
      i += 1;
      // iterate
      Tfk = FKinSpace(thetalist);
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
   * @param thetaList joint configuration
   * @return 6xn Spatial Jacobian
   */
  inline Eigen::MatrixXd
  JacobianSpace(const Eigen::MatrixXd& thetaList) const {
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
   * @param thetaList joint configuration
   * @return 6xn Spatial Jacobian
   */
  inline Eigen::MatrixXd
  JacobianBody(const Eigen::MatrixXd& thetaList) const {
    Eigen::MatrixXd Jb = Blist;
    Eigen::MatrixXd T = Eigen::MatrixXd::Identity(4, 4);
    Eigen::VectorXd bListTemp(Blist.col(0).size());
    for (int i = thetaList.size() - 2; i >= 0; i--) {
      bListTemp << Blist.col(i + 1) * thetaList(i + 1);
      T = T * Algebra::MatrixExp6(Algebra::VecTose3(-1 * bListTemp));
      // std::cout << "array: " << sListTemp << std::endl;
      Jb.col(i) = Algebra::Adjoint(T) * Blist.col(i);
    }
    return Jb;
  }  
};
