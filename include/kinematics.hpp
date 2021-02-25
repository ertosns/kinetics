#include <Eigen/Dense>
#include <iostream>
#include "algebra/algebra.hpp"
#include <cassert>
#include "logger.hpp"

//TODO add eomg, ev to auto generated configuration file
//TODO add checks on the length of given matrices, and vectors for example length of thetalist must match cols of the Slist, Blist.
class Kinematics : public Logger{

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
    int maxiterations;
    bool log;
    /*
      note those must be resized at runtime,
      and make sure assertion isn't on.
    */
    Eigen::MatrixXd Slist;
    Eigen::MatrixXd Blist;
    Kinematics(Eigen::MatrixXd _S, Eigen::MatrixXd _B,
               Eigen::MatrixXd _M, double _eomg=0.01,
               double _ev=0.001, int iterations=20, bool _log=false)
        : M(_M), eomg(_eomg), ev(_ev), log(_log),
          Logger("kinematics-log.csv"){
        space_frame= (_S.rows()>0) ? true : false;
        //TODO (res) redundancy! actually it can work without the follwoing as long as Slist, Blist isn't initialized yet, you reassign those values directly.
        int nrow,ncol;
        if (space_frame) {
            //nrow=_S.rows();
            //ncol=_S.cols();
            //Slist.resize(nrow,ncol);
            Slist=_S;
        } else {
            //nrow=_B.rows();
            //ncol=_B.cols();
            //Blist.resize(nrow,ncol);
            Blist=_B;
        }
        if (space_frame)
            //"Slist is 6 rows of corresponding (w,v)"
            assert((Slist.rows()==6));
        else
            //"Blist is 6 rows of corresponding (w,v)"
            assert((Blist.rows()==6));
        //TODO add this to a config file
        maxiterations = iterations;
        //
        if (log) {
            //write("home configuration", M);
            //write("screw list in space frame", Slist);
            //write("screw list in body frame", Blist);
            //write("angular error", eomg);
            //write("linear error", ev);
        }
    }
    Kinematics(const Kinematics &copy) : space_frame(copy.space_frame),
                                         eomg(copy.eomg),
                                         ev(copy.ev),
                                         M(copy.M),
                                         maxiterations(copy.maxiterations),
                                         log(copy.log),
                                         Slist(copy.Slist),
                                         Blist(copy.Blist) {
    }
  /** Compute end effector frame
   * @param thetaList A list of joint coordinates.
   * @return T transformation matrix representing the end-effector frame when the joints are at the specified coordinates
   */
  Eigen::MatrixXd ForwardKin(const Eigen::VectorXd& thetaList);

  /** Compute joints thetas given the end-effector configuration
   *
   * @param T is the target configuration
   * @param thetalist: the initial thetalist, and output the target joint angles.
   * @return boolean success flag
   */
  Eigen::VectorXd InverseKin(const Eigen::MatrixXd& T,
                             Eigen::VectorXd thetalist);

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
  FKinSpace(const Eigen::VectorXd& thetaList) const{
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
   * @param thetalist initial angles configuration
   * @param T Transfomation matrix representing the end-effector frame when the joints are at the specified coordinates
   * @return thetaList A list of joint coordinates.
   */
  inline Eigen::VectorXd
  IKinBody(const Eigen::MatrixXd& T, Eigen::VectorXd thetalist) {
    int i = 0;
    //add this to the auto-generated configuration file
    Eigen::MatrixXd Tfk = FKinBody(thetalist);
    Eigen::MatrixXd Tdiff = Algebra::TransInv(Tfk)*T;
    Eigen::VectorXd Vb = Algebra::se3ToVec(Algebra::MatrixLog6(Tdiff));
    Eigen::Vector3d angular(Vb(0), Vb(1), Vb(2));
    Eigen::Vector3d linear(Vb(3), Vb(4), Vb(5));

    bool err = (angular.norm() > eomg || linear.norm() > ev);
    Eigen::MatrixXd Jb;
    double omg_norm, v_norm;
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
      omg_norm=angular.norm();
      v_norm=linear.norm();
      err = (omg_norm > eomg || v_norm > ev);
      if (log) {
        write("iteration(space)", i);
        write("transformation(space)", Tfk);
        write("twist(space)", Vb);
        write("angular norm(space)", angular.norm());
        write("linear norm(space)", linear.norm());
        write("thetalist(space)", thetalist);
      }
    }
    // err boolean True if the iteration is ended without reaching the allowable limits determined by eomg, and ev.
    //assert(!err);
    return thetalist;
  }

  /** Inverse Kinematics in Space frame, given the mechanism configurations, IKinSpace derives the joints angles.
   *
   * @param thetalist initial angles configurations.
   * @param T Transfomation matrix representing the end-effector frame when the joints are at the specified coordinates.
   * @return thetalist angles configurations.
   */
  inline Eigen::VectorXd
  IKinSpace(Eigen::MatrixXd T, Eigen::VectorXd thetalist) {
    int i = 0;
    Eigen::MatrixXd Tfk = FKinSpace(thetalist);
    Eigen::MatrixXd Tdiff = Algebra::TransInv(Tfk)*T;
    Eigen::VectorXd Vs = Algebra::Adjoint(Tfk)*Algebra::se3ToVec(Algebra::MatrixLog6(Tdiff));
    Eigen::Vector3d angular(Vs(0), Vs(1), Vs(2));
    Eigen::Vector3d linear(Vs(3), Vs(4), Vs(5));

    bool err = (angular.norm() > eomg || linear.norm() > ev);
    Eigen::MatrixXd Js;
    double omg_norm,v_norm;
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
      omg_norm=angular.norm();
      v_norm=linear.norm();
      err = (omg_norm > eomg || v_norm > ev);
      if (log) {
        write("iteration(body)", i);
        write("transformation(body)", Tfk);
        write("twist(body)", Vs);
        write("angular norm(body)", angular.norm());
        write("linear norm(body)", linear.norm());
        write("thetalist(body)", thetalist);
      }
    }
    //err boolean True if the iteration is ended without reaching the allowable limits determined by eomg, and ev.
    //assert(!err);
    return thetalist;
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

class SpaceKinematics : public Kinematics {
public:
    SpaceKinematics(Eigen::MatrixXd _S,
                    Eigen::MatrixXd _M,
                    double _eomg=0.01,
                    double _ev=0.001,
                    int iterations=20,
                    bool _log=false) :
        Kinematics(_S,
                   Eigen::MatrixXd::Zero(0,0),
                   _M,
                   _eomg, _ev, iterations, _log) {
        //
    }
};

class BodyKinematics : public Kinematics {
public:
    BodyKinematics(Eigen::MatrixXd _B,
                   Eigen::MatrixXd _M,
                   double _eomg=0.01,
                   double _ev=0.001,
                   int iterations=20,
                   bool _log=false) :
        Kinematics(Eigen::MatrixXd::Zero(0,0),
                   _B, _M,
                   _eomg, _ev, iterations, _log){
        //
    }
};
