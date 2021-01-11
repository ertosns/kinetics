#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include "gmock/gmock.h"
//#include "gtest/gtest.h"
#include "../include/kinetics.hpp"
#include "../include/logger.hpp"

# define M_PI           3.14159265358979323846 

using namespace testing;

class WrapKinetics : public Kinetics {
public:
  WrapKinetics(Eigen::VectorXd _thetalist,
                  Eigen::VectorXd _dthetalist,
                  Eigen::VectorXd _ddthetalist,
                  std::vector<Eigen::MatrixXd> _Mlist,
                  std::vector<Eigen::MatrixXd> _Glist,
                  Eigen::MatrixXd _Slist,
                  Eigen::VectorXd _g=Eigen::Vector3d(0,0,-9.81),
                  double kp=0.1,
                  double ki=0.1,
                  double kd=0.1) :
    Kinetics(_thetalist,
             _dthetalist,
             _ddthetalist,
             _Mlist,
             _Glist,
             _Slist, _g, kp, ki, kd) {
  }
  void wrap_EulerStep(double dt) {
    EulerStep(dt);
  }
  Eigen::VectorXd wrap_GravityForces() const {
    return GravityForces();
  }
  Eigen::MatrixXd wrap_MassMatrix() const {
    return MassMatrix();
  }
  Eigen::VectorXd wrap_VelQuadraticForces() const {
    return VelQuadraticForces();
  }
  Eigen::VectorXd wrap_EndEffectorForces(const Eigen::VectorXd& Ftip) const {
    return EndEffectorForces(Ftip);
  }
};

TEST(KINETICS, ForwardDynamicsTest) {
  Eigen::VectorXd thetalist(3);
  thetalist << 0.1, 0.1, 0.1;
  Eigen::VectorXd dthetalist(3);
  dthetalist << 0.1, 0.2, 0.3;
  Eigen::VectorXd taulist(3);
  taulist << 0.5, 0.6, 0.7;
  Eigen::VectorXd g(3);
  g << 0, 0, -9.8;
  Eigen::VectorXd Ftip(6);
  Ftip << 1, 1, 1, 1, 1, 1;
  
  std::vector<Eigen::MatrixXd> Mlist;
  std::vector<Eigen::MatrixXd> Glist;
  
  Eigen::Matrix4d M01;
  M01 << 1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0.089159,
    0, 0, 0, 1;
  Eigen::Matrix4d M12;
  M12 << 0, 0, 1, 0.28,
    0, 1, 0, 0.13585,
    -1, 0, 0, 0,
    0, 0, 0, 1;
  Eigen::Matrix4d M23;
  M23 << 1, 0, 0, 0,
    0, 1, 0, -0.1197,
    0, 0, 1, 0.395,
    0, 0, 0, 1;
  Eigen::Matrix4d M34;
  M34 << 1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0.14225,
    0, 0, 0, 1;
  
  Mlist.push_back(M01);
  Mlist.push_back(M12);
  Mlist.push_back(M23);
  Mlist.push_back(M34);
  
  Eigen::VectorXd G1(6);
  G1 << 0.010267, 0.010267, 0.00666, 3.7, 3.7, 3.7;
  Eigen::VectorXd G2(6);
  G2 << 0.22689, 0.22689, 0.0151074, 8.393, 8.393, 8.393;
  Eigen::VectorXd G3(6);
  G3 << 0.0494433, 0.0494433, 0.004095, 2.275, 2.275, 2.275;
  
  Glist.push_back(G1.asDiagonal());
  Glist.push_back(G2.asDiagonal());
  Glist.push_back(G3.asDiagonal());
  
  Eigen::MatrixXd SlistT(3, 6);
  SlistT << 1, 0, 1, 0, 1, 0,
    0, 1, 0, -0.089, 0, 0,
    0, 1, 0, -0.089, 0, 0.425;
  Eigen::MatrixXd Slist = SlistT.transpose();
  Eigen::VectorXd ddthetalist;
  auto kinetics = Kinetics(thetalist, dthetalist, ddthetalist,
                           Mlist, Glist, Slist, g);
  //ddthetalist = kinetics.ForwardDynamics(const_cast<const Eigen::VectorXd>(taulist), const_cast<const Eigen::VectorXd>(Ftip));
  ddthetalist = kinetics.ForwardDynamics(taulist,
                                         Ftip
                                         );
  
  Eigen::VectorXd result(3);
  result << -0.9739, 25.5847, -32.9150;
  
  ASSERT_TRUE(ddthetalist.isApprox(result, 4));
}

//euler function need to be part of the Forward kinematics only.
TEST(KINETICS, EulerStepTest) {
  Eigen::VectorXd thetalist(3);
  thetalist << 0.1, 0.1, 0.1;
  Eigen::VectorXd dthetalist(3);
  dthetalist << 0.1, 0.2, 0.3;
  Eigen::VectorXd ddthetalist(3);
  ddthetalist << 2, 1.5, 1;
  double dt = 0.1;
  
  std::vector<Eigen::MatrixXd> dummyMlist, dummyGlist;
  Eigen::MatrixXd dummySlist;
  auto ket = WrapKinetics(thetalist, dthetalist, ddthetalist,
                             dummyMlist, dummyGlist, dummySlist);
  ket.wrap_EulerStep(dt);
  
  Eigen::VectorXd result_thetalistNext(3);
  result_thetalistNext << 0.11, 0.12, 0.13;
  Eigen::VectorXd result_dthetalistNext(3);
  result_dthetalistNext << 0.3, 0.35, 0.4;
  
  ASSERT_TRUE(ket.get_pos().isApprox(result_thetalistNext, 4));
  ASSERT_TRUE(ket.get_vel().isApprox(result_dthetalistNext, 4));
}

TEST(KINETICS, InverseDynamicsTest) {
  Eigen::VectorXd thetalist(3);
  thetalist << 0.1, 0.1, 0.1;
  Eigen::VectorXd dthetalist(3);
  dthetalist << 0.1, 0.2, 0.3;
  Eigen::VectorXd ddthetalist(3);
  ddthetalist << 2, 1.5, 1;
  Eigen::VectorXd g(3);
  g << 0, 0, -9.8;
  Eigen::VectorXd Ftip(6);
  Ftip << 1, 1, 1, 1, 1, 1;
  
  std::vector<Eigen::MatrixXd> Mlist;
  std::vector<Eigen::MatrixXd> Glist;
  
  Eigen::Matrix4d M01;
  M01 << 1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0.089159,
    0, 0, 0, 1;
  Eigen::Matrix4d M12;
  M12 << 0, 0, 1, 0.28,
    0, 1, 0, 0.13585,
    -1, 0, 0, 0,
    0, 0, 0, 1;
  Eigen::Matrix4d M23;
  M23 << 1, 0, 0, 0,
    0, 1, 0, -0.1197,
    0, 0, 1, 0.395,
    0, 0, 0, 1;
  Eigen::Matrix4d M34;
  M34 << 1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0.14225,
    0, 0, 0, 1;
  
  Mlist.push_back(M01);
  Mlist.push_back(M12);
  Mlist.push_back(M23);
  Mlist.push_back(M34);
  
  Eigen::VectorXd G1(6);
  G1 << 0.010267, 0.010267, 0.00666, 3.7, 3.7, 3.7;
  Eigen::VectorXd G2(6);
  G2 << 0.22689, 0.22689, 0.0151074, 8.393, 8.393, 8.393;
  Eigen::VectorXd G3(6);
  G3 << 0.0494433, 0.0494433, 0.004095, 2.275, 2.275, 2.275;
  
  Glist.push_back(G1.asDiagonal());
  Glist.push_back(G2.asDiagonal());
  Glist.push_back(G3.asDiagonal());
  
  Eigen::MatrixXd SlistT(3, 6);
  SlistT << 1, 0, 1, 0, 1, 0,
    0, 1, 0, -0.089, 0, 0,
    0, 1, 0, -0.089, 0, 0.425;
  Eigen::MatrixXd Slist = SlistT.transpose();
  
  Eigen::VectorXd taulist = Kinetics(thetalist,
                                     dthetalist,
                                     ddthetalist,
                                     Mlist,
                                     Glist,
                                     Slist,
                                     g)
    .InverseDynamics(Ftip);
  Eigen::VectorXd result(3);
  result << 74.6962, -33.0677, -3.23057;
  
  ASSERT_TRUE(taulist.isApprox(result, 4));
}

//those classes are private, if you want, make the KINETICS to inherit Kinetics in protected access mode.

TEST(KINETICS, GravityForcesTest) {
  Eigen::VectorXd thetalist(3);
  thetalist << 0.1, 0.1, 0.1;
  Eigen::VectorXd g(3);
  g << 0, 0, -9.8;
  
  std::vector<Eigen::MatrixXd> Mlist;
  std::vector<Eigen::MatrixXd> Glist;
  
  Eigen::Matrix4d M01;
  M01 << 1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0.089159,
    0, 0, 0, 1;
  Eigen::Matrix4d M12;
  M12 << 0, 0, 1, 0.28,
    0, 1, 0, 0.13585,
    -1, 0, 0, 0,
    0, 0, 0, 1;
  Eigen::Matrix4d M23;
  M23 << 1, 0, 0, 0,
    0, 1, 0, -0.1197,
    0, 0, 1, 0.395,
    0, 0, 0, 1;
  Eigen::Matrix4d M34;
  M34 << 1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0.14225,
    0, 0, 0, 1;
  
  Mlist.push_back(M01);
  Mlist.push_back(M12);
  Mlist.push_back(M23);
  Mlist.push_back(M34);
  
  Eigen::VectorXd G1(6);
  G1 << 0.010267, 0.010267, 0.00666, 3.7, 3.7, 3.7;
  Eigen::VectorXd G2(6);
  G2 << 0.22689, 0.22689, 0.0151074, 8.393, 8.393, 8.393;
  Eigen::VectorXd G3(6);
  G3 << 0.0494433, 0.0494433, 0.004095, 2.275, 2.275, 2.275;
  
  Glist.push_back(G1.asDiagonal());
  Glist.push_back(G2.asDiagonal());
  Glist.push_back(G3.asDiagonal());
  
  Eigen::MatrixXd SlistT(3, 6);
  SlistT << 1, 0, 1, 0, 1, 0,
    0, 1, 0, -0.089, 0, 0,
    0, 1, 0, -0.089, 0, 0.425;
  Eigen::MatrixXd Slist = SlistT.transpose();
  Eigen::VectorXd dummydtheta, dummyddtheta;
  auto ket = WrapKinetics(thetalist, dummydtheta, dummyddtheta, Mlist, Glist, Slist, g);
  
  Eigen::VectorXd grav = ket.wrap_GravityForces();
  
  Eigen::VectorXd result(3);
  result << 28.4033, -37.6409, -5.4416;
  
  ASSERT_TRUE(grav.isApprox(result, 4));
}

TEST(KINETICS, ForInvEuler) {
  Eigen::VectorXd thetalist(6);
  thetalist << 0,-1,0,0,0,0;
  Eigen::VectorXd dthetalist(6);
  dthetalist << 0,0,0,0,0,0;
  Eigen::VectorXd ddthetalist(6);
  ddthetalist << 0,0,0,0,0,0;
  
  Eigen::VectorXd g(3);
  g << 0, 0, -9.8;
  
  std::vector<Eigen::MatrixXd> Mlist;
  std::vector<Eigen::MatrixXd> Glist;
  
  Eigen::Matrix4d M01;
  M01 << 1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0.089159,
    0, 0, 0, 1;
  Eigen::Matrix4d M12;
  M12 << 0, 0, 1, 0.28,
    0, 1, 0, 0.13585,
    -1, 0, 0, 0,
    0, 0, 0, 1;
  Eigen::Matrix4d M23;
  M23 << 1, 0, 0, 0,
    0, 1, 0, -0.1197,
    0, 0, 1, 0.395,
    0, 0, 0, 1;
  Eigen::Matrix4d M34;
  M34 << 1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0.14225,
    0, 0, 0, 1;
  Eigen::Matrix4d M45;
  M45 << 1, 0, 0, 0,
    0, 1, 0, 0.093,
    0, 0, 1, 0,
    0, 0, 0, 1;
  Eigen::Matrix4d M56;
  M56 << 1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0.09465,
    0, 0, 0, 1;
  Eigen::Matrix4d M67;
  M67 << 1, 0, 0, 0,
    0, 0, 1, 0.0823,
    0, -1, 0, 0,
    0, 0, 0, 1;
  
  Mlist.push_back(M01);
  Mlist.push_back(M12);
  Mlist.push_back(M23);
  Mlist.push_back(M34);
  Mlist.push_back(M45);
  Mlist.push_back(M56);
  Mlist.push_back(M67);
  
  Eigen::VectorXd G1(6);
  G1 << 0.010267, 0.010267, 0.00666, 3.7, 3.7, 3.7;
  Eigen::VectorXd G2(6);
  G2 << 0.22689, 0.22689, 0.0151074, 8.393, 8.393, 8.393;
  Eigen::VectorXd G3(6);
  G3 << 0.0494433, 0.0494433, 0.004095, 2.275, 2.275, 2.275;
  Eigen::VectorXd G4(6);
  G4 << 0.111172755531, 0.111172755531,0.21942,1.219,1.219,1.219;
  Eigen::VectorXd G5(6);
  G5 << 0.111172755531, 0.111172755531,0.21942,1.219,1.219,1.219;
  Eigen::VectorXd G6(6);
  G6 << 0.0171364731454,0.0171364731454,0.033822,0.1879,0.1879,0.1879;
  
  Glist.push_back(G1.asDiagonal());
  Glist.push_back(G2.asDiagonal());
  Glist.push_back(G3.asDiagonal());
  Glist.push_back(G4.asDiagonal()); 
  Glist.push_back(G5.asDiagonal()); 
  Glist.push_back(G6.asDiagonal()); 

  Eigen::MatrixXd SlistT(6, 6);
  SlistT << 1, 0, 1, 0, 1, 0,
    0, 1, 0, -0.089, 0, 0,
    0, 1, 0, -0.089, 0, 0.425,
    0, 1, 0, -0.089159, 0, 0.81725,
    0, 0, -1, -0.10915, 0.81725, 0,
    0, 1, 0, 0.005491, 0, 0.81725;
  
  Eigen::MatrixXd Slist = SlistT.transpose();
  auto ket = WrapKinetics(thetalist, dthetalist, ddthetalist, Mlist, Glist, Slist, g);
  
  Eigen::VectorXd Ftip(6);
  Ftip << 0,0,0,0,0,0;
  //TODO what is the appropriate time step?!
  Logger log("simulation1.csv");
  double dt=0.01;
  for (int i=0; i < 500; i++) {
    auto taulist = ket.InverseDynamics(Ftip);
    ddthetalist = ket.ForwardDynamics(taulist, Ftip);
    ket.wrap_EulerStep(dt);
    log.write(ket.get_pos());
    //std::cout << "taulist: " << taulist << std::endl;
    //std::cout << "ddthetalist: " << ddthetalist << std::endl;
  }
}

TEST(KINETICS, MassMatrixTest) {
  Eigen::VectorXd thetalist(3);
  thetalist << 0.1, 0.1, 0.1;
  
  std::vector<Eigen::MatrixXd> Mlist;
  std::vector<Eigen::MatrixXd> Glist;
  
  Eigen::Matrix4d M01;
  M01 << 1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0.089159,
    0, 0, 0, 1;
  Eigen::Matrix4d M12;
  M12 << 0, 0, 1, 0.28,
    0, 1, 0, 0.13585,
    -1, 0, 0, 0,
    0, 0, 0, 1;
  Eigen::Matrix4d M23;
  M23 << 1, 0, 0, 0,
    0, 1, 0, -0.1197,
    0, 0, 1, 0.395,
    0, 0, 0, 1;
  Eigen::Matrix4d M34;
  M34 << 1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0.14225,
    0, 0, 0, 1;
  
  Mlist.push_back(M01);
  Mlist.push_back(M12);
  Mlist.push_back(M23);
  Mlist.push_back(M34);
  
  Eigen::VectorXd G1(6);
  G1 << 0.010267, 0.010267, 0.00666, 3.7, 3.7, 3.7;
  Eigen::VectorXd G2(6);
  G2 << 0.22689, 0.22689, 0.0151074, 8.393, 8.393, 8.393;
  Eigen::VectorXd G3(6);
  G3 << 0.0494433, 0.0494433, 0.004095, 2.275, 2.275, 2.275;
  
  Glist.push_back(G1.asDiagonal());
  Glist.push_back(G2.asDiagonal());
  Glist.push_back(G3.asDiagonal());
  
  Eigen::MatrixXd SlistT(3, 6);
  SlistT << 1, 0, 1, 0, 1, 0,
    0, 1, 0, -0.089, 0, 0,
    0, 1, 0, -0.089, 0, 0.425;
  Eigen::MatrixXd Slist = SlistT.transpose();

  Eigen::VectorXd dummydtheta, dummyddtheta;
  auto ket=WrapKinetics(thetalist, dummydtheta, dummyddtheta, Mlist, Glist, Slist);
  Eigen::MatrixXd M = ket.wrap_MassMatrix();
  
  Eigen::MatrixXd result(3, 3);
  result << 22.5433, -0.3071, -0.0072,
    -0.3071, 1.9685, 0.4322,
    -0.0072, 0.4322, 0.1916;
  
  ASSERT_TRUE(M.isApprox(result, 4));
}

TEST(KINETICS, VelQuadraticForcesTest) {
  Eigen::VectorXd thetalist(3);
  thetalist << 0.1, 0.1, 0.1;
  Eigen::VectorXd dthetalist(3);
  dthetalist << 0.1, 0.2, 0.3;
  
  std::vector<Eigen::MatrixXd> Mlist;
  std::vector<Eigen::MatrixXd> Glist;
  
  Eigen::Matrix4d M01;
  M01 << 1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0.089159,
    0, 0, 0, 1;
  Eigen::Matrix4d M12;
  M12 << 0, 0, 1, 0.28,
    0, 1, 0, 0.13585,
    -1, 0, 0, 0,
    0, 0, 0, 1;
  Eigen::Matrix4d M23;
  M23 << 1, 0, 0, 0,
    0, 1, 0, -0.1197,
    0, 0, 1, 0.395,
    0, 0, 0, 1;
  Eigen::Matrix4d M34;
  M34 << 1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0.14225,
    0, 0, 0, 1;
  
  Mlist.push_back(M01);
  Mlist.push_back(M12);
  Mlist.push_back(M23);
  Mlist.push_back(M34);
  
  Eigen::VectorXd G1(6);
  G1 << 0.010267, 0.010267, 0.00666, 3.7, 3.7, 3.7;
  Eigen::VectorXd G2(6);
  G2 << 0.22689, 0.22689, 0.0151074, 8.393, 8.393, 8.393;
  Eigen::VectorXd G3(6);
  G3 << 0.0494433, 0.0494433, 0.004095, 2.275, 2.275, 2.275;
  
  Glist.push_back(G1.asDiagonal());
  Glist.push_back(G2.asDiagonal());
  Glist.push_back(G3.asDiagonal());
  
  Eigen::MatrixXd SlistT(3, 6);
  SlistT << 1, 0, 1, 0, 1, 0,
    0, 1, 0, -0.089, 0, 0,
    0, 1, 0, -0.089, 0, 0.425;
  Eigen::MatrixXd Slist = SlistT.transpose();

  Eigen::VectorXd dummyddtheta;
  auto ket = WrapKinetics(thetalist, dthetalist, dummyddtheta, Mlist, Glist, Slist);
  Eigen::VectorXd c = ket.wrap_VelQuadraticForces();
  
  Eigen::VectorXd result(3);
  result << 0.2645, -0.0551, -0.0069;
  
  ASSERT_TRUE(c.isApprox(result, 4));
}

TEST(KINETICS, EndEffectorForcesTest) {
  Eigen::VectorXd thetalist(3);
  thetalist << 0.1, 0.1, 0.1;
  Eigen::VectorXd Ftip(6);
  Ftip << 1, 1, 1, 1, 1, 1;
  
  std::vector<Eigen::MatrixXd> Mlist;
  std::vector<Eigen::MatrixXd> Glist;
  
  Eigen::Matrix4d M01;
  M01 << 1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0.089159,
    0, 0, 0, 1;
  Eigen::Matrix4d M12;
  M12 << 0, 0, 1, 0.28,
    0, 1, 0, 0.13585,
    -1, 0, 0, 0,
    0, 0, 0, 1;
  Eigen::Matrix4d M23;
  M23 << 1, 0, 0, 0,
    0, 1, 0, -0.1197,
    0, 0, 1, 0.395,
    0, 0, 0, 1;
  Eigen::Matrix4d M34;
  M34 << 1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0.14225,
    0, 0, 0, 1;
  
  Mlist.push_back(M01);
  Mlist.push_back(M12);
  Mlist.push_back(M23);
  Mlist.push_back(M34);
  
  Eigen::VectorXd G1(6);
  G1 << 0.010267, 0.010267, 0.00666, 3.7, 3.7, 3.7;
  Eigen::VectorXd G2(6);
  G2 << 0.22689, 0.22689, 0.0151074, 8.393, 8.393, 8.393;
  Eigen::VectorXd G3(6);
  G3 << 0.0494433, 0.0494433, 0.004095, 2.275, 2.275, 2.275;
  
  Glist.push_back(G1.asDiagonal());
  Glist.push_back(G2.asDiagonal());
  Glist.push_back(G3.asDiagonal());
  
  Eigen::MatrixXd SlistT(3, 6);
  SlistT << 1, 0, 1, 0, 1, 0,
    0, 1, 0, -0.089, 0, 0,
    0, 1, 0, -0.089, 0, 0.425;
  Eigen::MatrixXd Slist = SlistT.transpose();

  Eigen::VectorXd dummydtheta, dummyddtheta;
  auto ket = WrapKinetics(thetalist, dummydtheta, dummyddtheta, Mlist, Glist, Slist);
  Eigen::VectorXd JTFtip = ket.wrap_EndEffectorForces(Ftip);
  
  Eigen::VectorXd result(3);
  result << 1.4095, 1.8577, 1.3924;
  
  ASSERT_TRUE(JTFtip.isApprox(result, 4));
}
