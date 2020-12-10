#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include "gtest/gtest.h"
#include "../include/kinetics.h"

# define M_PI           3.14159265358979323846 

class FxKet : public testing::Test {
  Kinetics ket;
};

TEST_F(FxKet, InverseDynamicsTest) {
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
  
  Eigen::VectorXd taulist = ket.InverseDynamics(thetalist, dthetalist, ddthetalist, g, Ftip, Mlist, Glist, Slist);
  
  Eigen::VectorXd result(3);
  result << 74.6962, -33.0677, -3.23057;
  
  ASSERT_TRUE(taulist.isApprox(result, 4));
}

//those classes are private, if you want, make the FxKet to inherit Kinetics in protected access mode.
/*
TEST_F(FxKet, GravityForcesTest) {
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
  
  Eigen::VectorXd grav = ket.GravityForces(thetalist, g, Mlist, Glist, Slist);
  
  Eigen::VectorXd result(3);
  result << 28.4033, -37.6409, -5.4416;
  
  ASSERT_TRUE(grav.isApprox(result, 4));
}

TEST_F(FxKet, MassMatrixTest) {
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
  
  Eigen::MatrixXd M = ket.MassMatrix(thetalist, Mlist, Glist, Slist);
  
  Eigen::MatrixXd result(3, 3);
  result << 22.5433, -0.3071, -0.0072,
    -0.3071, 1.9685, 0.4322,
    -0.0072, 0.4322, 0.1916;
  
  ASSERT_TRUE(M.isApprox(result, 4));
}

TEST_F(FxKet, VelQuadraticForcesTest) {
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
  
  Eigen::VectorXd c = ket.VelQuadraticForces(thetalist, dthetalist, Mlist, Glist, Slist);
  
  Eigen::VectorXd result(3);
  result << 0.2645, -0.0551, -0.0069;
  
  ASSERT_TRUE(c.isApprox(result, 4));
}

TEST_F(FxKet, EndEffectorForcesTest) {
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
  
  Eigen::VectorXd JTFtip = ket.EndEffectorForces(thetalist, Ftip, Mlist, Glist, Slist);
  
  Eigen::VectorXd result(3);
  result << 1.4095, 1.8577, 1.3924;
  
  ASSERT_TRUE(JTFtip.isApprox(result, 4));
}
*/

TEST_F(FxKet, ForwardDynamicsTest) {
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
  
  Eigen::VectorXd ddthetalist = ket.ForwardDynamics(thetalist, dthetalist, taulist, g, Ftip, Mlist, Glist, Slist);
  
  Eigen::VectorXd result(3);
  result << -0.9739, 25.5847, -32.9150;
  
  ASSERT_TRUE(ddthetalist.isApprox(result, 4));
}

TEST_F(FxKet, EulerStepTest) {
  Eigen::VectorXd thetalist(3);
  thetalist << 0.1, 0.1, 0.1;
  Eigen::VectorXd dthetalist(3);
  dthetalist << 0.1, 0.2, 0.3;
  Eigen::VectorXd ddthetalist(3);
  ddthetalist << 2, 1.5, 1;
  double dt = 0.1;
  
  ket.EulerStep(thetalist, dthetalist, ddthetalist, dt);
  
  Eigen::VectorXd result_thetalistNext(3);
  result_thetalistNext << 0.11, 0.12, 0.13;
  Eigen::VectorXd result_dthetalistNext(3);
  result_dthetalistNext << 0.3, 0.35, 0.4;
  
  ASSERT_TRUE(thetalist.isApprox(result_thetalistNext, 4));
  ASSERT_TRUE(dthetalist.isApprox(result_dthetalistNext, 4));
}
