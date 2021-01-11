#include <iostream>
#include <Eigen/Dense>
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "../include/kinematics.hpp"

# define M_PI           3.14159265358979323846 

/** \class FxKin
 *  fixture kinematics testing class
 */

using namespace testing;

/*
class FxKin : public testing::Test {
public:
  virtual void SetUp() {
    kin_s = Kinematics(true);
    kin_b = Kinematics(false);
  }
  Kinematics kin_s; //(true); // space frame kinematics
  Kinematics kin_b; //(false); // body frame kinematics
};
*/
Eigen::MatrixXd nullmat;

TEST(KINEMATICS, JacobianSpaceTest) {
  Eigen::MatrixXd s_list(6, 3);
  s_list << 0, 0, 0,
    0, 1, -1,
    1, 0, 0,
    0, -0.0711, 0.0711,
    0, 0, 0,
    0, 0, -0.2795;
  auto kin_s=Kinematics(s_list, nullmat, nullmat);
  Eigen::VectorXd theta(3);
  theta << 1.0472, 1.0472, 1.0472;
  Eigen::MatrixXd result(6, 3);
  result << 0, -0.866, 0.866,
    0, 0.5, -0.5,
    1, 0, 0,
    0, -0.0355, -0.0855,
    0, -0.0615, -0.1481,
    0, 0, -0.1398;
  Eigen::MatrixXd tmp_result = kin_s.Jacobian(theta);
  ASSERT_TRUE(tmp_result.isApprox(result, 4));
}

TEST(KINEMATICS, JacobianBodyTest) {
  Eigen::MatrixXd b_list(6, 3);
  b_list << 0, 0, 0,
    0, 1, -1,
    1, 0, 0,
    0.0425, 0, 0,
    0.5515, 0, 0,
    0, -0.5515, 0.2720;
  auto kin_b=Kinematics(nullmat, b_list, nullmat);
  Eigen::VectorXd theta(3);
  theta << 0, 0, 1.5708;
  Eigen::MatrixXd result(6, 3);
  result << 1, 0, 0,
    0, 1, -1,
    0, 0, 0,
    0, -0.2795, 0,
    0.2795, 0, 0,
    -0.0425, -0.2720, 0.2720;
  Eigen::MatrixXd tmp_result = kin_b.Jacobian(theta);
  ASSERT_TRUE(tmp_result.isApprox(result, 4));
}

TEST(KINEMATICS, FKInBodyTest) {
  Eigen::MatrixXd M(4, 4);
  M << -1, 0, 0, 0,
    0, 1, 0, 6,
    0, 0, -1, 2,
    0, 0, 0, 1;
  Eigen::MatrixXd Blist(6, 3);
  Blist << 0, 0, 0,
    0, 0, 0,
    -1, 0, 1,
    2, 0, 0,
    0, 1, 0,
    0, 0, 0.1;
  auto kin_b=Kinematics(nullmat, Blist, M);
  Eigen::VectorXd thetaList(3);
  thetaList << M_PI / 2.0, 3, M_PI;
  Eigen::MatrixXd result(4, 4);
  result << 0, 1, 0, -5,
    1, 0, 0, 4,
    0, 0, -1, 1.68584073,
    0, 0, 0, 1;
  Eigen::MatrixXd FKCal = kin_b.ForwardKin(thetaList);
  ASSERT_TRUE(FKCal.isApprox(result, 4));
}

TEST(KINEMATICS, FKInSpaceTest) {
  Eigen::MatrixXd M(4, 4);
  M << -1, 0, 0, 0,
    0, 1, 0, 6,
    0, 0, -1, 2,
    0, 0, 0, 1;
  Eigen::MatrixXd Slist(6, 3);
  Slist << 0, 0, 0,
    0, 0, 0,
    1, 0, -1,
    4, 0, -6,
    0, 1, 0,
    0, 0, -0.1;
  auto kin_s=Kinematics(Slist, nullmat, M);
  Eigen::VectorXd thetaList(3);
  thetaList << M_PI / 2.0, 3, M_PI;
  
  Eigen::MatrixXd result(4, 4);
  result << 0, 1, 0, -5,
    1, 0, 0, 4,
    0, 0, -1, 1.68584073,
    0, 0, 0, 1;
  Eigen::MatrixXd FKCal = kin_s.ForwardKin(thetaList);
  ASSERT_TRUE(FKCal.isApprox(result, 4));
}

TEST(KINEMATICS, IKinBodyTest) {
  Eigen::MatrixXd BlistT(3, 6);
  BlistT << 0, 0, -1, 2, 0, 0,
    0, 0, 0, 0, 1, 0,
    0, 0, 1, 0, 0, 0.1;
  Eigen::MatrixXd Blist = BlistT.transpose();
  Eigen::Matrix4d M;
  M << -1, 0, 0, 0,
    0, 1, 0, 6,
    0, 0, -1, 2,
    0, 0, 0, 1;
  auto kin_b=Kinematics(nullmat, Blist, M);
  Eigen::Matrix4d T;
  T << 0, 1, 0, -5,
    1, 0, 0, 4,
    0, 0, -1, 1.6858,
    0, 0, 0, 1;
  Eigen::VectorXd thetalist(3);
  thetalist << 1.5, 2.5, 3;
  double eomg = 0.01;
  double ev = 0.001;
  bool b_result = true;
  Eigen::VectorXd theta_result(3);
  theta_result << 1.57073819, 2.999667, 3.14153913;
  thetalist = kin_b.InverseKin(T, thetalist);
  ASSERT_TRUE(thetalist.isApprox(theta_result, 4));
}
/*
TEST(KINEMATICS, IKinBodyTest2) {
  double eomg = 0.01;
  double ev = 0.001;
  Eigen::MatrixXd BlistT(6, 6);
  BlistT << 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0, -1, 0,
    0, 1, 1, 1, 0, 1,
    0.191, 0.095, 0.095, 0.095, -0.082, 0,
    0, -0.817, -0.392, 0, 0, 0,
    0.817, 0, 0, 0, 0, 0;
  Eigen::MatrixXd Blist = BlistT.transpose();
  Eigen::Matrix4d M;
  M << -1, 0, 0, 0.817,
    0, 0, 1, 0.191,
    0, 1, 0, -0.006,
    0, 0, 0, 1;
  auto kin_b=Kinematics(nullmat, Blist.transpose(), M, eomg, ev, 20, true);
  Eigen::Matrix4d T;
  T << 0, 1, 0, -0.5,
    0, 0, -1, 0.1,
    -1, 0, 0, 0.1,
    0, 0, 0, 1;
  Eigen::VectorXd thetalist(6);
  //thetalist << 2.590, -0.988, 1.705, -0.637, -0.571, -1.608;
  thetalist << -4.156,-1.669,1.705,-0.637,-3.914,-1.426;
  kin_b.InverseKin(T, thetalist);
  kin_b.close();
  //Eigen::VectorXd theta_result(3);
  //theta_result << 1.57073819, 2.999667, 3.14153913;
  //thetalist = kin_b.InverseKin(T, thetalist);
  //ASSERT_TRUE(thetalist.isApprox(theta_result, 4));
}
*/

TEST(KINEMATICS, IKinSpaceTest) {
  Eigen::MatrixXd SlistT(3, 6);
  SlistT << 0, 0, 1, 4, 0, 0,
    0, 0, 0, 0, 1, 0,
    0, 0, -1, -6, 0, -0.1;
  Eigen::MatrixXd Slist = SlistT.transpose();
  Eigen::Matrix4d M;
  M << -1, 0, 0, 0,
    0, 1, 0, 6,
    0, 0, -1, 2,
    0, 0, 0, 1;
  auto kin_s=Kinematics(Slist, nullmat, M);
  Eigen::Matrix4d T;
  T << 0, 1, 0, -5,
    1, 0, 0, 4,
    0, 0, -1, 1.6858,
    0, 0, 0, 1;
  Eigen::VectorXd thetalist(3);
  thetalist << 1.5, 2.5, 3;
  double eomg = 0.01;
  double ev = 0.001;
  bool b_result = true;
  Eigen::VectorXd theta_result(3);
  theta_result << 1.57073783, 2.99966384, 3.1415342;
  thetalist = kin_s.InverseKin(T, thetalist);
  ASSERT_TRUE(thetalist.isApprox(theta_result, 4));
}

int main(int argc, char **args) {
  InitGoogleTest(&argc, args);
  return RUN_ALL_TESTS();
}
