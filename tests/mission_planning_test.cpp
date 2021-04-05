#include <Eigen/Dense>
#include "gmock/gmock.h"
#include <iostream>
#include <string>
#include <list>
#include <vector>
#include <cmath>
#include <exception>
#include "algebra/algebra.hpp"

#ifndef MANIPULATION_HDR
#include "../include/manipulation.hpp"
#endif

using namespace testing;

TEST(MANIPULATION, manipulate) {
    Eigen::MatrixXd F(3,4);
    F << 0,0,-1,2,
        -1,0,1,0,
        0,-1,0,1;
    auto A = -1*Eigen::Matrix4d::Identity();
    auto b = -1*Eigen::Vector4d::Ones();
    auto Aeq = F;
    auto beq = Eigen::Vector3d::Zero();
    std::cout << "Aeq(lu).rows: " << Aeq.rows() << std::endl;
    std::cout << "beq(rhs).rows: " << beq.rows() << std::endl;
    auto k = A.lu().solve(b);
    std::cout << "K: " << k << std::endl;
    auto ak_eq= Aeq*k;
    bool equality_check=true;
    for (int i=0; i < b.cols(); i++) {
        equality_check = equality_check && ak_eq(i) <= beq(i);
    }
    ASSERT_TRUE(equality_check);
}

TEST(MANIPULATION, closedForm) {
    Eigen::Vector4d f(1,1,1,1);
    Eigen::VectorXd K;
    Eigen::MatrixXd F(3,4);
    F << 0,0,-1,2,
        -1,0,1,0,
        0,-1,0,1;
    //
    auto Aieq = -1*Eigen::Matrix4d::Identity();
    auto bieq = -1*Eigen::Vector4d::Ones();
    //
    auto Aeq = F;
    auto beq = Eigen::Vector3d::Zero();
    try {
        ASSERT_TRUE(form_closure(f, Aieq, bieq, Aeq, beq, K));
    } catch(NotFormClosure e) {
        std::cerr << e.what() << std::endl;
        ASSERT_TRUE(false);
    }
}

TEST(MANIPULATION, ClosedForm) {
    //
    int n=4;

    Eigen::MatrixXd F (3,4); //wrench |m f|.T
    F << 1,1,-1,-1, //m
        1,0,-1,0,
        0,1,0,-1;
    Eigen::Vector4d f(1,1,1,1); //TODO (res)
    Eigen::VectorXd K;
    auto Aeq = F;
    auto beq = Eigen::Vector3d::Zero();
    auto Aieq = -1*Eigen::Matrix4d::Identity(n, n);
    auto bieq = -1*Eigen::VectorXd::Ones(n);
    try {
        ASSERT_TRUE(form_closure(f, Aieq, bieq, Aeq, beq, K));
    } catch(NotFormClosure e) {
        std::cerr << e.what() << std::endl;
        ASSERT_TRUE(false);
    } catch(exception e) {
        std::cerr << e.what() << std::endl;
    }
}

TEST(MANIPULATION, notClosedForm) {
    int n=4;
    Eigen::MatrixXd F (3,n);
    F << 1,1,1,1,
        0,1,2,3,
        0,0,0,0;
    //
    Eigen::VectorXd K;
    auto f = Eigen::VectorXd::Ones(n);
    auto Aeq = F;
    auto beq = Eigen::Vector3d::Zero();
    auto Aieq = -1*Eigen::Matrix4d::Identity(n, n);
    auto bieq = -1*Eigen::VectorXd::Ones(n);

    try {
        ASSERT_FALSE(form_closure(f, Aieq, bieq, Aeq, beq, K));
    } catch(NotFormClosure e) {
        std::cerr << e.what() << std::endl;
    } catch(exception e) {
        std::cerr << e.what() << std::endl;
    }
}

/*
  Eigen::Vector3d r1(1,0,0);
  Eigen::Vector3d r2(2,1,0);
  //
  Eigen::Vector3d wc(0,0,1); //angular velocity in clock wise direction
  Eigen::Vector3d wcc(0,0,-1); //angular velocity in counter clock wise direction
  //
  Eigen::Vector3d v1 = Algebra::VecToso3(wc)*r1;
  Eigen::Vector3d v2 = Algebra::VecToso3(wcc)*r1;
  Eigen::Vector3d v3 = Algebra::VecToso3(wc)*r2;
  Eigen::Vector3d v4 = Algebra::VecToso3(wcc)*r2;
  //
  Eigen::Vector4d V1(wc(2), v1(0), v1(1),0); //
  Eigen::Vector4d V2(wcc(2), v2(0), v2(1),0); // = Algebra::VecToso3(wcc)*r1;
  Eigen::Vector4d V3(wc(2), v3(0), v3(1),0); // = Algebra::VecToso3(wc)*r2;
  Eigen::Vector3d V4(wcc(2), v4(0), v4(1)); // = Algebra::VecToso3(wcc)*r2;
  //
  Eigen::Vector3d f1(1,0,0);
  Eigen::Vector3d f2(0,1,0);
  Eigen::Vector3d f3(-1,0,0);
  Eigen::Vector3d f4(0,-1,0);
  //
  int m1 = (Algebra::VecToso3(r1)*f1)(2);
  int m2 = (Algebra::VecToso3(r1)*f2)(2);
  int m3 = (Algebra::VecToso3(r2)*f3)(2);
  int m4 = (Algebra::VecToso3(r2)*f4)(2);
*/
