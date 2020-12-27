#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include "kinematics.hpp"
#include <complex>
//TODO use standard PI definition

# define PI           3.14159265358979323846

Eigen::MatrixXd nullmat;


void q1() {
  //2.1
  Eigen::MatrixXd M(4,4);
  M << 1,0,0,3.73,
    0,1,0,0,
    0,0,1,2.73,
    0,0,0,1;
  Eigen::MatrixXd S(6,6);
  S << 0,0,0,0,0,0,
    0,1,1,1,0,0,
    1,0,0,0,0,1,
    0,0,1,-0.73,0,0,
    -1,0,0,0,0,-3.73,
    0,1,2.73,3.73,1,0;
  Eigen::VectorXd Theta(6);
  Theta << -1*PI/2, PI/2, PI/3, -1*PI/4, 1, PI/6;
  Eigen::MatrixXd B(6,6);
  B << 0,0,0,0,0,0,
    0,1,1,1,0,0,
    1,0,0,0,0,1,
    0,2.73,3.73,2,0,0,
    2.73,0,0,0,0,0,
    0,-2.73,-1,0,1,0;
  auto kin_space = Kinematics(S, nullmat, M);
  auto kin_body = Kinematics(nullmat, B, M);
  
  auto T=kin_space.ForwardKin(Theta);
  std::cout << "Ts: " << T << std::endl;
  
  T=kin_body.ForwardKin(Theta);
  std::cout << "Tb: " << T << std::endl;
  
  //c2/w2
  Eigen::MatrixXd Sw2(6,3);
  Sw2 << 0,1,0,
    0,0,0,
    1,0,0,
    0,0,0,
    0,2,1,
    0,0,0;
  Eigen::Vector3d theta_list(PI/2,PI/2,1);
  auto ks = Kinematics(Sw2, nullmat, nullmat);
  auto Js = ks.Jacobian(theta_list);
  std::cout << "Js: " << Js << std::endl;
  //
  Eigen::MatrixXd Bw2(6,3);
  Bw2 << 0,-1,0,
    1,0,0,
    0,0,0,
    3,0,0,
    0,3,0,
    0,0,1;
  auto kb = Kinematics(nullmat, Bw2, nullmat);
  auto Jb = kb.Jacobian(theta_list);
  std::cout << "Jb: " << Jb << std::endl;

  Eigen::MatrixXd Jw(3,7);
  Jw << 0,    -1,    0,    0,   -1,   0,   0, //row1
    0,     0,    1,    0,    0,   1,   0, //row2
    1,     0,    0,    1,    0,   0,   1;  //row3
  
  auto Aw = Jw*Jw.transpose();
  Eigen::EigenSolver<Eigen::MatrixXd> eigw(Aw);
 

  //TODO add this function ot the kinematics class.
  // linear ellipsoid
  Eigen::MatrixXd Jv(3,7);
  Jv << -0.105, 0, 0.006, -0.045, 0, 0.006, 0, //row4
    -0.889,0.006, 0,  -0.844,0.006, 0,  0, //row5
    0, -0.105, 0.889,  0,    0,   0,   0; //row6
  auto Av = Jv*Jv.transpose();
  Eigen::EigenSolver<Eigen::MatrixXd> eigv(Av);
  auto res=eigv.eigenvalues();
  Eigen::Vector3d resv;
  resv << res(0).real() ,res(1).real(), res(2).real();
  std::cout << "linear ellipsoid semi-axis: " << resv.array().sqrt() << std::endl;
  std::cout << "linear ellipsoid vectors: " << eigv.eigenvectors() << std::endl;
  //
  return;
}

void ikin() {
  Eigen::MatrixXd Tt(4,4);
  Tt << -0.585,-0.811,0,0.076,
    0.811,-0.585,0,2.608,
    0,0,1,0,
    0,0,0,1;
  Eigen::MatrixXd Mi(4,4);
  Mi << 1,0,0,3,
    0,1,0,0,
    0,0,1,0,
    0,0,0,1;
  Eigen::MatrixXd Si(6,3);
  Si << 0,0,0,
    0,0,0,
    1,1,1,
    0,0,0,
    0,-1,-2,
    0,0,0;
  Eigen::MatrixXd Bi(6,3);
  Bi << 0,0,0,
    0,0,0,
    1,1,1,
    -0.707,-1,-0.707,
    -0.707,0,0.707,
    0,0,0;
  //
  Eigen::Vector3d thetalist(PI/4,PI/4,PI/4); //0.0448,0.74,1.41);
  //TODO remove the extra constructor.
  std::cout << "Space Frame: " << std::endl;
  auto kin_s = Kinematics(Si, nullmat, Mi, 0.001, 0.0001, 20, true);
  thetalist=kin_s.InverseKin(Tt, thetalist);
  std::cout << "thetalist: " << thetalist << std::endl;

  /*
    std::cout << "Body Frame: " << std::endl;
    auto kin_b = Kinematics(nullmat, Bi, Mi, 0.001, 0.0001);
    thetalist=kin_b.InverseKin(T, thetalist);
    std::cout << "thetalist home: " << thetalist << std::endl;
  */
  return;
}

void w3() {
  //example 4.5
  double W1=0.109;
  double W2=0.082;
  //
  double L1=0.425;
  double L2=0.392;
  //
  double H1=0.089;
  double H2=0.095;
  //
  Eigen::MatrixXd M(4,4);
  M << -1,0,0,L1+L2,
    0,    0,1,W1+W2,
    0,    1,0,H1-H2,
    0,    0,0,1;
  Eigen::MatrixXd Blist(6,6);
  Blist << 0,1,0,W1+W2,     0,L1+L2,
    0,       0,1,   H2,-L1-L2,    0,
    0,       0,1,   H2,   -L2,    0,
    0,       0,1,   H2,     0,    0,
    0,      -1,0,  -W2,     0,    0,
    0,       0,1,    0,     0,    0;
  Eigen::MatrixXd T(4,4);
  T << 0, 1, 0, -0.5,
    0,    0,-1, 0.1,
    -1,   0, 0, 0.1,
    0,    0, 0,   1;
  double ew=0.001;
  double ev=0.0001;

  Eigen::VectorXd thetalist(6);
  //thetalist << 2.7,5.734,2.093,-4.156,0.394,1.001;
  //thetalist << 2.821,5.309,1.608,-3.550,-2.761,1.790;
  //thetalist << -0.152,-2.215,-1.790,-6.280,-3.186,-5.552;
  //thetalist << -6.280,-1.911,-1.972,4.581,-3.003,-0.880;
  //thetalist << -6.28,-2.163,-2.024,-2.255,-3.157,-1.770;
  //thetalist << -0.145,-2.038,4.311,-1.449,-3.244,-0.757;
  //thetalist << -0.144,-2.076,4.573,-2.531,-3.294,4.655;
  //thetalist << -0.144, -5.78316967, -3.4252148,  44.19717166, -4.82926673, 23.05371185;
  //thetalist << -0.144,-8.01763,-4.55516,51.0526,-9.19444,28.9865;
  //thetalist << 2.639,-0.940,1.759,-0.979,-0.5,-1.608;
  //thetalist << 2.557,-1,1.759,-0.7,-0.59,-1.6;
  //
  //thetalist << 2.557,-1.053,1.759,-0.708,-0.590,-1.608;
  //thetalist << 2.575,-1.042,1.767,-0.758,-0.565,-1.536;
  //thetalist << 2.579,-1020,-4.581,-0.734,-0.637,-1.487;
  thetalist << 2.59,-0.988,1.705,-0.637,-0.571,-1.608;

  auto kin=Kinematics(nullmat, Blist, M, ew, ev, 300, true);
  thetalist=kin.InverseKin(T, thetalist);
  std::cout << "res: "<< thetalist << std::endl;
}

void free_fall() {
  //iterate 300 times (that is isn't accurate, i need to know the position in n seconds)
  //starting from zero{theta, dtheta, ddtheta}.
  //Ftip <- is can be the GravityForce(it's function of theta only)
  //if i do calculate gravityforce, i should add endeffector forces to the Ftip value.
  //tau <- InverseDynamics(thetalist, dthetalist, ddthetalist, g, Ftip, Mlist, Glist, Slist);
  //
  // this need to be under iterations on trajectory of 3 seconds, or 300 times.
  //ddtheta <-ForwardDynamics
  //theta, dtheta <- EulerStep
  // 
}

int main(int argc, char **args) {
  //q1();
  //ikin();
  //w3();
  free_fall();
  return 0;
}
