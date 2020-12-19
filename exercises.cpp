#include <iostream>
#include "src/kinematics.hpp"

//TODO use standard PI definition

# define PI           3.14159265358979323846

int main(int argc, char **args) {
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
  Eigen::MatrixXd T(4,4);
  Eigen::VectorXd Theta(6);
  Theta << -1*PI/2, PI/2, PI/3, -1*PI/4, 1, PI/6;
  Eigen::MatrixXd B(6,6);
  B << 0,0,0,0,0,0,
    0,1,1,1,0,0,
    1,0,0,0,0,1,
    0,3.73,3.73,2,0,0,
    0,0,0,0,0,0,
    2.73,-2.73,-1,0,1,0;
  auto Ts_forward=Algebra::IKinSpace(S, M, T, Theta);
  std::cout << "Ts: " << Ts_forward << std::endl;
  auto Tb_forward=Algebra::IKinBody(B, M, T, Theta);
  std::cout << "Tb: " << Tb_forward << std::endl;
}
