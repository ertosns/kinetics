#include "../include/trajectory.hpp"
#include  "algebra/algebra.hpp"

Eigen::MatrixXd
Trajectory::JointTrajectory(const Eigen::VectorXd& thetastart,
                            const Eigen::VectorXd& thetaend) {
  Eigen::MatrixXd trajT =
    Eigen::MatrixXd::Zero(thetastart.size(), N);
  double st;
  for (int i = 0; i < N; ++i) {
    if (method == 3)
      st = CubicTimeScaling(dt*i);
    else
      st = QuinticTimeScaling(dt*i);
    trajT.col(i) = st * thetaend + (1 - st)*thetastart;
  }
  Eigen::MatrixXd traj = trajT.transpose();
  return traj;
}

std::vector<Eigen::MatrixXd>
Trajectory::ScrewTrajectory(const Eigen::MatrixXd& Xstart,
                            const Eigen::MatrixXd& Xend) {
  std::vector<Eigen::MatrixXd> traj(N);
  double st;
  for (int i = 0; i < N; ++i) {
    if (method == 3)
      st = CubicTimeScaling(dt*i);
    else
      st = QuinticTimeScaling(dt*i);
    Eigen::MatrixXd Ttemp = Algebra::MatrixLog6(Algebra::TransInv(Xstart)*Xend);
    traj.at(i) = Xstart * Algebra::MatrixExp6(Ttemp*st);
  }
  return traj;
}

std::vector<Eigen::MatrixXd>
Trajectory::CartesianTrajectory(const Eigen::MatrixXd& Xstart,
                                const Eigen::MatrixXd& Xend) {
  std::vector<Eigen::MatrixXd> traj(N);
  std::vector<Eigen::MatrixXd> Rpstart = Algebra::TransToRp(Xstart);
  std::vector<Eigen::MatrixXd> Rpend = Algebra::TransToRp(Xend);
  Eigen::Matrix3d Rstart =
    Rpstart[0]; Eigen::Vector3d pstart = Rpstart[1];
  Eigen::Matrix3d Rend =
    Rpend[0]; Eigen::Vector3d pend = Rpend[1];
  double st;
  for (int i = 0; i < N; ++i) {
    if (method == 3)
      st = CubicTimeScaling(dt*i);
    else
      st = QuinticTimeScaling(dt*i);
    Eigen::Matrix3d Ri = Rstart *
      Algebra::MatrixExp3(Algebra::MatrixLog3(Rstart.transpose() * Rend)*st);
    Eigen::Vector3d pi = st*pend + (1 - st)*pstart;
    Eigen::MatrixXd traji(4, 4);
    traji << Ri, pi,
      0, 0, 0, 1;
    traj.at(i) = traji;
  }
  return traj;
}
