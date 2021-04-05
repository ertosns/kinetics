/*
class Mission{
  Mission() {
  }
  Eigen::MatrixXd
  InverseDynamicsTrajectory(const Eigen::MatrixXd& thetamat,
                            const Eigen::MatrixXd& dthetamat,
                            const Eigen::MatrixXd& ddthetamat,
                            const Eigen::VectorXd& g,
                            const Eigen::MatrixXd& Ftipmat,
                            const std::vector<Eigen::MatrixXd>& Mlist,
                            const std::vector<Eigen::MatrixXd>& Glist,
                            const Eigen::MatrixXd& Slist);
  std::vector<Eigen::MatrixXd>
  ForwardDynamicsTrajectory(const Eigen::VectorXd& thetalist,
                            const Eigen::VectorXd& dthetalist,
                            const Eigen::MatrixXd& taumat,
                            const Eigen::VectorXd& g,
                            const Eigen::MatrixXd& Ftipmat,
                            const std::vector<Eigen::MatrixXd>& Mlist,
                            const std::vector<Eigen::MatrixXd>& Glist,
                            const Eigen::MatrixXd& Slist,
                            double dt, int intRes);
};

#include "../include/mission.hpp"
#include "kinetics.hpp"

//TODO make kinetics more general to work on matrices, and vectors.
Eigen::MatrixXd
Mission::InverseDynamicsTrajectory(const Eigen::MatrixXd& thetamat,
                                   const Eigen::MatrixXd& dthetamat,
                                   const Eigen::MatrixXd& ddthetamat,
                                   const Eigen::VectorXd& g,
                                   const Eigen::MatrixXd& Ftipmat,
                                   const std::vector<Eigen::MatrixXd>& Mlist,
                                   const std::vector<Eigen::MatrixXd>& Glist,

                                   const Eigen::MatrixXd& Slist) {
  Eigen::MatrixXd thetamatT = thetamat.transpose();
  Eigen::MatrixXd dthetamatT = dthetamat.transpose();
  Eigen::MatrixXd ddthetamatT = ddthetamat.transpose();
  Eigen::MatrixXd FtipmatT = Ftipmat.transpose();

  int N = thetamat.rows();  // trajectory points
  int dof = thetamat.cols();
  Eigen::MatrixXd taumatT = Eigen::MatrixXd::Zero(dof, N);
  for (int i = 0; i < N; ++i) {
    //TODO (fix) this is more expensive.
    taumatT.col(i) = Kinetics(thetamatT.col(i),
                              dthetamatT.col(i),
                              ddthetamatT.col(i),
                              Mlist, Glist, Slist, g)
      .InverseDynamics(FtipmatT.col(i))

    taumatT.col(i) =
      kinetics.InverseDynamics(thetamatT.col(i),
                               dthetamatT.col(i),
                               ddthetamatT.col(i),
                               g, FtipmatT.col(i),
                               Mlist, Glist, Slist);

  }
  Eigen::MatrixXd taumat = taumatT.transpose();
  return taumat;
}

std::vector<Eigen::MatrixXd>
Mission::ForwardDynamicsTrajectory(const Eigen::VectorXd& thetalist,
                                      const Eigen::VectorXd& dthetalist,
                                      const Eigen::MatrixXd& taumat,
                                      const Eigen::VectorXd& g,
                                      const Eigen::MatrixXd& Ftipmat,
                                      const std::vector<Eigen::MatrixXd>& Mlist,
                                      const std::vector<Eigen::MatrixXd>& Glist,

                                      const Eigen::MatrixXd& Slist,
                                      double dt, int intRes) {
  Eigen::MatrixXd taumatT = taumat.transpose();
  Eigen::MatrixXd FtipmatT = Ftipmat.transpose();
  int N = taumat.rows();  // force/torque points
  int dof = taumat.cols();
  Eigen::MatrixXd thetamatT = Eigen::MatrixXd::Zero(dof, N);
  Eigen::MatrixXd dthetamatT = Eigen::MatrixXd::Zero(dof, N);
  thetamatT.col(0) = thetalist;
  dthetamatT.col(0) = dthetalist;
  Eigen::VectorXd thetacurrent = thetalist;
  Eigen::VectorXd dthetacurrent = dthetalist;
  Eigen::VectorXd ddthetalist = Eigen::VectorXd::Zero(dof);
  for (int i = 0; i < N - 1; ++i) {
    for (int j = 0; j < intRes; ++j) {
      auto kinetics = Kinetics(thetacurrent,
                               dthetacurrent,
                               ddthetalist,
                               Mlist,
                               Glist,
                               Slist,
                               g)
        kinetics.ForwardDynamics(taumatT.col(i), FtipmatT.col(i));

      ddthetalist = kinetics.ForwardDynamics(thetacurrent, dthetacurrent,
                                             taumatT.col(i), g,
                                             FtipmatT.col(i), Mlist,
                                             Glist, Slist);

      kinetics.EulerStep(1.0*dt / intRes);
    }
    thetamatT.col(i + 1) = get_pos();
    dthetamatT.col(i + 1) = get_vel();
  }
  std::vector<Eigen::MatrixXd> JointTraj_ret;
  JointTraj_ret.push_back(thetamatT.transpose());
  JointTraj_ret.push_back(dthetamatT.transpose());
  return JointTraj_ret;
}
*/
