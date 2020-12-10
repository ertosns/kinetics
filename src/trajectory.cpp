#include "../include/trajectory.hpp"
  
Eigen::MatrixXd
Trajectory::JointTrajectory(const Eigen::VectorXd& thetastart,
                            const Eigen::VectorXd& thetaend,
                            double Tf, int N, int method) {
  double timegap = Tf / (N - 1);
  Eigen::MatrixXd trajT =
    Eigen::MatrixXd::Zero(thetastart.size(), N);
  double st;
  for (int i = 0; i < N; ++i) {
    if (method == 3)
      st = CubicTimeScaling(Tf, timegap*i);
    else
      st = QuinticTimeScaling(Tf, timegap*i);
    trajT.col(i) = st * thetaend + (1 - st)*thetastart;
  }
  Eigen::MatrixXd traj = trajT.transpose();
  return traj;
}

std::vector<Eigen::MatrixXd>
Trajectory::ScrewTrajectory(const Eigen::MatrixXd& Xstart,
                            const Eigen::MatrixXd& Xend,
                            double Tf, int N, int method) {
  double timegap = Tf / (N - 1);
  std::vector<Eigen::MatrixXd> traj(N);
  double st;
  for (int i = 0; i < N; ++i) {
    if (method == 3)
      st = CubicTimeScaling(Tf, timegap*i);
    else
      st = QuinticTimeScaling(Tf, timegap*i);
    Eigen::MatrixXd Ttemp = Algebra::MatrixLog6(TransInv(Xstart)*Xend);
    traj.at(i) = Xstart * Algebra::MatrixExp6(Ttemp*st);
  }
  return traj;
}

std::vector<Eigen::MatrixXd>
Trajectory::CartesianTrajectory(const Eigen::MatrixXd& Xstart,
                                const Eigen::MatrixXd& Xend,
                                double Tf, int N, int method) {
  double timegap = Tf / (N - 1);
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
      st = CubicTimeScaling(Tf, timegap*i);
    else
      st = QuinticTimeScaling(Tf, timegap*i);
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

Eigen::MatrixXd
Trajectory::InverseDynamicsTrajectory(const Eigen::MatrixXd& thetamat,
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
    taumatT.col(i) =
      kinetics.InverseDynamics(thetamatT.col(i), dthetamatT.col(i),
                               ddthetamatT.col(i), g, FtipmatT.col(i),
                               Mlist, Glist, Slist);
  }
  Eigen::MatrixXd taumat = taumatT.transpose();
  return taumat;
}

std::vector<Eigen::MatrixXd>
Trajectory::ForwardDynamicsTrajectory(const Eigen::VectorXd& thetalist,
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
  Eigen::VectorXd ddthetalist;
  for (int i = 0; i < N - 1; ++i) {
    for (int j = 0; j < intRes; ++j) {
      ddthetalist = kinetics.ForwardDynamics(thetacurrent, dthetacurrent,
                                             taumatT.col(i), g,
                                             FtipmatT.col(i), Mlist,
                                             Glist, Slist);
      kinetics.EulerStep(thetacurrent, dthetacurrent, ddthetalist,
                         1.0*dt / intRes);
    }
    thetamatT.col(i + 1) = thetacurrent;
    dthetamatT.col(i + 1) = dthetacurrent;
  }
  std::vector<Eigen::MatrixXd> JointTraj_ret;
  JointTraj_ret.push_back(thetamatT.transpose());
  JointTraj_ret.push_back(dthetamatT.transpose());
  return JointTraj_ret;
}

Eigen::VectorXd
Trajectory::ComputedTorque(const Eigen::VectorXd& thetalist,
                           const Eigen::VectorXd& dthetalist,
                           const Eigen::VectorXd& eint,
                           const Eigen::VectorXd& g,
                           const std::vector<Eigen::MatrixXd>& Mlist,
                           const std::vector<Eigen::MatrixXd>& Glist,
                           const Eigen::MatrixXd& Slist,
                           const Eigen::VectorXd& thetalistd,
                           const Eigen::VectorXd& dthetalistd,
                           const Eigen::VectorXd& ddthetalistd,
                           double Kp, double Ki, double Kd) {
  
  Eigen::VectorXpd e = thetalistd - thetalist;  // position err
  Eigen::VectorXd tau_feedforward =
    kinetics.MassMatrix(thetalist, Mlist, Glist, Slist)*
    (Kp*e + Ki * (eint + e) + Kd * (dthetalistd - dthetalist));
  
  Eigen::VectorXd Ftip = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd tau_inversedyn =
    kinetics.InverseDynamics(thetalist, dthetalist, ddthetalistd,
                             g, Ftip, Mlist, Glist, Slist);
  Eigen::VectorXd tau_computed =
    tau_feedforward + tau_inversedyn;
  return tau_computed;
}  

std::vector<Eigen::MatrixXd>
Trajectory::SimulateControl(const Eigen::VectorXd& thetalist,
                            const Eigen::VectorXd& dthetalist,
                            const Eigen::VectorXd& g,
                            const Eigen::MatrixXd& Ftipmat,
                            const std::vector<Eigen::MatrixXd>& Mlist,
                            const std::vector<Eigen::MatrixXd>& Glist,
                            const Eigen::MatrixXd& Slist,
                            const Eigen::MatrixXd& thetamatd,
                            const Eigen::MatrixXd& dthetamatd,
                            const Eigen::MatrixXd& ddthetamatd,
                            const Eigen::VectorXd& gtilde,
                            const std::vector<Eigen::MatrixXd>& Mtildelist,
                            const std::vector<Eigen::MatrixXd>& Gtildelist,
                            double Kp, double Ki, double Kd, double dt,
                            int intRes) {
  Eigen::MatrixXd FtipmatT = Ftipmat.transpose();
  Eigen::MatrixXd thetamatdT = thetamatd.transpose();
  Eigen::MatrixXd dthetamatdT = dthetamatd.transpose();
  Eigen::MatrixXd ddthetamatdT = ddthetamatd.transpose();
  int m = thetamatdT.rows();
  int n = thetamatdT.cols();
  Eigen::VectorXd thetacurrent = thetalist;
  Eigen::VectorXd dthetacurrent = dthetalist;
  Eigen::VectorXd eint = Eigen::VectorXd::Zero(m);
  Eigen::MatrixXd taumatT = Eigen::MatrixXd::Zero(m, n);
  Eigen::MatrixXd thetamatT = Eigen::MatrixXd::Zero(m, n);
  Eigen::VectorXd taulist;
  Eigen::VectorXd ddthetalist;
  for (int i = 0; i < n; ++i) {
    taulist = kinetics.ComputedTorque(thetacurrent, dthetacurrent, eint,
                                      gtilde, Mtildelist, Gtildelist,
                                      Slist, thetamatdT.col(i),
                                      dthetamatdT.col(i),
                                      ddthetamatdT.col(i), Kp, Ki, Kd);
    for (int j = 0; j < intRes; ++j) {
      ddthetalist = kinetics.ForwardDynamics(thetacurrent, dthetacurrent,
                                             taulist, g, FtipmatT.col(i),
                                             Mlist, Glist, Slist);
      kinetics.EulerStep(thetacurrent, dthetacurrent, ddthetalist,
                         dt / intRes);
    }
    taumatT.col(i) = taulist;
    thetamatT.col(i) = thetacurrent;
    eint += dt * (thetamatdT.col(i) - thetacurrent);
  }
  std::vector<Eigen::MatrixXd> ControlTauTraj_ret;
  ControlTauTraj_ret.push_back(taumatT.transpose());
  ControlTauTraj_ret.push_back(thetamatT.transpose());
  return ControlTauTraj_ret;
}

