#include <Eigen/Dense>
#include <vector>
#include "../include/kinetics.hpp"

inline Eigen::VectorXd
Kinetics::InverseDynamics(const Eigen::VectorXd& thetalist,
                          const Eigen::VectorXd& dthetalist,
                          const Eigen::VectorXd& ddthetalist,
                          const Eigen::VectorXd& g,
                          const Eigen::VectorXd& Ftip,
                          const std::vector<Eigen::MatrixXd>& Mlist,
                          const std::vector<Eigen::MatrixXd>& Glist,
                          const Eigen::MatrixXd& Slist) const {
  // the size of the lists
  int n = thetalist.size();
  
  Eigen::MatrixXd Mi = Eigen::MatrixXd::Identity(4, 4);
  Eigen::MatrixXd Ai = Eigen::MatrixXd::Zero(6,n);
  std::vector<Eigen::MatrixXd> AdTi;
  for (int i = 0; i < n+1; i++) {
    AdTi.push_back(Eigen::MatrixXd::Zero(6,6));
  }
  // velocity
  Eigen::MatrixXd Vi = Eigen::MatrixXd::Zero(6,n+1);
  // acceleration
  Eigen::MatrixXd Vdi = Eigen::MatrixXd::Zero(6,n+1);
  
  Vdi.block(3, 0, 3, 1) = - g;
  AdTi[n] = Algebra::Adjoint(Algebra::TransInv(Mlist[n]));
  Eigen::VectorXd Fi = Ftip;
  
  Eigen::VectorXd taulist = Eigen::VectorXd::Zero(n);
  
  // forward pass
  for (int i = 0; i < n; i++) {
    Mi = Mi * Mlist[i];
    Ai.col(i) = Algebra::Adjoint(Algebra::TransInv(Mi))*Slist.col(i);
    
    AdTi[i] =
      Algebra::Adjoint(Algebra::MatrixExp6(Algebra::VecTose3(Ai.col(i)*-thetalist(i))) * Algebra::TransInv(Mlist[i]));
    
    Vi.col(i+1) =
      AdTi[i] * Vi.col(i) + Ai.col(i) * dthetalist(i)
      //TODO (res)
      // this index is different from book!;
      Vdi.col(i+1) =
      AdTi[i] * Vdi.col(i) + Ai.col(i) * ddthetalist(i)
      + ad(Vi.col(i+1)) * Ai.col(i) * dthetalist(i); 
  }
  
  // backward pass
  for (int i = n-1; i >= 0; i--) {
    Fi = AdTi[i+1].transpose() * Fi + Glist[i] * Vdi.col(i+1)
      - ad(Vi.col(i+1)).transpose() * (Glist[i] * Vi.col(i+1));
    taulist(i) = Fi.transpose() * Ai.col(i);
  }
  return taulist;
}


inline Eigen::VectorXd
Kinetics::ForwardDynamics(const Eigen::VectorXd& thetalist,
                          const Eigen::VectorXd& dthetalist,
                          const Eigen::VectorXd& taulist,
                          const Eigen::VectorXd& g,
                          const Eigen::VectorXd& Ftip,
                          const std::vector<Eigen::MatrixXd>& Mlist,
                          const std::vector<Eigen::MatrixXd>& Glist,
                          const Eigen::MatrixXd& Slist) const {
  
  Eigen::VectorXd totalForce =
    taulist - VelQuadraticForces(thetalist, dthetalist,
                                 Mlist, Glist, Slist)
    - GravityForces(thetalist, g, Mlist, Glist, Slist)
    - EndEffectorForces(thetalist, Ftip, Mlist, Glist, Slist);
  
  Eigen::MatrixXd M = MassMatrix(thetalist, Mlist, Glist, Slist);
  
  // Use LDLT since M is positive definite
  Eigen::VectorXd ddthetalist = M.ldlt().solve(totalForce);
  
  return ddthetalist;
}

inline void
Kinetics::EulerStep(Eigen::VectorXd& thetalist,
                    Eigen::VectorXd& dthetalist,
                    const Eigen::VectorXd& ddthetalist,
                    double dt) const {
  thetalist += dthetalist * dt;
  dthetalist += ddthetalist * dt;
  return;
}

Eigen::VectorXd
Kinetics::ComputedTorque(const Eigen::VectorXd& thetalist,
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
    ket.MassMatrix(thetalist, Mlist, Glist, Slist)*
    (Kp*e + Ki * (eint + e) + Kd * (dthetalistd - dthetalist));
  
  Eigen::VectorXd Ftip = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd tau_inversedyn =
    ket.InverseDynamics(thetalist, dthetalist, ddthetalistd,
                    g, Ftip, Mlist, Glist, Slist);
  Eigen::VectorXd tau_computed =
    tau_feedforward + tau_inversedyn;
  return tau_computed;
}

std::vector<Eigen::MatrixXd>
Kinetics::SimulateControl(const Eigen::VectorXd& thetalist,
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
    taulist = ket.ComputedTorque(thetacurrent, dthetacurrent, eint,
                                 gtilde, Mtildelist, Gtildelist,
                                 Slist, thetamatdT.col(i),
                                 dthetamatdT.col(i),
                                 ddthetamatdT.col(i), Kp, Ki, Kd);
    for (int j = 0; j < intRes; ++j) {
      ddthetalist = ket.ForwardDynamics(thetacurrent, dthetacurrent,
                                        taulist, g, FtipmatT.col(i),
                                        Mlist, Glist, Slist);
      ket.EulerStep(thetacurrent, dthetacurrent, ddthetalist,
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
