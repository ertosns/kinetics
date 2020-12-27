#include <Eigen/Dense>
#include <vector>
#include "../include/kinetics.hpp"

//TODO divide this class up into two different classes, forward, and backward, with base kinetics class, similarly for kinematics.

inline Eigen::VectorXd
Kinetics::InverseDynamics(const Eigen::VectorXd& Ftip) {
  int n=N;
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
      AdTi[i] * Vi.col(i) + Ai.col(i) * dthetalist(i);
    //TODO (res)
    // this index is different from book!;
    Vdi.col(i+1) =
      AdTi[i] * Vdi.col(i) + Ai.col(i) * ddthetalist(i)
      + Algebra::ad(Vi.col(i+1)) * Ai.col(i) * dthetalist(i); 
  }
  
  // backward pass
  for (int i = n-1; i >= 0; i--) {
    Fi = AdTi[i+1].transpose() * Fi + Glist[i] * Vdi.col(i+1)
      - Algebra::ad(Vi.col(i+1)).transpose() * (Glist[i] * Vi.col(i+1));
    taulist(i) = Fi.transpose() * Ai.col(i);
  }
  return taulist;
}


//TODO call the private(to be) euler function here, and then update the test files, and calling function (i only see Simulation funciton down).
Eigen::VectorXd
Kinetics::ForwardDynamics(const Eigen::VectorXd &taulist,
                          const Eigen::VectorXd &Ftip)  {
  Eigen::VectorXd tau = taulist;
  Eigen::VectorXd Ft = Ftip;
  
  // sum of end effector, velocity&coriolis, and gravity forces.
  Eigen::VectorXd totalForce = tau -
    VelQuadraticForces()-GravityForces()-EndEffectorForces(Ft);
  
  // rotational inertial mass matrix.
  Eigen::MatrixXd M = MassMatrix();
  
  // Use LDLT since M is positive definite
  //Eigen::VectorXd ddthetalist = M.ldlt().solve(totalForce);
  ddthetalist = M.ldlt().solve(totalForce);
  
  return ddthetalist;
  
  return Eigen::VectorXd(3);
}


Eigen::VectorXd
Kinetics::ComputedTorque(const Eigen::VectorXd& thetalistd,
                         const Eigen::VectorXd& dthetalistd,
                         const Eigen::VectorXd& ddthetalistd, //TODO (FIX) it's not used! supposed to pass to inversedynamics!
                         const Eigen::VectorXd& eint) {
  Eigen::VectorXd Ftip = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd e = thetalistd - thetalist;  // position err
  //
  Eigen::VectorXd tau_feedforward = MassMatrix()*
    (Kp*e + Ki * (eint + e) +
     Kd * (dthetalistd - dthetalist));
  Eigen::VectorXd tau_inversedyn = InverseDynamics(Ftip);
  //
  Eigen::VectorXd tau_computed = tau_feedforward + tau_inversedyn;
  return tau_computed;
}

/*
std::vector<Eigen::MatrixXd>
Kinetics::SimulateControl(const Eigen::MatrixXd& thetamatd,
                          const Eigen::MatrixXd& dthetamatd,
                          const Eigen::MatrixXd& ddthetamatd,
                          const Eigen::MatrixXd& Ftipmat,
                          const Eigen::VectorXd& gtilde,
                          const std::vector<Eigen::MatrixXd>& Mtildelist,
                          const std::vector<Eigen::MatrixXd>& Gtildelist,
                          double dt, int intRes) {
  Eigen::MatrixXd FtipmatT = Ftipmat.transpose();
  Eigen::MatrixXd thetamatdT = thetamatd.transpose();
  Eigen::MatrixXd dthetamatdT = dthetamatd.transpose();
  Eigen::MatrixXd ddthetamatdT = ddthetamatd.transpose();
  int m = thetamatdT.rows();
  int n = thetamatdT.cols();
  Eigen::VectorXd eint = Eigen::VectorXd::Zero(m);
  Eigen::MatrixXd taumatT = Eigen::MatrixXd::Zero(m, n);
  Eigen::MatrixXd thetamatT = Eigen::MatrixXd::Zero(m, n);
  Eigen::VectorXd taulist;
  Eigen::VectorXd ddthetalist;
  for (int i = 0; i < n; ++i) {
    //TODO (res) tilde controlled values, update ComputedTroque
    taulist = ComputedTorque(thetamatdT.col(i),
                             dthetamatdT.col(i),
                             ddthetamatdT.col(i),
                             eint,
                             gtilde, Mtildelist, Gtildelist);
    for (int j = 0; j < intRes; ++j) {
      ddthetalist = ForwardDynamics(taulist, FtipmatT.col(i));
      EulerStep(thetacurrent, dthetacurrent, ddthetalist,
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
*/
