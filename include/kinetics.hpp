#include "algebra.hpp"

class Kinetics : protected Algebra {

public:
  /*
   * Function: This function uses forward-backward 
   *   Newton-Euler iterations to solve the equation:
   * taulist = Mlist(thetalist) * ddthetalist + 
   *           c(thetalist, dthetalist)  + 
   *           g(thetalist) + Jtr(thetalist) * Ftip
   * Inputs:
   *  thetalist: n-vector of joint variables
   *  dthetalist: n-vector of joint rates
   *  ddthetalist: n-vector of joint accelerations
   *  g: Gravity vector g
   *  Ftip: Spatial force applied by the end-effector 
   *        expressed in frame {n+1}
   *  Mlist: List of link frames {i} relative to {i-1} 
   *         at the home position
   *  Glist: Spatial inertia matrices Gi of the links
   *  Slist: Screw axes Si of the joints in a space frame, 
   *         in the format
   *         of a matrix with the screw axes as the columns.
   *
   * Outputs:
   *  taulist: The n-vector of required joint forces/torques
   *
   */
  inline Eigen::VectorXd
  InverseDynamics(const Eigen::VectorXd& thetalist,
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
    AdTi[n] = Adjoint(TransInv(Mlist[n]));
    Eigen::VectorXd Fi = Ftip;
    
    Eigen::VectorXd taulist = Eigen::VectorXd::Zero(n);
    
    // forward pass
    for (int i = 0; i < n; i++) {
      Mi = Mi * Mlist[i];
      Ai.col(i) = Adjoint(TransInv(Mi))*Slist.col(i);

      AdTi[i] =
        Adjoint(MatrixExp6(VecTose3(Ai.col(i)*-thetalist(i)))
                * TransInv(Mlist[i]));

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

  /*
   * Function: This function calls InverseDynamics with 
   *           Ftip = 0, dthetalist = 0, and
   *           ddthetalist = 0. The purpose is to calculate one 
   *           important term in the dynamics equation
   * Inputs:
   *  thetalist: n-vector of joint variables
   *  g: Gravity vector g
   *  Mlist: List of link frames {i} relative to {i-1} 
   *  at the home position
   *  Glist: Spatial inertia matrices Gi of the links
   *  Slist: Screw axes Si of the joints in a space frame, 
   *  in the format
   *         of a matrix with the screw axes as the columns.
   *
   * Outputs:
   *  grav: The 3-vector showing the effect force of 
   *  gravity to the dynamics
   *
   */
  inline Eigen::VectorXd
  GravityForces(const Eigen::VectorXd& thetalist,
                const Eigen::VectorXd& g,
                const std::vector<Eigen::MatrixXd>& Mlist,
                const std::vector<Eigen::MatrixXd>& Glist,
                const Eigen::MatrixXd& Slist) const {
    int n = thetalist.size();
    Eigen::VectorXd dummylist = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd dummyForce = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd grav =
      InverseDynamics(thetalist, dummylist, dummylist, g,
                      dummyForce, Mlist, Glist, Slist);
    return grav;
  }
  
  /*
   * Function: This function calls InverseDynamics n times,
   *           each time passing a ddthetalist vector with 
   *           a single element equal to one and all other
   *           inputs set to zero. Each call of InverseDynamics 
   *           generates a single column, and these columns are 
   *           assembled to create the inertia matrix.
   *
   * Inputs:
   *  thetalist: n-vector of joint variables
   *  Mlist: List of link frames {i} relative to {i-1} 
   *         at the home position
   *  Glist: Spatial inertia matrices Gi of the links
   *  Slist: Screw axes Si of the joints in a space frame, 
   *         in the format
   *         of a matrix with the screw axes as the columns.
   *
   * Outputs:
   *  M: The numerical inertia matrix M(thetalist) of 
   *     an n-joint serial
   *     chain at the given configuration thetalist.
   */
  inline Eigen::MatrixXd
  MassMatrix(const Eigen::VectorXd& thetalist,
             const std::vector<Eigen::MatrixXd>& Mlist,
             const std::vector<Eigen::MatrixXd>& Glist,
             const Eigen::MatrixXd& Slist) const {
    int n = thetalist.size();
    Eigen::VectorXd dummylist = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd dummyg = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd dummyforce = Eigen::VectorXd::Zero(6);
    Eigen::MatrixXd M = Eigen::MatrixXd::Zero(n,n);
    for (int i = 0; i < n; i++) {
      Eigen::VectorXd ddthetalist = Eigen::VectorXd::Zero(n);
      ddthetalist(i) = 1;
      M.col(i) = InverseDynamics(thetalist, dummylist, ddthetalist,
                                 dummyg, dummyforce, Mlist, Glist,
                                 Slist);
    }
    return M;
  }
  
  /*
   * Function: This function calls InverseDynamics with g = 0, 
   *           Ftip = 0, and
   * ddthetalist = 0.
   *
   * Inputs:
   *  thetalist: n-vector of joint variables
   *  dthetalist: A list of joint rates
   *  Mlist: List of link frames {i} relative to {i-1} 
   *         at the home position
   *  Glist: Spatial inertia matrices Gi of the links
   *  Slist: Screw axes Si of the joints in a space frame, 
   *         in the format
   *         of a matrix with the screw axes as the columns.
   *
   * Outputs:
   *  c: The vector c(thetalist,dthetalist) of Coriolis and 
   *     centripetal
   *     terms for a given thetalist and dthetalist.
   */
  inline Eigen::VectorXd
  VelQuadraticForces(const Eigen::VectorXd& thetalist,
                     const Eigen::VectorXd& dthetalist,
                     const std::vector<Eigen::MatrixXd>& Mlist,
                     const std::vector<Eigen::MatrixXd>& Glist,
                     const Eigen::MatrixXd& Slist) const {
    int n = thetalist.size();
    Eigen::VectorXd dummylist = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd dummyg = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd dummyforce = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd c = InverseDynamics(thetalist, dthetalist,
                                        dummylist, dummyg,
                                        dummyforce, Mlist,
                                        Glist, Slist);
    return c;
  }
  
  /*
   * Function: This function calls InverseDynamics with g = 0, 
   *           dthetalist = 0, and
   * ddthetalist = 0.
   *
   * Inputs:
   *  thetalist: n-vector of joint variables
   *  Ftip: Spatial force applied by the end-effector 
   *        expressed in frame {n+1}
   *  Mlist: List of link frames {i} relative to {i-1} 
   *         at the home position
   *  Glist: Spatial inertia matrices Gi of the links
   *  Slist: Screw axes Si of the joints in a space frame, 
   *         in the format of a matrix with the screw axes 
   *         as the columns.
   *
   * Outputs:
   *  JTFtip: The joint forces and torques required only 
   *          to create the end-effector force Ftip.
   */
  inline Eigen::VectorXd
  EndEffectorForces(const Eigen::VectorXd& thetalist,
                    const Eigen::VectorXd& Ftip,
                    const std::vector<Eigen::MatrixXd>& Mlist,
                    const std::vector<Eigen::MatrixXd>& Glist,
                    const Eigen::MatrixXd& Slist) const {
    int n = thetalist.size();
    Eigen::VectorXd dummylist = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd dummyg = Eigen::VectorXd::Zero(3);
    
    Eigen::VectorXd JTFtip = InverseDynamics(thetalist, dummylist,
                                             dummylist, dummyg,
                                             Ftip, Mlist, Glist,
                                             Slist);
    return JTFtip;
  }
  
  /*
   * Function: This function computes ddthetalist by solving:
   * Mlist(thetalist) * ddthetalist = taulist - 
   *                        c(thetalist,dthetalist)
   *                       - g(thetalist) - Jtr(thetalist) * Ftip
   * Inputs:
   *  thetalist: n-vector of joint variables
   *  dthetalist: n-vector of joint rates
   *  taulist: An n-vector of joint forces/torques
   *  g: Gravity vector g
   *  Ftip: Spatial force applied by the end-effector 
   *        expressed in frame {n+1}
   *  Mlist: List of link frames {i} relative to {i-1} 
   *         at the home position
   *  Glist: Spatial inertia matrices Gi of the links
   *  Slist: Screw axes Si of the joints in a space frame, 
   *         in the format
   *         of a matrix with the screw axes as the columns.
   *
   * Outputs:
   *  ddthetalist: The resulting joint accelerations
   *
   */
  inline Eigen::VectorXd
  ForwardDynamics(const Eigen::VectorXd& thetalist,
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
  
  inline void EulerStep(Eigen::VectorXd& thetalist,
                        Eigen::VectorXd& dthetalist,
                        const Eigen::VectorXd& ddthetalist,
                        double dt) const {
    thetalist += dthetalist * dt;
    dthetalist += ddthetalist * dt;
    return;
  }
  
};


