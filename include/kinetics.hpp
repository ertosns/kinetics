#include "algebra.hpp"

class Kinetics {
  
public:   
  Kinetics() {
  }
  /**
   * Uses forward-backward Newton-Euler iterations to solve the equation:
   *   taulist = Mlist(thetalist) * ddthetalist + 
   *           c(thetalist, dthetalist)  + 
   *           g(thetalist) + Jtr(thetalist) * Ftip
   * 
   * @param thetalist n-vector of joint variables
   * @param dthetalist n-vector of joint rates
   * @param ddthetalist n-vector of joint accelerations
   * @param g Gravity vector g
   * @param Ftip Spatial force applied by the end-effector 
   *        expressed in frame {n+1}
   * @param Mlist List of link frames {i} relative to {i-1} 
   *         at the home position
   * @param Glist Spatial inertia matrices Gi of the links
   * @param Slist Screw axes Si of the joints in a space frame, 
   *         in the format
   *         of a matrix with the screw axes as the columns.
   * @return taulist The n-vector of required joint forces/torques
   */
  inline Eigen::VectorXd
  InverseDynamics(const Eigen::VectorXd& thetalist,
                  const Eigen::VectorXd& dthetalist,
                  const Eigen::VectorXd& ddthetalist,
                  const Eigen::VectorXd& g,
                  const Eigen::VectorXd& Ftip,
                  const std::vector<Eigen::MatrixXd>& Mlist,
                  const std::vector<Eigen::MatrixXd>& Glist,
                  const Eigen::MatrixXd& Slist) const;

  /**
   * Computes ddthetalist by solving:
   * Mlist(thetalist) * ddthetalist = taulist - 
   *                        c(thetalist,dthetalist)
   *                       - g(thetalist) - Jtr(thetalist) * Ftip
   *
   * @param thetalist n-vector of joint variables
   * @param dthetalist n-vector of joint rates
   * @param taulist An n-vector of joint forces/torques
   * @param g Gravity vector g
   * @param Ftip Spatial force applied by the end-effector 
   *        expressed in frame {n+1}
   * @param Mlist List of link frames {i} relative to {i-1} 
   *         at the home position
   * @param Glist Spatial inertia matrices Gi of the links
   * @param Slist Screw axes Si of the joints in a space frame, 
   *         in the format
   *         of a matrix with the screw axes as the columns.
   * @return ddthetalist The resulting joint accelerations.
   */
  inline Eigen::VectorXd
  ForwardDynamics(const Eigen::VectorXd& thetalist,
                  const Eigen::VectorXd& dthetalist,
                  const Eigen::VectorXd& taulist,
                  const Eigen::VectorXd& g,
                  const Eigen::VectorXd& Ftip,
                  const std::vector<Eigen::MatrixXd>& Mlist,
                  const std::vector<Eigen::MatrixXd>& Glist,
                  const Eigen::MatrixXd& Slist) const;


  //TODO move it to private
  inline void
  EulerStep(Eigen::VectorXd& thetalist,
                      Eigen::VectorXd& dthetalist,
                      const Eigen::VectorXd& ddthetalist,
                      double dt) const;

  Eigen::VectorXd
  ComputedTorque(const Eigen::VectorXd& thetalist,
                 const Eigen::VectorXd& dthetalist,
                 const Eigen::VectorXd& eint,
                 const Eigen::VectorXd& g,
                 const std::vector<Eigen::MatrixXd>& Mlist,
                 const std::vector<Eigen::MatrixXd>& Glist,
                 const Eigen::MatrixXd& Slist,
                 const Eigen::VectorXd& thetalistd,
                 const Eigen::VectorXd& dthetalistd,
                 const Eigen::VectorXd& ddthetalistd,
                 double Kp, double Ki, double Kd);

  std::vector<Eigen::MatrixXd>
  SimulateControl(const Eigen::VectorXd& thetalist,
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
                  int intRes);
private:
  
  /**
   * Calls InverseDynamics with 
   *           Ftip = 0, dthetalist = 0, and
   *           ddthetalist = 0. The purpose is to calculate one 
   *           important term in the dynamics equation
   *
   * @param thetalist n-vector of joint variables
   * @param  g Gravity vector g
   * @param  Mlist List of link frames {i} relative to {i-1}  at the home position
   * @param Glist Spatial inertia matrices Gi of the links
   * @param  Slist Screw axes Si of the joints in a space frame, 
   *  in the format of a matrix with the screw axes as the columns.
   *
   * @return grav The 3-vector showing the effect force of gravity to the dynamics
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
   * Calls InverseDynamics n times,
   *           each time passing a ddthetalist vector with 
   *           a single element equal to one and all other
   *           inputs set to zero. Each call of InverseDynamics 
   *           generates a single column, and these columns are 
   *           assembled to create the inertia matrix.
   *
   * @param thetalist n-vector of joint variables
   * @param Mlist List of link frames {i} relative to {i-1} 
   *         at the home position
   * @param Glist Spatial inertia matrices Gi of the links
   * @param Slist Screw axes Si of the joints in a space frame, 
   *         in the format
   *         of a matrix with the screw axes as the columns.
   * @return M The numerical inertia matrix M(thetalist) of an n-joint serial chain at the given configuration thetalist.
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
  
  /* Calls InverseDynamics with g = 0, 
   *           Ftip = 0, and ddthetalist = 0.
   *
   * @param thetalist n-vector of joint variables
   * @param dthetalist A list of joint rates
   * @param  Mlist List of link frames {i} relative to {i-1} 
   *         at the home position
   * @param Glist Spatial inertia matrices Gi of the links
   * @param Slist Screw axes Si of the joints in a space frame, 
   *         in the format
   *         of a matrix with the screw axes as the columns.
   * @return c The vector c(thetalist,dthetalist) of Coriolis and centripetal terms for a given thetalist and dthetalist.
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
  
  /* Calls InverseDynamics with g = 0, 
   *           dthetalist = 0, and ddthetalist = 0.
   *
   * @param thetalist n-vector of joint variables
   * @param Ftip Spatial force applied by the end-effector 
   *        expressed in frame {n+1}
   * @param Mlist List of link frames {i} relative to {i-1} 
   *         at the home position
   * @param Glist Spatial inertia matrices Gi of the links
   * @param Slist Screw axes Si of the joints in a space frame, 
   *         in the format of a matrix with the screw axes 
   *         as the columns.    
   * @return JTFtip The joint forces and torques required only 
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
  
};
