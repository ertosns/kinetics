#include  "algebra.hpp"

//TODO create stand-alone PID-controller
class Kinetics {
public:
  //position
  Eigen::VectorXd thetalist;
  //velocity
  Eigen::VectorXd dthetalist;
  //acceleration
  Eigen::VectorXd ddthetalist;
  
  //TODO how to allocate, and pass const vector?
  /** Constructor
   *
   * @param thetalist n-vector of  joint variables
   * @param dthetalist n-vector of joint rates
   * @param ddthetalist n-vector of joint accelerations
   * @param g Gravity vector g
   * @param Mlist List of link frames {i} relative to {i-1} 
   *         at the home position
   * @param Glist Spatial inertia matrices Gi of the links
   * @param Slist Screw axes Si of the joints in a space frame, 
   *         in the format of a matrix with the screw axes as the columns.
   */
  //TODO pass heavy variables in a unique_ptr
  Kinetics(Eigen::VectorXd _thetalist,
           Eigen::VectorXd _dthetalist,
           Eigen::VectorXd _ddthetalist,
           std::vector<Eigen::MatrixXd> _Mlist,
           std::vector<Eigen::MatrixXd> _Glist,
           Eigen::MatrixXd _Slist,
           Eigen::VectorXd _g=Eigen::Vector3d(0,0,-9.81),
           double kp=0.1,
           double ki=0.1,
           double kd=0.1) : thetalist(_thetalist),
                            dthetalist(_dthetalist),
                            ddthetalist(_ddthetalist),
                            N(_thetalist.size()),
                            Mlist(_Mlist),
                            Glist(_Glist),
                            Slist(_Slist),
                            g(_g), Kp(kp), Ki(ki), Kd(kd) {
  }
  
  Eigen::VectorXd get_pos() const {
    return thetalist;
  }
  
  Eigen::VectorXd get_vel() const {
    return dthetalist;
  }
  
  Eigen::VectorXd get_accl() const {
    return ddthetalist;
  }
  
  //dummy initialization
  /*
  Kinetics(int n, const std::vector<Eigen::MatrixXd> &_Mlist,
           const std::vector<Eigen::MatrixXd> &_Glist,
           const Eigen::MatrixXd &_Slist,
           const Eigen::VectorXd& _thetalist=Eigen::VectorXd::Zero(n),
           const Eigen::VectorXd& _dthetalist=Eigen::VectorXd::Zero(n),
           const Eigen::VectorXd& _ddthetalist=Eigen::VectorXd::Zero(n),
           const Eigen::VectorXd &_g=Eigen::VectorXd(0,0,-9.81),
           const double kp=0.1, const double ki=0.1,
           const double kd=0.1) {
    
  }
  */
  //
  /**
   * Uses forward-backward Newton-Euler iterations to solve the equation:
   *   taulist = Mlist(thetalist) * ddthetalist + 
   *           c(thetalist, dthetalist)  + 
   *           g(thetalist) + Jtr(thetalist) * Ftip
   * 
   * @param Ftip Spatial force applied by the end-effector 
   *        expressed in frame {n+1}
   * @return taulist The n-vector of required joint forces/torques
   */
  Eigen::VectorXd
  InverseDynamics(const Eigen::VectorXd& Ftip);

  /**
   * Computes ddthetalist by solving:
   * Mlist(thetalist) * ddthetalist = taulist - 
   *                        c(thetalist,dthetalist)
   *                       - g(thetalist) - Jtr(thetalist) * Ftip
   *
   * @param taulist An n-vector of joint forces/torques required at each joint.
   * @param Ftip Spatial force applied by the end-effector 
   *        expressed in frame {n+1}
   * @return ddthetalist The resulting joint accelerations.
   */
  Eigen::VectorXd
  ForwardDynamics(const Eigen::VectorXd &taulist, const Eigen::VectorXd &Ftip);


  Eigen::VectorXd
  ComputedTorque(const Eigen::VectorXd& thetalistd,
                 const Eigen::VectorXd& dthetalistd,
                 const Eigen::VectorXd& ddthetalistd,
                 const Eigen::VectorXd& eint);

  /*
    on the naming, why tilde?!
    const Eigen::VectorXd& gtilde,
    const std::vector<Eigen::MatrixXd>& Mtildelist,
    const std::vector<Eigen::MatrixXd>& Gtildelist,
  */
  /*
    std::vector<Eigen::MatrixXd>
    SimulateControl(const Eigen::MatrixXd& thetamatd,
                  const Eigen::MatrixXd& dthetamatd,
                  const Eigen::MatrixXd& ddthetamatd,
                  const Eigen::MatrixXd& Ftipmat,
                  const Eigen::VectorXd& gtilde,
                  const std::vector<Eigen::MatrixXd>& Mtildelist,
                  const std::vector<Eigen::MatrixXd>& Gtildelist,
                  double dt, int intRes);
  */
protected:

  /** calculate the first-order Euler iteration
   *
   * @param dt = $\frac{total time}{number of iterations}$
   */
  inline void EulerStep(double dt) {
    thetalist += dthetalist * dt;
    dthetalist += ddthetalist * dt;
    return;
  }
  
  /** calculate the Forces due to the effect of gravity for the current joints configurations thetalist.
   *
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
  inline Eigen::VectorXd GravityForces() const {
    int n = N;
    Eigen::VectorXd dummylist = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd dummyForce = Eigen::VectorXd::Zero(6);
    auto gforce = Kinetics(thetalist, dummylist, dummylist, Mlist, Glist, Slist);
    Eigen::VectorXd grav = gforce.InverseDynamics(dummyForce);
    return grav;
  }
  
  /** Calculate the current Mass Matrix for the current joint configuration thetalist
   *
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
  inline Eigen::MatrixXd MassMatrix() const {
    int n = N;
    Eigen::VectorXd dummylist = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd dummyg = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd dummyforce = Eigen::VectorXd::Zero(6);
    Eigen::MatrixXd M = Eigen::MatrixXd::Zero(n,n);
    for (int i = 0; i < n; i++) {
      Eigen::VectorXd ddthetalist = Eigen::VectorXd::Zero(n);
      ddthetalist(i) = 1;
      /*
      M.col(i) = InverseDynamics(thetalist, dummylist, ddthetalist,
                                 dummyg, dummyforce, Mlist, Glist,
                                 Slist);
      */
      //
      auto iforce = Kinetics(thetalist, dummylist, ddthetalist, Mlist, Glist, Slist, dummyg);
      M.col(i) = iforce.InverseDynamics(dummyforce);
    }
    return M;
  }
  /** Calculate the coriolis, and velocity forces for the current joint configurations.
   *
   * Calls InverseDynamics with g = 0, 
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
  inline Eigen::VectorXd VelQuadraticForces() const {
    int n = N;
    Eigen::VectorXd dummylist = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd dummyg = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd dummyforce = Eigen::VectorXd::Zero(6);
    /*
      Eigen::VectorXd c = InverseDynamics(thetalist, dthetalist,
                                        dummylist, dummyg,
                                        dummyforce, Mlist,
                                        Glist, Slist);
    */
    auto cforce = Kinetics(thetalist, dthetalist, dummylist, Mlist, Glist, Slist, dummyg);
    Eigen::VectorXd c= cforce.InverseDynamics(dummyforce);
    return c;
  }
  
  /** Calculate End-Effector combined Forces due to applied end effector force, as well as the forces generated due to gravity, inertial, coriolis, and velocities.
   * Calls InverseDynamics with g = 0, 
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
  EndEffectorForces(const Eigen::VectorXd& Ftip) const {
    int n = N;
    Eigen::VectorXd dummylist = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd dummyg = Eigen::VectorXd::Zero(3);
    /*
      Eigen::VectorXd JTFtip = InverseDynamics(thetalist, dummylist,
                                             dummylist, dummyg,
                                             Ftip, Mlist, Glist,
                                             Slist);
    */
    auto eforce = Kinetics(thetalist, dummylist, dummylist, Mlist, Glist, Slist, dummyg);
    Eigen::VectorXd JTFtip= eforce.InverseDynamics(Ftip);
    return JTFtip;
  }

  const std::vector<Eigen::MatrixXd> Mlist;
  const std::vector<Eigen::MatrixXd> Glist;
  const Eigen::MatrixXd Slist;
  const Eigen::VectorXd g;
  // pid controller constants
  const double Kp, Ki, Kd;
  // number of joints
  const int N;
};
