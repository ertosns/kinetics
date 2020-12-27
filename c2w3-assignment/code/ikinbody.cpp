
/** Inverse Kinematics in body frame, given the mechanism configurations, IKinSpace derives the joints angles.
   *
   * @param thetalist initial angles configuration
   * @param T Transfomation matrix representing the end-effector frame when the joints are at the specified coordinates
   * @return thetaList A list of joint coordinates.
   */
  inline Eigen::VectorXd
  IKinBody(const Eigen::MatrixXd& T, Eigen::VectorXd thetalist) {
    int i = 0;
    //add this to the auto-generated configuration file
    Eigen::MatrixXd Tfk = FKinBody(thetalist);
    Eigen::MatrixXd Tdiff = Algebra::TransInv(Tfk)*T;
    Eigen::VectorXd Vb = Algebra::se3ToVec(Algebra::MatrixLog6(Tdiff));
    Eigen::Vector3d angular(Vb(0), Vb(1), Vb(2));
    Eigen::Vector3d linear(Vb(3), Vb(4), Vb(5));
    
    bool err = (angular.norm() > eomg || linear.norm() > ev);
    Eigen::MatrixXd Jb;
    double omg_norm, v_norm;
    while (err && i < maxiterations) {
      Jb = JacobianBody(thetalist);
      thetalist += Jb.bdcSvd(Eigen::ComputeThinU |
                             Eigen::ComputeThinV).solve(Vb);
      i += 1;
      // iterate
      Tfk = FKinBody(thetalist);
      Tdiff = Algebra::TransInv(Tfk)*T;
      Vb = Algebra::se3ToVec(Algebra::MatrixLog6(Tdiff));
      angular = Eigen::Vector3d(Vb(0), Vb(1), Vb(2));
      linear = Eigen::Vector3d(Vb(3), Vb(4), Vb(5));
      omg_norm=angular.norm();
      v_norm=linear.norm();
      err = (omg_norm > eomg || v_norm > ev);
      if (log) {
        write("iteration(space)", i);
        write("transformation(space)", Tfk);
        write("twist(space)", Vb);
        write("angular norm(space)", angular.norm());
        write("linear norm(space)", linear.norm());
        write("thetalist(space)", thetalist);
      }
    }
    // err boolean True if the iteration is ended without reaching the allowable limits determined by eomg, and ev.
    //assert(!err);
    return thetalist;
  }
