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
