#include "algebra.hpp"
#include "kinetics.hpp"

class Trajectory {
public:
  Kinetics kinetics;
  Trajectory() {
  }
  Eigen::MatrixXd
  JointTrajectory(const Eigen::VectorXd& thetastart,
                  const Eigen::VectorXd& thetaend,
                  double Tf, int N, int method);
  
  std::vector<Eigen::MatrixXd>
  ScrewTrajectory(const Eigen::MatrixXd& Xstart,
                  const Eigen::MatrixXd& Xend,
                  double Tf, int N, int method);
  
  std::vector<Eigen::MatrixXd>
  CartesianTrajectory(const Eigen::MatrixXd& Xstart,
                      const Eigen::MatrixXd& Xend,
                      double Tf, int N, int method);
  
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
  
private:
  double CubicTimeScaling(double Tf, double t) {
    double timeratio = 1.0*t / Tf;
    double st = 3 * pow(timeratio, 2) - 2 * pow(timeratio, 3);
    return st;
  }
  
  double QuinticTimeScaling(double Tf, double t) {
    double timeratio = 1.0*t / Tf;
    double st = 10 * pow(timeratio, 3) - 15 * pow(timeratio, 4)
      + 6 * pow(timeratio, 5);
    return st;
  }
};
