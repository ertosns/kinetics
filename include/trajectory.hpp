#include <Eigen/Dense>
#include <vector>

class Trajectory {
public:
  double Tf;
  int N;
  int method;
  double dt;
  Trajectory(double Tf, int N, int m=5) : Tf(Tf), N(N), method(m){
    dt = Tf / (N - 1);
  }
  
  Eigen::MatrixXd
  JointTrajectory(const Eigen::VectorXd& thetastart,
                  const Eigen::VectorXd& thetaend);

  std::vector<Eigen::MatrixXd>
  ScrewTrajectory(const Eigen::MatrixXd& Xstart,
                  const Eigen::MatrixXd& Xend);
  
  std::vector<Eigen::MatrixXd>
  CartesianTrajectory(const Eigen::MatrixXd& Xstart,
                      const Eigen::MatrixXd& Xend);

private:
  inline double CubicTimeScaling(double t) const {
    double timeratio = t / Tf;
    double st = 3 * pow(timeratio, 2) - 2 * pow(timeratio, 3);
    return st;
  }
  
  inline double QuinticTimeScaling(double t) const {
    double timeratio = t / Tf;
    double st = 10 * pow(timeratio, 3) - 15 * pow(timeratio, 4)
      + 6 * pow(timeratio, 5);
    return st;
  }
};
