#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

constexpr double pi() { return M_PI; }
constexpr double MPC_LF = 2.67;
constexpr double MPC_LATENCY = 0.1;
constexpr int MPC_MAX_STEER = 25;
constexpr int MPC_POLY_ORDER = 3;

class MPC {
 public:
  MPC();

  virtual ~MPC();
  
  double deg2rad( const double x );

  double rad2deg( const double x );

  void Polyfit( const Eigen::VectorXd xvec, 
                const Eigen::VectorXd yvec, 
                const int order, 
                Eigen::VectorXd& result ); 

  double Polyeval( const Eigen::VectorXd coeffs, double x );

  vector<double> Solve( Eigen::VectorXd state, Eigen::VectorXd coeffs );

  vector<double> OptimizeMpc( const double px, 
                              const double py, 
                              const double psi, 
                              const double v, 
                              const double delta, 
                              const double a, 
                              const vector<double>& ptsx, 
                              const vector<double>& ptsy,
                              Eigen::VectorXd& ptsx_car,
                              Eigen::VectorXd& ptsy_car);

};

#endif /* MPC_H */
