#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC {
 public:
  MPC();

  virtual ~MPC();
  
  double deg2rad( const double x );

  double rad2deg( const double x );

  void GetVehicleCoords( const double px, 
                         const double py, 
                         const double psi, 
                         vector<double> &xvec, 
                         vector<double> &yvec ); 

  void Polyfit( const Eigen::VectorXd xvec, 
                const Eigen::VectorXd yvec, 
                const int order, 
                Eigen::VectorXd& result ); 

  double Polyeval( const Eigen::VectorXd coeffs, 
                   double x );

  vector<double> Solve( Eigen::VectorXd state, 
                        Eigen::VectorXd coeffs );

  void GetMpcOutputs( const double px, 
                      const double py, 
                      const double psi, 
                      const double v, 
                      vector<double>& ptsx, 
                      vector<double>& ptsy,
                      double& steer_value, 
                      double& throttle_value ) ;

  vector<double> x_vals;
  vector<double> y_vals;
};

#endif /* MPC_H */
