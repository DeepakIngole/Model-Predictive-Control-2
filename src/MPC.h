#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC {
 public:
  MPC();

  virtual ~MPC();
  
  void GetVehicleCoords( double px, 
                         double py, 
                         double psi, 
                         vector<double> &xvec, 
                         vector<double> &yvec ); 

  void Polyfit( Eigen::VectorXd xvec, 
                Eigen::VectorXd yvec, 
                int order, 
                Eigen::VectorXd& result ); 

  double Polyeval( Eigen::VectorXd coeffs, double x );

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

  vector<double> x_data;
  vector<double> y_data;
};

#endif /* MPC_H */
