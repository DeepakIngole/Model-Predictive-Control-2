#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

using CppAD::AD;

// Set the timestep length and duration
size_t N = 10;
double dt = 0.1;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.

double ref_cte = 0;
double ref_epsi = 0;
double ref_v = 70;

// All the state and actuator variable are in one big vector
// We need to define start index in the vector for each segment.
size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;

size_t i;

class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

  void operator()(ADvector& fg, const ADvector& vars) {
    fg[0] = 0;

    // The part of the cost based on the reference state.
    for( i = 0; i < N; i++ ) {
      fg[0] += 10*CppAD::pow(vars[cte_start + i] - ref_cte, 2);
      fg[0] += 10*CppAD::pow(vars[epsi_start + i] - ref_epsi, 2);
      fg[0] += CppAD::pow(vars[v_start + i] - ref_v, 2);
    }

    // Minimize the use of actuators.
    for( i = 0; i< N - 1; i++ ) {
      fg[0] += 1000*CppAD::pow(vars[delta_start + i], 2);
      fg[0] += 10*CppAD::pow(vars[a_start + i], 2);
    }

    // Minimize the value gap between sequential actuations.
    for( i = 0; i < N - 2; i++ ) {
      fg[0] += 10000*CppAD::pow(vars[delta_start + i + 1] - vars[delta_start + i], 2);
      //fg[0] += max_int*CppAD::pow(vars[delta_start + i + 1] - vars[delta_start + i], 2);
      fg[0] += 1*CppAD::pow(vars[a_start + i + 1] - vars[a_start + i], 2);
    }

    // Initial constraints.
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    // The rest of the constraints 
    for( i = 1; i < N; i++ ) {
      // The state at time t+1 .
      AD<double> x1 = vars[x_start + i];
      AD<double> y1 = vars[y_start + i];
      AD<double> psi1 = vars[psi_start + i];
      AD<double> v1 = vars[v_start + i];
      AD<double> cte1 = vars[cte_start + i];
      AD<double> epsi1 = vars[epsi_start + i];

      // The state at time t.
      AD<double> x0 = vars[x_start + i - 1];
      AD<double> y0 = vars[y_start + i - 1];
      AD<double> psi0 = vars[psi_start + i - 1];
      AD<double> v0 = vars[v_start + i - 1];
      AD<double> cte0 = vars[cte_start + i - 1];
      AD<double> epsi0 = vars[epsi_start + i - 1];

      // Only consider the actuation at time t.
      AD<double> delta0 = vars[delta_start + i - 1];
      AD<double> a0 = vars[a_start + i - 1];

      AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * CppAD::pow(x0, 2) + coeffs[3] * CppAD::pow(x0, 3);
      AD<double> psides0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * CppAD::pow(x0, 2));

      fg[1 + x_start + i] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + i] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psi_start + i] = psi1 - (psi0 - v0 / MPC_LF * delta0 * dt);
      fg[1 + v_start + i] = v1 - (v0 + a0 * dt);
      fg[1 + cte_start + i] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[1 + epsi_start + i] = epsi1 - ((psi0 - psides0) - v0 / MPC_LF * delta0 * dt);
    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

//=============================================================================
//  @brief: MPC::deg2rad()
//
//  @param  x:  double x (angle in degrees)
//
//  @return angle in radians 
//=============================================================================
double MPC::deg2rad( const double x ) { return x * pi() / 180; }

//=============================================================================
//  @brief: MPC::rad2deg()
//
//  @param  x:  double x (angle in radians)
//
//  @return angle in degrees 
//=============================================================================
double MPC::rad2deg( const double x ) { return x * 180 / pi(); }

//=============================================================================
//  @brief  Polyfit()
//          Fit a polynomial given x and y vectors.
//
//  @param  x:      Eigen vector for x coord 
//  @param  y:      Eigen vector for y coord 
//  @param  order:  Order of polynomial  
//  @param  result: Reference to Eigen vector for fitted result 
//
//  @return void
//=============================================================================
void MPC::Polyfit( const Eigen::VectorXd x, 
                   const Eigen::VectorXd y, 
                   const int order, 
                   Eigen::VectorXd& result ) 
{
  assert( x.size() == y.size());
  assert( order >= 1 && order <= x.size() - 1);

  Eigen::MatrixXd A( x.size(), order + 1 );
  int i,j;

  for( i = 0; i < x.size(); i++ ) {
    A(i, 0) = 1.0;
  }

  for( j = 0; j < x.size(); j++ ) {
    for( i = 0; i < order; i++ ) {
      A( j, i + 1 ) = A( j,i ) * x( j );
    }
  }

  auto Q = A.householderQr();

  result = Q.solve( y );
}

//=============================================================================
//  @brief  Polyeval()
//          Evaluate a polynomial.
//
//  @param  coeffs: polyfit coefficients vector
//  @param  x:      base value used for std::pow(base, exp)
//
//  @return result: evaluated result (cte) 
//=============================================================================
double MPC::Polyeval( const Eigen::VectorXd coeffs, double x ) 
{

  double result = 0.0;
  
  for( int i = 0; i < coeffs.size(); i++ ) {
    result += coeffs[i] * pow(x, i);
  }

  return result;
}

//=============================================================================
//  @brief  MPC::Solve()
//          Solve the model given an initial state and polynomial coefficients.
//          Return the first actuations.
//
//  @param  state:  Eigen vector for state
//  @param  coeffs: Eigen vector for coefficents
//
//  @return result: vector to hold solve result
//=============================================================================
vector<double> MPC::Solve( const Eigen::VectorXd state, const Eigen::VectorXd coeffs ) 
{

  bool ok = true;
  size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;
  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];
  double epsi = state[5];
  
  //  Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  //
  // 4 * 10 + 2 * 9
  size_t n_vars =  N * 6 + (N - 1) * 2;;
  //  Set the number of constraints
  size_t n_constraints = N * 6;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for( i = 0; i < n_vars; i++ ) {
    vars[i] = 0;
  }

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  //  Set lower and upper limits for variables.

  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  for( i = 0; i < delta_start; i++ ) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians).

  double angle_radians = deg2rad( MPC_MAX_STEER );
  for( i = delta_start; i < a_start; i++ ) {
    vars_lowerbound[i] = -angle_radians;
    vars_upperbound[i] = angle_radians;
  }

  // Acceleration upper and lower bounds.
  for( i = a_start; i < n_vars; i++ ) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for( i = 0; i < n_constraints; i++ ) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  constraints_lowerbound[x_start] = x;
  constraints_lowerbound[y_start] = y;
  constraints_lowerbound[psi_start] = psi;
  constraints_lowerbound[v_start] = v;
  constraints_lowerbound[cte_start] = cte;
  constraints_lowerbound[epsi_start] = epsi;

  constraints_upperbound[x_start] = x;
  constraints_upperbound[y_start] = y;
  constraints_upperbound[psi_start] = psi;
  constraints_upperbound[v_start] = v;
  constraints_upperbound[cte_start] = cte;
  constraints_upperbound[epsi_start] = epsi;

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.1\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  //auto cost = solution.obj_value;
  //std::cout << "Cost " << cost << std::endl;

  // Return the first actuator values. 
  
  vector<double> result;
  double steering_angle = solution.x[delta_start];
  double throttle = solution.x[a_start];

  result.push_back( steering_angle );
  result.push_back( throttle );

  for( i = 0; i < N - 2; i++ ) {
    result.push_back( solution.x[x_start + i + 1] );
    result.push_back( solution.x[y_start + i + 1] );
  }
  return result;
}

//=============================================================================
//  @brief: OptimizeMpc()
//          An MPC class wrapper function:
//          1. Convert Map coordinates to vehicle coordinates
//          2. Run polyfit to fit a polynomial given x and y vectors.
//          3. Run polyeval to evaluate the polynomial and compute the cte 
//          4. Run solve to get predicted vars[] 
//
//  @param  px:       const double x 
//  @param  py:       const double y 
//  @param  psi:      const double psi 
//  @param  v:        const double psi 
//  @param  ptsx:     const reference to ptsx map coords 
//  @param  ptsy:     const reference to ptsy map coords 
//  @param  ptsx_car: reference to ptsx car coords 
//  @param  ptsy_car: reference to ptsy car coords 
//
//  @return void
//=============================================================================
vector<double> MPC::OptimizeMpc( const double px, 
                                 const double py, 
                                 const double psi, 
                                 const double v, 
                                 const double delta, 
                                 const double a, 
                                 const vector<double>& ptsx, 
                                 const vector<double>& ptsy, 
                                 Eigen::VectorXd& ptsx_car, 
                                 Eigen::VectorXd& ptsy_car )
{
  Eigen::VectorXd coeffs;
  Eigen::VectorXd state( 6 );
  vector<double> vars; 
  
  // Convert Map to Vehicle coords
  for( size_t i = 0; i < ptsx.size(); i++ ) {
    auto dx = ( ptsx[i] - px );
    auto dy = ( ptsy[i] - py );

    ptsx_car[i] = ( ( dx * cos( -psi ) ) - ( dy * sin( -psi ) ) );
    ptsy_car[i] = ( ( dx * sin( -psi ) ) + ( dy * cos( -psi ) ) );
  }

  Polyfit( ptsx_car, ptsy_car, MPC_POLY_ORDER, coeffs );

  // Initial State vector (no latency).
  const double x0 = 0;
  const double y0 = 0;
  const double psi0 = 0;
  const double v0 = v;
  const double cte0 = coeffs[0];
  const double epsi0 = -atan( coeffs[1] );

  // State vector (with 100ms latency).
  auto pxd = x0 + ( v0 * cos( psi0 ) * MPC_LATENCY );
  auto pyd = y0 + ( v0 * sin( psi0 ) * MPC_LATENCY );
  auto psid = psi0 - ( v0 * delta * ( MPC_LATENCY/MPC_LF ) );
  auto vd = v0 + ( a * MPC_LATENCY );
  auto cted = cte0 + ( v0 * sin( epsi0 ) * MPC_LATENCY );
  auto epsid = epsi0 - ( v0 * atan( coeffs[1] ) * ( MPC_LATENCY/MPC_LF ) );

  state << pxd, pyd, psid, vd, cted, epsid;
  vars = Solve( state, coeffs );

  return vars;
}
