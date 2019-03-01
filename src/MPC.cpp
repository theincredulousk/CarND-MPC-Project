#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;
using Eigen::VectorXd;

// Samples to consider
// Notes: larger numbers seem to produce control oscillation, lower numbers cause complete failure to control vehicle.
//size_t N = 6;
size_t N = 6;

// 100ms, value seems to work best when near actuator delay?
double dt = 0.1;

size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;


// Lf based on steering geometry of simulator car
const double Lf = 2.67;

// Target average speed
double ref_v = 100;


class FG_eval {
 public:
  // Fitted polynomial coefficients
  VectorXd coeffs;
  FG_eval(VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) 
  {

    fg[0] = 0;

    // Minimize cross track and steering angle error
    // Penalize "future" cross track error over immediate cross track error (smooths trajectory)
    // Penalize immediate steering angle error over "future" angle error (keeps steering responsive)
    for (unsigned int t = 0; t < N; t++) 
    {
      fg[0] += ((t+1) / 0.2) * 15 * CppAD::pow(vars[cte_start + t], 2);
      fg[0] += (60.0 / (t+1)) * 40 * CppAD::pow(vars[epsi_start + t], 2);

      //fg[0] += 150 * CppAD::pow(vars[cte_start + t], 2);
      //fg[0] += 400 * CppAD::pow(vars[epsi_start + t], 2);

      fg[0] += CppAD::pow(vars[v_start + t] - ref_v, 2);
    }

    // Minimize the value gap between sequential actuations.
    // Penalize "future" gaps over immediate gaps (smooths trajectory)
    for (unsigned int t = 0; t < N - 2; t++) 
    {
      //fg[0] += 4000 * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += (300.0 / (t+1)) * 400 * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    } 

    //
    // Setup Constraints
    //
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    // The rest of the constraints
    for (unsigned int t = 1; t < N; ++t) 
    {
       // The state at time t+1 .
      AD<double> x1 = vars[x_start + t];
      AD<double> y1 = vars[y_start + t];
      AD<double> psi1 = vars[psi_start + t];
      AD<double> v1 = vars[v_start + t];
      AD<double> cte1 = vars[cte_start + t];
      AD<double> epsi1 = vars[epsi_start + t];

      // The state at time t.
      AD<double> x0 = vars[x_start + t - 1];
      AD<double> y0 = vars[y_start + t - 1];
      AD<double> psi0 = vars[psi_start + t - 1];
      AD<double> v0 = vars[v_start + t - 1];
      AD<double> cte0 = vars[cte_start + t - 1];
      AD<double> epsi0 = vars[epsi_start + t - 1];

      AD<double> delta0 = vars[delta_start + t - 1];
      AD<double> a0 = vars[a_start + t - 1];

      // Modified from lesson exmaple for 3rd degree polynomial
      AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * CppAD::pow(x0,2) + coeffs[3] * CppAD::pow(x0,3);
      AD<double> psides0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * CppAD::pow(x0,2));
      

      // x_[t] = x[t-1] + v[t-1] * cos(psi[t-1]) * dt
      fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);

      // y_[t] = y[t-1] + v[t-1] * sin(psi[t-1]) * dt
      fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);

      // psi_[t] = psi[t-1] + v[t-1] / Lf * delta[t-1] * dt
      fg[1 + psi_start + t] = psi1 - (psi0 + v0 * delta0 / Lf * dt);

      // v_[t] = v[t-1] + a[t-1] * dt
      fg[1 + v_start + t] = v1 - (v0 + a0 * dt);

      // cte[t] = f(x[t-1]) - y[t-1] + v[t-1] * sin(epsi[t-1]) * dt
      fg[1 + cte_start + t] =
          cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));

      // epsi[t] = psi[t] - psides[t-1] + v[t-1] * delta[t-1] / Lf * dt
      fg[1 + epsi_start + t] =
          epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);    
    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

std::vector<double> MPC::Solve(const VectorXd &state, const VectorXd &coeffs, std::vector<std::tuple<double, double>>& predicted_path) {
  bool ok = true;
  typedef CPPAD_TESTVECTOR(double) Dvector;

 double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];
  double epsi = state[5];

  // number of independent variables
  // N timesteps == N - 1 actuations
  size_t n_vars = N * 6 + (N - 1) * 2;
  // Number of constraints
  size_t n_constraints = N * 6;

  // Initial value of the independent variables.
  // Should be 0 except for the initial values.
  Dvector vars(n_vars);
  for (unsigned int i = 0; i < n_vars; ++i) {
    vars[i] = 0.0;
  }
  // Set the initial variable values
  vars[x_start] = x;
  vars[y_start] = y;
  vars[psi_start] = psi;
  vars[v_start] = v;
  vars[cte_start] = cte;
  vars[epsi_start] = epsi;

  // Lower and upper limits for x
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);

  // Generic Limits
  for (unsigned int i = 0; i < delta_start; ++i) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // Steering Limits
  for (unsigned int i = delta_start; i < a_start; ++i) {
    // Artifically limit steeting range available.  
    // This forces the model to pick smoother trajectories.
    // Also helps prevent oscillation, "jerky" steering, and overcorrection in sharp corners.
    //vars_lowerbound[i] = -0.15;//1236;
    //vars_upperbound[i] = 0.15; //1236;
     vars_lowerbound[i] = -0.25;//1236;
    vars_upperbound[i] = 0.25; //1236;
  }

  // Acceleration Limits
  for (unsigned int i = a_start; i < n_vars; ++i) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }

  // Lower and upper limits for constraints
  // All of these should be 0 except the initial
  // state indices.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (unsigned int i = 0; i < n_constraints; ++i) {
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

  // Object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  // options
  std::string options;
  options += "Integer print_level  0\n";
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  //
  // Check some of the solution values
  //
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  auto cost = solution.obj_value;
  std::cout << "COST: " << cost << std::endl;
  std::vector<double> return_value{solution.x[x_start + 1],   solution.x[y_start + 1],
          solution.x[psi_start + 1], solution.x[v_start + 1],
          solution.x[cte_start + 1], solution.x[epsi_start + 1],
          solution.x[delta_start],   solution.x[a_start]};

  // Add predicted path for visual output
  for(unsigned int i = 1; i < N; i++)
  {
      predicted_path.push_back(std::tuple<double,double>(solution.x[i + x_start], solution.x[i + y_start]));
  }

  return return_value;
}