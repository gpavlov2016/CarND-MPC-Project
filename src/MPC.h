#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>

using namespace std;
using CppAD::AD;

typedef CPPAD_TESTVECTOR(double) Dvector;

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);

  void GetTrajectory(vector<double>& mpc_x_vals, vector<double>& mpc_y_vals);
  
  // place to return solution
  CppAD::ipopt::solve_result<Dvector> m_solution;

};

#endif /* MPC_H */
