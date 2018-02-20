#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"
#define W_CTE 1
#define W_EPSI 1
#define W_V 1
#define W_DELTA 700
#define W_A 10
#define W_DDELTA 900
#define W_DA 1

using namespace std;

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
	vector<double> mpc_x_vals;
  vector<double> mpc_y_vals;
  double steer_value_;
  double throttle_value_;
};

#endif /* MPC_H */
