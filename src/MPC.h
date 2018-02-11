#ifndef MPC_H
#define MPC_H

#include "Eigen-3.3/Eigen/Core"
#include <vector>

using namespace std;

class MPC {
public:
  MPC();

  MPC(double des_speed);

  MPC(double des_speed, std::array<double, 7> des_gains);

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuations.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
