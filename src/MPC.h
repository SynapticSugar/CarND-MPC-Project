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

  // This value assumes the model presented in the classroom is used.
  //
  // It was obtained by measuring the radius formed by running the vehicle in
  // the
  // simulator around in a circle with a constant steering angle and velocity on
  // a
  // flat terrain.
  //
  // Lf was tuned until the the radius formed by the simulating the model
  // presented in the classroom matched the previous radius.
  //
  // This is the length from front to CoG that has a similar radius.
  static constexpr double Lf = 2.67;
};

#endif /* MPC_H */
