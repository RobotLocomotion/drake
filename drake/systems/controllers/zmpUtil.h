#include <Eigen/Core>
#include "drake/systems/trajectories/ExponentialPlusPiecewisePolynomial.h"
#include "drake/drakeZMPUtil_export.h"

struct DRAKEZMPUTIL_EXPORT TVLQRData {
  // TODO: move into its own file
  // TODO: turn into class, private members
  Eigen::MatrixXd A;
  Eigen::MatrixXd B;
  Eigen::MatrixXd C;
  Eigen::MatrixXd D;
  Eigen::MatrixXd Qy;
  Eigen::MatrixXd R;
  Eigen::VectorXd u0;
  Eigen::MatrixXd Q1;
  Eigen::MatrixXd R1;
  Eigen::MatrixXd N;
};

DRAKEZMPUTIL_EXPORT ExponentialPlusPiecewisePolynomial<double> s1Trajectory(const TVLQRData &sys, const PiecewisePolynomial<double> &zmp_trajectory,const Eigen::Ref<const Eigen::MatrixXd> &S);
