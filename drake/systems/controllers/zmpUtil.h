#include <Eigen/Core>
#include "drake/ExponentialPlusPiecewisePolynomial.h"

#undef DLLEXPORT
#if defined(WIN32) || defined(WIN64)
  #if defined(drakeZMPUtil_EXPORTS)
    #define DLLEXPORT __declspec( dllexport )
  #else
    #define DLLEXPORT __declspec( dllimport )
  #endif
#else
    #define DLLEXPORT
#endif

struct DLLEXPORT TVLQRData {
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

DLLEXPORT ExponentialPlusPiecewisePolynomial<double> s1Trajectory(const TVLQRData &sys, const PiecewisePolynomial<double> &zmp_trajectory,const Eigen::Ref<const Eigen::MatrixXd> &S);