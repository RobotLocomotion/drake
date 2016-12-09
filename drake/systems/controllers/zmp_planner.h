#pragma once

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/common/trajectories/exponential_plus_piecewise_polynomial.h"
//#include "drake/lcmt_zmp_data.hpp"

namespace drake {
namespace systems {

/**
 *
 * See the following reference for more details about the algorithm.
 * R. Tedrake, S. Kuindersma, R. Deits and K. Miura, "A closed-form solution
 * for real-time ZMP gait generation and feedback stabilization,"
 * 2015 IEEE-RAS 15th International Conference on Humanoid Robots (Humanoids),
 * Seoul, 2015, pp. 936-940.
 */
class ZMPPlanner {
 public:
  ZMPPlanner() {}

  void Plan(const PiecewisePolynomial<double> &zmp_d, const Eigen::Vector4d &x0, double height);
  //drake::lcmt_zmp_data EncodeZMPData(double time) const;

  void WriteToFile(const std::string &name, double dt) const;

  inline Eigen::Vector2d GetDesiredZMP(double time) const {
    return zmp_traj_.value(time);
  }

  inline Eigen::Vector2d GetDesiredZMPd(double time) const {
    return zmpd_traj_.value(time);
  }

  inline Eigen::Vector2d GetNominalCOM(double time) const {
    return com_traj_.value(time);
  }

  inline Eigen::Vector2d GetNominalCOMd(double time) const {
    return comd_traj_.value(time);
  }

 private:
  PiecewisePolynomial<double> zmp_traj_;
  PiecewisePolynomial<double> zmpd_traj_;
  ExponentialPlusPiecewisePolynomial<double> com_traj_;
  ExponentialPlusPiecewisePolynomial<double> comd_traj_;
  // this is essentially the feedforward/nominal control input coming from lqr solution
  ExponentialPlusPiecewisePolynomial<double> comdd_traj_;
  ExponentialPlusPiecewisePolynomial<double> s1_traj_;

  Eigen::Matrix<double, 4, 4> A_;
  Eigen::Matrix<double, 4, 2> B_;
  Eigen::Matrix<double, 2, 4> C_;
  Eigen::Matrix<double, 2, 2> D_;
  Eigen::Matrix<double, 2, 2> Qy_, R_;
  Eigen::Matrix<double, 4, 4> S_;
  Eigen::Matrix<double, 2, 4> K_;

  Eigen::Vector4d s1_dot_;
  Eigen::Vector2d u0_;
};

}  // namespace systems
}  // namespace drake
