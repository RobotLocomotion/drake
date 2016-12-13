#pragma once

#include "drake/common/trajectories/exponential_plus_piecewise_polynomial.h"
#include "drake/common/trajectories/piecewise_polynomial.h"

namespace drake {
namespace systems {

/**
 * Given a desired two dimensional (X and Y) zero-moment point (ZMP) trajectory
 * parametrized as a piecewise polynomial, an optimal center of mass (CoM)
 * trajectory is planned using a linear inverted pendulum model (LIPM).
 * A second order value function and a linear policy are also computed along
 * the optimal trajectory.
 * The state of the system is CoM position and velocity, and the control is
 * CoM acceleration.
 * See the following reference for more details about the algorithm.
 * [1] R. Tedrake, S. Kuindersma, R. Deits and K. Miura, "A closed-form solution
 * for real-time ZMP gait generation and feedback stabilization,"
 * 2015 IEEE-RAS 15th International Conference on Humanoid Robots (Humanoids),
 * Seoul, 2015, pp. 936-940.
 */
class ZMPPlanner {
 public:
  ZMPPlanner() {}

  /**
   * Implements the algorithm in [1] that computes a nominal CoM trajectory,
   * and the corresponding second order value function and linear policy.
   * @param zmp_d, Desired two dimensional ZMP trajectory.
   * @param x0, Initial CoM state.
   * @param height, CoM height from the ground.
   * @param Qy, Quadratic cost term on ZMP deviation from the desired.
   * @param R, Quadratic cost term on CoM acceleration.
   */
  void Plan(const PiecewisePolynomial<double>& zmp_d, const Eigen::Vector4d& x0,
            const double height,
            const Eigen::Matrix2d& Qy = Eigen::Matrix2d::Identity(),
            const Eigen::Matrix2d& R = Eigen::Matrix2d::Zero());

  /**
   * Computes the optimal control (CoM acceleration) at `time` given CoM state
   * `x` using the linear policy.
   * @param time, Current time.
   * @param x, Current state.
   * @return Optimal CoMdd.
   */
  Eigen::Vector2d ComputeOptimalCoMdd(double time,
                                      const Eigen::Vector4d& x) const;

  /**
   * Returns the desired ZMP evaluated at `time`.
   */
  inline Eigen::Vector2d get_desired_zmp(double time) const {
    return zmp_d_.value(time);
  }

  /**
   * Returns the desired ZMP velocity evaluated at `time`.
   */
  inline Eigen::Vector2d get_desired_zmpd(double time) const {
    return zmpd_d_.value(time);
  }

  /**
   * Returns the nominal CoM evaluated at `time`.
   */
  inline Eigen::Vector2d get_nominal_com(double time) const {
    return com_.value(time);
  }

  /**
   * Returns the nominal CoM velocity evaluated at `time`.
   */
  inline Eigen::Vector2d get_nominal_comd(double time) const {
    return comd_.value(time);
  }

  /**
   * Returns the nominal CoM acceleration evaluated at `time`.
   */
  inline Eigen::Vector2d get_nominal_comdd(double time) const {
    return comdd_.value(time);
  }

  /**
   * Returns the desired ZMP trajectory.
   */
  inline const PiecewisePolynomial<double> get_desired_zmp() const {
    return zmp_d_;
  }

  /**
   * Returns the desired ZMP velocity trajectory.
   */
  inline const PiecewisePolynomial<double> get_desired_zmpd() const {
    return zmpd_d_;
  }

  /**
   * Returns the nominal CoM trajectory.
   */
  inline const ExponentialPlusPiecewisePolynomial<double> get_nominal_com()
      const {
    return com_;
  }

  /**
   * Returns the nominal CoM velocity trajectory.
   */
  inline const ExponentialPlusPiecewisePolynomial<double> get_nominal_comd()
      const {
    return comd_;
  }

  /**
   * Returns the nominal CoM acceleration trajectory.
   */
  inline const ExponentialPlusPiecewisePolynomial<double> get_nominal_comdd()
      const {
    return comdd_;
  }

  /**
   * Returns the time invariant second order term of the value function.
   */
  inline const Eigen::Matrix<double, 4, 4>
  get_value_function_second_derivative() const {
    return S1_;
  }

  /**
   * Returns the time varying first order term of the value function.
   */
  inline const ExponentialPlusPiecewisePolynomial<double>
  get_value_function_first_derivative() const {
    return s2_;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  // Symbols:
  // x: [com; comd]
  // y: zmp
  // y_tf: last zmp_d
  // u: comdd
  // x_bar = [x - y_tf, xd]
  // y_bar = y - y_tf

  // Desired ZMP and ZMPd trajectories.
  PiecewisePolynomial<double> zmp_d_;
  PiecewisePolynomial<double> zmpd_d_;

  // Nominal CoM, CoMd, and CoMdd trajectories.
  ExponentialPlusPiecewisePolynomial<double> com_;
  ExponentialPlusPiecewisePolynomial<double> comd_;
  ExponentialPlusPiecewisePolynomial<double> comdd_;

  // System dynamics matrices.
  Eigen::Matrix<double, 4, 4> A_;
  Eigen::Matrix<double, 4, 2> B_;
  Eigen::Matrix<double, 2, 4> C_;
  Eigen::Matrix<double, 2, 2> D_;

  // One step cost function:
  // L = (y - y_d)^T * Qy * (y - y_d)^T + u^T * R * u.
  Eigen::Matrix<double, 2, 2> Qy_, R_;

  // Value function
  // V = x_bar^T * S1 * x_bar + x_bar^T * s2 + constant_term.
  Eigen::Matrix<double, 4, 4> S1_;
  ExponentialPlusPiecewisePolynomial<double> s2_;

  // Linear policy.
  // u = K * x_bar + k2
  Eigen::Matrix<double, 2, 4> K_;
  ExponentialPlusPiecewisePolynomial<double> k2_;
};

}  // namespace systems
}  // namespace drake
