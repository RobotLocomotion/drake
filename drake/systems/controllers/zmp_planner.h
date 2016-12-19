#pragma once

#include "drake/common/trajectories/exponential_plus_piecewise_polynomial.h"
#include "drake/common/trajectories/piecewise_polynomial.h"

namespace drake {
namespace systems {

/**
 * Given a desired two dimensional (X and Y) zero-moment point (ZMP) trajectory
 * parametrized as a piecewise polynomial, an optimal center of mass (CoM)
 * trajectory is planned using a linear inverted pendulum model (LIPM).
 * A second order value function (cost-to-go) and a linear policy are also
 * computed along the optimal trajectory.
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
   * It is allowed to pass in a `zmp_d` with a non-stationary end point, but
   * the user should treat the result with caution, since the resulting nominal
   * CoM trajectory diverges exponentially fast past the end point.
   * @param zmp_d, Desired two dimensional ZMP trajectory.
   * @param x0, Initial CoM state.
   * @param height, CoM height from the ground.
   * @param Qy, Quadratic cost term on ZMP deviation from the desired.
   * @param R, Quadratic cost term on CoM acceleration.
   */
  void Plan(const PiecewisePolynomial<double>& zmp_d, const Eigen::Vector4d& x0,
            double height, double gravity = 9.81,
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
   * Converts CoM acceleration to center of pressure (CoP) using
   *    cop = C * x + D * u or
   *    cop = com - z / g * comdd
   * @param x, CoM position and velocity
   * @param u, CoM acceleration
   * @return center of pressure (CoP)
   */
  inline Eigen::Vector2d comdd_to_cop(const Eigen::Vector4d& x,
                                      const Eigen::Vector2d& u) const {
    return C_ * x + D_ * u;
  }

  /**
   * Getter for A matrix.
   */
  inline const Eigen::Matrix<double, 4, 4>& get_A() const { return A_; }

  /**
   * Getter for B matrix.
   */
  inline const Eigen::Matrix<double, 4, 2>& get_B() const { return B_; }

  /**
   * Getter for C matrix.
   */
  inline const Eigen::Matrix<double, 2, 4>& get_C() const { return C_; }

  /**
   * Getter for D matrix.
   */
  inline const Eigen::Matrix<double, 2, 2>& get_D() const { return D_; }

  /**
   * Getter for Qy matrix.
   */
  inline const Eigen::Matrix<double, 2, 2>& get_Qy() const { return Qy_; }

  /**
   * Getter for R matrix.
   */
  inline const Eigen::Matrix<double, 2, 2>& get_R() const { return R_; }

  /**
   * Returns the desired ZMP evaluated at `time`.
   */
  inline Eigen::Vector2d get_desired_zmp(double time) const {
    return zmp_d_.value(time);
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

  inline Eigen::Vector2d get_final_desired_zmp() const {
    return zmp_d_.value(zmp_d_.getEndTime());
  }

  /**
   * Returns the desired ZMP trajectory.
   */
  inline const PiecewisePolynomial<double>& get_desired_zmp() const {
    return zmp_d_;
  }

  /**
   * Returns the nominal CoM trajectory.
   */
  inline const ExponentialPlusPiecewisePolynomial<double>& get_nominal_com()
      const {
    return com_;
  }

  /**
   * Returns the nominal CoM velocity trajectory.
   */
  inline const ExponentialPlusPiecewisePolynomial<double>& get_nominal_comd()
      const {
    return comd_;
  }

  /**
   * Returns the nominal CoM acceleration trajectory.
   */
  inline const ExponentialPlusPiecewisePolynomial<double>& get_nominal_comdd()
      const {
    return comdd_;
  }

  /**
   * Returns the time invariant second order term (S1 in [1]) of the value
   * function (cost-to-go).
   */
  inline const Eigen::Matrix<double, 4, 4>& get_Vxx() const { return S1_; }

  /**
   * Returns the time varying first order term (s2 in [1]) of the value
   * function (cost-to-go).
   */
  inline const ExponentialPlusPiecewisePolynomial<double>& get_Vx() const {
    return s2_;
  }

  /**
   * Returns the time varying first order term (s2 in [1]) of the value
   * function (cost-to-go).
   */
  inline const Eigen::Vector4d get_Vx(double time) const {
    return s2_.value(time);
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  // Check if the last point of zmp_d is stationary (first and higher
  // derivatives are zero).
  bool CheckStationaryEndPoint(const PiecewisePolynomial<double>& zmp_d) const;

  // Symbols:
  // x: [com; comd]
  // y: zmp
  // y_tf: last zmp_d
  // u: comdd
  // x_bar = [x - y_tf, xd]
  // y_bar = y - y_tf

  // Desired ZMP trajectories.
  PiecewisePolynomial<double> zmp_d_;

  // Nominal CoM, CoMd, and CoMdd trajectories.
  ExponentialPlusPiecewisePolynomial<double> com_;
  ExponentialPlusPiecewisePolynomial<double> comd_;
  ExponentialPlusPiecewisePolynomial<double> comdd_;

  // System dynamics matrices.
  Eigen::Matrix<double, 4, 4> A_;
  Eigen::Matrix<double, 4, 2> B_;
  Eigen::Matrix<double, 2, 4> C_;
  Eigen::Matrix<double, 2, 2> D_;

  Eigen::Matrix<double, 2, 4> NB_;
  Eigen::Matrix<double, 2, 2> R1i_;
  Eigen::Matrix<double, 4, 4> A2_;
  Eigen::Matrix<double, 4, 2> B2_;

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
