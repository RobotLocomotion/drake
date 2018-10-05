#pragma once

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/trajectories/exponential_plus_piecewise_polynomial.h"
#include "drake/common/trajectories/piecewise_polynomial.h"

namespace drake {
namespace systems {
namespace controllers {

/**
 * Given a desired two dimensional (X and Y) zero-moment point (ZMP) trajectory
 * parameterized as a piecewise polynomial, an optimal center of mass (CoM)
 * trajectory is planned using a linear inverted pendulum model (LIPM).
 * A second order value function (optimal cost-to-go) and a linear policy are
 * also computed along the optimal trajectory.
 * The system dynamics for the X and Y directions are decoupled, however, we
 * plan the XY motion together for convenience.
 *
 * Let \f$ c \f$ be the CoM position, the The state of the system, \f$ x \f$,
 * is \f$ [c; \dot{c}] \f$, the control, \f$ u = \ddot{c} \f$,
 * and \f$ y \f$ represents the center of pressure (CoP).
 * For the X direction, the LIPM dynamics is:
 * \f[
 *   y = c - \frac{z}{g} * u,
 * \f]
 * where \f$ g \f$ is the gravity constant and \f$ z \f$ is the CoM height.
 * \f$ z \f$ is assumed to be constant in LIPM.
 * The full dynamics can also be written in the matrix form as:
 * \f[
 *   \dot{x} = A x + B u \\
 *         y = C x + D u
 * \f]
 *
 * The one step cost function \f$ L \f$ is defined as:
 * \f[
 *   L(y, u, t) = (y - y_d(t))^T Q_y (y - y_d(t)) + u^T R u,
 * \f]
 * where \f$ Q_y \f$ and \f$ R \f$ are weighting matrices, and \f$ y_d(t) \f$
 * is the desired ZMP trajectory at time \f$ t \f$.
 *
 * The value function is defined as
 * \f[
 *   V(x, t) = \min_{u[t:t_f]} \bar{x}(t_f)^T S \bar{x}(t_f)
 *           + \int_{t}^{t_f} L(y, u, \tau) d\tau,
 * \f]
 * subject to the dynamics, and \f$ t_f \f$ is the last time in the desired
 * ZMP trajectory, \f$ \bar{x} = [c - y_d(t_f); \dot{c}] \f$,
 * \f$ S \f$ is the quadratic term from the infinite horizon continuous time
 * LQR solution solved with the same dynamics and one step cost function.
 *
 * For this problem, \f$ V \f$ is known to have a quadratic form of:
 * \f[
 *   V(x, t) = \bar{x}^T V_{xx} \bar{x} + \bar{x}^T V_x(t) + V_0(t),
 * \f]
 * and the corresponding optimal control policy, \f$ u^* \f$, is linear
 * w.r.t. to \f$ x \f$:
 * \f[
 *   u^*(x, t) = K \bar{x} + u_0(t).
 * \f]
 *
 * See the following reference for more details about the algorithm:
 *
 * [1] R. Tedrake, S. Kuindersma, R. Deits and K. Miura, "A closed-form solution
 * for real-time ZMP gait generation and feedback stabilization,"
 * 2015 IEEE-RAS 15th International Conference on Humanoid Robots (Humanoids),
 * Seoul, 2015, pp. 936-940.
 */
class ZMPPlanner {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ZMPPlanner)

  ZMPPlanner() {}

  /**
   * Implements the algorithm in [1] that computes a nominal CoM trajectory,
   * and the corresponding second order value function and linear policy.
   *
   * None of the other public methods should be called until Plan is called.
   *
   * It is allowed to pass in a `zmp_d` with a non-stationary end point, but
   * the user should treat the result with caution, since the resulting nominal
   * CoM trajectory diverges exponentially fast past the end point.
   * @param zmp_d, Desired two dimensional ZMP trajectory.
   * @param x0, Initial CoM state.
   * @param height, CoM height from the ground.
   * @param gravity, Gravity constant, defaults to 9.81
   * @param Qy, Quadratic cost term on ZMP deviation from the desired, defaults
   * to identity.
   * @param R, Quadratic cost term on CoM acceleration, defaults to zero.
   */
  void Plan(const trajectories::PiecewisePolynomial<double>& zmp_d,
            const Eigen::Vector4d& x0, double height, double gravity = 9.81,
            const Eigen::Matrix2d& Qy = Eigen::Matrix2d::Identity(),
            const Eigen::Matrix2d& R = Eigen::Matrix2d::Zero());

  /**
   * Returns true if Plan has been called.
   */
  bool has_planned() const { return planned_; }

  /**
   * Computes the optimal control (CoM acceleration) at `time` given CoM state
   * `x` using the linear policy.
   * Should only be called after Plan is called.
   * @param time, Current time.
   * @param x, Current state.
   * @return Optimal CoMdd.
   */
  Eigen::Vector2d ComputeOptimalCoMdd(double time,
                                      const Eigen::Vector4d& x) const;

  /**
   * Converts CoM acceleration to center of pressure (CoP) using
   *    cop = C * x + D * u, which is equivalent to
   *    cop = com - z / g * comdd
   * Should only be called after Plan is called.
   * @param x, CoM position and velocity
   * @param u, CoM acceleration
   * @return center of pressure (CoP)
   */
  Eigen::Vector2d comdd_to_cop(const Eigen::Vector4d& x,
                               const Eigen::Vector2d& u) const {
    DRAKE_DEMAND(planned_);
    return C_ * x + D_ * u;
  }

  /**
   * Getter for A matrix.
   */
  const Eigen::Matrix<double, 4, 4>& get_A() const {
    DRAKE_DEMAND(planned_);
    return A_;
  }

  /**
   * Getter for B matrix.
   */
  const Eigen::Matrix<double, 4, 2>& get_B() const {
    DRAKE_DEMAND(planned_);
    return B_;
  }

  /**
   * Getter for C matrix.
   */
  const Eigen::Matrix<double, 2, 4>& get_C() const {
    DRAKE_DEMAND(planned_);
    return C_;
  }

  /**
   * Getter for D matrix.
   */
  const Eigen::Matrix<double, 2, 2>& get_D() const {
    DRAKE_DEMAND(planned_);
    return D_;
  }

  /**
   * Getter for Qy matrix.
   */
  const Eigen::Matrix<double, 2, 2>& get_Qy() const {
    DRAKE_DEMAND(planned_);
    return Qy_;
  }

  /**
   * Getter for R matrix.
   */
  const Eigen::Matrix<double, 2, 2>& get_R() const {
    DRAKE_DEMAND(planned_);
    return R_;
  }

  /**
   * Returns the desired ZMP evaluated at `time`.
   */
  Eigen::Vector2d get_desired_zmp(double time) const {
    DRAKE_DEMAND(planned_);
    return zmp_d_.value(time);
  }

  /**
   * Returns the nominal CoM evaluated at `time`.
   */
  Eigen::Vector2d get_nominal_com(double time) const {
    DRAKE_DEMAND(planned_);
    return com_.value(time);
  }

  /**
   * Returns the nominal CoM velocity evaluated at `time`.
   */
  Eigen::Vector2d get_nominal_comd(double time) const {
    DRAKE_DEMAND(planned_);
    return comd_.value(time);
  }

  /**
   * Returns the nominal CoM acceleration evaluated at `time`.
   */
  Eigen::Vector2d get_nominal_comdd(double time) const {
    DRAKE_DEMAND(planned_);
    return comdd_.value(time);
  }

  Eigen::Vector2d get_final_desired_zmp() const {
    DRAKE_DEMAND(planned_);
    return zmp_d_.value(zmp_d_.end_time());
  }

  /**
   * Returns the desired ZMP trajectory.
   */
  const trajectories::PiecewisePolynomial<double>& get_desired_zmp() const {
    DRAKE_DEMAND(planned_);
    return zmp_d_;
  }

  /**
   * Returns the nominal CoM trajectory.
   */
  const trajectories::ExponentialPlusPiecewisePolynomial<double>&
  get_nominal_com() const {
    DRAKE_DEMAND(planned_);
    return com_;
  }

  /**
   * Returns the nominal CoM velocity trajectory.
   */
  const trajectories::ExponentialPlusPiecewisePolynomial<double>&
  get_nominal_comd() const {
    DRAKE_DEMAND(planned_);
    return comd_;
  }

  /**
   * Returns the nominal CoM acceleration trajectory.
   */
  const trajectories::ExponentialPlusPiecewisePolynomial<double>&
  get_nominal_comdd() const {
    DRAKE_DEMAND(planned_);
    return comdd_;
  }

  /**
   * Returns the time invariant second order term (S1 in [1]) of the value
   * function.
   */
  const Eigen::Matrix<double, 4, 4>& get_Vxx() const {
    DRAKE_DEMAND(planned_);
    return S1_;
  }

  /**
   * Returns the time varying first order term (s2 in [1]) of the value
   * function.
   */
  const trajectories::ExponentialPlusPiecewisePolynomial<double>& get_Vx()
      const {
    DRAKE_DEMAND(planned_);
    return s2_;
  }

  /**
   * Returns the time varying first order term (s2 in [1]) of the value
   * function.
   */
  const Eigen::Vector4d get_Vx(double time) const {
    DRAKE_DEMAND(planned_);
    return s2_.value(time);
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  // Check if the last point of zmp_d is stationary (first and higher
  // derivatives are zero).
  bool CheckStationaryEndPoint(
      const trajectories::PiecewisePolynomial<double>& zmp_d) const;

  // Used to test whether the last point of the desired ZMP trajectory is
  // stationary or not in CheckStationaryEndPoint. This number is currently
  // arbitrarily chosen.
  static constexpr double kStationaryThreshold = 1e-8;

  // Symbols:
  // x: [com; comd]
  // y: zmp
  // y_tf: last zmp_d
  // u: comdd
  // x_bar = [x - y_tf, xd]
  // y_bar = y - y_tf

  // Desired ZMP trajectories.
  trajectories::PiecewisePolynomial<double> zmp_d_;

  // Nominal CoM, CoMd, and CoMdd trajectories.
  trajectories::ExponentialPlusPiecewisePolynomial<double> com_;
  trajectories::ExponentialPlusPiecewisePolynomial<double> comd_;
  trajectories::ExponentialPlusPiecewisePolynomial<double> comdd_;

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
  trajectories::ExponentialPlusPiecewisePolynomial<double> s2_;

  // Linear policy.
  // u = K * x_bar + k2
  Eigen::Matrix<double, 2, 4> K_;
  trajectories::ExponentialPlusPiecewisePolynomial<double> k2_;

  bool planned_{false};
};

}  // namespace controllers
}  // namespace systems
}  // namespace drake
