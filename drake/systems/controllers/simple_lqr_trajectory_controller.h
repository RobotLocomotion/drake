#pragma once

#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/systems/controllers/LQR.h"
#include "drake/systems/trajectories/PiecewisePolynomial.h"
#include "drake/systems/vector.h"
#include "drake/util/drakeUtil.h"

namespace drake {

/**
 * A time varying LQR controller to play back an input+state
 * trajectory.
 *
 * @concept{system_concept}
 */
template <typename System>
class SimpleLqrTrajectoryController {
 public:
  template <typename ScalarType>
  using InputVector = typename System::template StateVector<ScalarType>;
  template <typename ScalarType>
  using StateVector = drake::NullVector<ScalarType>;
  template <typename ScalarType>
  using OutputVector = typename System::template InputVector<ScalarType>;
  typedef PiecewisePolynomial<double> PiecewisePolynomialType;

  SimpleLqrTrajectoryController(std::shared_ptr<System> sys,
                                const PiecewisePolynomialType& utraj,
                                const PiecewisePolynomialType& xtraj,
                                const Eigen::MatrixXd& Q,
                                const Eigen::MatrixXd& R)
      : sys_(sys), utraj_(utraj), xtraj_(xtraj), Q_(Q), R_(R) {}

  template <typename ScalarType>
  StateVector<ScalarType> dynamics(const ScalarType& t,
                                   const StateVector<ScalarType>& x,
                                   const InputVector<ScalarType>& u) const {
    return StateVector<ScalarType>();
  }

  template <typename ScalarType>
  OutputVector<ScalarType> output(const ScalarType& t,
                                  const StateVector<ScalarType>& x,
                                  const InputVector<ScalarType>& u) const {
    const auto u_t = utraj_.value(t);
    const auto x_t = xtraj_.value(t);
    Eigen::MatrixXd K;
    // S is deliberately unused.
    Eigen::MatrixXd S;
    CalculateLqrMatrices(*sys_, x_t, u_t, Q_, R_, &K, &S);

    // The formula below is a simplified version of the AffineSystem
    // returned by MakeTimeInvariantLqrSystem().
    OutputVector<ScalarType> y = -K * toEigen(u) + u_t + K * x_t;
    return y;
  }

  bool isTimeVarying() const { return true; }
  bool isDirectFeedthrough() const { return false; }

 private:
  std::shared_ptr<System> sys_;
  const PiecewisePolynomialType& utraj_;
  const PiecewisePolynomialType& xtraj_;
  const Eigen::MatrixXd& Q_;
  const Eigen::MatrixXd& R_;
};

}  // namespace drake
