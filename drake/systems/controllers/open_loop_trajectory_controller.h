#pragma once

#include "drake/systems/trajectories/PiecewisePolynomial.h"
#include "drake/systems/vector.h"

namespace drake {

/**
 * An open-loop controller to play back an input trajectory.
 *
 * @concept{system_concept}
 */
template <typename System>
class OpenLoopTrajectoryController {
 public:
  template <typename ScalarType>
  using InputVector = drake::NullVector<ScalarType>;
  template <typename ScalarType>
  using StateVector = drake::NullVector<ScalarType>;
  template <typename ScalarType>
  using OutputVector = typename System::template InputVector<ScalarType>;
  typedef PiecewisePolynomial<double> PiecewisePolynomialType;

  /**
   * The @p pp_traj argument is aliased, and must be valid for the
   * life of this object.
   */
  explicit OpenLoopTrajectoryController(const PiecewisePolynomialType& pp_traj)
      : pp_traj_(pp_traj) {}

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
    OutputVector<ScalarType> y = pp_traj_.value(t);
    return y;
  }

  bool isTimeVarying() const { return true; }
  bool isDirectFeedthrough() const { return false; }

 private:
  const PiecewisePolynomialType& pp_traj_;
};

}  // namespace drake
