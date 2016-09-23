#pragma once

#include <memory>

#include "drake/examples/bouncing_ball/ball-inl.h"

namespace drake {
namespace bouncingball {

/// Dynamical representation of the idealized hybrid dynamics
/// of a ball dropping from a height and bouncing on a surface.
///
/// This class uses Drake's `-inl.h` pattern.  When seeing linker errors from
/// this class, please refer to http://drake.mit.edu/cxx_inl.html.
///
/// Instantiated templates for the following scalar types @p T are provided:
/// - double
/// - AutoDiffXd
///
/// To use other specific scalar types see bouncing_ball-inl.h.
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
///
/// They are already available to link against in drakeBouncingBall.
///
/// Inputs: no inputs.
/// States: vertical position and velocity, respectively, in units of m and m/s.
/// Outputs: vertical position and velocity, respectivelt, in units of m and
/// m/s.
template <typename T>
class BouncingBall : public Ball<T> {
 public:
  /// Constructor for the BouncingBall system.
  BouncingBall();

  /// TODO(jadecastro): This is a prototype implementation to be overridden from
  /// the system API, pending further discussions.
  ///
  /// Evaluate the guard function  associated with the system at a particular
  /// mode. If the EvalGuard returns a non-positive value, then the hybrid
  /// system is allowed to make a transition from the `pre` mode to `post` mode.
  T EvalGuard(const systems::Context<T>& context) const;

  /// TODO(jadecastro): This is a prototype implementation to be overridden from
  /// the system API, pending further discussions.
  ///
  /// Performs a reset mapping that occurs once a discrete mode if and only if a
  /// mode transition (discrete jump) has been made. It does so by mutating the
  /// context, so that, by default the reset mapping is the identity mapping.
  void PerformReset(systems::Context<T>* context) const;

  /// Getter for the model coefficient.
  double GetRestitutionCoef() const { return restitution_coef_; }

 private:
  const double restitution_coef_ = 0.8;  // coefficient of restitution
};

}  // namespace bouncingball
}  // namespace drake
