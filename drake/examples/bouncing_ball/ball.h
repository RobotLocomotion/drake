#pragma once

#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace bouncing_ball {

/// Dynamical system representation of the one-dimensional equations of
/// motion for a ball in flight, dropped with an initial position (height)
/// and velocity.
///
/// This class uses Drake's `-inl.h` pattern.  When seeing linker errors from
/// this class, please refer to http://drake.mit.edu/cxx_inl.html.
///
/// Instantiated templates for the following scalar types @p T are provided:
/// - double
/// - AutoDiffXd
///
/// To use other specific scalar types see ball-inl.h.
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
///
/// They are already available to link against in drakeBouncingBall.
///
/// Inputs: no inputs.
/// States: vertical position (state index 0) and velocity (state index 1) in
/// units of m and m/s, respectively.
/// Outputs: vertical position (state index 0) and velocity (state index 1) in
/// units of m and m/s, respectively.
template <typename T>
class Ball : public systems::LeafSystem<T> {
 public:
  // Constructor for the Ball system.
  Ball();

  /// Gets the signed acceleration due to gravity. Since initial positions
  /// correspond to heights, acceleration should be negative.
  double get_gravitational_acceleration() const { return -9.81; }

 protected:
  void DoCalcOutput(const systems::Context<T>& context,
                    systems::SystemOutput<T>* output) const override;

  void DoCalcTimeDerivatives(
      const systems::Context<T>& context,
      systems::ContinuousState<T>* derivatives) const override;

  void SetDefaultState(const systems::Context<T>& context,
                       systems::State<T>* state) const override;
};

}  // namespace bouncing_ball
}  // namespace drake
