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

  void EvalOutput(const systems::Context<T>& context,
                  systems::SystemOutput<T>* output) const override;

  void EvalTimeDerivatives(
      const systems::Context<T>& context,
      systems::ContinuousState<T>* derivatives) const override;

 protected:
  std::unique_ptr<systems::ContinuousState<T>> AllocateContinuousState()
    const override;
};

}  // namespace bouncing_ball
}  // namespace drake
