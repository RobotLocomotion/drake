#pragma once

#include <memory>
#include <vector>

#include "drake/examples/bouncing_ball/ball.h"
#include "drake/examples/bouncing_ball/signed_distance_witness.h"

namespace drake {
namespace bouncing_ball {

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
/// They are already available to link against in the containing library.
///
/// Inputs: no inputs.
/// States: vertical position (state index 0) and velocity (state index 1) in
/// units of m and m/s, respectively.
/// Outputs: vertical position (state index 0) and velocity (state index 1) in
/// units of m and m/s, respectively.
template <typename T>
class BouncingBall : public Ball<T> {
 public:
  /// Constructor for the BouncingBall system.
  BouncingBall();

  void DoCalcUnrestrictedUpdate(const systems::Context<T>& context,
      const std::vector<const systems::UnrestrictedUpdateEvent<T>*>& events,
      systems::State<T>* state) const override;

  void DoGetWitnessFunctions(
      const systems::Context<T>& context,
      std::vector<const systems::WitnessFunction<T>*>* witnesses)
      const override;

  /// Getter for the coefficient of restitution for this model.
  double get_restitution_coef() const { return restitution_coef_; }

 private:
  const double restitution_coef_ = 1.0;  // Coefficient of restitution.

  std::unique_ptr<SignedDistanceWitnessFunction<T>> signed_distance_witness_;

  // Numerically intolerant signum function.
  int sgn(T x) const {
    return (T(0) < x) - (x < T(0));
  }
};

}  // namespace bouncing_ball
}  // namespace drake
