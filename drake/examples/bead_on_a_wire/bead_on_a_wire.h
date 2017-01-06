#pragma once

#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace bead_on_a_wire {

/// Dynamical system of a point mass constrained to lie on a wire. The system
/// is currently frictionless. The equation for the wire can be provided
/// parametrically *by the user*. Equations for the dynamics are provided
/// by R. Rosales, "Bead Moving Along a Thin, Rigid Wire". Available from:
/// https://ocw.mit.edu/courses/mathematics/18-385j-nonlinear-dynamics-and-chaos-fall-2004/lecture-notes/bead_on_wire.pdf 
///
/// The presence of readily available solutions coupled with the potential
/// for highly irregular geometric constraints (which can be viewed 
/// as complex contact constraints), make this a powerful example.
///  
/// This class uses Drake's `-inl.h` pattern.  When seeing linker errors from
/// this class, please refer to http://drake.mit.edu/cxx_inl.html.
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
/// Instantiated templates for the following scalar types @p T are provided:
/// - double
///
/// They are already available to link against in drakeBeadOnAWire.
///
/// Inputs: no inputs.
/// States: 3D position (state indices 0,1, and 2), and linear velocity (state
///         indices 3, 4, and 5) in units of m and m/s, respectively.
/// Outputs: same as state.
template <typename T>
class BeadOnAWire : public systems::LeafSystem<T> {
 public:
  BeadOnAWire();

  void DoCalcOutput(const systems::Context<T>& context,
                  systems::SystemOutput<T>* output) const override;

  void DoCalcTimeDerivatives(
      const systems::Context<T>& context,
      systems::ContinuousState<T>* derivatives) const override;

  /// Sets the acceleration (with respect to the positive y-axis) due to
  /// gravity (i.e., this number should generally be negative).
  void set_gravitational_acceleration(double g) { g_ = g; }

  /// Gets the acceleration (with respect to the positive y-axis) due to
  /// gravity (i.e., this number should generally be negative).
  double get_gravitational_acceleration() const { return g_; }

 protected:
  void SetDefaultState(const systems::Context<T>& context,
                       systems::State<T>* state) const override;

 private:
  double g_{-9.81};         // The acceleration due to gravity.
};

}  // namespace bead_on_a_wire
}  // namespace drake
