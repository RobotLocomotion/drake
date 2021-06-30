#pragma once

#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace van_der_pol {

/// van der Pol oscillator
///
/// The van der Pol oscillator, governed by the following equations:
///   q̈ + μ(q² - 1)q̇ + q = 0, μ > 0
///   y₀ = q
///   y₁ = [q,q̇]'
/// is a canonical example of a nonlinear system that exhibits a
/// limit cycle stability.  As such it serves as an important for
/// examining nonlinear stability and stochastic stability.
///
/// (Examples involving region of attraction analysis and analyzing
/// the stationary distribution of the oscillator under process
/// noise are coming soon).
///
/// @tparam_default_scalar
template <typename T>
class VanDerPolOscillator final : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(VanDerPolOscillator)

  /// Constructs a default oscillator.
  VanDerPolOscillator();

  /// Scalar-converting copy constructor.
  template <typename U>
  explicit VanDerPolOscillator(const VanDerPolOscillator<U>&)
      : VanDerPolOscillator() {}

  /// Returns the output port containing the output configuration (only).
  const systems::OutputPort<T>& get_position_output_port() const {
    return this->get_output_port(0);
  }

  /// Returns the output port containing the full state.  This is
  /// provided primarily as a tool for debugging/visualization.
  const systems::OutputPort<T>& get_full_state_output_port() const {
    return this->get_output_port(1);
  }

  // TODO(russt): generalize this to any mu using trajectory optimization
  //  (this could also improve the numerical accuracy).
  /// Returns a 2-row matrix containing the result of simulating the oscillator
  /// with the default mu=1 from (approximately) one point on the limit cycle
  /// for (approximately) one period.  The first row is q, and the second row
  /// is q̇.
  static Eigen::Matrix2Xd CalcLimitCycle();

 private:
  void DoCalcTimeDerivatives(
      const systems::Context<T>& context,
      systems::ContinuousState<T>* derivatives) const override;

  void CopyPositionToOutput(const systems::Context<T>& context,
                            systems::BasicVector<T>* output) const;
};

}  // namespace van_der_pol
}  // namespace examples
}  // namespace drake
