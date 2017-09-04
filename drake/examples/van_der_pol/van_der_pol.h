#pragma once

#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace van_der_pol {

/// van der Pol oscillator
///   q̈ + μ(q² - 1)q̇ + q = 0, μ > 0
///   y₁ = q
///   y₂ = [q,q̇]'
template <typename T>
class VanDerPolOscillator : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(VanDerPolOscillator)

  VanDerPolOscillator();

  /// Scalar-converting copy constructor.
  template <typename U>
  explicit VanDerPolOscillator(const VanDerPolOscillator<U>&)
      : VanDerPolOscillator() {}

 private:
  void DoCalcTimeDerivatives(
      const systems::Context<T>& context,
      systems::ContinuousState<T>* derivatives) const override;

  void CopyPositionToOutput(const systems::Context<T>& context,
                            systems::BasicVector<T>* output) const;

  void CopyFullStateToOutput(const systems::Context<T>& context,
                             systems::BasicVector<T>* output) const;
};

}  // namespace van_der_pol
}  // namespace examples
}  // namespace drake
