#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace particles {

/// A linear 1DOF particle system.
///
/// With very simple dynamics @f$ \ddot x = a @f$, this system can be
/// described in terms of its:
///
/// - Inputs:
///   - linear acceleration (input index 0), in @f$ m/s^2 @f$ units.
/// - States/Outputs:
///   - linear position (state/output index 0), in @f$ m @f$ units.
///   - linear velocity (state/output index 1), in @f$ m/s @f$ units.
///
/// @tparam_double_only
template <typename T>
class Particle final : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Particle)

  Particle();

 protected:
  void DoCalcTimeDerivatives(const systems::Context<T>& context,
      systems::ContinuousState<T>* derivatives) const override;
};

}  // namespace particles
}  // namespace examples
}  // namespace drake
