#pragma once

#include "drake/common/default_scalars.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {
namespace analysis_test {

/// System with no state evolution for testing numerical differencing in
/// integrators that use it.
/// @tparam_default_scalar
template <typename T>
class StationarySystem final : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(StationarySystem)

  StationarySystem() : LeafSystem<T>(SystemTypeTag<StationarySystem>{}) {
    this->DeclareContinuousState(1 /* num q */, 1 /* num v */, 0 /* num z */);
  }

  /// Scalar-converting copy constructor.  See @ref system_scalar_conversion.
  template <typename U>
  explicit StationarySystem(const StationarySystem<U>&) : StationarySystem() {}

 protected:
  void DoCalcTimeDerivatives(const Context<T>&,
                             ContinuousState<T>* derivatives) const override {
    // State does not evolve.
    derivatives->get_mutable_vector().SetAtIndex(0, T(0.0));
    derivatives->get_mutable_vector().SetAtIndex(1, T(0.0));
  }
};

}  // namespace analysis_test
}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::systems::analysis_test::StationarySystem)
