#pragma once

#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {
namespace analysis_test {

/// System with no state evolution for testing numerical differencing in
/// integrators that use it.
class StationarySystem final : public LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(StationarySystem)

  StationarySystem() : LeafSystem<double>() {
    this->DeclareContinuousState(1 /* num q */, 1 /* num v */, 0 /* num z */);
  }

 protected:
  void DoCalcTimeDerivatives(
      const Context<double>& context,
      ContinuousState<double>* derivatives) const override {
    // State does not evolve.
    derivatives->get_mutable_vector().SetAtIndex(0, 0.0);
    derivatives->get_mutable_vector().SetAtIndex(1, 0.0);
  }
};

}  // namespace analysis_test
}  // namespace systems
}  // namespace drake
