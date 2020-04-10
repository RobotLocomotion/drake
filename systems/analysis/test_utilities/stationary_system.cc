#include "drake/systems/analysis/test_utilities/stationary_system.h"

#include "drake/systems/framework/context.h"
#include "drake/systems/framework/continuous_state.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/system_type_tag.h"

namespace drake {
namespace systems {
namespace analysis_test {

template <class T>
StationarySystem<T>::StationarySystem()
    : LeafSystem<T>(SystemTypeTag<StationarySystem>{}) {
  this->DeclareContinuousState(1 /* num q */, 1 /* num v */, 0 /* num z */);
}

template <class T>
void StationarySystem<T>::DoCalcTimeDerivatives(
    const Context<T>&, ContinuousState<T>* derivatives) const {
  // State does not evolve.
  derivatives->get_mutable_vector().SetAtIndex(0, T(0.0));
  derivatives->get_mutable_vector().SetAtIndex(1, T(0.0));
}

}  // namespace analysis_test
}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::systems::analysis_test::StationarySystem)
