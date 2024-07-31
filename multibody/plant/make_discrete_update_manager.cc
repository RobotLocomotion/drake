#include "drake/multibody/plant/make_discrete_update_manager.h"

#include "drake/multibody/plant/compliant_contact_manager.h"

namespace drake {
namespace multibody {
namespace internal {

template <typename T>
std::unique_ptr<DiscreteUpdateManager<T>> MakeDiscreteUpdateManager(
    DiscreteContactSolver) {
  return std::make_unique<CompliantContactManager<T>>();
}

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    (&MakeDiscreteUpdateManager<T>));

}  // namespace internal
}  // namespace multibody
}  // namespace drake
