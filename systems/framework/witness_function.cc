#include "drake/systems/framework/witness_function.h"

#include "drake/common/default_scalars.h"
#include "drake/systems/framework/system.h"

namespace drake {
namespace systems {

template <class T>
T WitnessFunction<T>::Evaluate(const Context<T>& context) const {
  DRAKE_ASSERT_VOID(system_.CheckValidContext(context));
  return DoEvaluate(context);
}

// The Vector2/3 instantiations here are for the benefit of some
// older unit tests but are not otherwise advertised.
template class WitnessFunction<Eigen::AutoDiffScalar<Eigen::Vector2d>>;
template class WitnessFunction<Eigen::AutoDiffScalar<Eigen::Vector3d>>;

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::WitnessFunction)
