#include "drake/systems/framework/witness_function.h"

#include "drake/common/eigen_autodiff_types.h"
#include "drake/systems/framework/system.h"

namespace drake {
namespace systems {

template <class T>
T WitnessFunction<T>::Evaluate(const Context<T>& context) const {
  DRAKE_ASSERT_VOID(system_.CheckValidContext(context));
  return DoEvaluate(context);
}

template class WitnessFunction<double>;
template class WitnessFunction<AutoDiffXd>;
template class WitnessFunction<Eigen::AutoDiffScalar<Eigen::Vector2d>>;
template class WitnessFunction<Eigen::AutoDiffScalar<Eigen::Vector3d>>;
template class WitnessFunction<symbolic::Expression>;

}  // namespace systems
}  // namespace drake
