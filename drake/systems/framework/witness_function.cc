#include "drake/systems/framework/witness_function.h"

#include "drake/common/eigen_autodiff_types.h"
#include "drake/systems/framework/system.h"

namespace drake {
namespace systems {

template class WitnessFunction<double>;
template class WitnessFunction<AutoDiffXd>;

}  // namespace systems
}  // namespace drake
