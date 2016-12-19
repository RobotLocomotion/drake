#include "drake/systems/framework/continuous_state.h"

#include "drake/common/autodiff_overloads.h"
#include "drake/common/eigen_autodiff_types.h"

namespace drake {
namespace systems {

template class ContinuousState<double>;
template class ContinuousState<AutoDiffXd>;

}  // namespace systems
}  // namespace drake
