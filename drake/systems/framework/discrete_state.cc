#include "drake/systems/framework/discrete_state.h"

#include "drake/common/autodiff_overloads.h"
#include "drake/common/eigen_autodiff_types.h"

namespace drake {
namespace systems {

template class DiscreteState<double>;
template class DiscreteState<AutoDiffXd>;

}  // namespace systems
}  // namespace drake
