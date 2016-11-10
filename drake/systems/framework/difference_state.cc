#include "drake/systems/framework/difference_state.h"

#include "drake/common/eigen_autodiff_types.h"
#include "drake/math/autodiff_overloads.h"

namespace drake {
namespace systems {

template class DifferenceState<double>;
template class DifferenceState<AutoDiffXd>;

}  // namespace systems
}  // namespace drake
