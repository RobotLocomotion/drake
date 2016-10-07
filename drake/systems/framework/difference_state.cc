#include "drake/systems/framework/difference_state.h"


#include "drake/common/eigen_autodiff_types.h"

namespace drake {
namespace systems {

template class DifferenceState<double>;
template class DifferenceState<AutoDiffXd>;

}  // namespace systems
}  // namespace drake
