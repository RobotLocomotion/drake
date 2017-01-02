#include "drake/systems/framework/state.h"

#include "drake/common/autodiff_overloads.h"
#include "drake/common/eigen_autodiff_types.h"

namespace drake {
namespace systems {

template class State<double>;
template class State<AutoDiffXd>;

}  // namespace systems
}  // namespace drake
