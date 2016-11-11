#include "drake/systems/framework/parameters.h"

#include "drake/common/autodiff_overloads.h"
#include "drake/common/eigen_autodiff_types.h"

namespace drake {
namespace systems {

template class Parameters<double>;
template class Parameters<AutoDiffXd>;

}  // namespace systems
}  // namespace drake
