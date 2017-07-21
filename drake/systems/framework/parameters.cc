#include "drake/systems/framework/parameters.h"

#include "drake/common/autodiff_overloads.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/symbolic.h"

namespace drake {
namespace systems {

template class Parameters<double>;
template class Parameters<AutoDiffXd>;
template class Parameters<symbolic::Expression>;

}  // namespace systems
}  // namespace drake
