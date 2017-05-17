#include "drake/systems/framework/discrete_values.h"

#include "drake/common/autodiff_overloads.h"
#include "drake/common/eigen_autodiff_types.h"

namespace drake {
namespace systems {

template class DiscreteValues<double>;
template class DiscreteValues<AutoDiffXd>;

}  // namespace systems
}  // namespace drake
