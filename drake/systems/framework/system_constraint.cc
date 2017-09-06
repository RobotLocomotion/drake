#include "drake/systems/framework/system_constraint.h"

#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/symbolic.h"

namespace drake {
namespace systems {

template class SystemConstraint<double>;
template class SystemConstraint<AutoDiffXd>;
template class SystemConstraint<symbolic::Expression>;

}  // namespace systems
}  // namespace drake
