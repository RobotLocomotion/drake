#include "drake/systems/framework/basic_vector.h"

#include "drake/common/autodiff_overloads.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/symbolic_expression.h"

namespace drake {
namespace systems {

template class BasicVector<double>;
template class BasicVector<AutoDiffXd>;
template class BasicVector<symbolic::Expression>;

}  // namespace systems
}  // namespace drake
