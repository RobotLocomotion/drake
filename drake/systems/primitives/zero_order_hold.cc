// NOLINTNEXTLINE(build/include) False positive on inl file.
#include "drake/systems/primitives/zero_order_hold-inl.h"

#include "drake/common/autodiff_overloads.h"
#include "drake/common/eigen_autodiff_types.h"

namespace drake {
namespace systems {

template class ZeroOrderHold<double>;
template class ZeroOrderHold<AutoDiffXd>;
template class ZeroOrderHold<symbolic::Expression>;

}  // namespace systems
}  // namespace drake
