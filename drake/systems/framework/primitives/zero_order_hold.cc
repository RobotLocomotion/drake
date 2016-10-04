#include "drake/systems/framework/primitives/zero_order_hold-inl.h"

#include "drake/common/eigen_autodiff_types.h"

namespace drake {
namespace systems {

template class DRAKESYSTEMFRAMEWORK_EXPORT ZeroOrderHold<double>;
template class DRAKESYSTEMFRAMEWORK_EXPORT ZeroOrderHold<AutoDiffXd>;

}  // namespace systems
}  // namespace drake
