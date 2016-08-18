#include "drake/systems/framework/primitives/gain-inl.h"

#include "drake/common/eigen_autodiff_types.h"

namespace drake {
namespace systems {

template class DRAKESYSTEMFRAMEWORK_EXPORT Gain<double>;
template class DRAKESYSTEMFRAMEWORK_EXPORT Gain<AutoDiffXd>;

}  // namespace systems
}  // namespace drake
