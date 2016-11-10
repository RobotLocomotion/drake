// NOLINTNEXTLINE(build/include) False positive on inl file.
#include "drake/systems/framework/primitives/gain-inl.h"

#include "drake/common/eigen_autodiff_types.h"

namespace drake {
namespace systems {

template class Gain<double>;
template class Gain<AutoDiffXd>;

}  // namespace systems
}  // namespace drake
