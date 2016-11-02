// NOLINTNEXTLINE(build/include) False positive on inl file.
#include "drake/systems/framework/primitives/pass_through-inl.h"

#include "drake/common/eigen_autodiff_types.h"

namespace drake {
namespace systems {

template class DRAKE_EXPORT PassThrough<double>;
template class DRAKE_EXPORT PassThrough<AutoDiffXd>;

}  // namespace systems
}  // namespace drake
