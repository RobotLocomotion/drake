// NOLINTNEXTLINE(build/include) False positive on inl file.
#include "drake/systems/primitives/pass_through-inl.h"

#include "drake/common/autodiff_overloads.h"
#include "drake/common/eigen_autodiff_types.h"

namespace drake {
namespace systems {

template class PassThrough<double>;
template class PassThrough<AutoDiffXd>;
template class PassThrough<symbolic::Expression>;

}  // namespace systems
}  // namespace drake
