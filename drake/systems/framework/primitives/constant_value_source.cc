// NOLINTNEXTLINE(build/include) False positive on inl file.
#include "drake/systems/framework/primitives/constant_value_source-inl.h"

namespace drake {
namespace systems {

// Explicitly instantiates on the most common scalar types.
template class DRAKE_EXPORT ConstantValueSource<double>;

}  // namespace systems
}  // namespace drake
