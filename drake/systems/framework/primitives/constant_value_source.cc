#include "drake/systems/framework/primitives/constant_value_source-inl.h"

namespace drake {
namespace systems {

// Explicitly instantiates on the most common scalar types.
template class DRAKESYSTEMFRAMEWORK_EXPORT ConstantValueSource<double>;

}  // namespace systems
}  // namespace drake
