#include "drake/systems/framework/value.h"

namespace drake {
namespace systems {

AbstractValue::~AbstractValue() {}

// Explicit instantiations for VectorValue<double>.
template class Value<VectorBase<double>*>;
template class VectorValue<double>;

}  // namespace systems
}  // namespace drake
