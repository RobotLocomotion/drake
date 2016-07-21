// For now, this is an empty .cc file that only serves to confirm value.h is
// a stand-alone header and that an instantiation compiles.

#include "drake/systems/framework/value.h"

namespace drake {
namespace systems {

template class Value<double>;

} // namespace systems
} // namespace drake

