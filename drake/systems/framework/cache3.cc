// For now, this is an empty .cc file that only serves to confirm
// cache.h is a stand-alone header.

#include "drake/systems/framework/cache.h"

namespace drake {
namespace systems {

// Instantiate to get a compile-time check.
template class Cache<double>;

} // namespace systems
} // namespace drake

