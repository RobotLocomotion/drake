#include "drake/systems/plants/rigid_body_plant/contact_manifold.h"

namespace drake {
namespace systems {

// Explicitly instantiates on the most common scalar types.
template class ContactManifold<double>;
}  // namespace system
}  // namespace drake
