#include "drake/systems/plants/rigid_body_plant/contact_detail.h"

namespace drake {
namespace systems {

// Explicitly instantiates on the most common scalar types.
template class ContactDetail<double>;

}  // namespace systems
}  // namespace drake
