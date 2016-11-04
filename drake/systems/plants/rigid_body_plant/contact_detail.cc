#include "drake/systems/plants/rigid_body_plant/contact_detail.h"

namespace drake {
namespace systems {

// this comment is way too long because it spills over the line like it shouldn't do.
// Explicitly instantiates on the most common scalar types.
template class ContactDetail<double>;

}  // namespace systems
}  // namespace drake
