#include "drake/multibody/rigid_body_plant/contact_info.h"

using std::move;

namespace drake {
namespace systems {

template <typename T>
ContactInfo<T>::ContactInfo(drake::multibody::collision::ElementId element1,
                            drake::multibody::collision::ElementId element2)
    : element1_(element1),
      element2_(element2),
      resultant_force_(),
      contact_details_() {}

// Explicitly instantiates on the most common scalar types.
template class ContactInfo<double>;

}  // namespace systems
}  // namespace drake
