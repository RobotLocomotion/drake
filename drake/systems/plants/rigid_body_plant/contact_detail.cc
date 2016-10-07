#include "drake/systems/plants/rigid_body_plant/contact_detail.h"

namespace drake {
namespace systems {

template <typename T>
ContactDetail<T>::ContactDetail() {}

template <typename T>
ContactDetail<T>::ContactDetail(const Vector3<T>& point,
                                const WrenchVector<T>& wrench)
    : application_point_(point), wrench_(wrench) {}

// Explicitly instantiates on the most common scalar types.
template class DRAKE_EXPORT ContactDetail<double>;
}
}
