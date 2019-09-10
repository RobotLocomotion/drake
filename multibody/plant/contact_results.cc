#include "drake/multibody/plant/contact_results.h"

#include <utility>

namespace drake {
namespace multibody {

template <typename T>
void ContactResults<T>::Clear() {
  point_pairs_info_.clear();
  hydroelastic_contact_info_.clear();
}

template <typename T>
const PointPairContactInfo<T>&
ContactResults<T>::point_pair_contact_info(int i) const {
  DRAKE_DEMAND(i >= 0 && i < num_point_pair_contacts());
  return point_pairs_info_[i];
}

template <typename T>
const HydroelasticContactInfo<T>&
ContactResults<T>::hydroelastic_contact_info(int i) const {
  DRAKE_DEMAND(i >= 0 && i < num_hydroelastic_contacts());
  return hydroelastic_contact_info_[i];
}

}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::ContactResults)
