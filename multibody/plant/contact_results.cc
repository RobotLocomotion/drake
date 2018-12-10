#include "drake/multibody/plant/contact_results.h"

#include <utility>

#include "drake/common/default_scalars.h"

namespace drake {
namespace multibody {

template <typename T>
void ContactResults<T>::AddContactInfo(
    const PointPairContactInfo<T>& point_pair_info) {
  point_pairs_info_.push_back(point_pair_info);
}

template <typename T>
int ContactResults<T>::num_contacts() const {
  return static_cast<int>(point_pairs_info_.size());
}

template <typename T>
void ContactResults<T>::Clear() { point_pairs_info_.clear(); }

template <typename T>
const PointPairContactInfo<T>& ContactResults<T>::contact_info(int i) const {
  DRAKE_DEMAND(i >= 0 && i < num_contacts());
  return point_pairs_info_[i];
}

}  // namespace multibody
}  // namespace drake

// Explicitly instantiates on the most common scalar types.
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::ContactResults)
