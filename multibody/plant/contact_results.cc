#include "drake/multibody/plant/contact_results.h"

#include <utility>

namespace drake {
namespace multibody {

template <typename T>
ContactResults<T>::ContactResults(
    std::vector<PointPairContactInfo<T>>&& point_pair,
    std::vector<HydroelasticContactInfo<T>>&& hydroelastic,
    std::vector<DeformableContactInfo<T>>&& deformable,
    std::shared_ptr<const void>&& backing_store)
    : data_(std::make_shared<Data>(
          Data{std::move(point_pair), std::move(hydroelastic),
               std::move(deformable), std::move(backing_store)})) {}

template <typename T>
ContactResults<T>::~ContactResults() = default;

// TODO(jwnimmer-tri) The order of these definitions does not match the header
// file. We should shuffle them around to match.

template <typename T>
const MultibodyPlant<T>* ContactResults<T>::plant() const {
  return plant_;
}

template <typename T>
const PointPairContactInfo<T>& ContactResults<T>::point_pair_contact_info(
    int i) const {
  DRAKE_THROW_UNLESS(i >= 0 && i < num_point_pair_contacts());
  DRAKE_ASSERT(data_ != nullptr);
  return data_->point_pair[i];
}

template <typename T>
const HydroelasticContactInfo<T>& ContactResults<T>::hydroelastic_contact_info(
    int i) const {
  DRAKE_THROW_UNLESS(i >= 0 && i < num_hydroelastic_contacts());
  DRAKE_ASSERT(data_ != nullptr);
  return data_->hydroelastic[i];
}

template <typename T>
const DeformableContactInfo<T>& ContactResults<T>::deformable_contact_info(
    int i) const {
  DRAKE_THROW_UNLESS(i >= 0 && i < num_deformable_contacts());
  DRAKE_ASSERT(data_ != nullptr);
  return data_->deformable[i];
}

// TODO(jwnimmer-tri) For a function intended as a performance hack, this is
// sure doing a lot of copying. We should consider deprecating this for removal.
template <typename T>
ContactResults<T> ContactResults<T>::SelectHydroelastic(
    std::function<bool(const HydroelasticContactInfo<T>&)> selector) const {
  if (num_hydroelastic_contacts() == 0) {
    return *this;
  }
  DRAKE_DEMAND(data_ != nullptr);
  std::vector<HydroelasticContactInfo<T>> selected;
  std::copy_if(data_->hydroelastic.begin(), data_->hydroelastic.end(),
               std::back_inserter(selected), selector);
  ContactResults<T> result(
      std::vector<PointPairContactInfo<T>>(data_->point_pair),
      std::move(selected),
      std::vector<DeformableContactInfo<T>>(data_->deformable),
      std::shared_ptr<const void>(data_->backing_store));
  result.plant_ = this->plant_;
  return result;
}

template <typename T>
void ContactResults<T>::set_plant(const MultibodyPlant<T>* plant) {
  DRAKE_THROW_UNLESS(plant != nullptr);
  plant_ = plant;
}

}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::ContactResults);
