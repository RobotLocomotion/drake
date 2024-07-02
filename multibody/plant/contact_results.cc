#include "drake/multibody/plant/contact_results.h"

#include <utility>

namespace drake {
namespace multibody {

template <typename T>
ContactResults<T>::ContactResults(const ContactResults<T>&) = default;

template <typename T>
ContactResults<T>& ContactResults<T>::operator=(const ContactResults<T>&) =
    default;

template <typename T>
ContactResults<T>::~ContactResults() = default;

template <typename T>
const PointPairContactInfo<T>& ContactResults<T>::point_pair_contact_info(
    int i) const {
  DRAKE_THROW_UNLESS(i >= 0 && i < num_point_pair_contacts());
  return point_pair_[i];
}

template <typename T>
const HydroelasticContactInfo<T>& ContactResults<T>::hydroelastic_contact_info(
    int i) const {
  DRAKE_THROW_UNLESS(i >= 0 && i < num_hydroelastic_contacts());
  return hydroelastic_[i];
}

template <typename T>
const DeformableContactInfo<T>& ContactResults<T>::deformable_contact_info(
    int i) const {
  DRAKE_THROW_UNLESS(i >= 0 && i < num_deformable_contacts());
  return deformable_[i];
}

template <typename T>
ContactResults<T> ContactResults<T>::SelectHydroelastic(
    std::function<bool(const HydroelasticContactInfo<T>&)> selector) const {
  ContactResults<T> result;
  result.point_pair_ = point_pair_;
  std::copy_if(hydroelastic_.begin(), hydroelastic_.end(),
               std::back_inserter(result.hydroelastic_), selector);
  result.deformable_ = deformable_;
  result.plant_ = plant_;
  return result;
}

template <typename T>
void ContactResults<T>::set_plant(const MultibodyPlant<T>* plant) {
  DRAKE_THROW_UNLESS(plant != nullptr);
  plant_ = plant;
}

template <typename T>
void ContactResults<T>::Clear() {
  point_pair_.clear();
  hydroelastic_.clear();
  deformable_.clear();
  plant_ = nullptr;
}

template <typename T>
void ContactResults<T>::SetContactInfo(
    std::vector<HydroelasticContactInfo<T>>&& hydroelastic) {
  hydroelastic_ = std::move(hydroelastic);
}

template <typename T>
void ContactResults<T>::AddContactInfo(
    const PointPairContactInfo<T>& point_pair) {
  point_pair_.push_back(point_pair);
}

template <typename T>
void ContactResults<T>::AddContactInfo(
    HydroelasticContactInfo<T> hydroelastic) {
  hydroelastic_.push_back(std::move(hydroelastic));
}

template <typename T>
void ContactResults<T>::AddContactInfo(DeformableContactInfo<T> deformable) {
  deformable_.push_back(std::move(deformable));
}

}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::ContactResults);
