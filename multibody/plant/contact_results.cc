#include "drake/multibody/plant/contact_results.h"

#include <utility>

namespace drake {
namespace multibody {

template <typename T>
ContactResults<T>::ContactResults() {
  // Make this hold pointers at first (see comment on
  // hydroelastic_contact_info_).
  hydroelastic_contact_info_ = std::vector<const HydroelasticContactInfo<T>*>();
}

template <typename T>
ContactResults<T>::ContactResults(const ContactResults<T>& contact_results) {
  *this = contact_results;
}

template <typename T>
ContactResults<T>& ContactResults<T>::operator=(
    const ContactResults<T>& contact_results) {
  // Make the type a vector of const pointers if we can.
  if (contact_results.num_hydroelastic_contacts() == 0) {
    hydroelastic_contact_info_ =
        std::vector<const HydroelasticContactInfo<T>*>();
  } else {
    // If this currently holds pointers, we need to change the type.
    if (hydroelastic_contact_vector_ownership_mode() == kAliasedPointers) {
      hydroelastic_contact_info_ =
          std::vector<std::unique_ptr<HydroelasticContactInfo<T>>>();
    }

    // Copy the HydroelasticContactInfo data.
    std::vector<std::unique_ptr<HydroelasticContactInfo<T>>>&
        hydroelastic_contact_vector =
            hydroelastic_contact_vector_of_unique_ptrs();
    hydroelastic_contact_vector.resize(
        contact_results.num_hydroelastic_contacts());
    for (int i = 0; i < contact_results.num_hydroelastic_contacts(); ++i) {
      const HydroelasticContactInfo<T>& contact_info =
          contact_results.hydroelastic_contact_info(i);
      hydroelastic_contact_vector[i] =
          std::make_unique<HydroelasticContactInfo<T>>(contact_info);
    }
  }

  point_pairs_info_ = contact_results.point_pairs_info_;
  deformable_contact_info_ = contact_results.deformable_contact_info_;
  plant_ = contact_results.plant_;

  return *this;
}

template <typename T>
const MultibodyPlant<T>* ContactResults<T>::plant() const {
  return plant_;
}

template <typename T>
void ContactResults<T>::set_plant(const MultibodyPlant<T>* plant) {
  DRAKE_THROW_UNLESS(plant != nullptr);
  plant_ = plant;
}

template <typename T>
void ContactResults<T>::Clear() {
  point_pairs_info_.clear();
  if (hydroelastic_contact_vector_ownership_mode() == kAliasedPointers) {
    hydroelastic_contact_vector_of_pointers().clear();
  } else {
    hydroelastic_contact_vector_of_unique_ptrs().clear();
  }
  deformable_contact_info_.clear();
  plant_ = nullptr;
}

template <typename T>
const PointPairContactInfo<T>& ContactResults<T>::point_pair_contact_info(
    int i) const {
  DRAKE_THROW_UNLESS(i >= 0 && i < num_point_pair_contacts());
  return point_pairs_info_[i];
}

template <typename T>
const HydroelasticContactInfo<T>& ContactResults<T>::hydroelastic_contact_info(
    int i) const {
  DRAKE_THROW_UNLESS(i >= 0 && i < num_hydroelastic_contacts());
  if (hydroelastic_contact_vector_ownership_mode() == kAliasedPointers) {
    return *hydroelastic_contact_vector_of_pointers()[i];
  } else {
    return *hydroelastic_contact_vector_of_unique_ptrs()[i];
  }
}

template <typename T>
const DeformableContactInfo<T>& ContactResults<T>::deformable_contact_info(
    int i) const {
  DRAKE_THROW_UNLESS(i >= 0 && i < num_deformable_contacts());
  return deformable_contact_info_[i];
}

template <typename T>
void ContactResults<T>::AddContactInfo(
    const HydroelasticContactInfo<T>* hydroelastic_contact_info) {
  DRAKE_DEMAND(hydroelastic_contact_vector_ownership_mode() ==
               kAliasedPointers);
  hydroelastic_contact_vector_of_pointers().push_back(
      hydroelastic_contact_info);
}

template <typename T>
int ContactResults<T>::num_hydroelastic_contacts() const {
  if (hydroelastic_contact_vector_ownership_mode() == kAliasedPointers) {
    return static_cast<int>(hydroelastic_contact_vector_of_pointers().size());
  } else {
    return static_cast<int>(
        hydroelastic_contact_vector_of_unique_ptrs().size());
  }
}

template <typename T>
ContactResults<T> ContactResults<T>::SelectHydroelastic(
    std::function<bool(const HydroelasticContactInfo<T>&)> selector) const {
  ContactResults<T> selected_alias_pointers;
  if (this->plant() != nullptr) {
    selected_alias_pointers.set_plant(this->plant());
  }
  int num_hydroelastic_contacts = this->num_hydroelastic_contacts();
  for (int i = 0; i < num_hydroelastic_contacts; ++i) {
    const HydroelasticContactInfo<T>& contact_info =
        this->hydroelastic_contact_info(i);
    if (selector(contact_info)) {
      selected_alias_pointers.AddContactInfo(&contact_info);
    }
  }
  // Deep copy of the selected hydroelastic contact info.
  ContactResults<T> output_unique_pointers = selected_alias_pointers;
  // Deep copy of the point pair contact info.
  output_unique_pointers.point_pairs_info_ = this->point_pairs_info_;
  // Deep copy of the deformable contact info.
  output_unique_pointers.deformable_contact_info_ =
      this->deformable_contact_info_;

  return output_unique_pointers;
}

}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::ContactResults)
