#pragma once

#include <memory>
#include <utility>
#include <variant>
#include <vector>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/ssize.h"
#include "drake/multibody/plant/deformable_contact_info.h"
#include "drake/multibody/plant/hydroelastic_contact_info.h"
#include "drake/multibody/plant/point_pair_contact_info.h"

namespace drake {
namespace multibody {

#ifndef DRAKE_DOXYGEN_CXX
template <typename T>
class MultibodyPlant;
#endif

/** A container class storing the contact results information for each contact
 pair for a given state of the simulation. Note that copying this data structure
 is expensive.
 @tparam_default_scalar */
template <typename T>
class ContactResults {
 public:
  DRAKE_DECLARE_COPY_AND_MOVE_AND_ASSIGN(ContactResults);
  ContactResults() = default;
  ~ContactResults();

  /** Returns the number of point pair contacts. */
  int num_point_pair_contacts() const { return ssize(point_pair_); }

  /** Returns the number of hydroelastic contacts. */
  int num_hydroelastic_contacts() const { return ssize(hydroelastic_); }

  /** Returns the number of deformable contacts. */
  int num_deformable_contacts() const { return ssize(deformable_); }

  /** Retrieves the ith PointPairContactInfo instance. The input index i
   must be in the range [0, `num_point_pair_contacts()`) or this method
   throws. */
  const PointPairContactInfo<T>& point_pair_contact_info(int i) const;

  /** Retrieves the ith HydroelasticContactInfo instance. The input index i
   must be in the range [0, `num_hydroelastic_contacts()`) or this
   method throws. */
  const HydroelasticContactInfo<T>& hydroelastic_contact_info(int i) const;

  /** Retrieves the ith DeformableContactInfo instance. The input index i
   must be in the range [0, `num_deformable_contacts()`) or this method
   throws. */
  const DeformableContactInfo<T>& deformable_contact_info(int i) const;

  /** Returns the plant that produced these contact results. In most cases the
  result will be non-null, but default-constructed results might have nulls. */
  const MultibodyPlant<T>* plant() const { return plant_; }

  /** Returns ContactResults with only HydroelasticContactInfo instances
   satisfying the selection criterion; all other contacts (point_pair and
   deformable) are retained, unchanged.

   @param selector Boolean predicate that returns true to select which
                   HydroelasticContactInfo.

   @note It uses deep copy. */
  ContactResults<T> SelectHydroelastic(
      std::function<bool(const HydroelasticContactInfo<T>&)> selector) const;

// The following methods should only be called by MultibodyPlant and testing
// fixtures and are undocumented rather than being made private with friends.
#ifndef DRAKE_DOXYGEN_CXX
  /* Sets the plant that produced these contact results. */
  void set_plant(const MultibodyPlant<T>* plant);

  /* Clears the set of contact information for when the old data becomes
   invalid. This includes clearing the `plant` back to nullptr. */
  void Clear();

  /* Overwrites the hydroelastic contact in `this`. */
  void SetContactInfo(std::vector<HydroelasticContactInfo<T>>&& hydroelastic);

  /* Adds a new contact pair result to `this`. */
  void AddContactInfo(const PointPairContactInfo<T>& point_pair);

  /* Adds a new hydroelastic contact to `this`. */
  void AddContactInfo(HydroelasticContactInfo<T> hydroelastic);

  /* Adds a new deformable contact result to `this`. */
  void AddContactInfo(DeformableContactInfo<T> deformable);
#endif

 private:
  std::vector<PointPairContactInfo<T>> point_pair_;
  std::vector<HydroelasticContactInfo<T>> hydroelastic_;
  std::vector<DeformableContactInfo<T>> deformable_;
  const MultibodyPlant<T>* plant_{nullptr};
};

template <typename T>
ContactResults<T>::ContactResults(ContactResults<T>&&) = default;

template <typename T>
ContactResults<T>& ContactResults<T>::operator=(ContactResults<T>&&) = default;

}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::ContactResults);
