#pragma once

#include "drake/common/drake_assert.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace multibody {
namespace internal {

// This class is used to grant access to a selected collection of
// MultibodyPlant's private members and/or methods. It is meant to be used by
// internal:: implementations that might need it. In particular, originally
// designed to enable the implementation of ContactComputationManager classes
// that will often need private access into MultibodyPlant.
// Examples of usage:
//   const MultibodyPlantAccess<T> plant_access(&my_plant);
//   plant_access.ConstFooMethod();
//   plant_access.const_foo_member_;
// for const access and:
//   MultibodyPlantAccess<T> plant_access(&my_plant);
//   plant_access.MutableFooMethod();
//   plant_access.mutable_foo_member_;
// for mutable access.
template <typename T>
class MultibodyPlantAccess {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MultibodyPlantAccess);

  // Provides access to const private methods and members.
  explicit MultibodyPlantAccess(const MultibodyPlant<T>* plant)
      : plant_(plant) {
    DRAKE_DEMAND(plant != nullptr);
  }

  // Provides access to mutable private methods and members.
  // TODO(xuchenhan-tri): unit test this.
  explicit MultibodyPlantAccess(MultibodyPlant<T>* plant)
      : mutable_plant_(plant) {
    DRAKE_DEMAND(plant != nullptr);
  }

  const MultibodyTree<T>& internal_tree() const {
    DRAKE_DEMAND((plant_ == nullptr) ^ (mutable_plant_ == nullptr));
    if (plant_) return plant_->internal_tree();
    return mutable_plant_->internal_tree();
  }

 private:
  const MultibodyPlant<T>* plant_{nullptr};
  MultibodyPlant<T>* mutable_plant_{nullptr};
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
