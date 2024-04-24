#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "drake/multibody/plant/deformable_model.h"
#include "drake/multibody/plant/dummy_physical_model.h"
#include "drake/multibody/plant/physical_model.h"

namespace drake {
namespace multibody {

template <typename T>
class MultibodyPlant;

namespace internal {

/* A collection of PhysicalModels. Each type of concrete PhysicalModel is stored
 at most once. */
template <typename T>
class PhysicalModelCollection : public ScalarConvertibleComponent<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PhysicalModelCollection);

  PhysicalModelCollection() = default;

  ~PhysicalModelCollection() = default;

  /* Adds a DeformableModel to the collection and returns the added model if
   successful.
   @throws std::exception if a DeformableModel is already in the collection.
   @throws std::exception if `model` is not a DeformableModel. */
  DeformableModel<T>* AddDeformableModel(
      std::unique_ptr<PhysicalModel<T>> model);

  /* (For testing only) Adds a DummyPhysicalModel to the collection and returns
   the added model if successful.
   @throws std::exception if a DummyPhysicalModel is already in the collection.
   @throws std::exception if `model` is not a DummyPhysicalModel. */
  internal::DummyPhysicalModel<T>* AddDummyModel(
      std::unique_ptr<PhysicalModel<T>> model);

  /* For the given `plant`, removes support for scalars that are not supported
   any of the PhysicalModels owned by `this` PhysicalModelCollection. */
  void RemoveUnsupportedScalars(MultibodyPlant<T>* plant);

  /* Returns a pointer to the stored DeformableModel if one exists and nullptr
   otherwise. */
  const DeformableModel<T>* deformable_model() const {
    return deformable_model_;
  }
  DeformableModel<T>* mutable_deformable_model() { return deformable_model_; }

  /* Returns a pointer to the stored DummyPhysicalModel if one exists and
   nullptr otherwise. */
  const internal::DummyPhysicalModel<T>* dummy_model() const {
    return dummy_model_;
  }
  internal::DummyPhysicalModel<T>* mutable_dummy_model() {
    return dummy_model_;
  }

  const std::vector<std::unique_ptr<PhysicalModel<T>>>& owned_models() const {
    return owned_models_;
  }

  bool is_cloneable_to_double() const override;

  bool is_cloneable_to_autodiff() const override;

  bool is_cloneable_to_symbolic() const override;

  template <typename ScalarType>
  std::unique_ptr<PhysicalModelCollection<ScalarType>> CloneToScalar(
      MultibodyPlant<ScalarType>* plant) const {
    std::unique_ptr<PhysicalModelCollection<ScalarType>> clone =
        std::make_unique<PhysicalModelCollection<ScalarType>>();
    if (deformable_model_) {
      clone->AddDeformableModel(
          deformable_model_->template CloneToScalar<ScalarType>(plant));
    }
    if (dummy_model_) {
      clone->AddDummyModel(
          dummy_model_->template CloneToScalar<ScalarType>(plant));
    }
    return clone;
  }

 private:
  std::vector<std::unique_ptr<PhysicalModel<T>>> owned_models_;
  DeformableModel<T>* deformable_model_{nullptr};
  internal::DummyPhysicalModel<T>* dummy_model_{nullptr};
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
