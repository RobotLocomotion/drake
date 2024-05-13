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

  /* Constructs a PhysicalModelCollection owned by the given `plant`. The given
   * `plant` must outlive `this` PhysicalModelCollection. */
  explicit PhysicalModelCollection(const MultibodyPlant<T>* plant)
      : owning_plant_(plant) {}

  ~PhysicalModelCollection() = default;

  /* Returns the back pointer to the MultibodyPlant owning `this`
   PhysicalModel before DeclareSystemResources() are called and nullptr after.
  */
  const MultibodyPlant<T>* plant() const { return owning_plant_; }

  /* Adds a DeformableModel to the collection and returns the added model if
   successful.
   @throws std::exception if a DeformableModel is already in the collection.
   @throws std::exception if model is nullptr.
   @throws std::exception if model->plant() != plant(). */
  DeformableModel<T>* AddDeformableModel(
      std::unique_ptr<DeformableModel<T>> model);

  /* (For testing only) Adds a DummyPhysicalModel to the collection and returns
   the added model if successful.
   @throws std::exception if a DummyPhysicalModel is already in the collection.
   @throws std::exception if model is nullptr.
   @throws std::exception if model->plant() != plant(). */
  DummyPhysicalModel<T>* AddDummyModel(
      std::unique_ptr<DummyPhysicalModel<T>> model);

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

  /* Declare system resources on all owned models and nulls the back pointer to
   the owning plant. */
  void DeclareSystemResources();

  /* Creates a clone of `this` PhysicalModelCollection with the scalar type
   `ScalarType` to be owned by the given `plant`. The clone should be a deep
   scalar transmogrification of the original PhysicalModelCollection with the
   exception of back pointer to the owning plant.
   @tparam_default_scalar
   @param[in] plant pointer to the MultibodyPlant owning the clone.
   @throw std::exception if plant is nullptr.
   @throw std::exception if DeclareSystemResources() has not been called on
   `this` PhysicalModelCollection
   @note `DeclareSystemResources()` is not called on the clone and needs to be
   called from the plant owning the clone. */
  template <typename ScalarType>
  std::unique_ptr<PhysicalModelCollection<ScalarType>> CloneToScalar(
      MultibodyPlant<ScalarType>* plant) const {
    DRAKE_THROW_UNLESS(owning_plant_ == nullptr);
    std::unique_ptr<PhysicalModelCollection<ScalarType>> clone =
        std::make_unique<PhysicalModelCollection<ScalarType>>(plant);
    if (deformable_model_) {
      auto deformable_model_clone =
          std::unique_ptr<DeformableModel<ScalarType>>(
              static_cast<DeformableModel<ScalarType>*>(
                  deformable_model_->template CloneToScalar<ScalarType>(plant)
                      .release()));
      clone->AddDeformableModel(std::move(deformable_model_clone));
    }
    if (dummy_model_) {
      auto dummy_model_clone = std::unique_ptr<DummyPhysicalModel<ScalarType>>(
          static_cast<DummyPhysicalModel<ScalarType>*>(
              dummy_model_->template CloneToScalar<ScalarType>(plant)
                  .release()));
      clone->AddDummyModel(std::move(dummy_model_clone));
    }
    return clone;
  }

 private:
  const MultibodyPlant<T>* owning_plant_{nullptr};
  /* We maintain the invariant such that each `model` in `owned_models_`
   satisfies `model->plant() == owning_plant_`. */
  std::vector<std::unique_ptr<PhysicalModel<T>>> owned_models_;
  DeformableModel<T>* deformable_model_{nullptr};
  DummyPhysicalModel<T>* dummy_model_{nullptr};
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
