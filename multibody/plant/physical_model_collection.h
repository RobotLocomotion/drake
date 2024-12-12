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

/* A PhysicalModelCollection is a scalar convertible component of a
 MultibodyPlant. Each MultibodyPlant owns exactly one PhysicalModelCollection.
 It helps its owning MultibodyPlant manage its PhysicalModel components and
 ensures that each type of PhysicalModel appears at most once in the
 MultibodyPlant. PhysicalModels can be added prior to the call to
 `DeclareSystemResources`. Once `DeclareSystemResources` is called, a
 PhysicalModelCollection is considered to be "finalized" and new PhysicalModels
 can no longer be added. The collection of PhysicalModel types is closed and
 currently the only two types are DeformableModel and DummyPhysicalModel. */
template <typename T>
class PhysicalModelCollection : public ScalarConvertibleComponent<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PhysicalModelCollection);

  /* Constructs an empty PhysicalModelCollection. */
  PhysicalModelCollection() = default;

  ~PhysicalModelCollection() override;

  /* Adds a DeformableModel to the collection and returns the added model if
   successful.
   @throws std::exception if a DeformableModel is already in the collection.
   @throws std::exception if model is nullptr.
   @throws std::exception if model.plant() is different from plant(). */
  DeformableModel<T>& AddDeformableModel(
      std::unique_ptr<DeformableModel<T>> model);

  /* (For testing only) Adds a DummyPhysicalModel to the collection and returns
   the added model if successful.
   @throws std::exception if a DummyPhysicalModel is already in the collection.
   @throws std::exception if model is nullptr.
   @throws std::exception if model.plant() is different from plant(). */
  DummyPhysicalModel<T>& AddDummyModel(
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
   the owning plant. For each PhysicalModelCollection object, this function
   should be called once and only once by the owning plant, typically in the
   process of MultibodyPlant::Finalize(). After the call to this function, new
   models can no longer be added to `this` PhysicalModelCollection. */
  void DeclareSystemResources();

  /* Creates a clone of `this` PhysicalModelCollection with the scalar type
   `ScalarType`. The clone should be a deep scalar conversion of the original
   PhysicalModelCollection with the exception of back pointer to the owning
   plant. This function should only called from the scalar conversion
   constructor of MultibodyPlant, with `new_plant` being the scalar converted
   MultibodyPlant under construction.
   @tparam ScalarType double, AutoDiffXd, or symbolic::Expression.
   @param[in] new_plant pointer to the MultibodyPlant that will own the cloned
   PhysicalModelCollection. Note that we take a mutable pointer to
   MultibodyPlant here as required by cloning each individual PhysicalModel.
   @pre is_cloneable_to_<ScalarType>() returns true.
   @throw std::exception if plant is nullptr.
   @throw std::exception if DeclareSystemResources() has not been called on
   `this` PhysicalModelCollection
   @note `DeclareSystemResources()` is not called on the clone and needs to be
   called from the plant owning the clone. */
  template <typename ScalarType>
  std::unique_ptr<PhysicalModelCollection<ScalarType>> CloneToScalar(
      MultibodyPlant<ScalarType>* new_plant) const {
    DRAKE_THROW_UNLESS(system_resources_declared_);
    std::unique_ptr<PhysicalModelCollection<ScalarType>> clone =
        std::make_unique<PhysicalModelCollection<ScalarType>>();
    if (deformable_model_) {
      auto deformable_model_clone =
          std::unique_ptr<DeformableModel<ScalarType>>(
              static_cast<DeformableModel<ScalarType>*>(
                  deformable_model_
                      ->template CloneToScalar<ScalarType>(new_plant)
                      .release()));
      clone->AddDeformableModel(std::move(deformable_model_clone));
    }
    if (dummy_model_) {
      auto dummy_model_clone = std::unique_ptr<DummyPhysicalModel<ScalarType>>(
          static_cast<DummyPhysicalModel<ScalarType>*>(
              dummy_model_->template CloneToScalar<ScalarType>(new_plant)
                  .release()));
      clone->AddDummyModel(std::move(dummy_model_clone));
    }
    return clone;
  }

  /* For each PhysicalModel in `this` collection, declares zero or more output
   ports in the MultibodyPlant owning the PhysicalModel to communicate with a
   SceneGraph.
   @throw std::exception if called after call to DeclareSystemResources().
   @throws std::exception if called more than when at least one output port is
   created. */
  void DeclareSceneGraphPorts();

  /* Throws an std::exception if the given `model` belongs to a different
   MultibodyPlant from that of existing models. */
  void ThrowForIncompatibleModel(const PhysicalModel<T>& model) const;

 private:
  bool system_resources_declared_{false};
  std::vector<std::unique_ptr<PhysicalModel<T>>> owned_models_;
  DeformableModel<T>* deformable_model_{nullptr};
  DummyPhysicalModel<T>* dummy_model_{nullptr};
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
