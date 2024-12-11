#include "drake/multibody/plant/physical_model_collection.h"

namespace drake {
namespace multibody {
namespace internal {

template <typename T>
PhysicalModelCollection<T>::~PhysicalModelCollection() = default;

template <typename T>
DeformableModel<T>& PhysicalModelCollection<T>::AddDeformableModel(
    std::unique_ptr<DeformableModel<T>> model) {
  DRAKE_THROW_UNLESS(deformable_model_ == nullptr);
  DRAKE_THROW_UNLESS(model != nullptr);
  ThrowForIncompatibleModel(*model);
  deformable_model_ = model.get();
  owned_models_.emplace_back(std::move(model));
  return *deformable_model_;
}

template <typename T>
DummyPhysicalModel<T>& PhysicalModelCollection<T>::AddDummyModel(
    std::unique_ptr<DummyPhysicalModel<T>> model) {
  DRAKE_THROW_UNLESS(dummy_model_ == nullptr);
  DRAKE_THROW_UNLESS(model != nullptr);
  ThrowForIncompatibleModel(*model);
  dummy_model_ = model.get();
  owned_models_.emplace_back(std::move(model));
  return *dummy_model_;
}
template <typename T>
bool PhysicalModelCollection<T>::is_cloneable_to_double() const {
  for (const auto& model : owned_models_) {
    if (!model->is_cloneable_to_double()) {
      return false;
    }
  }
  return true;
}

template <typename T>
bool PhysicalModelCollection<T>::is_cloneable_to_autodiff() const {
  for (const auto& model : owned_models_) {
    if (!model->is_cloneable_to_autodiff()) {
      return false;
    }
  }
  return true;
}

template <typename T>
bool PhysicalModelCollection<T>::is_cloneable_to_symbolic() const {
  for (const auto& model : owned_models_) {
    if (!model->is_cloneable_to_symbolic()) {
      return false;
    }
  }
  return true;
}

template <typename T>
void PhysicalModelCollection<T>::DeclareSystemResources() {
  for (auto& model : owned_models_) {
    model->DeclareSystemResources();
  }
  system_resources_declared_ = true;
}

template <typename T>
void PhysicalModelCollection<T>::DeclareSceneGraphPorts() {
  DRAKE_THROW_UNLESS(!system_resources_declared_);
  for (std::unique_ptr<PhysicalModel<T>>& model : owned_models_) {
    model->DeclareSceneGraphPorts();
  }
}

template <typename T>
void PhysicalModelCollection<T>::ThrowForIncompatibleModel(
    const PhysicalModel<T>& model) const {
  if (!owned_models_.empty() &&
      &model.plant() != &owned_models_.back()->plant()) {
    throw std::runtime_error(
        "The given model belongs to a different MultibodyPlant.");
  }
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::multibody::internal::PhysicalModelCollection);
