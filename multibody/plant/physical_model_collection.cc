#include "drake/multibody/plant/physical_model_collection.h"

#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace multibody {
namespace internal {

template <typename T>
DeformableModel<T>* PhysicalModelCollection<T>::AddDeformableModel(
    std::unique_ptr<PhysicalModel<T>> model) {
  DRAKE_THROW_UNLESS(std::holds_alternative<const DeformableModel<T>*>(
      model->ToPhysicalModelPointerVariant()));
  DRAKE_THROW_UNLESS(deformable_model_ == nullptr);
  owned_models_.push_back(std::move(model));
  deformable_model_ =
      static_cast<DeformableModel<T>*>(owned_models_.back().get());
  return deformable_model_;
}

template <typename T>
internal::DummyPhysicalModel<T>* PhysicalModelCollection<T>::AddDummyModel(
    std::unique_ptr<PhysicalModel<T>> model) {
  DRAKE_THROW_UNLESS(std::holds_alternative<std::monostate>(
      model->ToPhysicalModelPointerVariant()));
  DRAKE_THROW_UNLESS(dummy_model_ == nullptr);
  owned_models_.push_back(std::move(model));
  dummy_model_ =
      static_cast<internal::DummyPhysicalModel<T>*>(owned_models_.back().get());
  return dummy_model_;
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

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::multibody::internal::PhysicalModelCollection)
