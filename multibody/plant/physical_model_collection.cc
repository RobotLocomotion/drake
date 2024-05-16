#include "drake/multibody/plant/physical_model_collection.h"

namespace drake {
namespace multibody {
namespace internal {

template <typename T>
DeformableModel<T>& PhysicalModelCollection<T>::AddDeformableModel(
    std::unique_ptr<DeformableModel<T>> model) {
  DRAKE_THROW_UNLESS(deformable_model_ == nullptr);
  DRAKE_THROW_UNLESS(model != nullptr);
  DRAKE_THROW_UNLESS(model->plant() == plant());
  deformable_model_ = model.get();
  owned_models_.emplace_back(std::move(model));
  return *deformable_model_;
}

template <typename T>
DummyPhysicalModel<T>& PhysicalModelCollection<T>::AddDummyModel(
    std::unique_ptr<DummyPhysicalModel<T>> model) {
  DRAKE_THROW_UNLESS(dummy_model_ == nullptr);
  DRAKE_THROW_UNLESS(model != nullptr);
  DRAKE_THROW_UNLESS(model->plant() == plant());
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
  owning_plant_ = nullptr;
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::multibody::internal::PhysicalModelCollection)
