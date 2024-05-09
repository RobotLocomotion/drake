#include "drake/multibody/plant/physical_model.h"

#include <utility>

#include "drake/multibody/plant/multibody_plant_model_attorney.h"

namespace drake {
namespace multibody {

template <typename T>
std::unique_ptr<PhysicalModel<double>> PhysicalModel<T>::CloneToDouble(
    MultibodyPlant<double>*) const {
  throw std::logic_error(
      "Scalar conversion to double is not supported by this PhysicalModel.");
}

template <typename T>
std::unique_ptr<PhysicalModel<AutoDiffXd>> PhysicalModel<T>::CloneToAutoDiffXd(
    MultibodyPlant<AutoDiffXd>*) const {
  throw std::logic_error(
      "Scalar conversion to AutoDiffXd is not supported by this "
      "PhysicalModel.");
}

template <typename T>
std::unique_ptr<PhysicalModel<symbolic::Expression>>
PhysicalModel<T>::CloneToSymbolic(MultibodyPlant<symbolic::Expression>*) const {
  throw std::logic_error(
      "Scalar conversion to symbolic::Expression is not supported by this "
      "PhysicalModel.");
}

template <typename T>
bool PhysicalModel<T>::is_cloneable_to_double() const {
  return false;
}

template <typename T>
bool PhysicalModel<T>::is_cloneable_to_autodiff() const {
  return false;
}

template <typename T>
bool PhysicalModel<T>::is_cloneable_to_symbolic() const {
  return false;
}

template <typename T>
void PhysicalModel<T>::ThrowIfSystemResourcesDeclared(
    const char* function_name) const {
  if (owning_plant_ == nullptr) {
    throw std::logic_error(
        fmt::format("Calls to {}() after system resources have been declared "
                    "are not allowed.",
                    function_name));
  }
}

template <typename T>
void PhysicalModel<T>::ThrowIfSystemResourcesNotDeclared(
    const char* function_name) const {
  if (owning_plant_ != nullptr) {
    throw std::logic_error(
        fmt::format("Calls to {}() before system resources have been declared "
                    "are not allowed.",
                    function_name));
  }
}

template <typename T>
geometry::SceneGraph<T>& PhysicalModel<T>::mutable_scene_graph() {
  DRAKE_THROW_UNLESS(owning_plant_ != nullptr);
  return internal::MultibodyPlantModelAttorney<T>::mutable_scene_graph(
      owning_plant_);
}

template <typename T>
systems::DiscreteStateIndex PhysicalModel<T>::DeclareDiscreteState(
    const VectorX<T>& model_value) {
  DRAKE_THROW_UNLESS(owning_plant_ != nullptr);
  return internal::MultibodyPlantModelAttorney<T>::DeclareDiscreteState(
      owning_plant_, model_value);
}

template <typename T>
systems::LeafOutputPort<T>& PhysicalModel<T>::DeclareAbstractOutputPort(
    std::string name,
    typename systems::LeafOutputPort<T>::AllocCallback alloc_function,
    typename systems::LeafOutputPort<T>::CalcCallback calc_function,
    std::set<systems::DependencyTicket> prerequisites_of_calc) {
  DRAKE_THROW_UNLESS(owning_plant_ != nullptr);
  return internal::MultibodyPlantModelAttorney<T>::DeclareAbstractOutputPort(
      owning_plant_, std::move(name), std::move(alloc_function),
      std::move(calc_function), std::move(prerequisites_of_calc));
}

template <typename T>
systems::LeafOutputPort<T>& PhysicalModel<T>::DeclareVectorOutputPort(
    std::string name, const systems::BasicVector<T>& model_vector,
    typename systems::LeafOutputPort<T>::CalcVectorCallback
        vector_calc_function,
    std::set<systems::DependencyTicket> prerequisites_of_calc) {
  DRAKE_THROW_UNLESS(owning_plant_ != nullptr);
  return internal::MultibodyPlantModelAttorney<T>::DeclareVectorOutputPort(
      owning_plant_, std::move(name), model_vector,
      std::move(vector_calc_function), std::move(prerequisites_of_calc));
}

}  // namespace multibody
}  // namespace drake
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::PhysicalModel);
