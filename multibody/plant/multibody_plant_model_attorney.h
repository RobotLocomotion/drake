#pragma once

#include <set>
#include <string>
#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace multibody {

template <typename T>
class PhysicalModel;

namespace internal {

/* This class is used to grant access to a selected collection of
 MultibodyPlant's private methods to PhysicalModel.

 @tparam_default_scalar */
template <typename T>
class MultibodyPlantModelAttorney {
 private:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MultibodyPlantModelAttorney);

  friend class PhysicalModel<T>;

  /* Returns the SceneGraph with which the given `plant` has been registered.
   @pre plant != nullptr.
   @pre Finalize() has not been called on `plant`
   @pre `plant` has been registered with some SceneGraph. */
  static geometry::SceneGraph<T>& mutable_scene_graph(
      MultibodyPlant<T>* plant) {
    DRAKE_DEMAND(plant != nullptr);
    auto* scene_graph = plant->scene_graph_;
    DRAKE_DEMAND(scene_graph != nullptr);
    return *scene_graph;
  }

  static const internal::MultibodyTree<T>& internal_tree(
      const MultibodyPlant<T>& plant) {
    return plant.internal_tree();
  }

  static systems::DiscreteStateIndex DeclareDiscreteState(
      MultibodyPlant<T>* plant, const VectorX<T>& model_value) {
    DRAKE_DEMAND(plant != nullptr);
    return plant->DeclareDiscreteState(model_value);
  }

  static systems::AbstractParameterIndex DeclareAbstractParameter(
      MultibodyPlant<T>* plant, const AbstractValue& model_value) {
    DRAKE_DEMAND(plant != nullptr);
    return systems::AbstractParameterIndex(
        plant->DeclareAbstractParameter(model_value));
  }

  static systems::LeafOutputPort<T>& DeclareAbstractOutputPort(
      MultibodyPlant<T>* plant, std::string name,
      typename systems::LeafOutputPort<T>::AllocCallback alloc_function,
      typename systems::LeafOutputPort<T>::CalcCallback calc_function,
      std::set<systems::DependencyTicket> prerequisites_of_calc = {
          systems::System<T>::all_sources_ticket()}) {
    DRAKE_DEMAND(plant != nullptr);
    return plant->DeclareAbstractOutputPort(
        std::move(name), std::move(alloc_function), std::move(calc_function),
        std::move(prerequisites_of_calc));
  }

  static systems::LeafOutputPort<T>& DeclareVectorOutputPort(
      MultibodyPlant<T>* plant, std::string name,
      const systems::BasicVector<T>& model_vector,
      typename systems::LeafOutputPort<T>::CalcVectorCallback
          vector_calc_function,
      std::set<systems::DependencyTicket> prerequisites_of_calc = {
          systems::System<T>::all_sources_ticket()}) {
    DRAKE_DEMAND(plant != nullptr);
    return plant->DeclareVectorOutputPort(std::move(name), model_vector,
                                          std::move(vector_calc_function),
                                          std::move(prerequisites_of_calc));
  }
};
}  // namespace internal
}  // namespace multibody
}  // namespace drake
