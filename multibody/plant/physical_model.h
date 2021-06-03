#pragma once
#include <set>
#include <string>

#include "drake/common/default_scalars.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace multibody {
template <typename T>
class MultibodyPlant;

namespace internal {

/* PhysicalModel provides the functionalities to extend the type of
 physical model of MultibodyPlant. Developers can derive from this
 PhysicalModel to incorporate additional model elements coupled with the
 rigid body dynamics. For instance, simulation of deformable objects requires
 additional state and ports to interact with externals systems such as
 visualization.

 Similar to the routine of adding multiple model elements in MultibodyPlant,
 users should add all the model elements they wish to add to a PhysicalModel
 before the owning MultibodyPlant calls `Finalize()`. When `Finalize()` is
 invoked, MultibodyPlant will allocate the system level context resources for
 each PhysicalModel it owns. After the system resources are allocated, model
 mutation in the PhysicalModels owned by MultibodyPlant is not allowed.

 @tparam_default_scalar */
template <typename T>
class PhysicalModel {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PhysicalModel);

  PhysicalModel() = default;

  virtual ~PhysicalModel() = default;

  /* (Internal) MultibodyPlant calls this from within Finalize() to declare
   additional system resources. This method is only meant to be called by
   MultibodyPlant. We pass in a MultibodyPlant pointer so that derived
   PhysicalModels can use specific MultibodyPlant cache tickets.
   @pre plant != nullptr. */
  void DeclareSystemResources(MultibodyPlant<T>* plant) {
    DRAKE_DEMAND(plant != nullptr);
    DoDeclareSystemResources(plant);
    system_resources_declared = true;
  }

 protected:
  /* Derived class must override this to declare system resources for its
   specific model. */
  virtual void DoDeclareSystemResources(MultibodyPlant<T>* plant) = 0;

  /* Helper method for throwing an exception within public methods that should
   not be called after system resources are declared. The invoking method should
   pass its name so that the error message can include that detail. */
  void ThrowIfSystemResourcesDeclared(const char* source_method) const {
    if (system_resources_declared) {
      throw std::logic_error(
          "Calls to '" + std::string(source_method) +
          "()' after system resources have been declared are not allowed.");
    }
  }

  /* Helper method for throwing an exception within public methods that should
   not be called before system resources are declared. The invoking method
   should pass its name so that the error message can include that detail. */
  void ThrowIfSystemResourcesNotDeclared(const char* source_method) const {
    if (!system_resources_declared) {
      throw std::logic_error(
          "Calls to '" + std::string(source_method) +
          "()' before system resources have been declared are not allowed.");
    }
  }

  /* Protected LeafSystem methods exposed through MultibodyPlant. */
  static systems::DiscreteStateIndex DeclareDiscreteState(
      MultibodyPlant<T>* plant, const VectorX<T>& model_value);

  static systems::LeafOutputPort<T>& DeclareAbstractOutputPort(
      MultibodyPlant<T>* plant, std::string name,
      typename systems::LeafOutputPort<T>::AllocCallback alloc_function,
      typename systems::LeafOutputPort<T>::CalcCallback calc_function,
      std::set<systems::DependencyTicket> prerequisites_of_calc = {
          systems::System<T>::all_sources_ticket()});

  static systems::LeafOutputPort<T>& DeclareVectorOutputPort(
      MultibodyPlant<T>* plant, std::string name,
      const systems::BasicVector<T>& model_vector,
      typename systems::LeafOutputPort<T>::CalcVectorCallback
          vector_calc_function,
      std::set<systems::DependencyTicket> prerequisites_of_calc = {
          systems::System<T>::all_sources_ticket()});

 private:
  /* Flag to track whether the system resources requested by `this`
   PhysicalModel have been declared. */
  bool system_resources_declared{false};
};
}  // namespace internal
}  // namespace multibody
}  // namespace drake
DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::PhysicalModel);
