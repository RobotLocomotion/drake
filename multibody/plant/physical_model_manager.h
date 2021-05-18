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

/* PhysicalModelManager provides the functionalities to extend the type of
 physical model of MultibodyPlant. Developers can derive from this
 PhysicalModelManager to incorporate additional discrete models coupled with the
 rigid body dynamics. For instance, simulation of deformable objects requires
 additional state and ports to interact with externals systems such as
 visualization.

 Similar to the routine of adding multiple model instances in MultibodyPlant,
 users should add all the model instances they wish to add to a certain
 manager before the owning MultibodyPlant calls `Finalize()`. When `Finalize()`
 is invoked, MultibodyPlant will allocate the context resources for the state,
 cache and ports for each PhysicalModelManager it owns. After `Finalize()` is
 invoked, adding more model instances in any model managers that the
 MultibodyPlant owns is not allowed.

 @tparam_default_scalar */
template <typename T>
class PhysicalModelManager {
 public:
  explicit PhysicalModelManager(MultibodyPlant<T>* plant) : plant_(plant) {
    DRAKE_DEMAND(plant_ != nullptr);
  }

  virtual ~PhysicalModelManager() = default;

  /* MultibodyPlant calls this from within Finalize() to declare additional
   state, cache and ports. `DeclareContextResources()` will throw an
   exception when called before `Finalize()`.*/
  void DeclareContextResources() {
    ThrowIfNotFinalized(__func__);
    DoDeclareContextResources();
  }

  const MultibodyPlant<T>& plant() const { return *plant_; }

 protected:
  /* Derived class must override this to declare the state, cache and ports for
   its specific model. */
  virtual void DoDeclareContextResources() = 0;

  /* Helper method for throwing an exception within public methods that should
   not be called after the owning MultibodyPlant is Finalized. The invoking
   method should pass it's name so that the error message can include that
   detail. */
  void ThrowIfFinalized(const char* source_method) const {
    if (plant_->is_finalized()) {
      throw std::logic_error("Post-finalize calls to '" +
                             std::string(source_method) +
                             "()' are not allowed; calls to this method must "
                             "happen before the owning "
                             "MultibodyPlant calls Finalize().");
    }
  }

  /* Helper method for throwing an exception within public methods that should
   not be called before the owning MultibodyPlant is Finalized. The invoking
   method should pass it's name so that the error message can include that
   detail. */
  void ThrowIfNotFinalized(const char* source_method) const {
    if (!plant_->is_finalized()) {
      throw std::logic_error("Pre-finalize calls to '" +
                             std::string(source_method) +
                             "()' are not allowed; calls to this method must "
                             "happen after the owning "
                             "MultibodyPlant calls Finalize().");
    }
  }

  /* Protected LeafSystem methods exposed through MultibodyPlant. */
  systems::DiscreteStateIndex DeclareDiscreteState(
      const VectorX<T>& model_value);

  systems::LeafOutputPort<T>& DeclareAbstractOutputPort(
      std::string name,
      typename systems::LeafOutputPort<T>::AllocCallback alloc_function,
      typename systems::LeafOutputPort<T>::CalcCallback calc_function,
      std::set<systems::DependencyTicket> prerequisites_of_calc = {
          systems::System<T>::all_sources_ticket()});

  systems::LeafOutputPort<T>& DeclareVectorOutputPort(
      std::string name, const systems::BasicVector<T>& model_vector,
      typename systems::LeafOutputPort<T>::CalcVectorCallback
          vector_calc_function,
      std::set<systems::DependencyTicket> prerequisites_of_calc = {
          systems::System<T>::all_sources_ticket()});

 private:
  MultibodyPlant<T>* plant_{nullptr};
};
}  // namespace internal
}  // namespace multibody
}  // namespace drake
DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::internal::PhysicalModelManager);
