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

/* PhysicalModelManager provides the functionalities to extend the physical
 model of MultibodyPlant. Developers can derive from this PhysicalModelManager
 to incorporate additional discrete models coupled with the rigid body dynamics.
 For instance, simulation of deformable objects requires additional state and
 ports to interact with externals systems such as visualization. Similar to the
 routine of adding models in MultibodyPlant, users should add all the models
 they wish to add with a certain manager and then call `Finalize()`. After
 `Finalize()` is called, adding more models should not be allowed. When
 MultibodyPlant::Finalize() is invoked, MultibodyPlant will allocate the context
 resources for the state, cache and ports for each PhysicalModelManager it owns.
 At that moment, all the PhysicalModelManager must be finalized.

 @tparam_default_scalar */
template <typename T>
class PhysicalModelManager {
 public:
  explicit PhysicalModelManager(MultibodyPlant<T>* plant) : plant_(plant) {
    DRAKE_DEMAND(plant_ != nullptr);
  }

  virtual ~PhysicalModelManager() = default;

  /* MultibodyPlant calls this from within Finalize() to declare additional
   state, cache and ports. `DeclareStateCacheAndPorts()` will throw an
   exception when called before `Finalize()`.*/
  void DeclareStateCacheAndPorts() {
    DRAKE_THROW_UNLESS(finalized_);
    DoDeclareStateCacheAndPorts();
  }

  /* `Finalize()` should be called when all the models have been added. */
  void Finalize() {
    ThrowIfFinalized();
    finalized_ = true;
  }

  bool is_finalized() const { return finalized_; }

 protected:
  const MultibodyPlant<T>& plant() const { return *plant_; }

  /* Derived class must override this to declare the state, cache and ports for
   its specific model. */
  virtual void DoDeclareStateCacheAndPorts() = 0;

  void ThrowIfFinalized() const { DRAKE_THROW_UNLESS(!finalized_); }

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
  bool finalized_{false};
};
}  // namespace internal
}  // namespace multibody
}  // namespace drake
DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::internal::PhysicalModelManager);
