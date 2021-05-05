#pragma once

#include <set>
#include <string>
#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace multibody {
namespace internal {
// TODO(xuchenhan-tri): Split this class into more fine-grained accessors that
// cater to more than just ContactComputationManager when there are more
// components that need access to MbP's internal methods.
/* This class is used to grant access to a selected collection of
 MultibodyPlant's private members and methods to ContactComputationManager. It
 achieves that by forwarding the invoked method to the associated
 MultibodyPlant. Users of this class can either own an instance of the class or
 privately inherit from this class to gain access to a particular instance of
 MultibodyPlant that the user specifies at construction.
 @tparam_default_scalar */
template <typename T>
class MultibodyPlantAccess {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MultibodyPlantAccess);

  /* Constructs MultibodyPlantAccess that gains access to the given `plant`. */
  explicit MultibodyPlantAccess(MultibodyPlant<T>* plant) : plant_(plant) {
    DRAKE_DEMAND(plant_ != nullptr);
  }

  // TODO(xuchenhan-tri): At the moment we are lumping all methods we need to
  // expose for experimenting with integration with the new contact solver and
  // softsim. The access to private methods should be better tailored/restricted
  // to each individual class.
  /*               Exposed MultibodyPlant private methods.                   */
  //@{
  const MultibodyTree<T>& internal_tree() const {
    return plant_->internal_tree();
  }

  systems::DiscreteStateIndex DeclareDiscreteState(
      const VectorX<T>& model_value) {
    return plant_->DeclareDiscreteState(model_value);
  }

  systems::LeafOutputPort<T>& DeclareAbstractOutputPort(
      std::string name,
      typename systems::LeafOutputPort<T>::AllocCallback alloc_function,
      typename systems::LeafOutputPort<T>::CalcCallback calc_function,
      std::set<systems::DependencyTicket> prerequisites_of_calc = {
          systems::System<T>::all_sources_ticket()}) {
    return plant_->DeclareAbstractOutputPort(
        std::move(name), std::move(alloc_function), std::move(calc_function),
        std::move(prerequisites_of_calc));
  }

  systems::LeafOutputPort<T>& DeclareVectorOutputPort(
      std::string name, const systems::BasicVector<T>& model_vector,
      typename systems::LeafOutputPort<T>::CalcVectorCallback
          vector_calc_function,
      std::set<systems::DependencyTicket> prerequisites_of_calc = {
          systems::System<T>::all_sources_ticket()}) {
    return plant_->DeclareVectorOutputPort(std::move(name), model_vector,
                                           std::move(vector_calc_function),
                                           std::move(prerequisites_of_calc));
  }

  const systems::OutputPort<T>& get_output_port(
      systems::OutputPortIndex output_port_index) const {
    return plant_->get_output_port(output_port_index);
  }

  const contact_solvers::internal::ContactSolverResults<T>&
  EvalContactSolverResults(const systems::Context<T>& context) const {
    return plant_->EvalContactSolverResults(context);
  }
  //@}

 private:
  MultibodyPlant<T>* plant_{nullptr};
};
}  // namespace internal
}  // namespace multibody
}  // namespace drake
