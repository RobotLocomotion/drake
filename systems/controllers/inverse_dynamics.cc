#include "drake/systems/controllers/inverse_dynamics.h"

#include <vector>

using drake::multibody::MultibodyForces;
using drake::multibody::MultibodyPlant;

namespace drake {
namespace systems {
namespace controllers {

template <typename T>
InverseDynamics<T>::InverseDynamics(const MultibodyPlant<T>* plant,
                                    const InverseDynamicsMode mode)
    : multibody_plant_(plant),
      mode_(mode),
      q_dim_(plant->num_positions()),
      v_dim_(plant->num_velocities()) {
  DRAKE_DEMAND(multibody_plant_ != nullptr);
  DRAKE_DEMAND(plant->is_finalized());

  input_port_index_state_ =
      this->DeclareInputPort(kUseDefaultName, kVectorValued, q_dim_ + v_dim_)
          .get_index();
  output_port_index_force_ =
      this->DeclareVectorOutputPort(kUseDefaultName, v_dim_,
                                    &InverseDynamics<T>::CalcOutputForce)
          .get_index();

  auto multibody_plant_context = multibody_plant_->CreateDefaultContext();
  // Gravity compensation mode requires velocities to be zero.
  if (this->is_pure_gravity_compensation()) {
    plant->SetVelocities(
        multibody_plant_context.get(),
        VectorX<T>::Zero(plant->num_velocities()));
  }

  // Declare cache entry for the multibody plant context.
  multibody_plant_context_cache_index_ =
      this->DeclareCacheEntry(
              "multibody_plant_context_cache",
              *multibody_plant_context,
              &InverseDynamics<T>::SetMultibodyContext,
              {this->input_port_ticket(
                  get_input_port_estimated_state().get_index())})
          .cache_index();

  // Declare external forces cache entry and desired acceleration input port if
  // this is doing inverse dynamics.
  if (!this->is_pure_gravity_compensation()) {
    external_forces_cache_index_ =
        this->DeclareCacheEntry("external_forces_cache",
                                MultibodyForces<T>(*plant),
                                &InverseDynamics<T>::CalcMultibodyForces,
                                {this->cache_entry_ticket(
                                    multibody_plant_context_cache_index_)})
            .cache_index();

    input_port_index_desired_acceleration_ =
        this->DeclareInputPort(kUseDefaultName, kVectorValued, v_dim_)
            .get_index();
  }
}

template <typename T>
InverseDynamics<T>::~InverseDynamics() = default;

template <typename T>
void InverseDynamics<T>::SetMultibodyContext(
    const Context<T>& context,
    Context<T>* multibody_plant_context) const {
  const VectorX<T>& x = get_input_port_estimated_state().Eval(context);

  if (this->is_pure_gravity_compensation()) {
    // Velocities remain zero, as set in the constructor, for pure gravity
    // compensation mode.
    const VectorX<T> q = x.head(multibody_plant_->num_positions());
    multibody_plant_->SetPositions(multibody_plant_context, q);
  } else {
    // Set the plant positions and velocities.
    multibody_plant_->SetPositionsAndVelocities(multibody_plant_context, x);
  }
}

template <typename T>
void InverseDynamics<T>::CalcMultibodyForces(
    const Context<T>& context, MultibodyForces<T>* cache_value) const {
  const auto& multibody_plant_context =
      this->get_cache_entry(multibody_plant_context_cache_index_)
          .template Eval<Context<T>>(context);

  multibody_plant_->CalcForceElementsContribution(multibody_plant_context,
                                                  cache_value);
}

template <typename T>
void InverseDynamics<T>::CalcOutputForce(const Context<T>& context,
                                         BasicVector<T>* output) const {
  auto& plant = *multibody_plant_;

  const auto& multibody_plant_context =
      this->get_cache_entry(multibody_plant_context_cache_index_)
          .template Eval<Context<T>>(context);

  if (this->is_pure_gravity_compensation()) {
    output->get_mutable_value() =
        -plant.CalcGravityGeneralizedForces(multibody_plant_context);
  } else {
    const auto& external_forces =
        this->get_cache_entry(external_forces_cache_index_)
            .template Eval<MultibodyForces<T>>(context);

    // Compute inverse dynamics.
    const VectorX<T>& desired_vd =
        get_input_port_desired_acceleration().Eval(context);
    output->get_mutable_value() = plant.CalcInverseDynamics(
        multibody_plant_context, desired_vd, external_forces);
  }
}

template class InverseDynamics<double>;
// TODO(siyuan) template on autodiff.
// template class InverseDynamics<AutoDiffXd>;

}  // namespace controllers
}  // namespace systems
}  // namespace drake
