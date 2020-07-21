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
  DRAKE_DEMAND(multibody_plant_);
  DRAKE_DEMAND(plant->is_finalized());

  input_port_index_state_ =
      this->DeclareInputPort(kVectorValued, q_dim_ + v_dim_).get_index();
  output_port_index_force_ =
      this->DeclareVectorOutputPort(BasicVector<T>(v_dim_),
                                    &InverseDynamics<T>::CalcOutputForce)
          .get_index();

  // Declare cache entry for the multibody plant context.
  multibody_plant_context_cache_index_ =
      this->DeclareCacheEntry(
              "multibody_plant_context_cache",
              [this]() { return this->MakeMultibodyContext(); },
              [this](const ContextBase& context, AbstractValue* cache_value) {
                this->SetMultibodyContext(
                    dynamic_cast<const Context<T>&>(context), cache_value);
              },
              {this->input_port_ticket(
                  get_input_port_estimated_state().get_index())})
          .cache_index();

  // Declare external forces cache entry and desired acceleration input port if
  // this is doing inverse dynamics.
  if (!this->is_pure_gravity_compensation()) {
    external_forces_cache_index_ =
        this->DeclareCacheEntry("external_forces_cache",
                                &InverseDynamics<T>::MakeMultibodyForces,
                                &InverseDynamics<T>::CalcMultibodyForces,
                                {this->cache_entry_ticket(
                                    multibody_plant_context_cache_index_)})
            .cache_index();

    input_port_index_desired_acceleration_ =
        this->DeclareInputPort(kVectorValued, v_dim_).get_index();
  }
}

template <typename T>
InverseDynamics<T>::~InverseDynamics() = default;

template <typename T>
std::unique_ptr<AbstractValue> InverseDynamics<T>::MakeMultibodyContext()
    const {
  auto multibody_plant_context = multibody_plant_->CreateDefaultContext();
  // Gravity compensation mode requires velocities to be zero.
  if (this->is_pure_gravity_compensation()) {
    multibody_plant_->SetVelocities(
        multibody_plant_context.get(),
        VectorX<T>::Zero(multibody_plant_->num_velocities()));
  }
  return AbstractValue::Make(*multibody_plant_context);
}

template <typename T>
void InverseDynamics<T>::SetMultibodyContext(const Context<T>& context,
                                             AbstractValue* cache_value) const {
  auto& multibody_plant_context =
      cache_value->template get_mutable_value<Context<T>>();

  Eigen::VectorBlock<const VectorX<T>> x =
      get_input_port_estimated_state().Eval(context);

  if (this->is_pure_gravity_compensation()) {
    // Velocities remain zero, as set in the cache allocation
    // function, for pure gravity compensation mode.
    multibody_plant_->SetPositions(&multibody_plant_context,
                                   x.head(multibody_plant_->num_positions()));
  } else {
    // Set the plant positions and velocities.
    multibody_plant_->SetPositionsAndVelocities(&multibody_plant_context, x);
  }
}

template <typename T>
MultibodyForces<T> InverseDynamics<T>::MakeMultibodyForces() const {
  return MultibodyForces<T>(*multibody_plant_);
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
    Eigen::VectorBlock<const VectorX<T>> desired_vd =
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
