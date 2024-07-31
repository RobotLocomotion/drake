#include "drake/systems/controllers/inverse_dynamics.h"

#include <utility>
#include <vector>

using drake::multibody::MultibodyForces;
using drake::multibody::MultibodyPlant;

namespace drake {
namespace systems {
namespace controllers {

template <typename T>
InverseDynamics<T>::InverseDynamics(
    std::unique_ptr<multibody::MultibodyPlant<T>> owned_plant,
    const MultibodyPlant<T>* plant, const InverseDynamicsMode mode,
    const Context<T>* plant_context)
    : LeafSystem<T>(SystemTypeTag<InverseDynamics>{}),
      owned_plant_(std::move(owned_plant)),
      plant_(owned_plant_ ? owned_plant_.get() : plant),
      mode_(mode),
      q_dim_(plant_->num_positions()),
      v_dim_(plant_->num_velocities()) {
  // Check that only one of owned_plant and plant where set.
  DRAKE_DEMAND(owned_plant_ == nullptr || plant == nullptr);
  DRAKE_DEMAND(plant_ != nullptr);
  DRAKE_THROW_UNLESS(plant_->is_finalized());
  if (plant_context != nullptr) {
    plant_->ValidateContext(*plant_context);
  }

  estimated_state_ =
      this->DeclareInputPort("estimated_state", kVectorValued, q_dim_ + v_dim_)
          .get_index();
  // We declare the all_input_ports ticket so that GetDirectFeedthrough does
  // not try to cast to Symbolic for feedthrough evaluation.
  generalized_force_ =
      this->DeclareVectorOutputPort("generalized_force", v_dim_,
                                    &InverseDynamics<T>::CalcOutputForce,
                                    {this->all_input_ports_ticket()})
          .get_index();

  std::unique_ptr<Context<T>> model_plant_context =
      plant_context == nullptr ? plant_->CreateDefaultContext()
                               : plant_context->Clone();

  // Gravity compensation mode requires velocities to be zero.
  if (this->is_pure_gravity_compensation()) {
    plant_->SetVelocities(model_plant_context.get(),
                          VectorX<T>::Zero(plant_->num_velocities()));
  }

  // Declare cache entry for the multibody plant context.
  plant_context_cache_index_ =
      this->DeclareCacheEntry("plant_context_cache", *model_plant_context,
                              &InverseDynamics<T>::SetMultibodyContext,
                              {this->input_port_ticket(estimated_state_)})
          .cache_index();

  // Declare external forces cache entry and desired acceleration input port if
  // this is doing inverse dynamics.
  if (!this->is_pure_gravity_compensation()) {
    external_forces_cache_index_ =
        this->DeclareCacheEntry(
                "external_forces_cache", MultibodyForces<T>(*plant_),
                &InverseDynamics<T>::CalcMultibodyForces,
                {this->cache_entry_ticket(plant_context_cache_index_)})
            .cache_index();

    desired_acceleration_ =
        this->DeclareInputPort("desired_acceleration", kVectorValued, v_dim_)
            .get_index();
  }
}

template <typename T>
InverseDynamics<T>::InverseDynamics(const MultibodyPlant<T>* plant,
                                    const InverseDynamicsMode mode,
                                    const Context<T>* plant_context)
    : InverseDynamics(nullptr, plant, mode, plant_context) {}

template <typename T>
InverseDynamics<T>::InverseDynamics(
    std::unique_ptr<multibody::MultibodyPlant<T>> plant,
    const InverseDynamicsMode mode, const Context<T>* plant_context)
    : InverseDynamics(std::move(plant), nullptr, mode, plant_context) {}

template <typename T>
template <typename U>
typename InverseDynamics<T>::ScalarConversionData
InverseDynamics<T>::ScalarConvertHelper(const InverseDynamics<U>& other) {
  // Convert plant.
  auto this_plant = systems::System<U>::template ToScalarType<T>(*other.plant_);
  auto this_plant_context = this_plant->CreateDefaultContext();

  // Create context and connect required port.
  auto other_context = other.CreateDefaultContext();
  const int num_states = other.get_input_port_estimated_state().size();
  other.get_input_port_estimated_state().FixValue(
      other_context.get(),
      Eigen::Matrix<U, Eigen::Dynamic, 1>::Zero(num_states));

  // Convert plant context.
  const auto& other_plant_context =
      other.get_cache_entry(other.plant_context_cache_index_)
          .template Eval<Context<U>>(*other_context);
  this_plant_context->SetStateAndParametersFrom(other_plant_context);
  InverseDynamicsMode mode = other.is_pure_gravity_compensation()
                                 ? kGravityCompensation
                                 : kInverseDynamics;
  return ScalarConversionData{std::move(this_plant), mode,
                              std::move(this_plant_context)};
}

template <typename T>
template <typename U>
InverseDynamics<T>::InverseDynamics(const InverseDynamics<U>& other)
    : InverseDynamics(ScalarConvertHelper(other)) {}

template <typename T>
InverseDynamics<T>::InverseDynamics(ScalarConversionData&& data)
    : InverseDynamics(std::move(data.plant), data.mode,
                      data.plant_context.get()) {}

template <typename T>
InverseDynamics<T>::~InverseDynamics() = default;

template <typename T>
void InverseDynamics<T>::SetMultibodyContext(const Context<T>& context,
                                             Context<T>* plant_context) const {
  const VectorX<T>& x = get_input_port_estimated_state().Eval(context);

  if (this->is_pure_gravity_compensation()) {
    // Velocities remain zero, as set in the constructor, for pure gravity
    // compensation mode.
    const VectorX<T> q = x.head(plant_->num_positions());
    plant_->SetPositions(plant_context, q);
  } else {
    // Set the plant positions and velocities.
    plant_->SetPositionsAndVelocities(plant_context, x);
  }
}

template <typename T>
void InverseDynamics<T>::CalcMultibodyForces(
    const Context<T>& context, MultibodyForces<T>* cache_value) const {
  const auto& plant_context = this->get_cache_entry(plant_context_cache_index_)
                                  .template Eval<Context<T>>(context);

  plant_->CalcForceElementsContribution(plant_context, cache_value);
}

template <typename T>
void InverseDynamics<T>::CalcOutputForce(const Context<T>& context,
                                         BasicVector<T>* output) const {
  const auto& plant_context = this->get_cache_entry(plant_context_cache_index_)
                                  .template Eval<Context<T>>(context);

  if (this->is_pure_gravity_compensation()) {
    output->get_mutable_value() =
        -plant_->CalcGravityGeneralizedForces(plant_context);
  } else {
    const auto& external_forces =
        this->get_cache_entry(external_forces_cache_index_)
            .template Eval<MultibodyForces<T>>(context);

    // Compute inverse dynamics.
    const VectorX<T>& desired_vd =
        get_input_port_desired_acceleration().Eval(context);

    output->get_mutable_value() =
        plant_->CalcInverseDynamics(plant_context, desired_vd, external_forces);
  }
}

}  // namespace controllers
}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::controllers::InverseDynamics);
