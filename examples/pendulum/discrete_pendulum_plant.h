#pragma once

#include <memory>

#include "drake/examples/pendulum/gen/pendulum_input.h"
#include "drake/examples/pendulum/gen/pendulum_params.h"
#include "drake/examples/pendulum/gen/pendulum_state.h"
#include "drake/examples/pendulum/pendulum_plant.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace pendulum {

/// DiscretePendulumPlant implements a discrete time system approximation of the
/// continuous time PendulumPlant. It contains a PendulumPlant object, which
/// provides the plant's time derivatives used in an explicit Euler update
/// to the continuous state. The time step for this update is provided at
/// construction time.
template <typename T>
class DiscretePendulumPlant final : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DiscretePendulumPlant);

  /** Constructs a default plant */
  explicit DiscretePendulumPlant(double time_step)
      : systems::LeafSystem<T>(systems::SystemTypeTag<DiscretePendulumPlant>{}),
        pend_plant_(),
        pend_plant_context_(pend_plant_.CreateDefaultContext()),
        time_step_(time_step) {
    if (time_step_ == 0) {
      throw std::logic_error(
          "time_step must be greater than zero. For continuous time "
          "pendulum plant use the PendulumPlant system directly.");
    }
    // Ensure time step is positive.
    if (time_step_ < 0) {
      throw std::logic_error("time step must be > 0");
    }

    // The input and output port.
    this->DeclareVectorInputPort(PendulumInput<T>());
    this->DeclareVectorOutputPort(PendulumState<T>(),
                                  &DiscretePendulumPlant::CopyStateOut);

    // Declare the discrete state held by this class.
    this->DeclareDiscreteState(PendulumState<T>());
    this->DeclarePeriodicDiscreteUpdateEvent(
        time_step_, 0 /* time offset */,
        &DiscretePendulumPlant::DoDiscreteStateUpdate);
    this->DeclareForcedDiscreteUpdateEvent(
        &DiscretePendulumPlant::DoDiscreteStateUpdate);
  }

  /// Scalar-converting copy constructor.  See @ref system_scalar_conversion.
  template <typename U>
  explicit DiscretePendulumPlant(const DiscretePendulumPlant<U>& p)
      : DiscretePendulumPlant(p.time_step()) {}

  ~DiscretePendulumPlant() {}

  static const PendulumState<T>& get_discrete_state(
      const systems::DiscreteValues<T>& dstate) {
    return dynamic_cast<const PendulumState<T>&>(dstate.get_vector());
  }

  static const PendulumState<T>& get_discrete_state(
      const systems::Context<T>& context) {
    return get_discrete_state(context.get_discrete_state());
  }

  static PendulumState<T>& get_mutable_discrete_state(
      systems::DiscreteValues<T>* dstate) {
    return dynamic_cast<PendulumState<T>&>(dstate->get_mutable_vector());
  }

  static PendulumState<T>& get_mutable_discrete_state(
      systems::Context<T>* context) {
    return get_mutable_discrete_state(&context->get_mutable_discrete_state());
  }

  const systems::InputPort<T>& get_actuation_input_port() const {
    return systems::System<T>::get_input_port(0);
  }

  T get_tau(const systems::Context<T>& context) const {
    return this->get_actuation_input_port().Eval(context)(0);
  }

  double time_step() const { return time_step_; }

 private:
  /* The period and forced discrete state update handler */
  systems::EventStatus DoDiscreteStateUpdate(
      const systems::Context<T>& context,
      systems::DiscreteValues<T>* discrete_value_out) const {
    // Get the current discrete state.
    const PendulumState<T>& discrete_state = get_discrete_state(context);

    // Get discrete_value_out as a PendulumState.
    PendulumState<T>& mutable_discrete_state =
        get_mutable_discrete_state(discrete_value_out);

    auto& mutable_continuous_state =
        dynamic_cast<PendulumState<T>&>(
            pend_plant_context_->get_mutable_continuous_state()
                .get_mutable_vector());
    mutable_continuous_state.set_theta(discrete_state.theta());
    mutable_continuous_state.set_thetadot(discrete_state.thetadot());

    // Set the actuation input port.
    PendulumInput<T> pend_input;
    pend_input.set_tau(get_tau(context));
    pend_plant_.get_actuation_input_port().FixValue(pend_plant_context_.get(),
                                                    pend_input);

    // Compute the state derivatives. PendulumPlant is time-invariant, so no
    // need to set time in pend_plant_context_.
    auto const& derivatives = dynamic_cast<const PendulumState<T>&>(
        pend_plant_.EvalTimeDerivatives(*pend_plant_context_).get_vector());

    // Update the discrete state using explicit Euler.
    mutable_discrete_state.set_theta(discrete_state.theta() +
                                     derivatives.theta() * time_step_);
    mutable_discrete_state.set_thetadot(discrete_state.thetadot() +
                                        derivatives.thetadot() * time_step_);

    return systems::EventStatus::Succeeded();
  }

  // This is the calculator method for the state output port.
  void CopyStateOut(const systems::Context<T>& context,
                    PendulumState<T>* output) const {
    output->set_value(get_discrete_state(context).get_value());
  }

  PendulumPlant<T> pend_plant_;  /* The continuous time plant */
  std::unique_ptr<systems::Context<T>> pend_plant_context_;

  // For a discrete system with periodic updates, time_step_ corresponds to the
  // period of those updates. More explicitly, here it represents the time step
  // used for an explicit Euler update.
  double time_step_{-1};
};

}  // namespace pendulum
}  // namespace examples
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::examples::pendulum::DiscretePendulumPlant)
