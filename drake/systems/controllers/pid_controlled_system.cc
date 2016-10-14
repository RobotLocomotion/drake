#include "pid_controlled_system.h"

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace systems {

template <typename T>
PidControlledSystem<T>::PidControlledSystem(
    std::unique_ptr<System<T>> system,
    const T& Kp, const T& Ki, const T& Kd) {
  DiagramBuilder<T> builder;
  system_ = builder.template AddSystem(std::move(system));

  DRAKE_ASSERT(system_->get_num_input_ports() >= 1);
  DRAKE_ASSERT(system_->get_num_output_ports() >= 1);
  const int num_positions = system_->get_input_port(0).get_size();
  const int num_states = num_positions * 2;

  // TODO(sam.creasey) It would be nice to be able to handle the
  // existence of extra values in the output port which are discarded
  // because they're not relevant to the control we're applying here
  // (see PidControlledSpringMassSystem which discards the energy
  // information from the spring mass system).  Unfortunately, I can't
  // find an easy way to do this now, and the current implementation
  // is sufficient for most uses (including RigidBodyPlant).
  DRAKE_ASSERT(system_->get_output_port(0).get_size() == num_states);

  state_minus_target_ = builder.template AddSystem<Adder<T>>(
      2, num_states);
  controller_ = builder.template AddSystem<PidController<T>>(
      Kp, Ki, Kd, num_positions);

  // Split the input into two signals one with the positions and one
  // with the velocities.
  error_demux_ = builder.template AddSystem<Demultiplexer<T>>(
      num_states, num_positions);

  controller_inverter_ = builder.template AddSystem<Gain<T>>(
      -1.0, num_positions);
  error_inverter_ = builder.template AddSystem<Gain<T>>(
      -1.0, num_states);

  // Create an adder to sum the provided input with the output of the
  // controller.
  system_input_ = builder.template AddSystem<Adder<T>>(
      2, num_positions);

  builder.Connect(error_inverter_->get_output_port(),
                  state_minus_target_->get_input_port(0));
  builder.Connect(system_->get_output_port(0),
                  state_minus_target_->get_input_port(1));

  // Splits the error signal into positions and velocities components.
  builder.Connect(state_minus_target_->get_output_port(),
                  error_demux_->get_input_port(0));

  // Connects PID controller.
  builder.Connect(error_demux_->get_output_port(0),
                  controller_->get_error_port());
  builder.Connect(error_demux_->get_output_port(1),
                  controller_->get_error_derivative_port());
  // Adds feedback.
  builder.Connect(controller_->get_output_port(0),
                  controller_inverter_->get_input_port());
  builder.Connect(controller_inverter_->get_output_port(),
                  system_input_->get_input_port(0));

  builder.Connect(system_input_->get_output_port(),
                  system_->get_input_port(0));

  builder.ExportInput(system_input_->get_input_port(1));
  builder.ExportInput(error_inverter_->get_input_port());
  builder.ExportOutput(system_->get_output_port(0));
  builder.BuildInto(this);
}


template <typename T>
PidControlledSystem<T>::~PidControlledSystem() {}

template <typename T>
void PidControlledSystem<T>::SetDefaultState(
    Context<T>* context) const {
  Context<T>* controller_context =
      Diagram<T>::GetMutableSubsystemContext(context, controller_);
  controller_->set_integral_value(
      controller_context,
      VectorX<T>::Zero(system_->get_input_port(0).get_size()));
}

template class DRAKE_EXPORT PidControlledSystem<double>;
template class DRAKE_EXPORT PidControlledSystem<AutoDiffXd>;

}  // namespace systems
}  // namespace drake
