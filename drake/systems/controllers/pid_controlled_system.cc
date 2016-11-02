#include "drake/systems/controllers/pid_controlled_system.h"

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace systems {

template <typename T>
PidControlledSystem<T>::PidControlledSystem(
    std::unique_ptr<System<T>> plant,
    const T& Kp, const T& Ki, const T& Kd)
    : PidControlledSystem(std::move(plant), nullptr /* feedback selector */,
        Kp, Ki, Kd) {}

template <typename T>
PidControlledSystem<T>::PidControlledSystem(
    std::unique_ptr<System<T>> plant,
    const VectorX<T>& Kp, const VectorX<T>& Ki, const VectorX<T>& Kd)
    : PidControlledSystem(std::move(plant), nullptr /* feedback selector */,
        Kp, Ki, Kd) {}

template <typename T>
PidControlledSystem<T>::PidControlledSystem(
    std::unique_ptr<System<T>> plant,
    std::unique_ptr<MatrixGain<T>> feedback_selector,
    const T& Kp, const T& Ki, const T& Kd) {
  const VectorX<T> Kp_v =
      VectorX<T>::Ones(plant->get_input_port(0).get_size()) * Kp;
  const VectorX<T> Ki_v =
      VectorX<T>::Ones(plant->get_input_port(0).get_size()) * Ki;
  const VectorX<T> Kd_v =
      VectorX<T>::Ones(plant->get_input_port(0).get_size()) * Kd;
  Initialize(std::move(plant), std::move(feedback_selector), Kp_v, Ki_v, Kd_v);
}

template <typename T>
PidControlledSystem<T>::PidControlledSystem(
    std::unique_ptr<System<T>> plant,
    std::unique_ptr<MatrixGain<T>> feedback_selector,
    const VectorX<T>& Kp, const VectorX<T>& Ki, const VectorX<T>& Kd) {
  Initialize(std::move(plant), std::move(feedback_selector), Kp, Ki, Kd);
}

template <typename T>
void PidControlledSystem<T>::Initialize(
    std::unique_ptr<System<T>> plant,
    std::unique_ptr<MatrixGain<T>> feedback_selector,
    const VectorX<T>& Kp, const VectorX<T>& Ki, const VectorX<T>& Kd) {
  DRAKE_DEMAND(plant != nullptr);
  DiagramBuilder<T> builder;
  plant_ = builder.template AddSystem(std::move(plant));
  if (feedback_selector == nullptr) {
    // No feedback selector was provided. Create a GainMatrix containing an
    // identity matrix, which results in every element of the plant's output
    // port zero being used as the feedback signal to the PID controller.
    feedback_selector =
        std::make_unique<MatrixGain<T>>(plant_->get_output_port(0).get_size());
  }
  feedback_selector_ = builder.template AddSystem(std::move(feedback_selector));

  DRAKE_ASSERT(plant_->get_num_input_ports() >= 1);
  DRAKE_ASSERT(plant_->get_num_output_ports() >= 1);
  DRAKE_ASSERT(plant_->get_output_port(0).get_size() ==
               feedback_selector_->get_input_port().get_size());
  const int num_effort_commands = plant_->get_input_port(0).get_size();
  const int num_states = num_effort_commands * 2;

  DRAKE_ASSERT(feedback_selector_->get_output_port().get_size() == num_states);

  state_minus_target_ = builder.template AddSystem<Adder<T>>(
      2, num_states);
  controller_ = builder.template AddSystem<PidController<T>>(
      Kp, Ki, Kd);

  // Split the input into two signals one with the positions and one
  // with the velocities.
  error_demux_ = builder.template AddSystem<Demultiplexer<T>>(
      num_states, num_effort_commands);

  controller_inverter_ = builder.template AddSystem<Gain<T>>(
      -1.0, num_effort_commands);
  error_inverter_ = builder.template AddSystem<Gain<T>>(
      -1.0, num_states);

  // Create an adder to sum the provided input with the output of the
  // controller.
  plant_input_ = builder.template AddSystem<Adder<T>>(2, num_effort_commands);

  builder.Connect(error_inverter_->get_output_port(),
                  state_minus_target_->get_input_port(0));
  builder.Connect(plant_->get_output_port(0),
                  feedback_selector_->get_input_port());
  builder.Connect(feedback_selector_->get_output_port(),
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
                  plant_input_->get_input_port(0));

  builder.Connect(plant_input_->get_output_port(),
                  plant_->get_input_port(0));

  builder.ExportInput(plant_input_->get_input_port(1));
  builder.ExportInput(error_inverter_->get_input_port());
  builder.ExportOutput(plant_->get_output_port(0));
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
      VectorX<T>::Zero(plant_->get_input_port(0).get_size()));
}

template class DRAKE_EXPORT PidControlledSystem<double>;
template class DRAKE_EXPORT PidControlledSystem<AutoDiffXd>;

}  // namespace systems
}  // namespace drake
