#include "drake/systems/controllers/pid_controlled_system.h"

#include "drake/common/autodiff_overloads.h"
#include "drake/common/drake_assert.h"
#include "drake/common/eigen_autodiff_types.h"

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
  DRAKE_ASSERT(plant_->get_num_input_ports() >= 1);
  DRAKE_ASSERT(plant_->get_num_output_ports() >= 1);

  auto input_ports = ConnectController(
      plant_->get_input_port(0), plant_->get_output_port(0),
      std::move(feedback_selector), Kp, Ki, Kd, &builder);

  builder.ExportInput(input_ports.first);
  builder.ExportInput(input_ports.second);
  builder.ExportOutput(plant_->get_output_port(0));
  builder.BuildInto(this);
}

template <typename T>
std::pair<const SystemPortDescriptor<T>,
          const SystemPortDescriptor<T>>
    PidControlledSystem<T>::ConnectController(
        const SystemPortDescriptor<T>& plant_input,
        const SystemPortDescriptor<T>& plant_output,
        std::unique_ptr<MatrixGain<T>> feedback_selector,
        const VectorX<T>& Kp, const VectorX<T>& Ki,
        const VectorX<T>& Kd,
        DiagramBuilder<T>* builder) {
  if (feedback_selector == nullptr) {
    // No feedback selector was provided. Create a GainMatrix containing an
    // identity matrix, which results in every element of the plant's output
    // port zero being used as the feedback signal to the PID controller.
    feedback_selector =
        std::make_unique<MatrixGain<T>>(plant_output.get_size());
  }
  auto feedback_selector_p =
      builder->template AddSystem(std::move(feedback_selector));

  DRAKE_ASSERT(plant_output.get_size() ==
               feedback_selector_p->get_input_port().get_size());
  const int num_effort_commands = plant_input.get_size();
  const int num_states = num_effort_commands * 2;

  DRAKE_ASSERT(feedback_selector_p->get_output_port().get_size() == num_states);

  auto state_minus_target = builder->template AddSystem<Adder<T>>(
      2, num_states);
  auto controller = builder->template AddSystem<PidController<T>>(
      Kp, Ki, Kd);

  // Split the input into two signals one with the positions and one
  // with the velocities.
  auto error_demux = builder->template AddSystem<Demultiplexer<T>>(
      num_states, num_effort_commands);

  auto controller_inverter = builder->template AddSystem<Gain<T>>(
      -1.0, num_effort_commands);
  auto error_inverter = builder->template AddSystem<Gain<T>>(
      -1.0, num_states);

  // Create an adder to sum the provided input with the output of the
  // controller.
  auto plant_input_adder =
      builder->template AddSystem<Adder<T>>(2, num_effort_commands);

  builder->Connect(error_inverter->get_output_port(),
                   state_minus_target->get_input_port(0));
  builder->Connect(plant_output,
                   feedback_selector_p->get_input_port());
  builder->Connect(feedback_selector_p->get_output_port(),
                   state_minus_target->get_input_port(1));

  // Splits the error signal into positions and velocities components.
  builder->Connect(state_minus_target->get_output_port(),
                   error_demux->get_input_port(0));

  // Connects PID controller.
  builder->Connect(error_demux->get_output_port(0),
                   controller->get_error_port());
  builder->Connect(error_demux->get_output_port(1),
                   controller->get_error_derivative_port());
  // Adds feedback.
  builder->Connect(controller->get_output_port(0),
                   controller_inverter->get_input_port());
  builder->Connect(controller_inverter->get_output_port(),
                   plant_input_adder->get_input_port(0));

  builder->Connect(plant_input_adder->get_output_port(),
                   plant_input);

  return std::make_pair(plant_input_adder->get_input_port(1),
                        error_inverter->get_input_port());
}

template <typename T>
PidControlledSystem<T>::~PidControlledSystem() {}

template class PidControlledSystem<double>;
template class PidControlledSystem<AutoDiffXd>;

}  // namespace systems
}  // namespace drake
