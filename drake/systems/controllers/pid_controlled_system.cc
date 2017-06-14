#include "drake/systems/controllers/pid_controlled_system.h"

#include "drake/common/autodiff_overloads.h"
#include "drake/common/drake_assert.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/systems/primitives/saturation.h"

namespace drake {
namespace systems {

template <typename T>
PidControlledSystem<T>::PidControlledSystem(std::unique_ptr<System<T>> plant,
                                            double Kp, double Ki, double Kd) {
  const int input_size = plant->get_input_port(0).size();
  const Eigen::VectorXd Kp_v = Eigen::VectorXd::Ones(input_size) * Kp;
  const Eigen::VectorXd Ki_v = Eigen::VectorXd::Ones(input_size) * Ki;
  const Eigen::VectorXd Kd_v = Eigen::VectorXd::Ones(input_size) * Kd;
  const MatrixX<double> selector =
    MatrixX<double>::Identity(plant->get_input_port(0).size() * 2,
                              plant->get_input_port(0).size() * 2);
  Initialize(std::move(plant), selector, Kp_v, Ki_v, Kd_v);
}

template <typename T>
PidControlledSystem<T>::PidControlledSystem(std::unique_ptr<System<T>> plant,
                                            const Eigen::VectorXd& Kp,
                                            const Eigen::VectorXd& Ki,
                                            const Eigen::VectorXd& Kd)
    : PidControlledSystem(
          std::move(plant),
          MatrixX<double>::Identity(2 * Kp.size(), 2 * Kp.size()), Kp, Ki, Kd) {
}

template <typename T>
PidControlledSystem<T>::PidControlledSystem(
    std::unique_ptr<System<T>> plant,
    const MatrixX<double>& feedback_selector, double Kp, double Ki,
    double Kd) {
  const int input_size = plant->get_input_port(0).size();
  const Eigen::VectorXd Kp_v = Eigen::VectorXd::Ones(input_size) * Kp;
  const Eigen::VectorXd Ki_v = Eigen::VectorXd::Ones(input_size) * Ki;
  const Eigen::VectorXd Kd_v = Eigen::VectorXd::Ones(input_size) * Kd;
  Initialize(std::move(plant), feedback_selector, Kp_v, Ki_v, Kd_v);
}

template <typename T>
PidControlledSystem<T>::PidControlledSystem(
    std::unique_ptr<System<T>> plant,
    const MatrixX<double>& feedback_selector, const Eigen::VectorXd& Kp,
    const Eigen::VectorXd& Ki, const Eigen::VectorXd& Kd) {
  Initialize(std::move(plant), feedback_selector, Kp, Ki, Kd);
}

template <typename T>
void PidControlledSystem<T>::Initialize(
    std::unique_ptr<System<T>> plant,
    const MatrixX<double>& feedback_selector, const Eigen::VectorXd& Kp,
    const Eigen::VectorXd& Ki, const Eigen::VectorXd& Kd) {
  DRAKE_DEMAND(plant != nullptr);

  if (plant->get_name().empty()) {
    plant->set_name("plant");
  }

  DiagramBuilder<T> builder;
  plant_ = builder.template AddSystem(std::move(plant));
  DRAKE_ASSERT(plant_->get_num_input_ports() >= 1);
  DRAKE_ASSERT(plant_->get_num_output_ports() >= 1);

  auto input_ports =
      ConnectController(plant_->get_input_port(0), plant_->get_output_port(0),
                        feedback_selector, Kp, Ki, Kd, &builder);

  builder.ExportInput(input_ports.control_input_port);
  builder.ExportInput(input_ports.state_input_port);
  builder.ExportOutput(plant_->get_output_port(0));
  builder.BuildInto(this);
}

template <typename T>
typename PidControlledSystem<T>::ConnectResult
PidControlledSystem<T>::ConnectController(
    const InputPortDescriptor<T>& plant_input,
    const OutputPort<T>& plant_output,
    const MatrixX<double>& feedback_selector, const Eigen::VectorXd& Kp,
    const Eigen::VectorXd& Ki, const Eigen::VectorXd& Kd,
    DiagramBuilder<T>* builder) {
  auto controller = builder->template AddSystem<PidController<T>>(
      feedback_selector,
      Kp, Ki, Kd);
  controller->set_name("pid_controller");

  auto plant_input_adder =
      builder->template AddSystem<Adder<T>>(2, plant_input.size());
  plant_input_adder->set_name("input_adder");

  builder->Connect(plant_output, controller->get_input_port_estimated_state());

  builder->Connect(controller->get_output_port_control(),
                   plant_input_adder->get_input_port(0));

  builder->Connect(plant_input_adder->get_output_port(), plant_input);

  return ConnectResult{
      plant_input_adder->get_input_port(1),
      controller->get_input_port_desired_state()};
}

template <typename T>
typename PidControlledSystem<T>::ConnectResult
PidControlledSystem<T>::ConnectController(
    const InputPortDescriptor<T>& plant_input,
    const OutputPort<T>& plant_output,
    const Eigen::VectorXd& Kp, const Eigen::VectorXd& Ki,
    const Eigen::VectorXd& Kd,
    DiagramBuilder<T>* builder) {
  return ConnectController(plant_input, plant_output,
      MatrixX<double>::Identity(plant_output.size(), plant_output.size()),
      Kp, Ki, Kd, builder);
}

template <typename T>
typename PidControlledSystem<T>::ConnectResult
PidControlledSystem<T>::ConnectControllerWithInputSaturation(
    const InputPortDescriptor<T>& plant_input,
    const OutputPort<T>& plant_output,
    const MatrixX<double>& feedback_selector, const Eigen::VectorXd& Kp,
    const Eigen::VectorXd& Ki, const Eigen::VectorXd& Kd,
    const VectorX<T>& min_plant_input, const VectorX<T>& max_plant_input,
    DiagramBuilder<T>* builder) {
  auto saturation = builder->template AddSystem<Saturation<T>>(
      min_plant_input, max_plant_input);
  saturation->set_name("saturation");

  builder->Connect(saturation->get_output_port(), plant_input);

  return
    PidControlledSystem<T>::ConnectController(saturation->get_input_port(),
    plant_output, feedback_selector, Kp, Ki, Kd, builder);
}

template <typename T>
typename PidControlledSystem<T>::ConnectResult
PidControlledSystem<T>::ConnectControllerWithInputSaturation(
    const InputPortDescriptor<T>& plant_input,
    const OutputPort<T>& plant_output,
    const Eigen::VectorXd& Kp, const Eigen::VectorXd& Ki,
    const Eigen::VectorXd& Kd, const VectorX<T>& min_plant_input,
    const VectorX<T>& max_plant_input, DiagramBuilder<T>* builder) {
  return ConnectControllerWithInputSaturation(plant_input, plant_output,
      MatrixX<double>::Identity(plant_output.size(), plant_output.size()),
      Kp, Ki, Kd, min_plant_input, max_plant_input, builder);
}

template <typename T>
PidControlledSystem<T>::~PidControlledSystem() {}

template class PidControlledSystem<double>;
template class PidControlledSystem<AutoDiffXd>;

}  // namespace systems
}  // namespace drake
