#include "drake/systems/controllers/pid_controlled_system.h"

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/systems/primitives/saturation.h"

namespace drake {
namespace systems {
namespace controllers {

template <typename T>
PidControlledSystem<T>::PidControlledSystem(std::unique_ptr<System<T>> plant,
                                            double Kp, double Ki, double Kd,
                                            int state_output_port_index,
                                            int plant_input_port_index)
    : state_output_port_index_(state_output_port_index),
      plant_input_port_index_{plant_input_port_index} {
  const int input_size = plant->get_input_port(plant_input_port_index_).size();
  const Eigen::VectorXd Kp_v = Eigen::VectorXd::Ones(input_size) * Kp;
  const Eigen::VectorXd Ki_v = Eigen::VectorXd::Ones(input_size) * Ki;
  const Eigen::VectorXd Kd_v = Eigen::VectorXd::Ones(input_size) * Kd;
  const MatrixX<double> selector = MatrixX<double>::Identity(
      plant->get_input_port(plant_input_port_index_).size() * 2,
      plant->get_input_port(plant_input_port_index_).size() * 2);
  Initialize(std::move(plant), selector, Kp_v, Ki_v, Kd_v);
}

template <typename T>
PidControlledSystem<T>::PidControlledSystem(std::unique_ptr<System<T>> plant,
                                            const Eigen::VectorXd& Kp,
                                            const Eigen::VectorXd& Ki,
                                            const Eigen::VectorXd& Kd,
                                            int state_output_port_index,
                                            int plant_input_port_index)
    : PidControlledSystem(
          std::move(plant),
          MatrixX<double>::Identity(2 * Kp.size(), 2 * Kp.size()), Kp, Ki, Kd,
          state_output_port_index, plant_input_port_index) {}

template <typename T>
PidControlledSystem<T>::PidControlledSystem(
    std::unique_ptr<System<T>> plant, const MatrixX<double>& feedback_selector,
    double Kp, double Ki, double Kd, int state_output_port_index,
    int plant_input_port_index)
    : state_output_port_index_(state_output_port_index),
      plant_input_port_index_{plant_input_port_index} {
  const int input_size = plant->get_input_port(plant_input_port_index_).size();
  const Eigen::VectorXd Kp_v = Eigen::VectorXd::Ones(input_size) * Kp;
  const Eigen::VectorXd Ki_v = Eigen::VectorXd::Ones(input_size) * Ki;
  const Eigen::VectorXd Kd_v = Eigen::VectorXd::Ones(input_size) * Kd;
  Initialize(std::move(plant), feedback_selector, Kp_v, Ki_v, Kd_v);
}

template <typename T>
PidControlledSystem<T>::PidControlledSystem(
    std::unique_ptr<System<T>> plant, const MatrixX<double>& feedback_selector,
    const Eigen::VectorXd& Kp, const Eigen::VectorXd& Ki,
    const Eigen::VectorXd& Kd, int state_output_port_index,
    int plant_input_port_index)
    : state_output_port_index_(state_output_port_index),
      plant_input_port_index_{plant_input_port_index} {
  Initialize(std::move(plant), feedback_selector, Kp, Ki, Kd);
}

template <typename T>
void PidControlledSystem<T>::Initialize(
    std::unique_ptr<System<T>> plant, const MatrixX<double>& feedback_selector,
    const Eigen::VectorXd& Kp, const Eigen::VectorXd& Ki,
    const Eigen::VectorXd& Kd) {
  DRAKE_DEMAND(plant != nullptr);

  DiagramBuilder<T> builder;
  plant_ = builder.template AddSystem(std::move(plant));
  DRAKE_ASSERT(plant_->num_input_ports() >= 1);
  DRAKE_ASSERT(plant_->num_output_ports() >= 1);
  // state_output_port_index_ will be checked by the get_output_port call below.

  auto input_ports =
      ConnectController(plant_->get_input_port(plant_input_port_index_),
                        plant_->get_output_port(state_output_port_index_),
                        feedback_selector, Kp, Ki, Kd, &builder);

  builder.ExportInput(input_ports.control_input_port);
  builder.ExportInput(input_ports.state_input_port);

  for (int i=0; i < plant_->num_output_ports(); i++) {
    builder.ExportOutput(plant_->get_output_port(i));
  }
  builder.BuildInto(this);
}

template <typename T>
typename PidControlledSystem<T>::ConnectResult
PidControlledSystem<T>::ConnectController(
    const InputPort<T>& plant_input,
    const OutputPort<T>& plant_output,
    const MatrixX<double>& feedback_selector, const Eigen::VectorXd& Kp,
    const Eigen::VectorXd& Ki, const Eigen::VectorXd& Kd,
    DiagramBuilder<T>* builder) {
  auto controller = builder->template AddSystem<PidController<T>>(
      feedback_selector,
      Kp, Ki, Kd);

  auto plant_input_adder =
      builder->template AddSystem<Adder<T>>(2, plant_input.size());

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
    const InputPort<T>& plant_input,
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
    const InputPort<T>& plant_input,
    const OutputPort<T>& plant_output,
    const MatrixX<double>& feedback_selector, const Eigen::VectorXd& Kp,
    const Eigen::VectorXd& Ki, const Eigen::VectorXd& Kd,
    const VectorX<T>& min_plant_input, const VectorX<T>& max_plant_input,
    DiagramBuilder<T>* builder) {
  auto saturation = builder->template AddSystem<Saturation<T>>(
      min_plant_input, max_plant_input);
  builder->Connect(saturation->get_output_port(), plant_input);

  return
    PidControlledSystem<T>::ConnectController(saturation->get_input_port(),
    plant_output, feedback_selector, Kp, Ki, Kd, builder);
}

template <typename T>
typename PidControlledSystem<T>::ConnectResult
PidControlledSystem<T>::ConnectControllerWithInputSaturation(
    const InputPort<T>& plant_input,
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

}  // namespace controllers
}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::systems::controllers::PidControlledSystem)
