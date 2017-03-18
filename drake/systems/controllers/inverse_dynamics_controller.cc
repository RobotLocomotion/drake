#include "drake/systems/controllers/inverse_dynamics_controller.h"

#include <memory>
#include <utility>

#include "drake/systems/controllers/inverse_dynamics.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/adder.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/demultiplexer.h"
#include "drake/systems/primitives/pass_through.h"

namespace drake {
namespace systems {

template <typename T>
void InverseDynamicsController<T>::SetUp(const VectorX<T>& kp,
                                         const VectorX<T>& ki,
                                         const VectorX<T>& kd) {
  DiagramBuilder<T> builder;
  this->set_name("InverseDynamicsController");

  const RigidBodyTree<T>& robot = this->get_robot_for_control();
  DRAKE_DEMAND(robot.get_num_positions() == kp.size());
  DRAKE_DEMAND(robot.get_num_positions() == robot.get_num_velocities());
  DRAKE_DEMAND(robot.get_num_positions() == robot.get_num_actuators());

  const int dim = robot.get_num_velocities();

  /*
  (vd*)
         --------------------
                            |
  (q*, v*)                  |
         ---------> |   |   |
  (q, v)            |PID|   |
         ---------> |   | --+--> |                  |
             |                   | inverse dynamics | ---> torque
             ------------------> |                  |

  */

  // Adds a PID.
  pid_ = builder.template AddSystem<PidController<T>>(kp, ki, kd);

  // Adds inverse dynamics.
  auto inverse_dynamics =
      builder.template AddSystem<InverseDynamics<T>>(robot, false);

  // Redirects estimated state input into PID and inverse dynamics.
  auto pass_through = builder.template AddSystem<PassThrough<T>>(2 * dim);

  // Adds a adder to do PID's acceleration + reference acceleration.
  auto adder = builder.template AddSystem<Adder<T>>(2, dim);

  // Connects estimated state to PID.
  builder.Connect(pass_through->get_output_port(),
                  pid_->get_input_port_estimated_state());

  // Connects estimated state to inverse dynamics.
  builder.Connect(pass_through->get_output_port(),
                  inverse_dynamics->get_input_port_estimated_state());

  // Adds PID's output with reference acceleration
  builder.Connect(pid_->get_output_port_control(), adder->get_input_port(0));

  // Connects desired acceleration to inverse dynamics
  builder.Connect(adder->get_output_port(),
                  inverse_dynamics->get_input_port_desired_acceleration());

  // Exposes estimated state input port.
  int index = builder.ExportInput(pass_through->get_input_port());
  this->set_input_port_index_estimated_state(index);

  // Exposes reference state input port.
  index = builder.ExportInput(pid_->get_input_port_desired_state());
  this->set_input_port_index_desired_state(index);

  if (!has_reference_acceleration_) {
    // Uses a zero constant source for reference acceleration.
    auto zero_feedforward_acceleartion =
        builder.template AddSystem<ConstantVectorSource<double>>(
            VectorX<T>::Zero(robot.get_num_velocities()));
    builder.Connect(zero_feedforward_acceleartion->get_output_port(),
                    adder->get_input_port(1));
  } else {
    // Exposes reference acceleration input port.
    input_port_index_desired_acceleration_ =
        builder.ExportInput(adder->get_input_port(1));
  }

  // Exposes inverse dynamics' output torque port.
  index = builder.ExportOutput(inverse_dynamics->get_output_port_torque());
  this->set_output_port_index_control(index);

  builder.BuildInto(this);
}

template <typename T>
void InverseDynamicsController<T>::set_integral_value(
    Context<T>* context, const Eigen::Ref<const VectorX<T>>& value) const {
  Context<T>* pid_context =
      Diagram<T>::GetMutableSubsystemContext(context, pid_);
  pid_->set_integral_value(pid_context, value);
}

template <typename T>
InverseDynamicsController<T>::InverseDynamicsController(
    const std::string& model_path,
    std::shared_ptr<RigidBodyFrame<double>> world_offset, const VectorX<T>& kp,
    const VectorX<T>& ki, const VectorX<T>& kd, bool has_reference_acceleration)
    : ModelBasedController<T>(model_path, world_offset,
                              multibody::joints::kFixed),
      has_reference_acceleration_(has_reference_acceleration) {
  SetUp(kp, ki, kd);
}

template <typename T>
InverseDynamicsController<T>::InverseDynamicsController(
    const RigidBodyTree<T>& robot, const VectorX<T>& kp, const VectorX<T>& ki,
    const VectorX<T>& kd, bool has_reference_acceleration)
    : ModelBasedController<T>(robot),
      has_reference_acceleration_(has_reference_acceleration) {
  SetUp(kp, ki, kd);
}

template <typename T>
InverseDynamicsController<T>::InverseDynamicsController(
    std::unique_ptr<RigidBodyTree<T>> robot, const VectorX<T>& kp,
    const VectorX<T>& ki, const VectorX<T>& kd, bool has_reference_acceleration)
    : ModelBasedController<T>(std::move(robot)),
      has_reference_acceleration_(has_reference_acceleration) {
  SetUp(kp, ki, kd);
}

template class InverseDynamicsController<double>;

}  // namespace systems
}  // namespace drake
