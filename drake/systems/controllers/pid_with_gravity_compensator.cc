#include "drake/systems/controllers/pid_with_gravity_compensator.h"

#include <memory>
#include <utility>

#include "drake/systems/controllers/pid_controller.h"
#include "drake/systems/controllers/gravity_compensator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/demultiplexer.h"

namespace drake {
namespace systems {

template<typename T>
void PidWithGravityCompensator<T>::SetUp(
    const VectorX<T>& kp, const VectorX<T>& ki, const VectorX<T>& kd) {
  DiagramBuilder<T> builder;
  this->set_name("PidWithGravityCompensator");

  const RigidBodyTree<T>& robot = this->get_robot_for_control();

  // Adds a PID.
  int state_dim = robot.get_num_positions() + robot.get_num_velocities();
  int control_dim = robot.get_num_velocities();
  auto pid = builder.template AddSystem<PidController<T>>(
      state_dim, control_dim, kp, ki, kd);

  // Adds a gravity compensator.
  auto grav_comp = builder.template AddSystem<GravityCompensator<T>>(robot);

  // Adds a adder to do PID + gravity compensation.
  auto adder = builder.template AddSystem<Adder<T>>(2, control_dim);

  // Splits state into q for gravity compensator.
  auto pass_through = builder.template AddSystem<PassThrough<T>>(
      robot.get_num_positions() + robot.get_num_velocities());

  auto rbp_state_demux = builder.template AddSystem<Demultiplexer<T>>(
      robot.get_num_positions() + robot.get_num_velocities(),
      robot.get_num_positions());

  // Splits q from state, and connects it to gravity compensator.
  builder.Connect(pass_through->get_output_port(0),
      rbp_state_demux->get_input_port(0));
  builder.Connect(rbp_state_demux->get_output_port(0),
      grav_comp->get_input_port(0));

  // Connects state to PID.
  builder.Connect(pass_through->get_output_port(0),
      pid->get_measured_state_input_port());

  // Create an adder to sum the provided input with the output of the
  // controller.
  builder.Connect(pid->get_control_output_port(),
                  adder->get_input_port(0));
  builder.Connect(grav_comp->get_output_port(0),
                  adder->get_input_port(1));

  // Exposes measured state input port.
  builder.ExportInput(pass_through->get_input_port(0));

  // Exposes PID's reference state input port.
  builder.ExportInput(pid->get_desired_state_input_port());

  // Exposes output torque port.
  builder.ExportOutput(adder->get_output_port());

  builder.BuildInto(this);
}

template<typename T>
PidWithGravityCompensator<T>::PidWithGravityCompensator(
    const std::string& model_path,
    std::shared_ptr<RigidBodyFrame<double>> world_offset,
    const VectorX<T>& kp, const VectorX<T>& ki, const VectorX<T>& kd)
    : ModelBasedController<T>(model_path, world_offset) {
  SetUp(kp, ki, kd);
}

template<typename T>
PidWithGravityCompensator<T>::PidWithGravityCompensator(
    const RigidBodyTree<T>& robot,
    const VectorX<T>& kp, const VectorX<T>& ki, const VectorX<T>& kd)
    : ModelBasedController<T>(robot) {
  SetUp(kp, ki, kd);
}

template<typename T>
PidWithGravityCompensator<T>::PidWithGravityCompensator(
    std::unique_ptr<RigidBodyTree<T>> robot,
    const VectorX<T>& kp, const VectorX<T>& ki, const VectorX<T>& kd)
    : ModelBasedController<T>(std::move(robot)) {
  SetUp(kp, ki, kd);
}

template class PidWithGravityCompensator<double>;

}  // namespace systems
}  // namespace drake
