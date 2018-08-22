#include "drake/examples/kuka_iiwa_arm/kuka_torque_controller.h"

#include <utility>
#include <vector>

#include "drake/systems/controllers/inverse_dynamics.h"
#include "drake/systems/controllers/pid_controller.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/adder.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/demultiplexer.h"
#include "drake/systems/primitives/multiplexer.h"
#include "drake/systems/primitives/pass_through.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

using drake::systems::DiagramBuilder;
using drake::systems::PassThrough;
using drake::systems::controllers::InverseDynamics;
using drake::systems::controllers::PidController;
using drake::systems::ConstantVectorSource;
using drake::systems::Demultiplexer;
using drake::systems::Multiplexer;
using drake::systems::Adder;

template <typename T>
void KukaTorqueController<T>::SetUp(const VectorX<double>& kp,
                                    const VectorX<double>& damping) {
  DiagramBuilder<T> builder;
  const RigidBodyTree<T>& tree = *robot_for_control_;
  DRAKE_DEMAND(tree.get_num_positions() == kp.size());
  DRAKE_DEMAND(tree.get_num_positions() == damping.size());

  const int dim = tree.get_num_positions();

  /*
  torque_in --------------------------
                                     |
  (q, v)    ------->|Gravity Comp|---+---> torque_out
               |                     |
               -------------->|   |  |
                              |PID|---
  (q*, v*)  ---|Mux/Demux|--->|   |
  */

  // Redirects estimated state input into PID and gravity compensation.
  auto pass_through = builder.template AddSystem<PassThrough<T>>(2 * dim);

  // Add gravity compensator.
  auto gravity_comp =
      builder.template AddSystem<InverseDynamics<T>>(tree, true);

  // Adds PID.
  const VectorX<double> ki = VectorX<double>::Zero(dim);
  auto pid = builder.template AddSystem<PidController<T>>(kp, ki, damping);

  // Adds constant zeros for desired velocity to simulate damping.
  auto zero_vel = builder.template AddSystem<ConstantVectorSource<T>>(
      VectorX<T>::Zero(tree.get_num_velocities()));

  // Adds Demultiplexer to pick off the desired position.
  auto demux = builder.template AddSystem<Demultiplexer<T>>(2 * dim, dim);

  // Adds Multiplexer to combine desired state
  std::vector<int> mux_input_sizes = {dim, dim};
  auto mux = builder.template AddSystem<Multiplexer>(mux_input_sizes);

  // Adds an adder to sum the gravity compensation, PID, and feedforward torque.
  auto adder = builder.template AddSystem<Adder<T>>(3, dim);

  // Connects the estimated state to the gravity compensator.
  builder.Connect(pass_through->get_output_port(),
                  gravity_comp->get_input_port_estimated_state());

  // Connects the estimated state to the PID controller.
  builder.Connect(pass_through->get_output_port(),
                  pid->get_input_port_estimated_state());

  // Concatenate the desired position with zero velocity.
  builder.Connect(demux->get_output_port(0), mux->get_input_port(0));
  builder.Connect(zero_vel->get_output_port(), mux->get_input_port(1));

  // Connects the desired state to the PID controller
  builder.Connect(mux->get_output_port(0), pid->get_input_port_desired_state());

  // Connects the gravity comp and PID torques to the adder.
  builder.Connect(pid->get_output_port(0), adder->get_input_port(1));
  builder.Connect(gravity_comp->get_output_port(0), adder->get_input_port(2));

  // Exposes the estimated state port.
  input_port_index_estimated_state_ =
      builder.ExportInput(pass_through->get_input_port());

  // Exposes the desired state port.
  input_port_index_desired_state_ =
      builder.ExportInput(demux->get_input_port(0));

  // Exposes the commanded torque port.
  input_port_index_commanded_torque_ =
      builder.ExportInput(adder->get_input_port(0));

  // Exposes controller output.
  output_port_index_control_ = builder.ExportOutput(adder->get_output_port());

  builder.BuildInto(this);
}

template <typename T>
KukaTorqueController<T>::KukaTorqueController(
    std::unique_ptr<RigidBodyTree<T>> tree, const VectorX<double>& kp,
    const VectorX<double>& damping) {
  robot_for_control_ = std::move(tree);
  SetUp(kp, damping);
}

template class KukaTorqueController<double>;

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
