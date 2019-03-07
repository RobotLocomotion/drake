#include "drake/systems/controllers/rbt_inverse_dynamics_controller.h"

#include <memory>
#include <utility>

#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/controllers/rbt_inverse_dynamics.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/adder.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/demultiplexer.h"
#include "drake/systems/primitives/pass_through.h"

namespace drake {
namespace systems {
namespace controllers {
namespace rbt {

template <typename T>
void InverseDynamicsController<T>::SetUp(const VectorX<double>& kp,
    const VectorX<double>& ki, const VectorX<double>& kd,
    const InverseDynamics<T>& inverse_dynamics, DiagramBuilder<T>* builder) {
  const int dim = kp.size();

  /*
  (vd*)
         --------------------
                            |
  (q*, v*)                  |
         ---------> |   |   |
  (q, v)            |PID|   |
         ---------> |   | --+--> |                  |
             |                   | inverse dynamics | ---> force
             ------------------> |                  |

  */

  // Adds a PID.
  pid_ = builder->template AddSystem<PidController<T>>(kp, ki, kd);

  // Redirects estimated state input into PID and inverse dynamics.
  auto pass_through = builder->template AddSystem<PassThrough<T>>(2 * dim);

  // Adds a adder to do PID's acceleration + reference acceleration.
  auto adder = builder->template AddSystem<Adder<T>>(2, dim);

  // Connects estimated state to PID.
  builder->Connect(pass_through->get_output_port(),
                   pid_->get_input_port_estimated_state());

  // Connects estimated state to inverse dynamics.
  builder->Connect(pass_through->get_output_port(),
                   inverse_dynamics.get_input_port_estimated_state());

  // Adds PID's output with reference acceleration
  builder->Connect(pid_->get_output_port_control(), adder->get_input_port(0));

  // Connects desired acceleration to inverse dynamics
  builder->Connect(adder->get_output_port(),
                   inverse_dynamics.get_input_port_desired_acceleration());

  // Exposes estimated state input port.
  input_port_index_estimated_state_ =
      builder->ExportInput(pass_through->get_input_port());

  // Exposes reference state input port.
  input_port_index_desired_state_ =
      builder->ExportInput(pid_->get_input_port_desired_state());

  if (!has_reference_acceleration_) {
    // Uses a zero constant source for reference acceleration.
    auto zero_feedforward_acceleration =
        builder->template AddSystem<ConstantVectorSource<T>>(
            VectorX<T>::Zero(dim));
    builder->Connect(zero_feedforward_acceleration->get_output_port(),
                     adder->get_input_port(1));
  } else {
    // Exposes reference acceleration input port.
    input_port_index_desired_acceleration_ =
        builder->ExportInput(adder->get_input_port(1));
  }

  // Exposes inverse dynamics' output force port.
  output_port_index_control_ =
      builder->ExportOutput(inverse_dynamics.get_output_port_force());

  builder->BuildInto(this);
}

template <typename T>
void InverseDynamicsController<T>::set_integral_value(
    Context<T>* context, const Eigen::Ref<const VectorX<T>>& value) const {
  Context<T>& pid_context =
      Diagram<T>::GetMutableSubsystemContext(*pid_, context);
  pid_->set_integral_value(&pid_context, value);
}

template <typename T>
InverseDynamicsController<T>::InverseDynamicsController(
    std::unique_ptr<RigidBodyTree<T>> robot, const VectorX<double>& kp,
    const VectorX<double>& ki, const VectorX<double>& kd,
    bool has_reference_acceleration)
    : rigid_body_tree_for_control_(std::move(robot)),
      has_reference_acceleration_(has_reference_acceleration) {
  DiagramBuilder<T> builder;
  auto inverse_dynamics =
      builder.template AddSystem<InverseDynamics<T>>(
          rigid_body_tree_for_control_.get(), false);

  const int num_positions = rigid_body_tree_for_control_->get_num_positions();
  const int num_velocities = rigid_body_tree_for_control_->get_num_velocities();
  const int num_actuators = rigid_body_tree_for_control_->get_num_actuators();
  DRAKE_DEMAND(num_positions == kp.size());
  DRAKE_DEMAND(num_positions == num_velocities);
  DRAKE_DEMAND(num_positions == num_actuators);
  SetUp(kp, ki, kd, *inverse_dynamics, &builder);
}

// We need this in the *.cc file so that rigid_body_tree.h does not need to be
// included by our header file.
template <typename T>
InverseDynamicsController<T>::~InverseDynamicsController() = default;

template class InverseDynamicsController<double>;

}  // namespace rbt
}  // namespace controllers
}  // namespace systems
}  // namespace drake
