#include "drake/systems/controllers/inverse_dynamics_controller.h"

#include <memory>
#include <utility>

#include "drake/systems/controllers/inverse_dynamics.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/adder.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/demultiplexer.h"

using drake::multibody::MultibodyPlant;

namespace drake {
namespace systems {
namespace controllers {

template <typename T>
void InverseDynamicsController<T>::SetUp(const VectorX<double>& kp,
                                         const VectorX<double>& ki,
                                         const VectorX<double>& kd) {
  DRAKE_DEMAND(multibody_plant_for_control_->is_finalized());

  DiagramBuilder<T> builder;
  auto inverse_dynamics = builder.template AddSystem<InverseDynamics<T>>(
      multibody_plant_for_control_, InverseDynamics<T>::kInverseDynamics);

  const int num_positions = multibody_plant_for_control_->num_positions();
  const int num_velocities = multibody_plant_for_control_->num_velocities();
  const int num_actuators = multibody_plant_for_control_->num_actuators();
  const int dim = kp.size();
  DRAKE_DEMAND(num_positions == dim);
  DRAKE_DEMAND(num_positions == num_velocities);
  DRAKE_DEMAND(num_positions == num_actuators);

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
  pid_ = builder.template AddSystem<PidController<T>>(kp, ki, kd);

  // Adds a adder to do PID's acceleration + reference acceleration.
  auto adder = builder.template AddSystem<Adder<T>>(2, dim);

  // Adds PID's output with reference acceleration
  builder.Connect(pid_->get_output_port_control(), adder->get_input_port(0));

  // Connects desired acceleration to inverse dynamics
  builder.Connect(adder->get_output_port(),
                  inverse_dynamics->get_input_port_desired_acceleration());


  // Exposes estimated state input port.
  // Connects estimated state to PID.
  input_port_index_estimated_state_ = builder.ExportInput(
      pid_->get_input_port_estimated_state(), "estimated_state");

  // Connects estimated state to inverse dynamics.
  builder.ConnectInput(input_port_index_estimated_state_,
                       inverse_dynamics->get_input_port_estimated_state());

  // Exposes reference state input port.
  input_port_index_desired_state_ = builder.ExportInput(
      pid_->get_input_port_desired_state(), "desired_state");

  if (!has_reference_acceleration_) {
    // Uses a zero constant source for reference acceleration.
    auto zero_feedforward_acceleration =
        builder.template AddSystem<ConstantVectorSource<T>>(
            VectorX<T>::Zero(dim));
    builder.Connect(zero_feedforward_acceleration->get_output_port(),
                    adder->get_input_port(1));
  } else {
    // Exposes reference acceleration input port.
    input_port_index_desired_acceleration_ =
        builder.ExportInput(adder->get_input_port(1), "desired_acceleration");
  }

  // Exposes inverse dynamics' output force port.
  output_port_index_control_ =
      builder.ExportOutput(inverse_dynamics->get_output_port_force(), "force");

  builder.BuildInto(this);
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
    const MultibodyPlant<T>& plant, const VectorX<double>& kp,
    const VectorX<double>& ki, const VectorX<double>& kd,
    bool has_reference_acceleration)
    : multibody_plant_for_control_(&plant),
      has_reference_acceleration_(has_reference_acceleration) {
  SetUp(kp, ki, kd);
}

template <typename T>
InverseDynamicsController<T>::InverseDynamicsController(
    std::unique_ptr<multibody::MultibodyPlant<T>> plant,
    const VectorX<double>& kp, const VectorX<double>& ki,
    const VectorX<double>& kd, bool has_reference_acceleration)
    : owned_plant_for_control_(std::move(plant)),
      multibody_plant_for_control_(owned_plant_for_control_.get()),
      has_reference_acceleration_(has_reference_acceleration) {
  SetUp(kp, ki, kd);
}

template <typename T>
InverseDynamicsController<T>::~InverseDynamicsController() = default;

template class InverseDynamicsController<double>;
// TODO(siyuan) template on autodiff.
// template class InverseDynamicsController<AutoDiffXd>;

}  // namespace controllers
}  // namespace systems
}  // namespace drake
