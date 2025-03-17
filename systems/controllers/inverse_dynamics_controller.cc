#include "drake/systems/controllers/inverse_dynamics_controller.h"

#include <memory>
#include <utility>

#include "drake/systems/controllers/inverse_dynamics.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/adder.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/demultiplexer.h"
#include "drake/systems/primitives/sparse_matrix_gain.h"

using drake::multibody::MultibodyPlant;

namespace drake {
namespace systems {
namespace controllers {

template <typename T>
void InverseDynamicsController<T>::SetUp(
    std::unique_ptr<multibody::MultibodyPlant<T>> owned_plant,
    const VectorX<double>& kp, const VectorX<double>& ki,
    const VectorX<double>& kd, const Context<T>* plant_context) {
  DRAKE_DEMAND(multibody_plant_for_control_->is_finalized());

  DiagramBuilder<T> builder;
  InverseDynamics<T>* inverse_dynamics{};
  if (owned_plant) {
    inverse_dynamics = builder.template AddNamedSystem<InverseDynamics<T>>(
        "InverseDynamics", std::move(owned_plant),
        InverseDynamics<T>::kInverseDynamics, plant_context);
  } else {
    inverse_dynamics = builder.template AddNamedSystem<InverseDynamics<T>>(
        "InverseDynamics", multibody_plant_for_control_,
        InverseDynamics<T>::kInverseDynamics, plant_context);
  }

  const int num_positions = multibody_plant_for_control_->num_positions();
  const int num_velocities = multibody_plant_for_control_->num_velocities();
  const int num_actuators = multibody_plant_for_control_->num_actuators();
  const int dim = kp.size();
  DRAKE_DEMAND(num_positions == dim);
  if (num_positions != num_actuators) {
    throw std::runtime_error(fmt::format(R"""(
Your plant has {} positions, but only {} actuators.

InverseDynamicsController (currently) only supports fully-actuated robots. For
instance, you cannot use this directly if your robot/model has an unactuated 
floating base.

Note that commonly, the MultibodyPlant used for control is not the same
one used for simulation; the simulation model might contain the robot and also
some objects in the world which the controller does not have direct
observations of nor control over. See 
https://stackoverflow.com/q/75917723/9510020 for some further discussion.)""",
                                         num_positions, num_actuators));
  }
  if (num_positions != num_velocities) {
    throw std::runtime_error(fmt::format(R"""(
Your plant has {} positions, but {} velocities. Likely you have a quaternion 
floating base. InverseDynamicsController currently requires that the 
number of positions matches the number of velocities, and does not support 
joints modeled with quaternions.)""",
                                         num_positions, num_velocities));
  }

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
  pid_ = builder.template AddNamedSystem<PidController<T>>("pid", kp, ki, kd);

  // Adds a adder to do PID's acceleration + reference acceleration.
  auto adder = builder.template AddNamedSystem<Adder<T>>("+", 2, dim);

  // Adds PID's output with reference acceleration
  builder.Connect(pid_->get_output_port_control(), adder->get_input_port(0));

  // Connects desired acceleration to inverse dynamics
  builder.Connect(adder->get_output_port(),
                  inverse_dynamics->get_input_port_desired_acceleration());

  // Exposes estimated state input port.
  // Connects estimated state to PID.
  estimated_state_ = builder.ExportInput(pid_->get_input_port_estimated_state(),
                                         "estimated_state");

  // Connects estimated state to inverse dynamics.
  builder.ConnectInput(estimated_state_,
                       inverse_dynamics->get_input_port_estimated_state());

  // Exposes reference state input port.
  desired_state_ = builder.ExportInput(pid_->get_input_port_desired_state(),
                                       "desired_state");

  if (!has_reference_acceleration_) {
    // Uses a zero constant source for reference acceleration.
    auto zero_feedforward_acceleration =
        builder.template AddNamedSystem<ConstantVectorSource<T>>(
            "desired_acceleration=0", VectorX<T>::Zero(dim));
    builder.Connect(zero_feedforward_acceleration->get_output_port(),
                    adder->get_input_port(1));
  } else {
    // Exposes reference acceleration input port.
    desired_acceleration_ =
        builder.ExportInput(adder->get_input_port(1), "desired_acceleration");
  }

  // Add B⁻¹ to the diagram.
  auto Binv = builder.template AddNamedSystem<SparseMatrixGain<T>>(
      "B⁻¹", multibody_plant_for_control_->MakeActuationMatrixPseudoinverse());
  builder.Connect(inverse_dynamics->get_output_port_generalized_force(),
                  Binv->get_input_port());

  // Expose the actuation output port.
  actuation_ = builder.ExportOutput(Binv->get_output_port(), "actuation");

  // Exposes the generalized force output port.
  generalized_force_ = builder.ExportOutput(
      inverse_dynamics->get_output_port_generalized_force(),
      "generalized_force");

  // Finalize ourself.
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
    bool has_reference_acceleration, const Context<T>* plant_context)
    : multibody_plant_for_control_(&plant),
      has_reference_acceleration_(has_reference_acceleration) {
  SetUp(nullptr, kp, ki, kd, plant_context);
}

template <typename T>
InverseDynamicsController<T>::InverseDynamicsController(
    std::unique_ptr<multibody::MultibodyPlant<T>> plant,
    const VectorX<double>& kp, const VectorX<double>& ki,
    const VectorX<double>& kd, bool has_reference_acceleration,
    const Context<T>* plant_context)
    : multibody_plant_for_control_(plant.get()),
      has_reference_acceleration_(has_reference_acceleration) {
  SetUp(std::move(plant), kp, ki, kd, plant_context);
}

template <typename T>
InverseDynamicsController<T>::~InverseDynamicsController() = default;

}  // namespace controllers
}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::controllers::InverseDynamicsController);
