#include "drake/systems/analysis/test/controlled_spring_mass_system/controlled_spring_mass_system.h"

#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/eigen_types.h"
#include "drake/systems/framework/diagram_builder.h"

using std::make_unique;

namespace drake {
namespace systems {

template <typename T>
PidControlledSpringMassSystem<T>::PidControlledSpringMassSystem(
    const T& spring_stiffness, const T& mass,
    const T& Kp, const T& Ki, const T& Kd,
    const T& target_position) : Diagram<T>() {
  DRAKE_ASSERT(spring_stiffness >= 0);
  DRAKE_ASSERT(mass >= 0);
  DRAKE_ASSERT(Kp >= 0);
  DRAKE_ASSERT(Ki >= 0);
  DRAKE_ASSERT(Kd >= 0);

  DiagramBuilder<T> builder;

  plant_ = builder.template
      AddSystem<SpringMassSystem>(spring_stiffness, mass, true /* is forced */);
  controller_ = builder.template
      AddSystem<PidController>(Kp, Ki, Kd, 1 /* size */);
  pid_inverter_ = builder.template
      AddSystem<Gain>(-1 /* gain */, 1 /* size */);
  target_inverter_ = builder.template
      AddSystem<Gain>(-1 /* gain */, 1 /* size */);
  target_ = builder.template AddSystem<ConstantVectorSource>(target_position);
  state_minus_target_ = builder.template
      AddSystem<Adder>(2 /* num inputs */, 1 /* size */);

  // A demultiplexer is used to split the output from the spring-mass system
  // into three ports. One port with the mass position and another port with the
  // mass velocity so that they can be connected to the controller.
  // The third output from the demultiplexer is the spring-mass system's energy
  // and it is left unconnected.
  demux_ = builder.template AddSystem<Demultiplexer>(3);

  builder.Connect(plant_->get_output_port(),
                  demux_->get_input_port(0));

  // Subtracts the target position from the spring position to obtain the error
  // signal.
  builder.Connect(target_->get_output_port(),
                  target_inverter_->get_input_port());
  builder.Connect(target_inverter_->get_output_port(),
                  state_minus_target_->get_input_port(0));
  builder.Connect(demux_->get_output_port(0),
                  state_minus_target_->get_input_port(1));

  // Connects the input error and rate signals to the PID controller.
  builder.Connect(state_minus_target_->get_output_port(),
                  controller_->get_error_port());
  builder.Connect(demux_->get_output_port(1),
                  controller_->get_error_derivative_port());

  // Closes the feedback loop.
  builder.Connect(controller_->get_control_output_port(),
                  pid_inverter_->get_input_port());
  builder.Connect(pid_inverter_->get_output_port(),
                  plant_->get_force_port());

  // The output to this system is the output of the spring-mass system which
  // consists of a vector with position, velocity and energy.
  builder.ExportOutput(plant_->get_output_port());
  builder.BuildInto(this);
}

template <typename T>
bool PidControlledSpringMassSystem<T>::has_any_direct_feedthrough() const {
  return false;
}

template <typename T>
void PidControlledSpringMassSystem<T>::SetDefaultState(
    Context<T>* context) const {
  Context<T>* controller_context =
      Diagram<T>::GetMutableSubsystemContext(context, controller_);
  controller_->set_integral_value(controller_context, VectorX<T>::Zero(1));

  Context<T>* plant_context =
      Diagram<T>::GetMutableSubsystemContext(context, plant_);
  plant_->set_position(plant_context, 0.0);
  plant_->set_velocity(plant_context, 0.0);
  plant_->set_conservative_work(plant_context, 0.0);
}

template <typename T>
T PidControlledSpringMassSystem<T>::get_position(
    const Context<T>& context) const {
  const Context<T>& plant_context =
      Diagram<T>::GetSubsystemContext(context, plant_);
  return plant_->get_position(plant_context);
}

template <typename T>
T PidControlledSpringMassSystem<T>::get_velocity(
    const Context<T>& context) const {
  const Context<T>& plant_context =
      Diagram<T>::GetSubsystemContext(context, plant_);
  return plant_->get_velocity(plant_context);
}

template <typename T>
T PidControlledSpringMassSystem<T>::get_conservative_work(
    const Context<T>& context) const {
  const Context<T>& plant_context =
      Diagram<T>::GetSubsystemContext(context, plant_);
  return plant_->get_conservative_work(plant_context);
}

template <typename T>
void PidControlledSpringMassSystem<T>::set_position(
    Context<T>* context, const T& position) const {
  Context<T>* plant_context =
      Diagram<T>::GetMutableSubsystemContext(context, plant_);
  plant_->set_position(plant_context, position);
}

template <typename T>
void PidControlledSpringMassSystem<T>::set_velocity(
    Context<T>* context, const T& velocity) const {
  Context<T>* plant_context =
      Diagram<T>::GetMutableSubsystemContext(context, plant_);
  plant_->set_velocity(plant_context, velocity);
}

template <typename T>
const SpringMassSystem<T>& PidControlledSpringMassSystem<T>::get_plant()
const {
  return *plant_;
}

template class
DRAKE_EXPORT
PidControlledSpringMassSystem<double>;
template class
DRAKE_EXPORT
PidControlledSpringMassSystem<AutoDiffXd>;

}  // namespace systems
}  // namespace drake
