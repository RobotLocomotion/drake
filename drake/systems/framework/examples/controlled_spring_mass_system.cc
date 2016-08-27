#include "drake/systems/framework/examples/controlled_spring_mass_system.h"

#include "drake/drakeSystemFramework_export.h"
#include "drake/systems/framework/diagram_builder.h"

using std::make_unique;

namespace drake {
namespace systems {

template <typename T>
PidControlledSpringMassSystem<T>::PidControlledSpringMassSystem(
    const T& spring_stiffness, const T& mass,
    const T& Kp, const T& Ki, const T& Kd) : Diagram<T>() {
  DRAKE_ASSERT(spring_stiffness >= 0);
  DRAKE_ASSERT(mass >= 0);
  DRAKE_ASSERT(Kp >= 0);
  DRAKE_ASSERT(Ki >= 0);
  DRAKE_ASSERT(Kd >= 0);

  plant_ = make_unique<SpringMassSystem>(
      spring_stiffness, mass, true /* is forced */);
  controller_ = make_unique<PidController<T>>(Kp, Ki, Kd, 1);

  // A demux is used to split the output from the spring-mass system into two
  // ports. One port with the mass position and another port with the mass
  // velocity so that they can be connected to the controller.
  // The third output from the demultiplexer is the spring-mass system energy
  // and it is left unconnected.
  demux_ = make_unique<Demultiplexer<T>>(3);

  DiagramBuilder<T> builder;
  builder.Connect(plant_->get_output_port(0),
                  demux_->get_input_port(0));

  // Connects the input error and rate signals to the PID controller.
  builder.Connect(demux_->get_output_port(0),
                  controller_->get_error_signal_port());
  builder.Connect(demux_->get_output_port(1),
                  controller_->get_error_signal_rate_port());

  // Close the feedback loop.
  builder.Connect(controller_->get_output_port(0),
                  plant_->get_input_port(0));

  // The output to this system is the output of the spring-mass system which
  // consists of a vector with position, velocity and energy.
  builder.ExportOutput(plant_->get_output_port(0));
  builder.BuildInto(this);
}

template <typename T>
bool PidControlledSpringMassSystem<T>::has_any_direct_feedthrough() const {
  return false;
}

template <typename T>
void PidControlledSpringMassSystem<T>::SetDefaultState(
    ContextBase<T>* context) const {
  ContextBase<T>* controller_context =
      Diagram<T>::GetMutableSubSystemContext(context, controller_.get());
  controller_->set_integral_value(controller_context, VectorX<T>::Zero(1));

  ContextBase<T>* plant_context =
      Diagram<T>::GetMutableSubSystemContext(context, plant_.get());
  plant_->set_position(plant_context, 0.0);
  plant_->set_velocity(plant_context, 0.0);
  plant_->set_conservative_work(plant_context, 0.0);
}

template <typename T>
void PidControlledSpringMassSystem<T>::set_position(
    ContextBase<T>* context, const T& position) const {
  ContextBase<T>* plant_context =
      Diagram<T>::GetMutableSubSystemContext(context, plant_.get());
  plant_->set_position(plant_context, position);
}

template <typename T>
void PidControlledSpringMassSystem<T>::set_velocity(
    ContextBase<T>* context, const T& velocity) const {
  ContextBase<T>* plant_context =
      Diagram<T>::GetMutableSubSystemContext(context, plant_.get());
  plant_->set_velocity(plant_context, velocity);
}

template class
DRAKESYSTEMFRAMEWORK_EXPORT PidControlledSpringMassSystem<double>;

}  // namespace systems
}  // namespace drake
