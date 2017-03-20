#include "drake/systems/analysis/test/controlled_spring_mass_system/controlled_spring_mass_system.h"

#include "drake/common/autodiff_overloads.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/eigen_types.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/demultiplexer.h"
#include "drake/systems/primitives/multiplexer.h"

using std::make_unique;

namespace drake {
namespace systems {

template <typename T>
PidControlledSpringMassSystem<T>::PidControlledSpringMassSystem(
    double spring_stiffness, double mass,
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
  controller_ = builder.template AddSystem<PidController>(
      VectorX<T>::Constant(1, Kp),
      VectorX<T>::Constant(1, Ki),
      VectorX<T>::Constant(1, Kd));
  VectorX<T> desired(2);
  desired << target_position, 0;
  target_ = builder.template AddSystem<ConstantVectorSource>(desired);

  // A demultiplexer is used to split the output from the spring-mass system
  // into three ports. One port with the mass position and another port with the
  // mass velocity so that they can be connected to the controller.
  // The third output from the demultiplexer is the spring-mass system's energy
  // and it is left unconnected.
  auto demux = builder.template AddSystem<Demultiplexer>(3);
  auto mux = builder.template AddSystem<Multiplexer>(2);

  builder.Connect(plant_->get_output_port(),
                  demux->get_input_port(0));

  builder.Connect(demux->get_output_port(0),
                  mux->get_input_port(0));
  builder.Connect(demux->get_output_port(1),
                  mux->get_input_port(1));

  // Connects the estimated state to PID.
  builder.Connect(mux->get_output_port(0),
                  controller_->get_input_port_estimated_state());

  // Connects the desired state to PID.
  builder.Connect(target_->get_output_port(),
                  controller_->get_input_port_desired_state());

  // Closes the feedback loop.
  builder.Connect(controller_->get_output_port_control(),
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

template class PidControlledSpringMassSystem<double>;
template class PidControlledSpringMassSystem<AutoDiffXd>;

}  // namespace systems
}  // namespace drake
