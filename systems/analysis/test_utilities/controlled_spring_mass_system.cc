#include "drake/systems/analysis/test_utilities/controlled_spring_mass_system.h"

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
    double Kp, double Ki, double Kd,
    const T& target_position) : Diagram<T>() {
  DRAKE_ASSERT(spring_stiffness >= 0);
  DRAKE_ASSERT(mass >= 0);
  DRAKE_ASSERT(Kp >= 0);
  DRAKE_ASSERT(Ki >= 0);
  DRAKE_ASSERT(Kd >= 0);

  DiagramBuilder<T> builder;

  plant_ = builder.template
      AddSystem<SpringMassSystem>(spring_stiffness, mass, true /* is forced */);
  plant_->set_name("plant");
  controller_ = builder.template AddSystem<controllers::PidController>(
      VectorX<double>::Constant(1, Kp), VectorX<double>::Constant(1, Ki),
      VectorX<double>::Constant(1, Kd));
  controller_->set_name("controller");
  VectorX<T> desired(2);
  desired << target_position, 0;
  target_ = builder.template AddSystem<ConstantVectorSource>(desired);
  target_->set_name("target");

  // A demultiplexer is used to split the output from the spring-mass system
  // into three ports. One port with the mass position and another port with the
  // mass velocity so that they can be connected to the controller.
  // The third output from the demultiplexer is the spring-mass system's energy
  // and it is left unconnected.
  auto demux = builder.template AddSystem<Demultiplexer>(3);
  demux->set_name("demux");
  auto mux = builder.template AddSystem<Multiplexer>(2);
  mux->set_name("mux");

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
T PidControlledSpringMassSystem<T>::get_position(
    const Context<T>& context) const {
  const Context<T>& plant_context =
      Diagram<T>::GetSubsystemContext(*plant_, context);
  return plant_->get_position(plant_context);
}

template <typename T>
T PidControlledSpringMassSystem<T>::get_velocity(
    const Context<T>& context) const {
  const Context<T>& plant_context =
      Diagram<T>::GetSubsystemContext(*plant_, context);
  return plant_->get_velocity(plant_context);
}

template <typename T>
T PidControlledSpringMassSystem<T>::get_conservative_work(
    const Context<T>& context) const {
  const Context<T>& plant_context =
      Diagram<T>::GetSubsystemContext(*plant_, context);
  return plant_->get_conservative_work(plant_context);
}

template <typename T>
void PidControlledSpringMassSystem<T>::set_position(
    Context<T>* context, const T& position) const {
  Context<T>& plant_context =
      Diagram<T>::GetMutableSubsystemContext(*plant_, context);
  plant_->set_position(&plant_context, position);
}

template <typename T>
void PidControlledSpringMassSystem<T>::set_velocity(
    Context<T>* context, const T& velocity) const {
  Context<T>& plant_context =
      Diagram<T>::GetMutableSubsystemContext(*plant_, context);
  plant_->set_velocity(&plant_context, velocity);
}

template <typename T>
const SpringMassSystem<T>& PidControlledSpringMassSystem<T>::get_plant()
const {
  return *plant_;
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class drake::systems::PidControlledSpringMassSystem)
