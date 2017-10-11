#include "drake/automotive/dev/traffic_light_aware_controller.h"

namespace drake {
using std::cout;
using std::endl;
using std::pow;
using systems::BasicVector;
using systems::Context;
using systems::System;

namespace automotive {

template <typename T>
TrafficLightAwareController<T>::TrafficLightAwareController()
    : car_state_input_index_{this->DeclareAbstractInputPort().get_index()},
      acceleration_input_index_{this->DeclareAbstractInputPort().get_index()},
      traffic_light_input_index_{this->DeclareAbstractInputPort().get_index()},
      output_index_{
          this->DeclareVectorOutputPort(
                  BasicVector<T>(2), &TrafficLightAwareController::DoCalcOutput)
              .get_index()}

{}

template <typename T>
const systems::InputPortDescriptor<T>&
TrafficLightAwareController<T>::car_state() const {
  return System<T>::get_input_port(car_state_input_index_);
}

template <typename T>
const systems::InputPortDescriptor<T>&
TrafficLightAwareController<T>::other_controller_acceleration() const {
  return System<T>::get_input_port(acceleration_input_index_);
}

template <typename T>
const systems::InputPortDescriptor<T>&
TrafficLightAwareController<T>::traffic_light_input() const {
  return System<T>::get_input_port(traffic_light_input_index_);
}

template <typename T>
const systems::OutputPort<T>&
TrafficLightAwareController<T>::acceleration_output() const {
  return System<T>::get_output_port(output_index_);
}

template <typename T>
void TrafficLightAwareController<T>::DoCalcOutput(
    const systems::Context<T>& context, systems::BasicVector<T>* output) const {
  VectorX<T> car_state = ReadInput(context, car_state_input_index_);
  VectorX<T> other_acceleration = ReadInput(context, acceleration_input_index_);
  VectorX<T> traffic_light_signal =
      ReadInput(context, traffic_light_input_index_);

  T car_x = car_state(0);
  T car_y = car_state(1);
  T signal_x = traffic_light_signal(0);
  T signal_y = traffic_light_signal(1);
  T signal_radius = traffic_light_signal(2);

  if ((traffic_light_signal(3) == 0) ||
      (pow(signal_x - car_x, 2) + pow(signal_y - car_y, 2) >
       pow(signal_radius, 2))) {
    // Either we can go through or we are too far to worry about it, continue.
    output->SetFromVector(other_acceleration);
  } else {
    // Slam on the brakes.
    DrivingCommand<T> braking_command;
    braking_command.set_steering_angle(other_acceleration(0));
    // TODO(nikos-tri) Replace this with a more sophisticated braking controller
    braking_command.set_acceleration(-100);
    output->set_value(braking_command.get_value());
  }
}

template <typename T>
const VectorX<T> TrafficLightAwareController<T>::ReadInput(
    const Context<T>& context, int input_index) const {
  const BasicVector<T>* input =
      this->template EvalVectorInput<BasicVector>(context, input_index);
  DRAKE_ASSERT((input != nullptr));
  return input->get_value();
}

template class TrafficLightAwareController<double>;

}  // namespace automotive
}  // namespace drake
