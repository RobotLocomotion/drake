#include "drake/automotive/dev/traffic_light.h"

#include <cmath>
#include <iostream>

namespace drake {
namespace automotive {

using drake::systems::System;
using std::fmod;
using systems::BasicVector;

template <typename T>
TrafficLight<T>::TrafficLight(T x_position, T y_position, T radius, T period)
    : output_index_{this->DeclareVectorOutputPort(BasicVector<T>(4),
                                                  &TrafficLight::DoCalcOutput)
                        .get_index()} {
  // Register parameters.
  const Vector4<T> parameters(x_position, y_position, radius, period);
  const BasicVector<T> p(parameters);
  parameters_index_ = this->DeclareNumericParameter(p);
}

template <typename T>
const systems::OutputPort<T>& TrafficLight<T>::output() const {
  return System<T>::get_output_port(output_index_);
}

template <typename T>
void TrafficLight<T>::DoCalcOutput(const systems::Context<T>& context,
                                   systems::BasicVector<T>* output) const {
  const VectorX<T> parameters = ReadParameters(context);
  const T x_position = parameters(0);
  const T y_position = parameters(1);
  const T radius = parameters(2);
  const T period = parameters(3);

  // Get time from context.
  const T time = context.get_time();

  // If time is in the first half of the period, signal should be open.
  // If time is in the second half of the period, signal should be closed.
  T signal = 0;
  if (fmod(time, period) >= (period / 2)) {
    signal = 1;
  }

  const Vector4<T> signal_data(x_position, y_position, radius, signal);
  WriteOutput(signal_data, output);
}

template <typename T>
const VectorX<T> TrafficLight<T>::ReadParameters(
    const systems::Context<T>& context) const {
  const BasicVector<T>& p = this->template GetNumericParameter<BasicVector>(
      context, parameters_index_);
  const VectorX<T> parameters = p.get_value();
  return parameters;
}

template <typename T>
void TrafficLight<T>::WriteOutput(const VectorX<T> value,
                                  BasicVector<T>* output) const {
  output->set_value(value);
}

template class TrafficLight<double>;

}  // namespace automotive
}  // namespace drake
