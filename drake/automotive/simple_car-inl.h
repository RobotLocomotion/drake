#pragma once

/// @file
/// Template method implementations for simple_car.h.
/// Most users should only include that file, not this one.
/// For background, see http://drake.mit.edu/cxx_inl.html.

#include "simple_car.h"

#include <algorithm>
#include <cmath>

#include <Eigen/Geometry>

#include "drake/common/drake_assert.h"
#include "drake/systems/framework/vector_base.h"

namespace drake {
namespace automotive {

template <typename ScalarType>
SimpleCar1::StateVector<ScalarType> SimpleCar1::dynamics(
    const ScalarType&,
    const StateVector<ScalarType>& state,
    const InputVector<ScalarType>& input) const {
  // Apply simplistic throttle.
  ScalarType new_velocity = state.velocity() +
                            input.throttle() * config_.max_acceleration *
                                config_.velocity_lookahead_time;
  new_velocity = std::min(
      new_velocity, static_cast<ScalarType>(config_.max_velocity));

  // Apply simplistic brake.
  new_velocity += input.brake() * -config_.max_acceleration *
                  config_.velocity_lookahead_time;
  new_velocity = std::max(new_velocity, static_cast<ScalarType>(0.));

  // Apply steering.
  ScalarType sane_steering_angle = input.steering_angle();
  DRAKE_ASSERT(static_cast<ScalarType>(-M_PI) < sane_steering_angle);
  DRAKE_ASSERT(sane_steering_angle < static_cast<ScalarType>(M_PI));
  sane_steering_angle = std::min(
      sane_steering_angle,
      static_cast<ScalarType>(config_.max_abs_steering_angle));
  sane_steering_angle = std::max(
      sane_steering_angle,
      static_cast<ScalarType>(-config_.max_abs_steering_angle));
  ScalarType curvature = tan(sane_steering_angle) / config_.wheelbase;

  StateVector<ScalarType> rates;
  rates.set_x(state.velocity() * cos(state.heading()));
  rates.set_y(state.velocity() * sin(state.heading()));
  rates.set_heading(curvature * state.velocity());
  rates.set_velocity((new_velocity - state.velocity()) * config_.velocity_kp);
  return rates;
}

template <typename ScalarType>
SimpleCar1::OutputVector<ScalarType> SimpleCar1::output(
    const ScalarType&,
    const StateVector<ScalarType>& state,
    const InputVector<ScalarType>&) const {
  return state;
}

template <typename T>
SimpleCar<T>::SimpleCar(const drake::lcmt_simple_car_config_t& config)
    : wrapped_(config) {
  this->DeclareInputPort(systems::kVectorValued,
                         DrivingCommandIndices::kNumCoordinates,
                         systems::kContinuousSampling);
  this->DeclareOutputPort(systems::kVectorValued,
                          SimpleCarStateIndices::kNumCoordinates,
                          systems::kContinuousSampling);
}

template <typename T>
bool SimpleCar<T>::has_any_direct_feedthrough() const {
  return wrapped_.isDirectFeedthrough();
}

namespace detail {
template <typename T>
std::pair<SimpleCarState1<T>, DrivingCommand1<T>> ConvertContextToSystem1(
    const systems::Context<T>& context) {
  // Convert the state into System1 data.
  const systems::VectorBase<T>& context_state =
      context.get_state().continuous_state->get_state();
  const SimpleCarState<T>* const simple_car_state =
      dynamic_cast<const SimpleCarState<T>*>(&context_state);
  DRAKE_ASSERT(simple_car_state != nullptr);
  const SimpleCarState1<T> state(simple_car_state->get_value());

  // Convert the input into System1 data.
  // TODO(jwnimmer-tri) Why does this return a pointer instead of ref?
  const systems::VectorBase<T>* const context_vector_input =
      context.get_vector_input(0);
  DRAKE_ASSERT(context_vector_input != nullptr);
  const DrivingCommand<T>* const driving_command_input =
      dynamic_cast<const DrivingCommand<T>*>(context_vector_input);
  DRAKE_ASSERT(driving_command_input != nullptr);
  const DrivingCommand1<T> input(driving_command_input->get_value());

  return std::make_pair(state, input);
}
}  // namespace detail

template <typename T>
void SimpleCar<T>::EvalOutput(const systems::Context<T>& context,
                              systems::SystemOutput<T>* output) const {
  DRAKE_ASSERT_VOID(systems::System<T>::CheckValidContext(context));
  DRAKE_ASSERT_VOID(systems::System<T>::CheckValidOutput(output));

  // Convert the context into System1 data.
  SimpleCarState1<T> state;
  DrivingCommand1<T> input;
  std::tie(state, input) = detail::ConvertContextToSystem1(context);

  // Obtain the structure we need to write into.
  systems::BasicVector<T>* const output_vector =
      output->GetMutableVectorData(0);
  DRAKE_ASSERT(output_vector != nullptr);

  // Delegate to the System1 version of this block.
  output_vector->get_mutable_value() =
      toEigen(wrapped_.output<T>(context.get_time(), state, input));
}

template <typename T>
void SimpleCar<T>::EvalTimeDerivatives(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  DRAKE_ASSERT_VOID(systems::System<T>::CheckValidContext(context));

  // Convert the context into System1 data.
  SimpleCarState1<T> state;
  DrivingCommand1<T> input;
  std::tie(state, input) = detail::ConvertContextToSystem1(context);

  // Obtain the structure we need to write into.
  DRAKE_ASSERT(derivatives != nullptr);
  systems::VectorBase<T>* const derivatives_state =
      derivatives->get_mutable_state();
  DRAKE_ASSERT(derivatives_state != nullptr);
  SimpleCarState<T>* const simple_car_derivatives =
      dynamic_cast<SimpleCarState<T>*>(derivatives_state);
  DRAKE_ASSERT(simple_car_derivatives != nullptr);

  // Delegate to the System1 version of this block.
  simple_car_derivatives->get_mutable_value() =
      toEigen(wrapped_.dynamics<T>(context.get_time(), state, input));
}

template <typename T>
std::unique_ptr<systems::ContinuousState<T>>
SimpleCar<T>::AllocateContinuousState() const {
  return std::make_unique<systems::ContinuousState<T>>(
      std::make_unique<SimpleCarState<T>>());
}

template <typename T>
std::unique_ptr<systems::BasicVector<T>> SimpleCar<T>::AllocateOutputVector(
    const systems::SystemPortDescriptor<T>& descriptor) const {
  return std::make_unique<SimpleCarState<T>>();
}

}  // namespace automotive
}  // namespace drake
