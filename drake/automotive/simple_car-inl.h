#pragma once

/// @file
/// Template method implementations for simple_car.h.
/// Most users should only include that file, not this one.
/// For background, see http://drake.mit.edu/cxx_inl.html.

#include "drake/automotive/simple_car.h"

#include <algorithm>
#include <cmath>

#include <Eigen/Geometry>

#include "drake/common/drake_assert.h"
#include "drake/systems/framework/vector_base.h"

namespace drake {
namespace automotive {

template <typename T>
SimpleCar<T>::SimpleCar(const SimpleCarConfig<T>& config)
    : config_(config) {
  this->DeclareInputPort(systems::kVectorValued,
                         DrivingCommandIndices::kNumCoordinates,
                         systems::kContinuousSampling);
  this->DeclareOutputPort(systems::kVectorValued,
                          SimpleCarStateIndices::kNumCoordinates,
                          systems::kContinuousSampling);
}

template <typename T>
SimpleCarConfig<T> SimpleCar<T>::get_default_config() {
  constexpr double kInchToMeter = 0.0254;
  constexpr double kDegToRadian = 0.0174532925199;
  // This approximates a 2010 Toyota Prius.
  SimpleCarConfig<T> result;
  result.set_wheelbase(static_cast<T>(106.3 * kInchToMeter));
  result.set_track(static_cast<T>(59.9 * kInchToMeter));
  result.set_max_abs_steering_angle(static_cast<T>(27 * kDegToRadian));
  result.set_max_velocity(static_cast<T>(45.0));  // meters/second
  result.set_max_acceleration(static_cast<T>(4.0));  // meters/second**2
  result.set_velocity_lookahead_time(static_cast<T>(1.0));  // second
  result.set_velocity_kp(static_cast<T>(1.0));  // Hz
  return result;
}

template <typename T>
bool SimpleCar<T>::has_any_direct_feedthrough() const {
  return false;
}

template <typename T>
void SimpleCar<T>::EvalOutput(const systems::Context<T>& context,
                              systems::SystemOutput<T>* output) const {
  DRAKE_ASSERT_VOID(systems::System<T>::CheckValidContext(context));
  DRAKE_ASSERT_VOID(systems::System<T>::CheckValidOutput(output));

  // Obtain the state.
  const systems::VectorBase<T>& context_state =
      context.get_continuous_state_vector();
  const SimpleCarState<T>* const state =
      dynamic_cast<const SimpleCarState<T>*>(&context_state);
  DRAKE_ASSERT(state);

  // Obtain the output pointer.
  SimpleCarState<T>* const output_vector =
      dynamic_cast<SimpleCarState<T>*>(output->GetMutableVectorData(0));
  DRAKE_ASSERT(output_vector);

  DoEvalOutput(*state, output_vector);
}

template <typename T>
void SimpleCar<T>::DoEvalOutput(const SimpleCarState<T>& state,
                                SimpleCarState<T>* output) const {
  output->set_value(state.get_value());
}

template <typename T>
void SimpleCar<T>::EvalTimeDerivatives(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  DRAKE_ASSERT_VOID(systems::System<T>::CheckValidContext(context));

  // Obtain the state.
  const systems::VectorBase<T>& context_state =
      context.get_continuous_state_vector();
  const SimpleCarState<T>* const state =
      dynamic_cast<const SimpleCarState<T>*>(&context_state);
  DRAKE_ASSERT(state);

  // Obtain the input.
  const systems::VectorBase<T>* const vector_input =
      this->EvalVectorInput(context, 0);
  DRAKE_ASSERT(vector_input);
  const DrivingCommand<T>* const input =
      dynamic_cast<const DrivingCommand<T>*>(vector_input);
  DRAKE_ASSERT(input);

  // Obtain the result structure.
  DRAKE_ASSERT(derivatives != nullptr);
  systems::VectorBase<T>* const vector_derivatives =
      derivatives->get_mutable_vector();
  DRAKE_ASSERT(vector_derivatives);
  SimpleCarState<T>* const rates =
      dynamic_cast<SimpleCarState<T>*>(vector_derivatives);
  DRAKE_ASSERT(rates);

  DoEvalTimeDerivatives(*state, *input, rates);
}

template <typename T>
void SimpleCar<T>::DoEvalTimeDerivatives(const SimpleCarState<T>& state,
                                         const DrivingCommand<T>& input,
                                         SimpleCarState<T>* rates) const {
  using std::max;
  using std::min;

  // Apply simplistic throttle.
  T new_velocity =
      state.velocity() + (input.throttle() * config_.max_acceleration() *
                          config_.velocity_lookahead_time());
  new_velocity = min(new_velocity, config_.max_velocity());

  // Apply simplistic brake.
  new_velocity += input.brake() * -config_.max_acceleration() *
                  config_.velocity_lookahead_time();
  new_velocity = max(new_velocity, static_cast<T>(0.));

  // Apply steering.
  T sane_steering_angle = input.steering_angle();
  DRAKE_ASSERT(static_cast<T>(-M_PI) < sane_steering_angle);
  DRAKE_ASSERT(sane_steering_angle < static_cast<T>(M_PI));
  sane_steering_angle = min(
      sane_steering_angle, config_.max_abs_steering_angle());
  sane_steering_angle = max(
      sane_steering_angle, static_cast<T>(-config_.max_abs_steering_angle()));
  const T curvature = tan(sane_steering_angle) / config_.wheelbase();

  rates->set_x(state.velocity() * cos(state.heading()));
  rates->set_y(state.velocity() * sin(state.heading()));
  rates->set_heading(curvature * state.velocity());
  rates->set_velocity((new_velocity - state.velocity()) *
                      config_.velocity_kp());
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
