#include "drake/automotive/simple_car.h"

#include <algorithm>
#include <cmath>
#include <utility>

#include <Eigen/Geometry>

#include "drake/common/autodiff_overloads.h"
#include "drake/common/cond.h"
#include "drake/common/double_overloads.h"
#include "drake/common/drake_assert.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/symbolic_expression.h"
#include "drake/systems/framework/vector_base.h"

// This is used indirectly to allow DRAKE_ASSERT on symbolic::Expression.
#include "drake/common/symbolic_formula.h"

namespace drake {
namespace automotive {

template <typename T>
SimpleCar<T>::SimpleCar() {
  this->DeclareInputPort(systems::kVectorValued,
                         DrivingCommandIndices::kNumCoordinates);
  this->DeclareOutputPort(systems::kVectorValued,
                          SimpleCarStateIndices::kNumCoordinates);
}

template <typename T>
bool SimpleCar<T>::has_any_direct_feedthrough() const {
  return false;
}

template <typename T>
void SimpleCar<T>::DoCalcOutput(const systems::Context<T>& context,
                                systems::SystemOutput<T>* output) const {
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

  ImplCalcOutput(*state, output_vector);
}

template <typename T>
void SimpleCar<T>::ImplCalcOutput(const SimpleCarState<T>& state,
                                  SimpleCarState<T>* output) const {
  output->set_value(state.get_value());

  // Don't allow small negative velocities to escape our state.
  using std::max;
  output->set_velocity(max(T(0), state.velocity()));
}

template <typename T>
void SimpleCar<T>::DoCalcTimeDerivatives(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  // Obtain the parameters.
  const SimpleCarConfig<T>& config =
      this->template GetNumericParameter<SimpleCarConfig>(context, 0);

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

  ImplCalcTimeDerivatives(config, *state, *input, rates);
}

namespace {
// If value is within [low, high] then return it; else return the boundary.
template <class T1, class T2, class T3>
T1 saturate(const T1& value, const T2& low, const T3& high) {
  DRAKE_ASSERT(low <= high);
  return cond(
      value < low, low,
      value > high, high,
      value);
}
}  // namespace

template <typename T>
void SimpleCar<T>::ImplCalcTimeDerivatives(const SimpleCarConfig<T>& config,
                                           const SimpleCarState<T>& state,
                                           const DrivingCommand<T>& input,
                                           SimpleCarState<T>* rates) const {
  using std::abs;
  using std::max;
  using std::min;
  using std::tanh;

  // Sanity check our input.
  DRAKE_DEMAND(abs(input.steering_angle()) < M_PI);
  DRAKE_DEMAND(input.throttle() >= 0);
  DRAKE_DEMAND(input.brake() >= 0);

  // Determine the requested acceleration, using throttle and brake.
  const T nominal_acceleration =
      config.max_acceleration() * (input.throttle() - input.brake());
  // If our current velocity is out of bounds, insist on damping that brings us
  // back toward the limit, but allow for the nominal_acceleration to win if it
  // is stronger than the damping and has the desired sign.
  const T underspeed = 0 - state.velocity();
  const T overspeed = state.velocity() - config.max_velocity();
  const T damped_acceleration = cond(
      // If velocity is too low, use positive damping or nominal_acceleration.
      underspeed > 0,
      max(nominal_acceleration, T(config.velocity_limit_kp() * underspeed)),
      // If velocity is too high, use negative damping or nominal_acceleration.
      overspeed > 0,
      min(nominal_acceleration, T(-config.velocity_limit_kp() * overspeed)),
      // Velocity is within limits.
      nominal_acceleration);
  // TODO(jwnimmer-tri) Declare witness functions for the above conditions,
  // once the framework support is in place.  Until then, smooth out the
  // acceleration using tanh centered around the limit we are headed towards
  // (max speed when accelerating; zero when decelerating).  The smoothing
  // constant within the tanh is arbitrary and un-tuned.
  const T relevant_limit = cond(
      damped_acceleration >= 0, config.max_velocity(), T(0));
  const T smoothing_factor =
      pow(tanh(20.0 * (state.velocity() - relevant_limit)), 2);
  const T smooth_acceleration = damped_acceleration * smoothing_factor;

  // Determine steering.
  const T saturated_steering_angle = saturate(
      input.steering_angle(),
      -config.max_abs_steering_angle(),
      config.max_abs_steering_angle());
  const T curvature = tan(saturated_steering_angle) / config.wheelbase();

  // Don't allow small negative velocities to affect position or heading.
  const T nonneg_velocity = max(T(0), state.velocity());

  rates->set_x(nonneg_velocity * cos(state.heading()));
  rates->set_y(nonneg_velocity * sin(state.heading()));
  rates->set_heading(curvature * nonneg_velocity);
  rates->set_velocity(smooth_acceleration);
}

template <typename T>
std::unique_ptr<systems::ContinuousState<T>>
SimpleCar<T>::AllocateContinuousState() const {
  return std::make_unique<systems::ContinuousState<T>>(
      std::make_unique<SimpleCarState<T>>());
}

template <typename T>
std::unique_ptr<systems::BasicVector<T>> SimpleCar<T>::AllocateOutputVector(
    const systems::OutputPortDescriptor<T>& descriptor) const {
  return std::make_unique<SimpleCarState<T>>();
}

template <typename T>
std::unique_ptr<systems::Parameters<T>>
SimpleCar<T>::AllocateParameters() const {
  auto params = std::make_unique<SimpleCarConfig<T>>();
  return std::make_unique<systems::Parameters<T>>(std::move(params));
}

template <typename T>
void SimpleCar<T>::SetDefaultParameters(const systems::LeafContext<T>& context,
                                        systems::Parameters<T>* params) const {
  SimpleCarConfig<T>* config = dynamic_cast<SimpleCarConfig<T>*>(
      params->get_mutable_numeric_parameter(0));
  DRAKE_DEMAND(config != nullptr);
  SetDefaultParameters(config);
}

template <typename T>
void SimpleCar<T>::SetDefaultParameters(SimpleCarConfig<T>* config) {
  DRAKE_DEMAND(config != nullptr);
  constexpr double kInchToMeter = 0.0254;
  constexpr double kDegToRadian = 0.0174532925199;
  // This approximates a 2010 Toyota Prius.
  config->set_wheelbase(static_cast<T>(106.3 * kInchToMeter));
  config->set_track(static_cast<T>(59.9 * kInchToMeter));
  config->set_max_abs_steering_angle(static_cast<T>(27 * kDegToRadian));
  config->set_max_velocity(static_cast<T>(45.0));       // meters/second
  config->set_max_acceleration(static_cast<T>(4.0));    // meters/second**2
  config->set_velocity_limit_kp(static_cast<T>(10.0));  // Hz
}

// These instantiations must match the API documentation in simple_car.h.
template class SimpleCar<double>;
#if EIGEN_VERSION_AT_LEAST(3, 2, 93)  // True when built via Drake superbuild.
template class SimpleCar<drake::AutoDiffXd>;
#endif
template class SimpleCar<drake::symbolic::Expression>;

}  // namespace automotive
}  // namespace drake
