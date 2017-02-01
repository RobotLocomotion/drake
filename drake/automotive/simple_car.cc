#include "drake/automotive/simple_car.h"

#include <algorithm>
#include <cmath>
#include <utility>

#include <Eigen/Geometry>

#include "drake/common/autodiff_overloads.h"
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

  ImplCalcOutput(*state, output_vector);
}

template <typename T>
void SimpleCar<T>::ImplCalcOutput(const SimpleCarState<T>& state,
                                  SimpleCarState<T>* output) const {
  output->set_value(state.get_value());
}

template <typename T>
void SimpleCar<T>::DoCalcTimeDerivatives(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  DRAKE_ASSERT_VOID(systems::System<T>::CheckValidContext(context));

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

template <typename T>
void SimpleCar<T>::ImplCalcTimeDerivatives(const SimpleCarConfig<T>& config,
                                           const SimpleCarState<T>& state,
                                           const DrivingCommand<T>& input,
                                           SimpleCarState<T>* rates) const {
  using std::max;
  using std::min;

  // Apply simplistic throttle.
  T new_velocity =
      state.velocity() + (input.throttle() * config.max_acceleration() *
                          config.velocity_lookahead_time());
  new_velocity = min(new_velocity, config.max_velocity());

  // Apply simplistic brake.
  new_velocity += input.brake() * -config.max_acceleration() *
                  config.velocity_lookahead_time();
  new_velocity = max(new_velocity, static_cast<T>(0.));

  // Apply steering.
  T sane_steering_angle = input.steering_angle();
  DRAKE_ASSERT(static_cast<T>(-M_PI) < sane_steering_angle);
  DRAKE_ASSERT(sane_steering_angle < static_cast<T>(M_PI));
  sane_steering_angle = min(
      sane_steering_angle, config.max_abs_steering_angle());
  sane_steering_angle = max(
      sane_steering_angle, static_cast<T>(-config.max_abs_steering_angle()));
  const T curvature = tan(sane_steering_angle) / config.wheelbase();

  rates->set_x(state.velocity() * cos(state.heading()));
  rates->set_y(state.velocity() * sin(state.heading()));
  rates->set_heading(curvature * state.velocity());
  rates->set_velocity((new_velocity - state.velocity()) *
                      config.velocity_kp());
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
  config->set_max_velocity(static_cast<T>(45.0));            // meters/second
  config->set_max_acceleration(static_cast<T>(4.0));         // meters/second**2
  config->set_velocity_lookahead_time(static_cast<T>(1.0));  // second
  config->set_velocity_kp(static_cast<T>(1.0));              // Hz
}

// These instantiations must match the API documentation in simple_car.h.
template class SimpleCar<double>;
#if EIGEN_VERSION_AT_LEAST(3, 2, 93)  // True when built via Drake superbuild.
template class SimpleCar<drake::AutoDiffXd>;
#endif
template class SimpleCar<drake::symbolic::Expression>;

}  // namespace automotive
}  // namespace drake
