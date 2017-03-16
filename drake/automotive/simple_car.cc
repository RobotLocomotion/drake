#include "drake/automotive/simple_car.h"

#include <algorithm>
#include <cmath>
#include <utility>

#include <Eigen/Geometry>

#include "drake/automotive/calc_smooth_acceleration.h"
#include "drake/common/autodiff_overloads.h"
#include "drake/common/cond.h"
#include "drake/common/double_overloads.h"
#include "drake/common/drake_assert.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/symbolic_expression.h"
#include "drake/common/symbolic_formula.h"
#include "drake/math/saturate.h"
#include "drake/systems/framework/vector_base.h"

namespace drake {

using systems::rendering::FrameVelocity;
using systems::rendering::PoseVector;

namespace automotive {

template <typename T>
SimpleCar<T>::SimpleCar() {
  this->DeclareVectorInputPort(DrivingCommand<T>());
  this->DeclareVectorOutputPort(SimpleCarState<T>());
  this->DeclareVectorOutputPort(PoseVector<T>());
  this->DeclareVectorOutputPort(FrameVelocity<T>());
  this->DeclareContinuousState(SimpleCarState<T>());
}

template <typename T>
const systems::OutputPortDescriptor<T>& SimpleCar<T>::state_output() const {
  return this->get_output_port(0);
}

template <typename T>
const systems::OutputPortDescriptor<T>& SimpleCar<T>::pose_output() const {
  return this->get_output_port(1);
}

template <typename T>
const systems::OutputPortDescriptor<T>& SimpleCar<T>::velocity_output() const {
  return this->get_output_port(2);
}

template <typename T>
void SimpleCar<T>::DoCalcOutput(const systems::Context<T>& context,
                                systems::SystemOutput<T>* output) const {
  // Obtain the parameters.
  const SimpleCarConfig<T>& config =
      this->template GetNumericParameter<SimpleCarConfig>(context, 0);

  // Obtain the input.
  const DrivingCommand<T>* const input =
      this->template EvalVectorInput<DrivingCommand>(context, 0);
  DRAKE_ASSERT(input);

  // Obtain the state.
  const systems::VectorBase<T>& context_state =
      context.get_continuous_state_vector();
  const SimpleCarState<T>* const state =
      dynamic_cast<const SimpleCarState<T>*>(&context_state);
  DRAKE_ASSERT(state);

  // Obtain the output pointer.
  SimpleCarState<T>* const output_vector =
      dynamic_cast<SimpleCarState<T>*>(output->GetMutableVectorData(0));
  DRAKE_ASSERT(output_vector != nullptr);
  ImplCalcOutput(*state, output_vector);

  PoseVector<T>* const pose =
      dynamic_cast<PoseVector<T>*>(output->GetMutableVectorData(1));
  DRAKE_ASSERT(pose != nullptr);
  ImplCalcPose(*state, pose);

  FrameVelocity<T>* const velocity =
      dynamic_cast<FrameVelocity<T>*>(output->GetMutableVectorData(2));
  DRAKE_ASSERT(pose != nullptr);
  ImplCalcVelocity(config, *state, *input, velocity);
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
void SimpleCar<T>::ImplCalcPose(const SimpleCarState<T>& state,
                                PoseVector<T>* pose) const {
  pose->set_translation(Eigen::Translation<T, 3>(state.x(), state.y(), 0));
  const Vector3<T> z_axis{0.0, 0.0, 1.0};
  const Eigen::AngleAxis<T> rotation(state.heading(), z_axis);
  pose->set_rotation(Eigen::Quaternion<T>(rotation));
}

template <typename T>
void SimpleCar<T>::ImplCalcVelocity(
    const SimpleCarConfig<T>& config, const SimpleCarState<T>& state,
    const DrivingCommand<T>& input,
    systems::rendering::FrameVelocity<T>* velocity) const {
  // Calculate the derivatives.
  SimpleCarState<T> rates;
  ImplCalcTimeDerivatives(config, state, input, &rates);

  // Convert the state derivatives into a spatial velocity.
  multibody::SpatialVelocity<T> output;
  output.translational().x() = rates.x();
  output.translational().y() = rates.y();
  output.translational().z() = T(0);
  output.rotational().x() = T(0);
  output.rotational().y() = T(0);
  // The rotational velocity around the z-axis is actually rates.heading(),
  // which is a function of the input steering angle. We set it to zero so that
  // this system is not direct-feedthrough.
  output.rotational().z() = T(0);
  velocity->set_velocity(output);
}

template <typename T>
void SimpleCar<T>::DoCalcTimeDerivatives(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  // Obtain the parameters.
  const SimpleCarConfig<T>& config =
      this->template GetNumericParameter<SimpleCarConfig>(context, 0);

  // Obtain the state.
  const SimpleCarState<T>* const state = dynamic_cast<const SimpleCarState<T>*>(
      &context.get_continuous_state_vector());
  DRAKE_ASSERT(state);

  // Obtain the input.
  const DrivingCommand<T>* const input =
      this->template EvalVectorInput<DrivingCommand>(context, 0);
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
  using std::abs;
  using std::cos;
  using std::max;
  using std::sin;

  // Sanity check our input.
  DRAKE_DEMAND(abs(input.steering_angle()) < M_PI);
  DRAKE_DEMAND(input.throttle() >= 0);
  DRAKE_DEMAND(input.brake() >= 0);

  // Determine the requested acceleration, using throttle and brake. Then
  // compute the smooth acceleration that the vehicle actually executes.
  const T desired_acceleration =
      config.max_acceleration() * (input.throttle() - input.brake());
  T smooth_acceleration =
    calc_smooth_acceleration(
        desired_acceleration, config.max_velocity(), config.velocity_limit_kp(),
        state.velocity());

  // Determine steering.
  const T saturated_steering_angle = math::saturate(
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
systems::System<AutoDiffXd>* SimpleCar<T>::DoToAutoDiffXd() const {
  return new SimpleCar<AutoDiffXd>;
}

template <typename T>
systems::System<symbolic::Expression>* SimpleCar<T>::DoToSymbolic() const {
  return new SimpleCar<symbolic::Expression>;
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
