#pragma once

/// @file
/// Template method implementations for endless_road_car.h.
/// Most users should only include that file, not this one.
/// For background, see http://drake.mit.edu/cxx_inl.html.

#include "endless_road_car.h"

#include <algorithm>
#include <cmath>

#include <Eigen/Geometry>

#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/automotive/maliput/utility/infinite_circuit_road.h"
#include "drake/common/drake_assert.h"
#include "drake/systems/framework/vector_base.h"

namespace drake {
namespace automotive {

template <typename T>
EndlessRoadCar<T>::EndlessRoadCar(
    const std::string& id,
    const maliput::utility::InfiniteCircuitRoad* road,
    const ControlType control_type,
    const SimpleCarConfig<T>& config)
    : id_(id), road_(road), control_type_(control_type), config_(config) {
  switch (control_type) {
    case kNone: { break; }
    case kUser: {
      this->DeclareInputPort(systems::kVectorValued,
                             DrivingCommandIndices::kNumCoordinates);
      break;
    }
    case kIdm: {
      this->DeclareInputPort(systems::kVectorValued,
                             EndlessRoadOracleOutputIndices::kNumCoordinates);
      break;
    }
    default: { DRAKE_ABORT(); }
  };
  this->DeclareOutputPort(systems::kVectorValued,
                          EndlessRoadCarStateIndices::kNumCoordinates);
}


template <typename T>
SimpleCarConfig<T> EndlessRoadCar<T>::get_default_config() {
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
bool EndlessRoadCar<T>::has_any_direct_feedthrough() const {
  return false;
}


template <typename T>
void EndlessRoadCar<T>::DoCalcOutput(const systems::Context<T>& context,
                                     systems::SystemOutput<T>* output) const {
  DRAKE_ASSERT_VOID(systems::System<T>::CheckValidContext(context));
  DRAKE_ASSERT_VOID(systems::System<T>::CheckValidOutput(output));

//DBG  std::cerr << "EvalOutput()  CAR " << id_ << std::endl;
  // Obtain the state.
  const systems::VectorBase<T>& context_state =
      context.get_continuous_state_vector();
  const EndlessRoadCarState<T>* const state =
      dynamic_cast<const EndlessRoadCarState<T>*>(&context_state);
  DRAKE_ASSERT(state);

  // Obtain the output pointer.
  EndlessRoadCarState<T>* const output_vector =
      dynamic_cast<EndlessRoadCarState<T>*>(output->GetMutableVectorData(0));
  DRAKE_ASSERT(output_vector);

  ImplCalcOutput(*state, output_vector);
}


template <typename T>
void EndlessRoadCar<T>::ImplCalcOutput(const EndlessRoadCarState<T>& state,
                                       EndlessRoadCarState<T>* output) const {
  output->set_value(state.get_value());
  // TODO(maddog)  Until we have a way to express this constraint to the
  //               simulator, forbid exposing negative speeds.
  if (output->speed() < 0.) {
    output->set_speed(0.);
  }
}


template <typename T>
void EndlessRoadCar<T>::DoCalcTimeDerivatives(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  DRAKE_ASSERT_VOID(systems::System<T>::CheckValidContext(context));

//DBG  std::cerr << "EvalTimeDerivatives()  CAR " << id_ << std::endl;

  // Obtain the state.
  const systems::VectorBase<T>& context_state =
      context.get_continuous_state_vector();
  const EndlessRoadCarState<T>* const state =
      dynamic_cast<const EndlessRoadCarState<T>*>(&context_state);
  DRAKE_ASSERT(state);

  // Obtain the result structure.
  DRAKE_ASSERT(derivatives != nullptr);
  systems::VectorBase<T>* const vector_derivatives =
      derivatives->get_mutable_vector();
  DRAKE_ASSERT(vector_derivatives);
  EndlessRoadCarState<T>* const rates =
      dynamic_cast<EndlessRoadCarState<T>*>(vector_derivatives);
  DRAKE_ASSERT(rates);

  const Accelerations accelerations = [&](){
    switch (control_type_) {
      case kNone: { return Accelerations(0., 0.); }
      case kUser: {
        // Obtain the DrivingCommand input.
        const systems::VectorBase<T>* const vector_input =
        this->EvalVectorInput(context, 0);
        DRAKE_ASSERT(vector_input);
        const DrivingCommand<T>* const input =
        dynamic_cast<const DrivingCommand<T>*>(vector_input);
        DRAKE_ASSERT(input);
        return ComputeUserAccelerations(*state, *input);
      }
      case kIdm: {
        // Obtain the EndlessRoadOracleOutput input.
        const systems::VectorBase<T>* const vector_input =
        this->EvalVectorInput(context, 0);
        DRAKE_ASSERT(vector_input);
        const EndlessRoadOracleOutput<T>* const input =
        dynamic_cast<const EndlessRoadOracleOutput<T>*>(vector_input);
        DRAKE_ASSERT(input);
        return ComputeIdmAccelerations(*state, *input);
      }
      default: { DRAKE_ABORT(); }
    };
  }();

  ImplCalcTimeDerivatives(*state, accelerations, rates);
}


template <typename T>
typename EndlessRoadCar<T>::Accelerations EndlessRoadCar<T>::ComputeUserAccelerations(
    const EndlessRoadCarState<T>& state,
    const DrivingCommand<T>& input) const {

  // Simplistic throttle and brake --> longitudinal acceleration.
  const T forward_acceleration = (input.throttle() - input.brake())
      * config_.max_acceleration();

  // Simplistic steering: lateral acceleration <-- centripetal acceleration.
  const T speed = state.speed();
  T sane_steering_angle = input.steering_angle();
  DRAKE_ASSERT(static_cast<T>(-M_PI) < sane_steering_angle);
  DRAKE_ASSERT(sane_steering_angle < static_cast<T>(M_PI));
  sane_steering_angle = std::min(
      sane_steering_angle, config_.max_abs_steering_angle());
  sane_steering_angle = std::max(
      sane_steering_angle, static_cast<T>(-config_.max_abs_steering_angle()));
  const T curvature = tan(sane_steering_angle) / config_.wheelbase();
  const T lateral_acceleration = speed * speed * curvature;

  return {forward_acceleration, lateral_acceleration};
}


template <typename T>
typename EndlessRoadCar<T>::Accelerations EndlessRoadCar<T>::ComputeIdmAccelerations(
    const EndlessRoadCarState<T>& state,
    const EndlessRoadOracleOutput<T>& input) const {

  // Adapted from https://en.wikipedia.org/wiki/Intelligent_driver_model
  const double v_0{30.0};  // desired velocity in free traffic.
  const double s_0{2.0};  // minimum desired net distance.
  const double h{1.0};  // desired time headway to vehicle in front.
  const double a{config_.max_acceleration()};  // max acceleration.
  const double b{3. * a};  // comfortable braking deceleration.
  const double delta{4.0};  // recommended choice of free-road exponent.
  // TODO(maddog)  This belongs somewhere.
  //  const double l_a{4.5};  // length of leading car.

  // Velocity difference to car ahead
  const double delta_v = input.delta_sigma_dot();
  // Net distance to car ahead (front bumper to rear bumper)
  const double s = input.net_delta_sigma();
  // Current velocity
  // TODO(maddog)  Demand that we are not pointing backwards, because we are
  //               not handling that correctly yet.
  DRAKE_DEMAND(std::cos(state.heading()) >= 0.);
  const double v = state.speed() * std::cos(state.heading());

  const double s_star = s_0 + (v * h) + (v * delta_v / 2. / std::sqrt(a * b));

  T forward_acceleration =
      a * (1. - std::pow(v / v_0, delta) - pow(s_star / s, 2.));

  const double kFudgeFactor = 9.0;
  if (std::abs(forward_acceleration) >
      kFudgeFactor * config_.max_acceleration()) {
    std::cerr << "TOO MUCH ACCEL "
              << "  fa " << forward_acceleration
              << "  max " << (kFudgeFactor * config_.max_acceleration())
              << "  v_0 " << v_0
              << "  delta_v " << delta_v
              << "  delta_s " << s
              << std::endl;
    // Clamp acceleration!
    forward_acceleration = std::copysign(
        kFudgeFactor * config_.max_acceleration(), forward_acceleration);
  }
  if ((state.speed() <= 0.) && (forward_acceleration < 0.)) {
    // Speed should be non-negative, so if it has dipped to/below zero,
    // then we want to clamp any negative acceleration.
    std::cerr << "CLAMP NEG ACCEL "
              << "  speed " << state.speed()
              << "  fa " << forward_acceleration
              << "  v_0 " << v_0
              << "  delta_v " << delta_v
              << "  delta_s " << s
              << std::endl;
    forward_acceleration = 0.;
  }

  return {forward_acceleration, 0.};
}


template <typename T>
void EndlessRoadCar<T>::ImplCalcTimeDerivatives(
    const EndlessRoadCarState<T>& state,
    const Accelerations& accelerations,
    EndlessRoadCarState<T>* rates) const {

  // Position + velocity ---> position derivatives.
  maliput::api::LanePosition lane_position(state.s(), state.r(), 0.);
  maliput::api::IsoLaneVelocity lane_velocity(
      state.speed() * std::cos(state.heading()),
      state.speed() * std::sin(state.heading()),
      0.);
  maliput::api::LanePosition derivatives =
      road_->lane()->EvalMotionDerivatives(lane_position, lane_velocity);

  // Magic Guard Rail:  If car is at driveable bounds, clamp r-derivative.
  maliput::api::RBounds bounds = road_->lane()->driveable_bounds(state.s());
  if (state.r() <= bounds.r_min) {
    derivatives.r = std::max(0., derivatives.r);
  } else if (state.r() >= bounds.r_max) {
    derivatives.r = std::min(0., derivatives.r);
  }

  rates->set_s(derivatives.s);
  rates->set_r(derivatives.r);
  // Ignore derivatives.h_, which should be zero anyhow.

  const double speed_dot = accelerations.forward;
  const double heading_dot =
      (state.speed() == 0.) ? 0. : (accelerations.lateral / state.speed());

  rates->set_speed(speed_dot);
  rates->set_heading(heading_dot);
}


template <typename T>
std::unique_ptr<systems::ContinuousState<T>>
EndlessRoadCar<T>::AllocateContinuousState() const {
  return std::make_unique<systems::ContinuousState<T>>(
      std::make_unique<EndlessRoadCarState<T>>());
}


template <typename T>
std::unique_ptr<systems::BasicVector<T>>
EndlessRoadCar<T>::AllocateOutputVector(
    const systems::OutputPortDescriptor<T>& descriptor) const {
  return std::make_unique<EndlessRoadCarState<T>>();
}

}  // namespace automotive
}  // namespace drake
