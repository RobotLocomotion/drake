#include "drake/automotive/dev/endless_road_car.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include <Eigen/Geometry>

#include "drake/automotive/dev/infinite_circuit_road.h"
#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/common/drake_assert.h"
#include "drake/common/text_logging.h"
#include "drake/systems/framework/vector_base.h"

namespace drake {
namespace automotive {

template <typename T>
EndlessRoadCar<T>::EndlessRoadCar(
    const std::string& id,
    const maliput::utility::InfiniteCircuitRoad* road,
    const ControlType control_type)
    : id_(id), road_(road), control_type_(control_type) {
  switch (control_type) {
    case kNone: {
      // No input ports.
      break;
    }
    case kUser: {
      // A single DrivingCommand input.
      this->DeclareInputPort(systems::kVectorValued,
                             DrivingCommandIndices::kNumCoordinates);
      break;
    }
    case kIdm: {
      // A single EndlessRoadOracleOutput as input.
      this->DeclareInputPort(systems::kVectorValued,
                             EndlessRoadOracleOutputIndices::kNumCoordinates);
      break;
    }
    default: { DRAKE_ABORT(); }
  }
  // A single EndlessRoadCarState output.
  this->DeclareOutputPort(systems::kVectorValued,
                          EndlessRoadCarStateIndices::kNumCoordinates);
}

template <typename T>
bool EndlessRoadCar<T>::DoHasDirectFeedthrough(
    const systems::SparsityMatrix* sparsity,
    int input_port, int output_port) const {
  return false;
}

template <typename T>
void EndlessRoadCar<T>::DoCalcOutput(const systems::Context<T>& context,
                                     systems::SystemOutput<T>* output) const {
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
  // TODO(maddog@tri.global) Until we have a way to express this constraint
  //                         to the simulator, forbid exposing negative speeds.
  if (output->speed() < 0.) {
    output->set_speed(0.);
  }
}


template <typename T>
void EndlessRoadCar<T>::DoCalcTimeDerivatives(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  // Obtain the parameters.
  const EndlessRoadCarConfig<T>& config =
      this->template GetNumericParameter<EndlessRoadCarConfig>(context, 0);

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

  const Accelerations accelerations = [&]() {
    switch (control_type_) {
      case kNone: {
        // No inputs; no accelerations.
        return Accelerations{0., 0.};
      }
      case kUser: {
        // Obtain the DrivingCommand input.
        const systems::VectorBase<T>* const vector_input =
            this->EvalVectorInput(context, 0);
        DRAKE_ASSERT(vector_input);
        const DrivingCommand<T>* const input =
            dynamic_cast<const DrivingCommand<T>*>(vector_input);
        DRAKE_ASSERT(input);
        return ComputeUserAccelerations(*state, *input, config);
      }
      case kIdm: {
        // Obtain the EndlessRoadOracleOutput input.
        const systems::VectorBase<T>* const vector_input =
            this->EvalVectorInput(context, 0);
        DRAKE_ASSERT(vector_input);
        const EndlessRoadOracleOutput<T>* const input =
            dynamic_cast<const EndlessRoadOracleOutput<T>*>(vector_input);
        DRAKE_ASSERT(input);
        return ComputeIdmAccelerations(*state, *input, config);
      }
      default: { DRAKE_ABORT(); }
    }
  }();

  ImplCalcTimeDerivatives(*state, accelerations, rates);
}


template <typename T>
typename EndlessRoadCar<T>::Accelerations
EndlessRoadCar<T>::ComputeUserAccelerations(
    const EndlessRoadCarState<T>& state,
    const DrivingCommand<T>& input,
    const EndlessRoadCarConfig<T>& config) const {

  // Simplistic throttle and brake yields longitudinal acceleration.
  const T forward_acceleration = input.acceleration();

  // Simplistic steering: centripetal acceleration --> lateral acceleration.
  const T speed = state.speed();
  T sane_steering_angle = input.steering_angle();
  DRAKE_DEMAND(static_cast<T>(-M_PI) < sane_steering_angle);
  DRAKE_DEMAND(sane_steering_angle < static_cast<T>(M_PI));
  sane_steering_angle = std::min(
      sane_steering_angle, config.max_abs_steering_angle());
  sane_steering_angle = std::max(
      sane_steering_angle, static_cast<T>(-config.max_abs_steering_angle()));
  const T curvature = tan(sane_steering_angle) / config.wheelbase();
  // Recall:  centripetal acceleration = v^2 / r.
  const T lateral_acceleration = speed * speed * curvature;

  return {forward_acceleration, lateral_acceleration};
}


template <typename T>
typename EndlessRoadCar<T>::Accelerations
EndlessRoadCar<T>::ComputeIdmAccelerations(
    const EndlessRoadCarState<T>& state,
    const EndlessRoadOracleOutput<T>& input,
    const EndlessRoadCarConfig<T>& config) const {

  // TODO(maddog@tri.global) These parameters are known to make the maliput
  //                         road demos in automotive_demo work well.  However,
  //                         there should be a better mechanism for managing
  //                         and/or deriving these values.
  // Adapted from https://en.wikipedia.org/wiki/Intelligent_driver_model
  const double v_0{30.0};  // desired velocity in free traffic.
  const double s_0{2.0};  // minimum desired net distance.
  const double h{1.0};  // desired time headway to vehicle in front.
  const double a{config.max_acceleration()};  // max acceleration.
  const double b{a};  // comfortable braking deceleration.
  const double delta{4.0};  // recommended choice of free-road exponent.

  // Velocity difference to car ahead
  const double delta_v = input.delta_sigma_dot();
  // Net distance to car ahead (front bumper to rear bumper)
  double s = input.net_delta_sigma();
  if (s <= 0.) {
    // Non-positive distances imply a collision or some other catastrophe.
    // Treat it as a very small positive value, which will lead to maximal
    // deceleration.
    drake::log()->warn(
        "Collision?  Clamping non-positive net_delta_sigma {}!", s);
    s = std::numeric_limits<double>::epsilon();
  }
  // TODO(maddog@tri.global) Demand that we are not pointing backwards, because
  //                         we are not handling that correctly yet.
  DRAKE_DEMAND(cos(state.heading()) >= 0.);
  // Current longitudinal velocity
  const double v = state.speed() * cos(state.heading());

  // TODO(maddog@tri.global)  Convene with IdmPlanner and decide on how to
  //                          de-duplicate this code.
  const double s_star = s_0 + (v * h) + (v * delta_v / 2. / std::sqrt(a * b));

  T forward_acceleration =
      a * (1. - std::pow(v / v_0, delta) - pow(s_star / s, 2.));

  // Clamp forward_acceleration to vehicle limits.
  forward_acceleration = std::min(forward_acceleration,
                                  config.max_acceleration());
  forward_acceleration = std::max(forward_acceleration,
                                  -config.max_deceleration());

  // Lateral acceleration is always zero, since we're just doing car-following.
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
      state.speed() * cos(state.heading()),
      state.speed() * sin(state.heading()),
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

  const double heading_dot =
      (state.speed() == 0.) ? 0. : (accelerations.lateral / state.speed());
  double speed_dot = accelerations.forward;
  // TODO(maddog@tri.global) Speed should be non-negative (because braking
  //                         deceleration should cause car to stop, not to
  //                         travel in reverse.)  Until we have a way to
  //                         express this constraint to the simulator (and/or
  //                         integrator, etc), clamp negative accelerations
  //                         if speed <= 0.
  if ((state.speed() <= 0.) && (speed_dot < 0.)) {
    drake::log()->warn(
        "Clamping negative acceleration for id {}:  speed {}  forward accel {}",
        id_, state.speed(), speed_dot);
    speed_dot = 0.;
  }

  rates->set_heading(heading_dot);
  rates->set_speed(speed_dot);
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


template <typename T>
std::unique_ptr<systems::Parameters<T>>
EndlessRoadCar<T>::AllocateParameters() const {
  auto params = std::make_unique<EndlessRoadCarConfig<T>>();
  return std::make_unique<systems::Parameters<T>>(std::move(params));
}


template <typename T>
void EndlessRoadCar<T>::SetDefaultParameters(
    const systems::LeafContext<T>& context,
    systems::Parameters<T>* params) const {
  EndlessRoadCarConfig<T>* config = dynamic_cast<EndlessRoadCarConfig<T>*>(
      params->get_mutable_numeric_parameter(0));
  DRAKE_DEMAND(config != nullptr);
  SetDefaultParameters(config);
}


template <typename T>
void EndlessRoadCar<T>::SetDefaultParameters(EndlessRoadCarConfig<T>* config) {
  DRAKE_DEMAND(config != nullptr);
  constexpr double kInchToMeter = 0.0254;
  constexpr double kDegToRadian = 0.0174532925199;
  // This approximates a 2010 Toyota Prius.
  config->set_wheelbase(static_cast<T>(106.3 * kInchToMeter));
  config->set_max_abs_steering_angle(static_cast<T>(27 * kDegToRadian));
  config->set_max_velocity(static_cast<T>(45.0));  // meters/second
  config->set_max_acceleration(static_cast<T>(2.5));  // meters/second**2
  config->set_max_deceleration(static_cast<T>(8.0));  // meters/second**2
}


// These instantiations must match the API documentation in endless_road_car.h.
template class EndlessRoadCar<double>;
// TODO(maddog@tri.global)  Eventually, AutoDiffXd, etc.

}  // namespace automotive
}  // namespace drake
