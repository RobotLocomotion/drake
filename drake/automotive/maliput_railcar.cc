#include "drake/automotive/maliput_railcar.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

#include "drake/automotive/calc_smooth_acceleration.h"
#include "drake/automotive/maliput/api/branch_point.h"
#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/common/cond.h"
#include "drake/common/drake_assert.h"
#include "drake/math/roll_pitch_yaw_using_quaternion.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/value.h"
#include "drake/systems/framework/vector_base.h"

namespace drake {

using maliput::api::GeoPosition;
using maliput::api::IsoLaneVelocity;
using maliput::api::Lane;
using maliput::api::LaneEnd;
using maliput::api::LanePosition;
using maliput::api::Rotation;
using systems::BasicVector;
using systems::Context;
using systems::ContinuousState;
using systems::InputPortDescriptor;
using systems::LeafContext;
using systems::OutputPortDescriptor;
using systems::Parameters;
using systems::SparsityMatrix;
using systems::State;
using systems::SystemOutput;
using systems::VectorBase;
using systems::rendering::PoseVector;

namespace automotive {

template <typename T> constexpr T MaliputRailcar<T>::kDefaultInitialS;
template <typename T> constexpr T MaliputRailcar<T>::kDefaultInitialSpeed;
template <typename T> constexpr double MaliputRailcar<T>::kLaneEndEpsilon;
template <typename T> constexpr double MaliputRailcar<T>::kTimeEpsilon;

template <typename T>
MaliputRailcar<T>::MaliputRailcar(const LaneDirection& initial_lane_direction)
    : initial_lane_direction_(initial_lane_direction) {
  command_input_port_index_ =
      this->DeclareInputPort(systems::kVectorValued, 1).get_index();
  state_output_port_index_ =
      this->DeclareVectorOutputPort(MaliputRailcarState<T>()).get_index();
  lane_state_output_port_index_ =
      this->DeclareAbstractOutputPort(
          systems::Value<LaneDirection>(initial_lane_direction)).get_index();
  pose_output_port_index_ =
      this->DeclareVectorOutputPort(PoseVector<T>()).get_index();
  this->DeclareContinuousState(MaliputRailcarState<T>());
  this->DeclareNumericParameter(MaliputRailcarParams<T>());
}

template <typename T>
const InputPortDescriptor<T>& MaliputRailcar<T>::command_input() const {
  return this->get_input_port(command_input_port_index_);
}

template <typename T>
const OutputPortDescriptor<T>& MaliputRailcar<T>::state_output() const {
  return this->get_output_port(state_output_port_index_);
}

template <typename T>
const OutputPortDescriptor<T>& MaliputRailcar<T>::lane_state_output() const {
  return this->get_output_port(lane_state_output_port_index_);
}

template <typename T>
const OutputPortDescriptor<T>& MaliputRailcar<T>::pose_output() const {
  return this->get_output_port(pose_output_port_index_);
}

template <typename T>
void MaliputRailcar<T>::DoCalcOutput(const Context<T>& context,
    SystemOutput<T>* output) const {
  // Obtains the parameters.
  const MaliputRailcarParams<T>& params =
      this->template GetNumericParameter<MaliputRailcarParams>(context, 0);

  // Obtains the state.
  const MaliputRailcarState<T>* const state =
      dynamic_cast<const MaliputRailcarState<T>*>(
          &context.get_continuous_state_vector());
  DRAKE_ASSERT(state != nullptr);

  const LaneDirection& lane_direction =
      context.template get_abstract_state<LaneDirection>(0);

  // Obtains and updates the output vectors.
  MaliputRailcarState<T>* const state_vector =
      dynamic_cast<MaliputRailcarState<T>*>(
          output->GetMutableVectorData(state_output_port_index_));
  DRAKE_ASSERT(state_vector != nullptr);
  ImplCalcOutput(*state, state_vector);

  LaneDirection& lane_direction_output =
      output->GetMutableData(lane_state_output_port_index_)->
          template GetMutableValue<LaneDirection>();
  ImplCalcLaneOutput(lane_direction, &lane_direction_output);

  PoseVector<T>* const pose_vector =
      dynamic_cast<PoseVector<T>*>(
          output->GetMutableVectorData(pose_output_port_index_));
  DRAKE_ASSERT(pose_vector != nullptr);
  ImplCalcPose(params, *state, lane_direction, pose_vector);
}

template <typename T>
void MaliputRailcar<T>::ImplCalcOutput(const MaliputRailcarState<T>& state,
    MaliputRailcarState<T>* output) const {
  output->set_value(state.get_value());

  // Don't allow small negative speed to escape our state.
  DRAKE_ASSERT(state.speed() >= -1e-3);
  using std::max;
  output->set_speed(max(T(0), state.speed()));
}

template <typename T>
void MaliputRailcar<T>::ImplCalcLaneOutput(const LaneDirection& lane_direction,
    LaneDirection* output) const {
  *output = lane_direction;
}

template <typename T>
T MaliputRailcar<T>::CalcR(const MaliputRailcarParams<T>& params,
    const LaneDirection& lane_direction) const {
  if (lane_direction.with_s == initial_lane_direction_.with_s) {
    return params.r();
  } else {
    return -params.r();
  }
}

template <typename T>
void MaliputRailcar<T>::ImplCalcPose(const MaliputRailcarParams<T>& params,
    const MaliputRailcarState<T>& state, const LaneDirection& lane_direction,
    PoseVector<T>* pose) const {

  const LanePosition lane_position(state.s(), CalcR(params, lane_direction),
                                   params.h());
  const GeoPosition geo_position =
      lane_direction.lane->ToGeoPosition(lane_position);
  const Rotation rotation =
      lane_direction.lane->GetOrientation(lane_position);

  using std::atan2;
  using std::sin;
  using std::cos;

  // Adjust the rotation based on whether the vehicle is traveling with s or
  // against s.
  const Rotation adjusted_rotation =
      (lane_direction.with_s ? rotation :
          Rotation(-rotation.roll,
                   -rotation.pitch,
                   atan2(-sin(rotation.yaw), -cos(rotation.yaw))));
  pose->set_translation(
      Eigen::Translation<T, 3>(geo_position.x, geo_position.y, geo_position.z));
  pose->set_rotation(math::RollPitchYawToQuaternion(
      Vector3<T>(adjusted_rotation.roll, adjusted_rotation.pitch,
                 adjusted_rotation.yaw)));
}

template <typename T>
void MaliputRailcar<T>::DoCalcTimeDerivatives(
    const Context<T>& context, ContinuousState<T>* derivatives) const {
  DRAKE_ASSERT(derivatives != nullptr);

  // Obtains the parameters.
  const MaliputRailcarParams<T>& params =
      this->template GetNumericParameter<MaliputRailcarParams>(context, 0);

  // Obtains the state.
  const VectorBase<T>& context_state = context.get_continuous_state_vector();
  const MaliputRailcarState<T>* const state =
      dynamic_cast<const MaliputRailcarState<T>*>(&context_state);
  DRAKE_ASSERT(state != nullptr);

  const LaneDirection& lane_direction =
      context.template get_abstract_state<LaneDirection>(0);

  // Obtains the input.
  const BasicVector<T>* input =
      this->template EvalVectorInput<BasicVector>(context,
          command_input_port_index_);

  // Allocates and uses a BasicVector containing a zero acceleration command in
  // case the input contains nullptr.
  const auto default_input = BasicVector<T>::Make(0);
  if (input == nullptr) {
    input = default_input.get();
  }
  DRAKE_ASSERT(input->size() == 1);

  // Obtains the result structure.
  VectorBase<T>* const vector_derivatives = derivatives->get_mutable_vector();
  DRAKE_ASSERT(vector_derivatives);
  MaliputRailcarState<T>* const rates =
      dynamic_cast<MaliputRailcarState<T>*>(vector_derivatives);
  DRAKE_ASSERT(rates != nullptr);

  ImplCalcTimeDerivatives(params, *state, lane_direction, *input, rates);
}

template<typename T>
void MaliputRailcar<T>::ImplCalcTimeDerivatives(
    const MaliputRailcarParams<T>& params,
    const MaliputRailcarState<T>& state,
    const LaneDirection& lane_direction,
    const BasicVector<T>& input,
    MaliputRailcarState<T>* rates) const {
  const T speed = state.speed();
  const T sigma_v = cond(lane_direction.with_s, speed, -speed);
  const LanePosition motion_derivatives =
      lane_direction.lane->EvalMotionDerivatives(
          LanePosition(state.s(), CalcR(params, lane_direction), params.h()),
          IsoLaneVelocity(sigma_v, 0 /* rho_v */, 0 /* eta_v */));
  // Since the railcar's IsoLaneVelocity's rho_v and eta_v values are both
  // zero, we expect the resulting motion derivative's r and h values to
  // also be zero. The IsoLaneVelocity's sigma_v, which may be non-zero, maps
  // to the motion derivative's s value.
  DRAKE_ASSERT(motion_derivatives.r == 0);
  DRAKE_ASSERT(motion_derivatives.h == 0);
  rates->set_s(motion_derivatives.s);

  const T desired_acceleration = input.GetAtIndex(0);
  const T smooth_acceleration = calc_smooth_acceleration(
      desired_acceleration, params.max_speed(), params.velocity_limit_kp(),
      state.speed());
  rates->set_speed(smooth_acceleration);
}

template <typename T>
std::unique_ptr<systems::AbstractValues>
MaliputRailcar<T>::AllocateAbstractState() const {
  std::vector<std::unique_ptr<systems::AbstractValue>> abstract_values;
  const LaneDirection lane_direction;
  abstract_values.push_back(std::unique_ptr<systems::AbstractValue>(
      std::make_unique<systems::Value<LaneDirection>>(lane_direction)));
  return std::make_unique<systems::AbstractValues>(std::move(abstract_values));
}

template <typename T>
bool MaliputRailcar<T>::DoHasDirectFeedthrough(const SparsityMatrix* sparsity,
    int input_port, int output_port) const {
  return false;
}

template <typename T>
void MaliputRailcar<T>::SetDefaultState(const Context<T>& context,
    State<T>* state) const {
  MaliputRailcarState<T>* railcar_state =
      dynamic_cast<MaliputRailcarState<T>*>(
          state->get_mutable_continuous_state()->get_mutable_vector());
  DRAKE_DEMAND(railcar_state != nullptr);
  SetDefaultState(railcar_state);

  LaneDirection& lane_direction =
      state->get_mutable_abstract_state()->get_mutable_value(0).
          template GetMutableValue<LaneDirection>();
  lane_direction = initial_lane_direction_;
}

template <typename T>
void MaliputRailcar<T>::SetDefaultState(
    MaliputRailcarState<T>* railcar_state) {
  railcar_state->set_s(kDefaultInitialS);
  railcar_state->set_speed(kDefaultInitialSpeed);
}

// TODO(liang.fok): Switch to guard functions once they are available. The
// following computes an estimate of when the vehicle will reach the end of
// its lane. This estimate will be off when r != 0 and the lane is very
// curvy because the scale factors used in Lane::EvalMotionDerivatives() will
// not be constant.
//
// Another reason why the estimate will be off is the acceleration of the
// vehicle is not considered (see #5532).
template <typename T>
void MaliputRailcar<T>::DoCalcNextUpdateTime(const systems::Context<T>& context,
    systems::UpdateActions<T>* actions) const {
  const VectorBase<T>& context_state = context.get_continuous_state_vector();
  const MaliputRailcarState<T>* const state =
      dynamic_cast<const MaliputRailcarState<T>*>(&context_state);
  DRAKE_ASSERT(state != nullptr);
  if (state->speed() == 0) {
    actions->time = T(std::numeric_limits<double>::infinity());
  } else {
    const MaliputRailcarParams<T>& params =
        this->template GetNumericParameter<MaliputRailcarParams>(context, 0);
    const LaneDirection& lane_direction =
        context.template get_abstract_state<LaneDirection>(0);

    const T& s = state->s();
    const T& speed = state->speed();
    const maliput::api::Lane* lane = lane_direction.lane;
    const bool with_s = lane_direction.with_s;

    DRAKE_ASSERT(lane != nullptr);

    // Computes `s_dot`, the time derivative of `s`.
    const T sigma_v = cond(with_s, speed, -speed);
    const LanePosition motion_derivatives =
        lane_direction.lane->EvalMotionDerivatives(
            LanePosition(s, CalcR(params, lane_direction), params.h()),
            IsoLaneVelocity(sigma_v, 0 /* rho_v */, 0 /* eta_v */));
    const T s_dot = motion_derivatives.s;

    const T distance = cond(with_s, T(lane->length()) - s, -s);

    actions->time = context.get_time() + distance / s_dot;
  }

  // Gracefully handle the situation when the next update time is equal to the
  // current time. Since the integrator requires that the next update time be
  // strictly greater than the current time, a small time epsilon is used.
  if (actions->time == context.get_time()) {
    actions->time = context.get_time() + kTimeEpsilon;
  }
  actions->events.push_back(systems::DiscreteEvent<T>());
  actions->events.back().action =
      systems::DiscreteEvent<T>::kUnrestrictedUpdateAction;
}

template <typename T>
void MaliputRailcar<T>::DoCalcUnrestrictedUpdate(
    const systems::Context<T>& context,
    systems::State<T>* next_state) const {
  const MaliputRailcarState<T>* const current_railcar_state =
      dynamic_cast<const MaliputRailcarState<T>*>(
          &context.get_continuous_state_vector());
  DRAKE_ASSERT(current_railcar_state != nullptr);
  const LaneDirection& current_lane_direction =
      context.template get_abstract_state<LaneDirection>(0);
  DRAKE_ASSERT(current_lane_direction.lane != nullptr);
  const bool current_with_s = current_lane_direction.with_s;
  const double current_s = current_railcar_state->s();
  const double current_length = current_lane_direction.lane->length();

  // Copies the present state into the new one.
  next_state->CopyFrom(context.get_state());

  ContinuousState<T>* cs = next_state->get_mutable_continuous_state();
  DRAKE_ASSERT(cs != nullptr);
  VectorBase<T>* cv = cs->get_mutable_vector();
  DRAKE_ASSERT(cv != nullptr);
  MaliputRailcarState<T>* const next_railcar_state =
      dynamic_cast<MaliputRailcarState<T>*>(cv);
  DRAKE_ASSERT(next_railcar_state != nullptr);

  // Handles the case where no lane change or speed adjustment is necessary. No
  // lane change is necessary when the vehicle is more than epilon away from the
  // next lane boundary.
  if ((current_with_s && current_s < current_length - kLaneEndEpsilon) ||
      (!current_with_s && current_s > kLaneEndEpsilon)) {
    return;
  }

  // Sets the speed to be zero if the car is at or is after the end of the road.
  if (current_with_s) {
    const int num_branches = current_lane_direction.lane->
        GetOngoingBranches(LaneEnd::kFinish)->size();
    if (num_branches == 0 && current_s >= current_length - kLaneEndEpsilon) {
      next_railcar_state->set_speed(0);
    }
  } else {
    const int num_branches = current_lane_direction.lane->
        GetOngoingBranches(LaneEnd::kStart)->size();
    if (num_branches == 0 && current_s <= kLaneEndEpsilon) {
      next_railcar_state->set_speed(0);
    }
  }

  if (next_railcar_state->speed() != 0) {
    LaneDirection& next_lane_direction =
        next_state->template get_mutable_abstract_state<LaneDirection>(0);
    // TODO(liang.fok) Generalize the following to support the selection of
    // non-default branches or non-zero ongoing branches. See #5702.
    std::unique_ptr<LaneEnd> next_branch;
    if (current_with_s) {
      next_branch = current_lane_direction.lane->GetDefaultBranch(
          LaneEnd::kFinish);
      if (next_branch == nullptr) {
        const maliput::api::LaneEndSet* ongoing_lanes =
            current_lane_direction.lane->GetOngoingBranches(LaneEnd::kFinish);
        if (ongoing_lanes != nullptr) {
          if (ongoing_lanes->size() > 0) {
            next_branch = std::make_unique<LaneEnd>(ongoing_lanes->get(0));
          }
        }
      }
    } else {
      next_branch = current_lane_direction.lane->GetDefaultBranch(
          LaneEnd::kStart);
      if (next_branch == nullptr) {
        const maliput::api::LaneEndSet* ongoing_lanes =
            current_lane_direction.lane->GetOngoingBranches(LaneEnd::kStart);
        if (ongoing_lanes != nullptr) {
          if (ongoing_lanes->size() > 0) {
            next_branch = std::make_unique<LaneEnd>(ongoing_lanes->get(0));
          }
        }
      }
    }

    if (next_branch == nullptr) {
      DRAKE_ABORT_MSG("MaliputRailcar::DoCalcUnrestrictedUpdate: ERROR: "
          "Vehicle should switch lanes but no default or ongoing branch "
          "exists.");
    } else {
      next_lane_direction.lane = next_branch->lane;
      if (next_branch->end == LaneEnd::kStart) {
        next_lane_direction.with_s = true;
        next_railcar_state->set_s(0);
      } else {
        next_lane_direction.with_s = false;
        next_railcar_state->set_s(next_lane_direction.lane->length());
      }
    }
  }
}

// This section must match the API documentation in maliput_railcar.h.
template class MaliputRailcar<double>;

}  // namespace automotive
}  // namespace drake
