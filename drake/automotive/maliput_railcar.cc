#include "drake/automotive/maliput_railcar.h"

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

#include "drake/automotive/calc_smooth_acceleration.h"
#include "drake/automotive/maliput/api/lane.h"
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

// Linkage for MaliputRailcar constants.
template <typename T> constexpr T MaliputRailcar<T>::kDefaultR;
template <typename T> constexpr T MaliputRailcar<T>::kDefaultH;
template <typename T> constexpr T MaliputRailcar<T>::kDefaultInitialS;
template <typename T> constexpr T MaliputRailcar<T>::kDefaultInitialSpeed;
template <typename T> constexpr T MaliputRailcar<T>::kDefaultMaxSpeed;
template <typename T> constexpr T MaliputRailcar<T>::kDefaultVelocityLimitKp;

template <typename T>
MaliputRailcar<T>::MaliputRailcar(const LaneDirection& initial_lane_direction,
    double start_time)
    : start_time_(start_time),
      initial_lane_direction_(initial_lane_direction) {
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
  const MaliputRailcarConfig<T>& config =
      this->template GetNumericParameter<MaliputRailcarConfig>(context, 0);

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
  ImplCalcPose(config, *state, lane_direction, pose_vector);
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
void MaliputRailcar<T>::ImplCalcPose(const MaliputRailcarConfig<T>& config,
    const MaliputRailcarState<T>& state, const LaneDirection& lane_direction,
    PoseVector<T>* pose) const {
  const LanePosition lane_position(state.s(), config.r(), config.h());
  const GeoPosition geo_position =
      lane_direction.lane->ToGeoPosition(lane_position);
  const Rotation rotation =
      lane_direction.lane->GetOrientation(lane_position);

  pose->set_translation(
      Eigen::Translation<T, 3>(geo_position.x, geo_position.y, geo_position.z));
  pose->set_rotation(math::RollPitchYawToQuaternion(
      Vector3<T>(rotation.roll, rotation.pitch, rotation.yaw)));
}

template <typename T>
void MaliputRailcar<T>::DoCalcTimeDerivatives(
    const Context<T>& context, ContinuousState<T>* derivatives) const {
  DRAKE_ASSERT(derivatives != nullptr);

  // Obtains the parameters.
  const MaliputRailcarConfig<T>& config =
      this->template GetNumericParameter<MaliputRailcarConfig>(context, 0);

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

  if (context.get_time() < T(start_time_)) {
    rates->set_s(T(0));
    rates->set_speed(T(0));
  } else {
    ImplCalcTimeDerivatives(config, *state, lane_direction, *input, rates);
  }
}

template<typename T>
void MaliputRailcar<T>::ImplCalcTimeDerivatives(
    const MaliputRailcarConfig<T>& config,
    const MaliputRailcarState<T>& state,
    const LaneDirection& lane_direction,
    const BasicVector<T>& input,
    MaliputRailcarState<T>* rates) const {
  if (state.s() < 0 || state.s() >= lane_direction.lane->length()) {
    rates->set_s(0);
  } else {
    const T speed = state.speed();
    const T sigma_v = cond(lane_direction.with_s, speed, -speed);
    const LanePosition motion_derivatives =
        lane_direction.lane->EvalMotionDerivatives(
            LanePosition(state.s(), config.r(), config.h()),
            IsoLaneVelocity(sigma_v, 0 /* rho_v */, 0 /* eta_v */));
    // Since the railcar's IsoLaneVelocity's rho_v and eta_v values are both
    // zero, we expect the resulting motion derivative's r and h values to
    // also be zero. The IsoLaneVelocity's sigma_v, which may be non-zero, maps
    // to the motion derivative's s value.
    DRAKE_ASSERT(motion_derivatives.r == 0);
    DRAKE_ASSERT(motion_derivatives.h == 0);
    rates->set_s(motion_derivatives.s);
  }

  const T desired_acceleration = input.GetAtIndex(0);
  const T smooth_acceleration = calc_smooth_acceleration(
      desired_acceleration, config.max_speed(), config.velocity_limit_kp(),
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
std::unique_ptr<systems::Parameters<T>>
MaliputRailcar<T>::AllocateParameters() const {
  auto params = std::make_unique<MaliputRailcarConfig<T>>();
  return std::make_unique<Parameters<T>>(std::move(params));
}

template <typename T>
bool MaliputRailcar<T>::DoHasDirectFeedthrough(const SparsityMatrix* sparsity,
    int input_port, int output_port) const {
  return false;
}

template <typename T>
void MaliputRailcar<T>::SetDefaultParameters(
    const LeafContext<T>& context, Parameters<T>* params) const {
  MaliputRailcarConfig<T>* config = dynamic_cast<MaliputRailcarConfig<T>*>(
      params->get_mutable_numeric_parameter(0));
  DRAKE_DEMAND(config != nullptr);
  SetDefaultParameters(config);
}

template <typename T>
void MaliputRailcar<T>::SetDefaultParameters(MaliputRailcarConfig<T>* config) {
  DRAKE_DEMAND(config != nullptr);
  config->set_r(kDefaultR);
  config->set_h(kDefaultH);
  config->set_initial_speed(kDefaultInitialSpeed);
  config->set_max_speed(kDefaultMaxSpeed);
  config->set_velocity_limit_kp(kDefaultVelocityLimitKp);
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

// This section must match the API documentation in maliput_railcar.h.
template class MaliputRailcar<double>;

}  // namespace automotive
}  // namespace drake
