#include "drake/automotive/maliput_railcar.h"

#include <algorithm>
#include <cmath>
#include <utility>

#include <Eigen/Geometry>

#include "drake/automotive/maliput/api/lane.h"
#include "drake/common/drake_assert.h"
#include "drake/math/roll_pitch_yaw_not_using_quaternion.h"
#include "drake/systems/framework/vector_base.h"

namespace drake {

using maliput::api::GeoPosition;
using maliput::api::IsoLaneVelocity;
using maliput::api::Lane;
using maliput::api::LanePosition;
using maliput::api::Rotation;
using systems::rendering::PoseVector;

namespace automotive {

// Linkage for MaliputRailcar constants.
template <typename T> constexpr double MaliputRailcar<T>::kDefaultR;
template <typename T> constexpr double MaliputRailcar<T>::kDefaultH;
template <typename T> constexpr double MaliputRailcar<T>::kDefaultSDot;

template <typename T>
MaliputRailcar<T>::MaliputRailcar(const Lane& lane, double start_time)
    : lane_(lane), start_time_(start_time) {
  state_output_port_index_ =
      this->DeclareOutputPort(systems::kVectorValued,
          MaliputRailcarStateIndices::kNumCoordinates).get_index();
  pose_output_port_index_ =
      this->DeclareOutputPort(systems::kVectorValued,
          PoseVector<T>::kSize).get_index();
}

template <typename T>
const systems::OutputPortDescriptor<T>& MaliputRailcar<T>::state_output()
    const {
  return this->get_output_port(state_output_port_index_);
}

template <typename T>
const systems::OutputPortDescriptor<T>& MaliputRailcar<T>::pose_output()
    const {
  return this->get_output_port(pose_output_port_index_);
}

template <typename T>
void MaliputRailcar<T>::DoCalcOutput(const systems::Context<T>& context,
    systems::SystemOutput<T>* output) const {
  // Obtains the parameters.
  const MaliputRailcarConfig<T>& config =
      this->template GetNumericParameter<MaliputRailcarConfig>(context, 0);

  // Obtains the state.
  const MaliputRailcarState<T>* const state =
      dynamic_cast<const MaliputRailcarState<T>*>(
          &context.get_continuous_state_vector());
  DRAKE_ASSERT(state != nullptr);

  // Obtains the output vectors.
  MaliputRailcarState<T>* const state_vector =
      dynamic_cast<MaliputRailcarState<T>*>(
          output->GetMutableVectorData(state_output_port_index_));
  DRAKE_ASSERT(state_vector != nullptr);

  ImplCalcOutput(*state, state_vector);

  PoseVector<T>* const pose_vector =
      dynamic_cast<PoseVector<T>*>(
          output->GetMutableVectorData(pose_output_port_index_));
  DRAKE_ASSERT(pose_vector != nullptr);

  ImplCalcPose(config, *state, pose_vector);
}

template <typename T>
void MaliputRailcar<T>::ImplCalcOutput(const MaliputRailcarState<T>& state,
    MaliputRailcarState<T>* output) const {
  output->set_value(state.get_value());
}

template <typename T>
void MaliputRailcar<T>::ImplCalcPose(const MaliputRailcarConfig<T>& config,
    const MaliputRailcarState<T>& state, PoseVector<T>* pose) const {
  const LanePosition lane_position(state.s(), config.r(), config.h());
  const GeoPosition geo_position = lane_.ToGeoPosition(lane_position);
  const Rotation rotation = lane_.GetOrientation(lane_position);

  pose->set_translation(
      Eigen::Translation<T, 3>(geo_position.x, geo_position.y, geo_position.z));

  const Vector4<T> quaternion =
      math::rpy2quat(Vector3<T>(rotation.roll, rotation.pitch, rotation.yaw));
  pose->set_rotation(Eigen::Quaternion<T>(
      quaternion(0), quaternion(1), quaternion(2), quaternion(3)));
}

template <typename T>
void MaliputRailcar<T>::DoCalcTimeDerivatives(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  DRAKE_ASSERT(derivatives != nullptr);

  // Obtains the parameters.
  const MaliputRailcarConfig<T>& config =
      this->template GetNumericParameter<MaliputRailcarConfig>(context, 0);

  // Obtains the state.
  const systems::VectorBase<T>& context_state =
      context.get_continuous_state_vector();
  const MaliputRailcarState<T>* const state =
      dynamic_cast<const MaliputRailcarState<T>*>(&context_state);
  DRAKE_ASSERT(state != nullptr);

  // Obtains the result structure.
  systems::VectorBase<T>* const vector_derivatives =
      derivatives->get_mutable_vector();
  DRAKE_ASSERT(vector_derivatives);
  MaliputRailcarState<T>* const rates =
      dynamic_cast<MaliputRailcarState<T>*>(vector_derivatives);
  DRAKE_ASSERT(rates != nullptr);

  ImplCalcTimeDerivatives(config, *state, rates);
}

template<typename T>
void MaliputRailcar<T>::ImplCalcTimeDerivatives(
    const MaliputRailcarConfig<T>& config,
    const MaliputRailcarState<T>& state,
    MaliputRailcarState<T>* rates) const {
  const T s_dot = config.initial_s_dot();
  if (state.s() < 0 || state.s() >= lane_.length()) {
    rates->set_s(0);
  } else {
    rates->set_s(s_dot);
  }
  // TODO(liang.fok): Set this to s_dot_dot once it is a system input.
  rates->set_s_dot(0);
}

template <typename T>
std::unique_ptr<systems::ContinuousState<T>>
MaliputRailcar<T>::AllocateContinuousState() const {
  return std::make_unique<systems::ContinuousState<T>>(
      std::make_unique<MaliputRailcarState<T>>());
}

template <typename T>
std::unique_ptr<systems::BasicVector<T>>
MaliputRailcar<T>::AllocateOutputVector(
    const systems::OutputPortDescriptor<T>& descriptor) const {
  DRAKE_DEMAND(descriptor.get_index() <= 1);
  if (descriptor.get_index() == state_output_port_index_) {
    return std::make_unique<MaliputRailcarState<T>>();
  } else if (descriptor.get_index() == pose_output_port_index_) {
    return std::make_unique<PoseVector<T>>();
  }
  return nullptr;
}

template <typename T>
std::unique_ptr<systems::Parameters<T>>
MaliputRailcar<T>::AllocateParameters() const {
  auto params = std::make_unique<MaliputRailcarConfig<T>>();
  return std::make_unique<systems::Parameters<T>>(std::move(params));
}

template <typename T>
void MaliputRailcar<T>::SetDefaultParameters(
    const systems::LeafContext<T>& context,
    systems::Parameters<T>* params) const {
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
  config->set_initial_s_dot(kDefaultSDot);
}

// This section must match the API documentation in maliput_railcar.h.
template class MaliputRailcar<double>;

}  // namespace automotive
}  // namespace drake
