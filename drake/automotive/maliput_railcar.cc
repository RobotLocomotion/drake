#include "drake/automotive/maliput_railcar.h"

#include <algorithm>
#include <cmath>
#include <utility>

#include <Eigen/Geometry>

#include "drake/automotive/maliput/api/lane.h"
#include "drake/common/drake_assert.h"
#include "drake/math/roll_pitch_yaw_using_quaternion.h"
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
template <typename T> constexpr double MaliputRailcar<T>::kDefaultSpeed;

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
  pose->set_rotation(math::RollPitchYawToQuaternion(
      Vector3<T>(rotation.roll, rotation.pitch, rotation.yaw)));
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
  if (state.s() < 0 || state.s() >= lane_.length()) {
    rates->set_s(0);
  } else {
    const T speed = config.initial_speed();
    const LanePosition motion_derivatives = lane_.EvalMotionDerivatives(
        LanePosition(state.s(), config.r(), config.h()),
        IsoLaneVelocity(speed /* sigma_v */, 0 /* rho_v */, 0 /* eta_v */));
    // Since the railcar's IsoLaneVelocity's rho_v and eta_v values are both
    // zero, we expect the resulting motion derivative's r and h values to
    // also be zero. The IsoLaneVelocity's sigma_v, which may be non-zero, maps
    // to the motion derivative's s value.
    DRAKE_ASSERT(motion_derivatives.r == 0);
    DRAKE_ASSERT(motion_derivatives.h == 0);
    rates->set_s(motion_derivatives.s);
  }
  // TODO(liang.fok): Set this to the desired acceleration once it is a system
  // input.
  rates->set_speed(0);
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
  config->set_initial_speed(kDefaultSpeed);
}

// This section must match the API documentation in maliput_railcar.h.
template class MaliputRailcar<double>;

}  // namespace automotive
}  // namespace drake
