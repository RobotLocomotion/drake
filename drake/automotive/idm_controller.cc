#include "drake/automotive/idm_controller.h"

#include <algorithm>
#include <limits>
#include <utility>
#include <vector>

#include "drake/common/cond.h"
#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/extract_double.h"
#include "drake/common/symbolic.h"

namespace drake {
namespace automotive {

using maliput::api::GeoPosition;
using maliput::api::LanePositionT;
using maliput::api::RoadGeometry;
using maliput::api::RoadPosition;
using maliput::api::Rotation;
using systems::rendering::FrameVelocity;
using systems::rendering::PoseBundle;
using systems::rendering::PoseVector;

static constexpr int kIdmParamsIndex{0};

template <typename T>
IdmController<T>::IdmController(const RoadGeometry& road)
    : systems::LeafSystem<T>(
          systems::SystemTypeTag<automotive::IdmController>{}),
      road_(road),
      ego_pose_index_{
          this->DeclareVectorInputPort(PoseVector<T>()).get_index()},
      ego_velocity_index_{
          this->DeclareVectorInputPort(FrameVelocity<T>()).get_index()},
      traffic_index_{this->DeclareAbstractInputPort().get_index()},
      acceleration_index_{
          this->DeclareVectorOutputPort(systems::BasicVector<T>(1),
                                        &IdmController::CalcAcceleration)
              .get_index()} {
  this->DeclareNumericParameter(IdmPlannerParameters<T>());
}

template <typename T>
IdmController<T>::~IdmController() {}

template <typename T>
const systems::InputPortDescriptor<T>& IdmController<T>::ego_pose_input()
    const {
  return systems::System<T>::get_input_port(ego_pose_index_);
}

template <typename T>
const systems::InputPortDescriptor<T>& IdmController<T>::ego_velocity_input()
    const {
  return systems::System<T>::get_input_port(ego_velocity_index_);
}

template <typename T>
const systems::InputPortDescriptor<T>& IdmController<T>::traffic_input() const {
  return systems::System<T>::get_input_port(traffic_index_);
}

template <typename T>
const systems::OutputPort<T>& IdmController<T>::acceleration_output() const {
  return systems::System<T>::get_output_port(acceleration_index_);
}

template <typename T>
void IdmController<T>::CalcAcceleration(
    const systems::Context<T>& context,
    systems::BasicVector<T>* accel_output) const {
  // Obtain the parameters.
  const IdmPlannerParameters<T>& idm_params =
      this->template GetNumericParameter<IdmPlannerParameters>(context,
                                                               kIdmParamsIndex);
  // Obtain the input/output data structures.
  const PoseVector<T>* const ego_pose =
      this->template EvalVectorInput<PoseVector>(context, ego_pose_index_);
  DRAKE_ASSERT(ego_pose != nullptr);

  const FrameVelocity<T>* const ego_velocity =
      this->template EvalVectorInput<FrameVelocity>(context,
                                                    ego_velocity_index_);
  DRAKE_ASSERT(ego_velocity != nullptr);

  const PoseBundle<T>* const traffic_poses =
      this->template EvalInputValue<PoseBundle<T>>(context, traffic_index_);
  DRAKE_ASSERT(traffic_poses != nullptr);

  ImplCalcAcceleration(*ego_pose, *ego_velocity, *traffic_poses, idm_params,
                       accel_output);
}

template <typename T>
void IdmController<T>::ImplCalcAcceleration(
    const PoseVector<T>& ego_pose, const FrameVelocity<T>& ego_velocity,
    const PoseBundle<T>& traffic_poses,
    const IdmPlannerParameters<T>& idm_params,
    systems::BasicVector<T>* command) const {
  using std::abs;
  using std::max;

  DRAKE_DEMAND(idm_params.IsValid());
  Isometry3<T> ego_pose_isometry = ego_pose.get_isometry();
  const auto translation = ego_pose_isometry.translation();

  const GeoPosition geo_position(ExtractDoubleOrThrow(translation.x()),
                                 ExtractDoubleOrThrow(translation.y()),
                                 ExtractDoubleOrThrow(translation.z()));
  const RoadPosition ego_position =
      road_.ToRoadPosition(geo_position, nullptr, nullptr, nullptr);

  // Find the single closest car ahead.
  const ClosestPose<T> lead_car_pose = PoseSelector<T>::FindSingleClosestPose(
      ego_position.lane, ego_pose, traffic_poses,
      idm_params.scan_ahead_distance(), AheadOrBehind::kAhead);
  const T headway_distance = lead_car_pose.distance;

  const LanePositionT<T> lane_position(T(ego_position.pos.s()),
                                       T(ego_position.pos.r()),
                                       T(ego_position.pos.h()));
  T s_dot_ego = PoseSelector<T>::GetSigmaVelocity(
      {ego_position.lane, lane_position, ego_velocity});
  T s_dot_lead =
      (abs(lead_car_pose.odometry.pos.s()) ==
       std::numeric_limits<T>::infinity())
          ? T(0.)
          : PoseSelector<T>::GetSigmaVelocity(lead_car_pose.odometry);

  // Saturate the net_distance at `idm_params.distance_lower_limit()` away from
  // the ego car to avoid near-singular solutions inherent to the IDM equation.
  const T actual_headway = headway_distance - idm_params.bloat_diameter();
  const T net_distance = max(actual_headway, idm_params.distance_lower_limit());
  const T closing_velocity = s_dot_ego - s_dot_lead;

  // Compute the acceleration command from the IDM equation.
  (*command)[0] = IdmPlanner<T>::Evaluate(idm_params, s_dot_ego, net_distance,
                                          closing_velocity);
}

}  // namespace automotive
}  // namespace drake

// These instantiations must match the API documentation in idm_controller.h.
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::automotive::IdmController)
