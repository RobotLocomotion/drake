#include "drake/automotive/idm_controller.h"

#include <algorithm>
#include <limits>
#include <utility>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"

namespace drake {
namespace automotive {

using maliput::api::GeoPosition;
using maliput::api::GeoPositionT;
using maliput::api::Lane;
using maliput::api::LanePosition;
using maliput::api::LanePositionT;
using maliput::api::RoadGeometry;
using maliput::api::RoadPosition;
using systems::rendering::FrameVelocity;
using systems::rendering::PoseBundle;
using systems::rendering::PoseVector;

static constexpr int kIdmParamsIndex{0};

template <typename T>
IdmController<T>::IdmController(const RoadGeometry& road,
                                ScanStrategy path_or_branches,
                                RoadPositionStrategy road_position_strategy,
                                double period_sec)
    : systems::LeafSystem<T>(
          systems::SystemTypeTag<automotive::IdmController>{}),
      road_(road),
      path_or_branches_(path_or_branches),
      road_position_strategy_(road_position_strategy),
      period_sec_(period_sec),
      ego_pose_index_(
          this->DeclareVectorInputPort(PoseVector<T>()).get_index()),
      ego_velocity_index_(
          this->DeclareVectorInputPort(FrameVelocity<T>()).get_index()),
      traffic_index_(this->DeclareAbstractInputPort(
          systems::kUseDefaultName, systems::Value<PoseBundle<T>>())
              .get_index()),
      acceleration_index_(
          this->DeclareVectorOutputPort(systems::BasicVector<T>(1),
                                        &IdmController::CalcAcceleration)
              .get_index()) {
  this->DeclareNumericParameter(IdmPlannerParameters<T>());
  // TODO(jadecastro) It is possible to replace the following AbstractState with
  // a caching scheme once #4364 lands, preventing the need to use abstract
  // states and periodic sampling time.
  if (road_position_strategy == RoadPositionStrategy::kCache) {
    this->DeclareAbstractState(systems::AbstractValue::Make<RoadPosition>(
        RoadPosition()));
    this->DeclarePeriodicUnrestrictedUpdate(period_sec, 0);
  }
}

template <typename T>
IdmController<T>::~IdmController() {}

template <typename T>
const systems::InputPort<T>& IdmController<T>::ego_pose_input()
    const {
  return systems::System<T>::get_input_port(ego_pose_index_);
}

template <typename T>
const systems::InputPort<T>& IdmController<T>::ego_velocity_input()
    const {
  return systems::System<T>::get_input_port(ego_velocity_index_);
}

template <typename T>
const systems::InputPort<T>& IdmController<T>::traffic_input() const {
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

  // Obtain the state if we've allocated it.
  RoadPosition ego_rp;
  if (context.get_state().get_abstract_state().size() != 0) {
    DRAKE_ASSERT(context.get_num_abstract_states() == 1);
    ego_rp = context.template get_abstract_state<RoadPosition>(0);
  }

  ImplCalcAcceleration(*ego_pose, *ego_velocity, *traffic_poses, idm_params,
                       ego_rp, accel_output);
}

template <typename T>
void IdmController<T>::ImplCalcAcceleration(
    const PoseVector<T>& ego_pose, const FrameVelocity<T>& ego_velocity,
    const PoseBundle<T>& traffic_poses,
    const IdmPlannerParameters<T>& idm_params,
    const RoadPosition& ego_rp,
    systems::BasicVector<T>* command) const {
  using std::abs;
  using std::max;

  DRAKE_DEMAND(idm_params.IsValid());
  RoadPosition ego_position = ego_rp;
  if (!ego_rp.lane) {
    const auto gp =
        GeoPositionT<T>::FromXyz(ego_pose.get_isometry().translation());
    ego_position =
        road_.ToRoadPosition(gp.MakeDouble(), nullptr, nullptr, nullptr);
  }

  // Find the single closest car ahead.
  const ClosestPose<T> lead_car_pose = PoseSelector<T>::FindSingleClosestPose(
      ego_position.lane, ego_pose, traffic_poses,
      idm_params.scan_ahead_distance(), AheadOrBehind::kAhead,
      path_or_branches_);
  const T headway_distance = lead_car_pose.distance;

  const LanePositionT<T> lane_position(T(ego_position.pos.s()),
                                       T(ego_position.pos.r()),
                                       T(ego_position.pos.h()));
  const T s_dot_ego = PoseSelector<T>::GetSigmaVelocity(
      {ego_position.lane, lane_position, ego_velocity});
  const T s_dot_lead =
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

template <typename T>
void IdmController<T>::DoCalcUnrestrictedUpdate(
    const systems::Context<T>& context,
    const std::vector<const systems::UnrestrictedUpdateEvent<T>*>&,
    systems::State<T>* state) const {
  DRAKE_ASSERT(context.get_num_abstract_states() == 1);

  // Obtain the input and state data.
  const PoseVector<T>* const ego_pose =
      this->template EvalVectorInput<PoseVector>(context, ego_pose_index_);
  DRAKE_ASSERT(ego_pose != nullptr);

  const FrameVelocity<T>* const ego_velocity =
      this->template EvalVectorInput<FrameVelocity>(context,
                                                    ego_velocity_index_);
  DRAKE_ASSERT(ego_velocity != nullptr);

  RoadPosition& rp =
      state->template get_mutable_abstract_state<RoadPosition>(0);

  CalcOngoingRoadPosition(*ego_pose, *ego_velocity, road_, &rp);
}

}  // namespace automotive
}  // namespace drake

// These instantiations must match the API documentation in idm_controller.h.
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::automotive::IdmController)
