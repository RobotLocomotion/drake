#include "drake/manipulation/perception/optitrack_pose_extractor.h"

#include <utility>
#include <vector>

#include <fmt/format.h>

#include "drake/common/text_logging.h"
#include "drake/math/quaternion.h"
#include "drake/math/rigid_transform.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace manipulation {
namespace perception {

using systems::Context;

Isometry3<double> ExtractOptitrackPose(
    const optitrack::optitrack_rigid_body_t& body) {
  // The optitrack quaternion ordering is X-Y-Z-W and this needs fitting into
  // Eigen's W-X-Y-Z ordering.
  auto& q_xyzw = body.quat;
  Eigen::Quaterniond q_wxyz(q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2]);
  // Quaternion arrived in float (single) precision so is not sufficiently
  // normalized for use in double precision.
  q_wxyz.normalize();
  // Pose of rigid body frame `B` in Optitrack frame `O`.
  const Eigen::Vector3d position(body.xyz[0], body.xyz[1], body.xyz[2]);
  const math::RigidTransform<double> X_OB(q_wxyz, position);
  return X_OB.GetAsIsometry3();
}

std::map<int, Isometry3<double>> ExtractOptitrackPoses(
    const optitrack::optitrack_frame_t& frame) {
  std::map<int, Isometry3<double>> poses;
  for (auto& body : frame.rigid_bodies) {
    poses[body.id] = ExtractOptitrackPose(body);
  }
  return poses;
}

std::optional<optitrack::optitrack_rigid_body_t> FindOptitrackBody(
      const optitrack::optitrack_frame_t& message, int object_id) {
  for (auto& body : message.rigid_bodies) {
    if (body.id == object_id) return body;
  }
  return std::nullopt;
}

std::optional<int> FindOptitrackObjectId(
    const optitrack::optitrack_data_descriptions_t& message,
    const std::string& object_name) {
  for (auto& desc : message.rigid_bodies) {
    if (desc.name == object_name) return desc.id;
  }
  return std::nullopt;
}

OptitrackPoseExtractor::OptitrackPoseExtractor(
    int object_id, const Isometry3<double>& X_WO,
    double optitrack_lcm_status_period)
    : object_id_(object_id),
      measured_pose_output_port_{
          this->DeclareAbstractOutputPort(
                  systems::kUseDefaultName,
                  &OptitrackPoseExtractor::OutputMeasuredPose)
              .get_index()},
      X_WO_(X_WO) {
  DeclareAbstractState(
      Value<Isometry3<double>>(
          Isometry3<double>::Identity()));
  this->DeclareAbstractInputPort(
      systems::kUseDefaultName,
      Value<optitrack::optitrack_frame_t>());
  // Internal state is an Isometry3d.
  this->DeclarePeriodicUnrestrictedUpdate(optitrack_lcm_status_period, 0);
}

void OptitrackPoseExtractor::DoCalcUnrestrictedUpdate(
    const Context<double>& context,
    const std::vector<const systems::UnrestrictedUpdateEvent<double>*>&,
    systems::State<double>* state) const {
  // Extract Internal state.
  Isometry3<double>& internal_state =
      state->get_mutable_abstract_state<Isometry3<double>>(0);

  // Update world state from inputs.
  const auto& input = this->get_input_port(0);
  const auto& message = input.Eval<optitrack::optitrack_frame_t>(context);
  auto body = FindOptitrackBody(message, object_id_);
  if (!body.has_value()) {
    throw std::runtime_error(fmt::format(
        "optitrack: id {} not found", object_id_));
  }
  internal_state = X_WO_ * ExtractOptitrackPose(*body);
}

void OptitrackPoseExtractor::OutputMeasuredPose(
    const Context<double>& context, Isometry3<double>* output) const {
  const auto state_value = context.get_abstract_state<Isometry3<double>>(0);
  *output = state_value;
}

}  // namespace perception
}  // namespace manipulation
}  // namespace drake
