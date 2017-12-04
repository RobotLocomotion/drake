#include "drake/manipulation/perception/optitrack_pose_extractor.h"

#include <utility>
#include <vector>

#include "optitrack/optitrack_frame_t.hpp"

#include "drake/common/text_logging.h"
#include "drake/math/quaternion.h"
#include "drake/math/rotation_matrix.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace manipulation {
namespace perception {
using systems::Context;
using systems::DiscreteValues;
using systems::BasicVector;

OptitrackPoseExtractor::OptitrackPoseExtractor(
    int object_id, const Isometry3<double>& X_WO,
    double optitrack_lcm_status_period)
    : object_id_(object_id),
      measured_pose_output_port_{
          this->DeclareAbstractOutputPort(
                  &OptitrackPoseExtractor::OutputMeasuredPose)
              .get_index()},
      X_WO_(X_WO) {
  DeclareAbstractState(
      systems::AbstractValue::Make<Isometry3<double>>(
          Isometry3<double>::Identity()));
  this->DeclareAbstractInputPort();
  // Internal state is an Isometry3d.
  this->DeclarePeriodicUnrestrictedUpdate(optitrack_lcm_status_period, 0);
}

void OptitrackPoseExtractor::DoCalcUnrestrictedUpdate(
    const systems::Context<double>& context,
    const std::vector<const systems::UnrestrictedUpdateEvent<double>*>&,
    systems::State<double>* state) const {
  // Extract Internal state.
  Isometry3<double>& internal_state =
      state->get_mutable_abstract_state<Isometry3<double>>(0);

  // Update world state from inputs.
  const systems::AbstractValue* input = this->EvalAbstractInput(context, 0);
  DRAKE_ASSERT(input != nullptr);
  const auto& pose_message = input->GetValue<optitrack::optitrack_frame_t>();

  internal_state = Isometry3<double>::Identity();

  std::vector<optitrack::optitrack_rigid_body_t> rigid_bodies =
      pose_message.rigid_bodies;

  // Lookup and verify if object exists and extract vector index.
  bool body_exists = false;
  size_t vector_index;
  for (vector_index = 0; vector_index < rigid_bodies.size(); ++vector_index) {
    if (rigid_bodies[vector_index].id == object_id_) {
      body_exists = true;
      break;
    }
  }
  DRAKE_THROW_UNLESS(body_exists);

  // The optitrack quaternion ordering is X-Y-Z-W and this needs fitting into
  // Eigen's W-X-Y-Z ordering.
  Eigen::Quaterniond quaternion(
      rigid_bodies[vector_index].quat[3], rigid_bodies[vector_index].quat[0],
      rigid_bodies[vector_index].quat[1], rigid_bodies[vector_index].quat[2]);

  // Transform from world frame W to rigid body frame B.
  Isometry3<double> X_OB;
  X_OB.linear() = quaternion.toRotationMatrix();
  X_OB.translation() = Eigen::Vector3d(rigid_bodies[vector_index].xyz[0],
                                       rigid_bodies[vector_index].xyz[1],
                                       rigid_bodies[vector_index].xyz[2]);

  X_OB.makeAffine();
  Isometry3<double> X_WB = X_WO_ * X_OB;

  internal_state = X_WB;
}

void OptitrackPoseExtractor::OutputMeasuredPose(
    const Context<double>& context, Isometry3<double>* output) const {
  const auto state_value = context.get_abstract_state<Isometry3<double>>(0);
  *output = state_value;
}

}  // namespace perception
}  // namespace manipulation
}  // namespace drake
