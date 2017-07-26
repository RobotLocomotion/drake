#include "drake/manipulation/perception/optitrack_pose_extractor.h"

#include <vector>
#include "optitrack/optitrack_frame_t.hpp"

#include "drake/math/quaternion.h"
#include "drake/math/rotation_matrix.h"
#include "drake/systems/framework/context.h"

namespace drake {
using systems::Context;
using systems::DiscreteValues;
using systems::BasicVector;

namespace manipulation {
namespace perception {

OptitrackPoseExtractor::OptitrackPoseExtractor(
    unsigned int object_id, const Isometry3<double>& world_X_optitrack,
    double optitrack_lcm_status_period)
    : object_id_(object_id),
      measured_pose_output_port_{
          this->DeclareVectorOutputPort(
                  BasicVector<double>(7),
                  &OptitrackPoseExtractor::OutputMeasuredPose)
              .get_index()},
      X_WOp_(world_X_optitrack) {
  this->set_name("Optitrack pose extractor");
  this->DeclareAbstractInputPort();
  // Internal state is stored with first 3 dimensions containing the object's
  // Cartesian position and the next 4, the orientation in a quaternion
  // parameterization.
  this->DeclareDiscreteState(7);
  this->DeclarePeriodicDiscreteUpdate(optitrack_lcm_status_period);
}

void OptitrackPoseExtractor::DoCalcDiscreteVariableUpdates(
    const Context<double>& context,
    const std::vector<const systems::DiscreteUpdateEvent<double>*>&,
    systems::DiscreteValues<double>* discrete_state) const {
  const systems::AbstractValue* input = this->EvalAbstractInput(context, 0);
  DRAKE_ASSERT(input != nullptr);
  const auto& pose_message = input->GetValue<optitrack::optitrack_frame_t>();
  BasicVector<double>* state = discrete_state->get_mutable_vector(0);
  auto state_value = state->get_mutable_value();
  state_value = VectorX<double>::Zero(7);

  std::vector<optitrack::optitrack_rigid_body_t> rigid_bodies =
      pose_message.rigid_bodies;

  DRAKE_THROW_UNLESS(object_id_ < rigid_bodies.size());
  DRAKE_THROW_UNLESS(rigid_bodies[object_id_].id == (int)object_id_);

  // The optitrack quaternion ordering is Z-W-X-Y.
  Eigen::Quaterniond quaternion(
      rigid_bodies[object_id_].quat[3], rigid_bodies[object_id_].quat[0],
      rigid_bodies[object_id_].quat[1], rigid_bodies[object_id_].quat[2]);

  // Transform pose to world frame.
  Isometry3<double> X_OpOb;
  X_OpOb.linear() = quaternion.toRotationMatrix();
  X_OpOb.translation() = Eigen::Vector3d(rigid_bodies[object_id_].xyz[0],
                                         rigid_bodies[object_id_].xyz[1],
                                         rigid_bodies[object_id_].xyz[2]);
  X_OpOb.makeAffine();
  Isometry3<double> X_WOb = X_WOp_ * X_OpOb;

  state_value.segment<3>(0) = X_WOb.translation();
  state_value.segment<4>(3) = Eigen::Quaterniond(X_WOb.linear()).coeffs();
}

void OptitrackPoseExtractor::OutputMeasuredPose(
    const Context<double>& context, BasicVector<double>* output) const {
  const auto state_value = context.get_discrete_state(0)->get_value();
  Eigen::VectorBlock<VectorX<double>> measured_pose_output =
      output->get_mutable_value();
  measured_pose_output = state_value;
}

}  // namespace perception
}  // namespace manipulation
}  // namespace drake
