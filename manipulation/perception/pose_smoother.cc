#include "drake/manipulation/perception/pose_smoother.h"

#include <utility>
#include <vector>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/eigen_types.h"
#include "drake/common/text_logging.h"
#include "drake/manipulation/util/moving_average_filter.h"
#include "drake/math/quaternion.h"
#include "drake/math/rigid_transform.h"
#include "drake/systems/framework/context.h"

namespace drake {

using Eigen::Quaterniond;
using Eigen::Isometry3d;
using math::RigidTransform;
using math::RotationMatrix;
namespace manipulation {

using util::MovingAverageFilter;
namespace perception {
namespace {
struct InternalState {
  explicit InternalState(int filter_window_size = 0) :
      filter(filter_window_size > 1?
             std::make_unique<MovingAverageFilter<VectorX<double>>>(
                 filter_window_size) : nullptr) {}

  RigidTransform<double> pose;  // Initializes to identity RigidTransform.
  Vector6<double> velocity{Vector6<double>::Zero()};
  double time_at_last_accepted_pose{0.0};
  bool is_first_time{true};
  copyable_unique_ptr<util::MovingAverageFilter<VectorX<double>>> filter;
};

/*
 * Computes velocity of the motion from pose_2 to pose_1 taking place in
 * delta_t seconds.
 */
VectorX<double> ComputeVelocities(const RigidTransform<double>& pose_1,
                                  const RigidTransform<double>& pose_2,
                                  double delta_t) {
  VectorX<double> velocities = VectorX<double>::Zero(6);

  // Since the condition delta_t = 0 can only occur at the first instance of
  // calling DoCalcUnrestrictedUpdate, it is sufficient to simply return a
  // velocity of VectorX<double>::Zero(6) under this condition. Otherwise,
  // compute the actual velocities from the poses.
  if (delta_t > 0) {
    Eigen::Vector3d translation_diff =
        pose_1.translation() - pose_2.translation();
    velocities.head<3>() = (translation_diff / delta_t).matrix();

    // Computes angular velocity from the angle difference.
    const RotationMatrix<double> R = pose_1.rotation()
                                   * pose_2.rotation().inverse();
    const Eigen::AngleAxisd angle_axis_diff = R.ToAngleAxis();
    velocities.tail<3>() =
        angle_axis_diff.axis() * angle_axis_diff.angle() / delta_t;
  }
  return velocities;
}

// TODO(naveenoid) : Replace the usage of these methods eventually with
// some struct with clearer pose semantics.
// Sets a pose from a 7-element array whose first 3 elements are position and
// last 4 elements are a quaternion (w, x, y, z) with w >= 0 (canonical form).
RigidTransform<double> PoseVector7ToRigidTransform(
    const VectorX<double>& pose_vector) {
  DRAKE_ASSERT(pose_vector.size() == 7);
  Quaterniond quaternion(pose_vector(3), pose_vector(4),
                         pose_vector(5), pose_vector(6));
  return RigidTransform<double>(quaternion, pose_vector.head<3>());
}

// Convert a pose into a 7 element array whose first 3 elements are position and
// last 4 elements are a quaternion (w, x, y, z) with w >= 0 (canonical form).
VectorX<double> RigidTransformdToVector7(const RigidTransform<double>& pose) {
  VectorX<double> pose_vector = VectorX<double>::Zero(7);
  pose_vector.head<3>() = pose.translation();
  const Quaterniond quat = pose.rotation().ToQuaternion();
  pose_vector.tail<4>() =
      (VectorX<double>(4) << quat.w(), quat.x(), quat.y(), quat.z()).finished();
  return pose_vector;
}
}  // namespace

PoseSmoother::PoseSmoother(double desired_max_linear_velocity,
                           double desired_max_angular_velocity,
                           double period_sec, int filter_window_size)
    : smoothed_pose_output_port_(
          this->DeclareAbstractOutputPort(
                  systems::kUseDefaultName,
                  &PoseSmoother::OutputSmoothedPose)
              .get_index()),
      smoothed_velocity_output_port_(
          this->DeclareAbstractOutputPort(
                  systems::kUseDefaultName,
                  &PoseSmoother::OutputSmoothedVelocity)
              .get_index()),
      max_linear_velocity_(desired_max_linear_velocity),
      max_angular_velocity_(desired_max_angular_velocity),
      is_filter_enabled_(filter_window_size > 1) {
  this->DeclareAbstractState(
      Value<InternalState>(
          InternalState(filter_window_size)));
  this->DeclareAbstractInputPort(
      systems::kUseDefaultName,
      Value<Isometry3d>(Isometry3d::Identity()));
  this->DeclarePeriodicUnrestrictedUpdate(period_sec, 0);
}

void PoseSmoother::DoCalcUnrestrictedUpdate(
    const systems::Context<double>& context,
    const std::vector<const systems::UnrestrictedUpdateEvent<double>*>&,
    systems::State<double>* state) const {
  // Extract Internal state.
  InternalState& internal_state =
      state->get_mutable_abstract_state<InternalState>(0);

  // Update world state from inputs.
  const RigidTransform<double> input_pose(
      this->get_input_port(0).Eval<Isometry3d>(context));

      double current_time = context.get_time();

  // Set the initial state of the smoother.
  if (internal_state.is_first_time) {
    internal_state.is_first_time = false;
    internal_state.pose = input_pose;
    internal_state.time_at_last_accepted_pose = current_time;
    drake::log()->debug("PoseSmoother initial state set.");
  }

  RigidTransform<double>& current_pose = internal_state.pose;
  double& time_at_last_accepted_pose =
      internal_state.time_at_last_accepted_pose;
  Vector6<double>& current_velocity = internal_state.velocity;
  Vector6<double> new_velocity = ComputeVelocities(input_pose, current_pose,
                                  current_time - time_at_last_accepted_pose);

  bool accept_data_point = true;
  for (int i = 0; i < 3; ++i) {
    if (new_velocity(i) >= max_linear_velocity_ ||
        new_velocity(3 + i) >= max_angular_velocity_) {
      accept_data_point = false;
      break;
    }
  }
  // If data is below threshold it can be added to the filter.
  if (accept_data_point) {
    RigidTransform<double> accepted_pose = input_pose;
    // If the smoother is enabled.
    if (is_filter_enabled_) {
      VectorX<double> temp = internal_state.filter->Update(
          RigidTransformdToVector7(input_pose));
      accepted_pose = PoseVector7ToRigidTransform(temp);
    }

    current_velocity = ComputeVelocities(
        accepted_pose, current_pose, current_time - time_at_last_accepted_pose);
    time_at_last_accepted_pose = current_time;
    current_pose = accepted_pose;
  } else {
    drake::log()->debug("Data point rejected");
  }
}

void PoseSmoother::OutputSmoothedPose(const systems::Context<double>& context,
                                      Isometry3d* output) const {
  const auto internal_state = context.get_abstract_state<InternalState>(0);
  *output = internal_state.pose.GetAsIsometry3();
  output->makeAffine();
}

void PoseSmoother::OutputSmoothedVelocity(
    const systems::Context<double>& context, Vector6<double>* output) const {
  const auto internal_state = context.get_abstract_state<InternalState>(0);
  *output = internal_state.velocity;
}
}  // namespace perception
}  // namespace manipulation
}  // namespace drake
