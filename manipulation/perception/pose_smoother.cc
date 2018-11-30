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
namespace manipulation {

using util::MovingAverageFilter;
namespace perception {
namespace {
struct InternalState {
  explicit InternalState(int filter_window_size = 0) :
      filter(filter_window_size > 1?
             std::make_unique<MovingAverageFilter<VectorX<double>>>(
                 filter_window_size) : nullptr) {}

  Isometry3<double> pose{Isometry3d::Identity()};
  Vector6<double> velocity{Vector6<double>::Zero()};
  double time_at_last_accepted_pose{0.0};
  bool is_first_time{true};
  copyable_unique_ptr<util::MovingAverageFilter<VectorX<double>>> filter;
};

/*
 * Computes velocity of the motion from pose_2 to pose_1 taking place in
 * delta_t seconds.
 */
VectorX<double> ComputeVelocities(const Isometry3d& pose_1,
                                  const Isometry3d& pose_2, double delta_t) {
  VectorX<double> velocities = VectorX<double>::Zero(6);

  // Since the condition delta_t = 0 can only occur at the first instance of
  // calling DoCalcUnrestrictedUpdate, it is sufficient to simply return a
  // velocity of VectorX<double>::Zero(6) under this condition. Otherwise,
  // compute the actual velocities from the poses.
  if (delta_t > 0) {
    Eigen::Vector3d translation_diff =
        pose_1.translation() - pose_2.translation();
    velocities.head<3>() = (translation_diff / delta_t).matrix();

    Eigen::AngleAxisd angle_axis_diff;
    // Computes angular velocity from the angle difference.
    angle_axis_diff =
        Eigen::AngleAxisd(pose_1.linear() * pose_2.linear().inverse());
    velocities.tail<3>() =
        angle_axis_diff.axis() * angle_axis_diff.angle() / delta_t;
  }
  return velocities;
}


}  // namespace

PoseSmoother::PoseSmoother(double desired_max_linear_velocity,
                           double desired_max_angular_velocity,
                           double period_sec, int filter_window_size)
    : smoothed_pose_output_port_(
          this->DeclareAbstractOutputPort(&PoseSmoother::OutputSmoothedPose)
              .get_index()),
      smoothed_velocity_output_port_(
          this->DeclareAbstractOutputPort(&PoseSmoother::OutputSmoothedVelocity)
              .get_index()),
      max_linear_velocity_(desired_max_linear_velocity),
      max_angular_velocity_(desired_max_angular_velocity),
      is_filter_enabled_(filter_window_size > 1) {
  this->DeclareAbstractState(
      systems::AbstractValue::Make<InternalState>(
          InternalState(filter_window_size)));
  this->DeclareAbstractInputPort(
      systems::kUseDefaultName,
      systems::Value<Isometry3d>(Isometry3d::Identity()));
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
  const systems::AbstractValue* input = this->EvalAbstractInput(context, 0);
  DRAKE_ASSERT(input != nullptr);
  const auto& input_pose = input->GetValue<Isometry3d>();

  double current_time = context.get_time();

  // Set the initial state of the smoother.
  if (internal_state.is_first_time) {
    internal_state.is_first_time = false;
    internal_state.pose = input_pose;
    internal_state.time_at_last_accepted_pose = current_time;
    drake::log()->debug("PoseSmoother initial state set.");
  }

  Isometry3d& current_pose = internal_state.pose;
  double& time_at_last_accepted_pose =
      internal_state.time_at_last_accepted_pose;
  Vector6<double>& current_velocity = internal_state.velocity;

  Vector6<double> new_velocity = ComputeVelocities(
      input_pose, current_pose, current_time - time_at_last_accepted_pose);

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
    const Quaterniond input_quaternion(input_pose.linear());
    const math::RotationMatrixd input_R(input_quaternion);
    const Vector3<double>& input_p = input_pose.translation();
    const math::RigidTransform corrected_input(input_R, input_p);

    math::RigidTransformd accepted_pose;  // Default is identity pose.
    // If the smoother is enabled.
    if (is_filter_enabled_) {
      // Create a 7-element array whose first four elements describe orientation
      // with a "canonical" quaternion (q0, q1, q2, q3) and whose last three
      // elements are the position (x, y, z).
      // Note: A "canonical" quaternion is a quaternion with q0 > = 0.
      const Quaterniond corrected_input_quat =
          corrected_input.rotation().ToQuaternion();
      VectorX<double> corrected_pose_vector(7);
      corrected_pose_vector << translation(), quat.matrix();

      VectorX<double> position_quat =
          internal_state.filter->Update(corrected_pose_vector);

      const Quaterniond quat(position_quat(3), position_quat(4),
                             position_quat(5), position_quat(6));
      accepted_pose.set(math::RotationMatrixd(quat), position_quat.head<3>());
    } else {
      accepted_pose = corrected_input;
    }
    current_velocity = ComputeVelocities(accepted_pose.GetAsIsometry3(),
        current_pose, current_time - time_at_last_accepted_pose);
    time_at_last_accepted_pose = current_time;
    current_pose = accepted_pose.GetAsIsometry3();
  } else {
    drake::log()->debug("Data point rejected");
  }
}

void PoseSmoother::OutputSmoothedPose(const systems::Context<double>& context,
                                      Isometry3d* output) const {
  const auto internal_state = context.get_abstract_state<InternalState>(0);
  *output = internal_state.pose;
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
