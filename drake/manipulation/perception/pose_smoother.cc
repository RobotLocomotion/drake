#include "drake/manipulation/perception/pose_smoother.h"

#include <utility>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/common/text_logging.h"
#include "drake/manipulation/util/moving_average_filter.h"
#include "drake/math/quaternion.h"
#include "drake/systems/framework/context.h"

namespace drake {
using systems::Context;
using systems::DiscreteValues;
using systems::BasicVector;
namespace manipulation {
using util::MovingAverageFilter;
namespace perception {
using Eigen::Quaterniond;

namespace {
struct InternalState {
  InternalState() {
    pose_ = Isometry3<double>::Identity();
    time_since_last_accepted_pose_ = 0.0;
    velocity_ = Vector6<double>::Zero();
    is_first_time_ = true;
  }
  Isometry3<double> pose_;
  Vector6<double> velocity_;
  double time_since_last_accepted_pose_;
  bool is_first_time_;
};

// Define some utility methods required for filtering.
Isometry3<double> VectorToIsometry3(const VectorX<double>& pose_vector) {
  Isometry3<double> pose;
  pose.linear() = Quaterniond(pose_vector.segment<4>(3)).matrix();
  pose.translation() = pose_vector.segment<3>(0);
  return pose;
}

VectorX<double> Isometry3ToVector(const Isometry3<double>& pose) {
  VectorX<double> pose_vector = VectorX<double>::Zero(7);
  pose_vector.segment<3>(0) = pose.translation();
  pose_vector.segment<4>(3) = Quaterniond(pose.linear()).coeffs();
  return pose_vector;
}

void FixQuaternionForCloseness(const Eigen::Quaterniond& q1,
                               Eigen::Quaterniond* q2) {
  double dot = q1.dot(*q2);
  if (dot < 0.0) {
    // Invert sign.
    q2->w() = -q2->w();
    q2->x() = -q2->x();
    q2->y() = -q2->y();
    q2->z() = -q2->z();
  }
}
// velocity from pose2 to pose1
VectorX<double> ComputeVelocities(const Isometry3<double>& pose_1,
                                  const Isometry3<double>& pose_2,
                                  double delta_t) {
  VectorX<double> velocities = VectorX<double>::Zero(6);

  Eigen::Array3d translation_diff =
      pose_1.translation().array() - pose_2.translation().array();
  velocities.segment<3>(0) = (translation_diff / delta_t).matrix();

  Eigen::AngleAxisd angle_axis_diff;
  // Computes angular velocity from the angle difference.
  angle_axis_diff =
      Eigen::AngleAxisd(pose_1.linear() * pose_2.linear().inverse());
  velocities.segment<3>(3) =
      angle_axis_diff.axis() * angle_axis_diff.angle() / delta_t;

  return (velocities);
}
}  // namespace

PoseSmoother::PoseSmoother(double max_linear_velocity,
                           double max_angular_velocity, int filter_window_size,
                           double optitrack_lcm_status_period)
    : smoothed_pose_output_port_(
          this->DeclareAbstractOutputPort(&PoseSmoother::OutputSmoothedPose)
              .get_index()),
      smoothed_velocity_output_port_(
          this->DeclareAbstractOutputPort(&PoseSmoother::OutputSmoothedVelocity)
              .get_index()),
      kMaxLinearVelocity(max_linear_velocity),
      kMaxAngularVelocity(max_angular_velocity),
      kDiscreteUpdateInSec(optitrack_lcm_status_period),
      filter_(std::make_unique<MovingAverageFilter<VectorX<double>>>(
          filter_window_size)),
      is_filter_enabled_(true) {
  this->set_name("Pose Smoother");
  this->DeclareAbstractInputPort();
  this->DeclarePeriodicUnrestrictedUpdate(optitrack_lcm_status_period, 0);
}

PoseSmoother::PoseSmoother(double max_linear_velocity,
                           double max_angular_velocity,
                           double optitrack_lcm_status_period)
    : smoothed_pose_output_port_(
          this->DeclareAbstractOutputPort(&PoseSmoother::OutputSmoothedPose)
              .get_index()),
      smoothed_velocity_output_port_(
          this->DeclareAbstractOutputPort(&PoseSmoother::OutputSmoothedVelocity)
              .get_index()),
      kMaxLinearVelocity(max_linear_velocity),
      kMaxAngularVelocity(max_angular_velocity),
      kDiscreteUpdateInSec(optitrack_lcm_status_period),
      filter_(nullptr),
      is_filter_enabled_(false) {
  this->set_name("Pose Smoother");
  this->DeclareAbstractInputPort();
  this->DeclarePeriodicUnrestrictedUpdate(optitrack_lcm_status_period, 0);
}

std::unique_ptr<systems::AbstractValues> PoseSmoother::AllocateAbstractState()
    const {
  std::vector<std::unique_ptr<systems::AbstractValue>> abstract_vals;
  abstract_vals.push_back(std::unique_ptr<systems::AbstractValue>(
      new systems::Value<InternalState>(InternalState())));
  return std::make_unique<systems::AbstractValues>(std::move(abstract_vals));
}

void PoseSmoother::SetDefaultState(const systems::Context<double>&,
                                   systems::State<double>* state) const {
  InternalState& internal_state =
      state->get_mutable_abstract_state<InternalState>(0);
  internal_state = InternalState();
}

void PoseSmoother::DoCalcUnrestrictedUpdate(
    const systems::Context<double>& context,
    const std::vector<const systems::UnrestrictedUpdateEvent<double>*>&,
    systems::State<double>* state) const {
  // Extract Internal state.
  InternalState& internal_state =
      state->get_mutable_abstract_state<InternalState>(0);

  Isometry3<double>& current_pose = internal_state.pose_;
  double& time_since_last_accepted_pose =
      internal_state.time_since_last_accepted_pose_;
  Vector6<double>& current_velocity = internal_state.velocity_;

  // Update world state from inputs.
  const systems::AbstractValue* input = this->EvalAbstractInput(context, 0);
  DRAKE_ASSERT(input != nullptr);
  const auto& input_pose = input->GetValue<Isometry3<double>>();

  // Set the initial state of the smoother.
  if (internal_state.is_first_time_) {
    internal_state.is_first_time_ = false;
    internal_state.pose_ = input_pose;
    internal_state.time_since_last_accepted_pose_ = kDiscreteUpdateInSec;
  }

  Vector6<double> new_velocity = ComputeVelocities(
      input_pose, current_pose, time_since_last_accepted_pose);

  bool accept_data_point = true;
  for (int i = 0; i < 3; ++i) {
    if (new_velocity(i) >= kMaxLinearVelocity ||
        new_velocity(3 + i) >= kMaxAngularVelocity) {
      accept_data_point = false;
      break;
    }
  }
  // If data is below threshold it can be added to the filter.
  if (accept_data_point) {
    Quaterniond input_quaternion = Quaterniond(input_pose.linear());
    Quaterniond current_quaternion = Quaterniond(current_pose.linear());

    FixQuaternionForCloseness(current_quaternion, &input_quaternion);
    Isometry3<double> corrected_input = input_pose;
    corrected_input.linear() = input_quaternion.matrix();

    Isometry3<double> accepted_pose;
    // If the smoother is enabled.
    if (is_filter_enabled_) {
      accepted_pose = VectorToIsometry3(
          filter_->Update(Isometry3ToVector(corrected_input)));
    } else {
      accepted_pose = corrected_input;
    }
    current_velocity = ComputeVelocities(accepted_pose, current_pose,
                                         time_since_last_accepted_pose);

    time_since_last_accepted_pose = kDiscreteUpdateInSec;
    current_pose = accepted_pose;
  } else {
    drake::log()->info("Data point rejected");
    // Since the current sample has been rejected, the time since the last
    // sample must be incremented suitably.
    time_since_last_accepted_pose += kDiscreteUpdateInSec;
  }
}

void PoseSmoother::OutputSmoothedPose(const systems::Context<double>& context,
                                      Isometry3<double>* output) const {
  const auto internal_state = context.get_abstract_state<InternalState>(0);
  *output = internal_state.pose_;
}

void PoseSmoother::OutputSmoothedVelocity(
    const systems::Context<double>& context, Vector6<double>* output) const {
  const auto internal_state = context.get_abstract_state<InternalState>(0);
  *output = internal_state.velocity_;
}
}  // namespace perception
}  // namespace manipulation
}  // namespace drake
