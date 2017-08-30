#include "drake/manipulation/perception/pose_smoother.h"

#include <utility>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/common/text_logging.h"
#include "drake/manipulation/util/moving_average_filter.h"
#include "drake/manipulation/util/perception_utils.h"
#include "drake/math/quaternion.h"
#include "drake/systems/framework/context.h"

namespace drake {

using systems::Context;
using systems::DiscreteValues;
using systems::BasicVector;
using Eigen::Quaterniond;
using Eigen::Isometry3d;
namespace manipulation {

using util::MovingAverageFilter;
using util::Isometry3dToVector;
using util::VectorToIsometry3d;
namespace perception {
namespace {
struct InternalState {
  Isometry3<double> pose_{Isometry3d::Identity()};
  Vector6<double> velocity_{Vector6<double>::Zero()};
  double time_since_last_accepted_pose_{0.0};
  bool is_first_time_{true};
};

// Computes velocity of the motion from pose_2 to pose_1 taking place in
// delta_t seconds.
VectorX<double> ComputeVelocities(const Isometry3d& pose_1,
                                  const Isometry3d& pose_2,
                                  double delta_t) {
  VectorX<double> velocities = VectorX<double>::Zero(6);

  Eigen::Vector3d translation_diff =
      pose_1.translation() - pose_2.translation();
  velocities.head<3>() = (translation_diff / delta_t).matrix();

  Eigen::AngleAxisd angle_axis_diff;
  // Computes angular velocity from the angle difference.

  drake::log()->info("system velocity check poses \n pose 1 rot:\n{}\n "
                         "pose 2 rot:\n{}", pose_1.linear(), pose_2.linear());

  angle_axis_diff =
      Eigen::AngleAxisd(pose_1.linear() * pose_2.linear().inverse());
  velocities.tail<3>() =
      angle_axis_diff.axis() * angle_axis_diff.angle() / delta_t;

  return velocities;
}
}  // namespace

PoseSmoother::PoseSmoother(double max_linear_velocity,
                           double max_angular_velocity, int filter_window_size,
                           double period_sec)
    : PoseSmoother(max_linear_velocity, max_angular_velocity, period_sec,
                   std::make_unique<MovingAverageFilter<VectorX<double>>>(
          filter_window_size)) {
}

PoseSmoother::PoseSmoother(double max_linear_velocity,
                           double max_angular_velocity,
                           double period_sec)
    : PoseSmoother(max_linear_velocity, max_angular_velocity, period_sec,
                   nullptr) {
}

PoseSmoother::PoseSmoother(
    double max_linear_velocity,
    double max_angular_velocity,
    double period_sec,
    std::unique_ptr<util::MovingAverageFilter<VectorX<double>>> filter) :
    smoothed_pose_output_port_(
        this->DeclareAbstractOutputPort(&PoseSmoother::OutputSmoothedPose)
            .get_index()),
    smoothed_velocity_output_port_(
        this->DeclareAbstractOutputPort(&PoseSmoother::OutputSmoothedVelocity)
            .get_index()),
    kMaxLinearVelocity(max_linear_velocity),
    kMaxAngularVelocity(max_angular_velocity),
    kDiscreteUpdateInSec(period_sec),
    filter_(std::move(filter)),
    is_filter_enabled_(filter_!= nullptr) {
  this->set_name("Pose Smoother");
  this->DeclareAbstractState(
      systems::AbstractValue::Make<InternalState>(InternalState()));
  this->DeclareAbstractInputPort();
  this->DeclarePeriodicUnrestrictedUpdate(period_sec, 0);
}

void PoseSmoother::DoCalcUnrestrictedUpdate(
    const systems::Context<double>& context,
    const std::vector<const systems::UnrestrictedUpdateEvent<double>*>&,
    systems::State<double>* state) const {
  // Extract Internal state.
  InternalState& internal_state =
      state->get_mutable_abstract_state<InternalState>(0);

  Isometry3d& current_pose = internal_state.pose_;
  double& time_since_last_accepted_pose =
      internal_state.time_since_last_accepted_pose_;
  Vector6<double>& current_velocity = internal_state.velocity_;

  // Update world state from inputs.
  const systems::AbstractValue* input = this->EvalAbstractInput(context, 0);
  DRAKE_ASSERT(input != nullptr);
  const auto& input_pose = input->GetValue<Isometry3d>();

  // Set the initial state of the smoother.
  if (internal_state.is_first_time_) {
    internal_state.is_first_time_ = false;
    internal_state.pose_ = input_pose;
    internal_state.time_since_last_accepted_pose_ = kDiscreteUpdateInSec;
    drake::log()->info("First time in system");
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

    input_quaternion = math::QuaternionToCanonicalForm(input_quaternion);
    Isometry3d corrected_input = input_pose;
    corrected_input.linear() = input_quaternion.toRotationMatrix();

    Isometry3d accepted_pose = Isometry3d::Identity();
    // If the smoother is enabled.
    if (is_filter_enabled_) {
      VectorX<double> temp =
          filter_->Update(Isometry3dToVector(corrected_input));
//      drake::log()->info("Data out of MAF : {}", temp.transpose());
      accepted_pose = VectorToIsometry3d(temp);
//      accepted_pose = VectorToIsometry3d(Isometry3dToVector(
//          corrected_input));
//      accepted_pose = corrected_input;
    } else {
      accepted_pose = corrected_input;
    }
    drake::log()->info("*********");
    drake::log()->info("System Corrected input :\n{}", corrected_input.linear());
    drake::log()->info("System Accepted pose :\n{}", accepted_pose.linear());
    current_velocity = ComputeVelocities(accepted_pose, current_pose,
                                         time_since_last_accepted_pose);

    time_since_last_accepted_pose = kDiscreteUpdateInSec;
    current_pose = accepted_pose;
  } else {
    drake::log()->debug("Data point rejected");
    // Since the current sample has been rejected, the time since the last
    // sample must be incremented suitably.
    time_since_last_accepted_pose += kDiscreteUpdateInSec;
  }
}

void PoseSmoother::OutputSmoothedPose(const systems::Context<double>& context,
                                      Isometry3d* output) const {
  const auto internal_state = context.get_abstract_state<InternalState>(0);
  *output = internal_state.pose_;
  output->makeAffine();
  drake::log()->info("System Accepted pose copied to output:\n{}",
                     output->linear());

}

void PoseSmoother::OutputSmoothedVelocity(
    const systems::Context<double>& context, Vector6<double>* output) const {
  const auto internal_state = context.get_abstract_state<InternalState>(0);
  *output = internal_state.velocity_;
}
}  // namespace perception
}  // namespace manipulation
}  // namespace drake
