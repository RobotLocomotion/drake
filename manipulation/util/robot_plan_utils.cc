#include "drake/manipulation/util/robot_plan_utils.h"

#include <map>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"

namespace drake {
namespace manipulation {
namespace util {

/// @return A vector of joint names corresponding to the positions in @plant
/// in the order of the joint indices.
template <typename T>
std::vector<std::string> GetJointNames(
    const multibody::MultibodyPlant<T>& plant) {

  std::map<int, std::string> position_names;
  const int num_positions = plant.num_positions();
  for (int i = 0; i < plant.num_joints(); ++i) {
    const multibody::Joint<T>& joint =
        plant.get_joint(multibody::JointIndex(i));
    if (joint.num_positions() == 0) {
      continue;
    }
    DRAKE_DEMAND(joint.num_positions() == 1);
    DRAKE_DEMAND(joint.position_start() < num_positions);

    position_names[joint.position_start()] = joint.name();
  }

  DRAKE_DEMAND(static_cast<int>(position_names.size()) == num_positions);
  std::vector<std::string> joint_names;
  for (int i = 0; i < num_positions; ++i) {
    joint_names.push_back(position_names[i]);
  }
  return joint_names;
}

void ApplyJointVelocityLimits(
    const std::vector<Eigen::VectorXd>& keyframes,
    const Eigen::VectorXd& limits,
    std::vector<double>* times) {
  DRAKE_DEMAND(keyframes.size() == times->size());
  DRAKE_DEMAND(times->front() ==0);
  const int num_time_steps = keyframes.size();

  // Calculate a matrix of velocities between each timestep.  We'll
  // use this later to determine by how much the plan exceeds the
  // joint velocity limits.
  Eigen::MatrixXd velocities(limits.size(), num_time_steps - 1);
  for (int i = 0; i < velocities.rows(); i++) {
    for (int j = 0; j < velocities.cols(); j++) {
      DRAKE_ASSERT((*times)[j + 1] > (*times)[j]);
      velocities(i, j) =
          std::abs((keyframes[j + 1](i) - keyframes[j](i)) /
                   ((*times)[j + 1] - (*times)[j]));
    }
  }

  Eigen::VectorXd velocity_ratios(velocities.rows());

  for (int i = 0; i < velocities.rows(); i++) {
    const double max_plan_velocity = velocities.row(i).maxCoeff();
    velocity_ratios(i) = max_plan_velocity / limits(i);
  }

  const double max_velocity_ratio = velocity_ratios.maxCoeff();
  if (max_velocity_ratio > 1) {
    // The code below slows the entire plan such that the fastest step
    // meets the limits.  If that step is much faster than the others,
    // the whole plan becomes very slow.
    drake::log()->debug("Slowing plan by {}", max_velocity_ratio);
    for (int j = 0; j < num_time_steps; j++) {
      (*times)[j] *= max_velocity_ratio;
    }
  }
}

lcmt_robot_plan EncodeKeyFrames(
    const std::vector<std::string>& joint_names,
    const std::vector<double>& times,
    const std::vector<Eigen::VectorXd>& keyframes) {
  DRAKE_DEMAND(keyframes.size() == times.size());

  const int num_time_steps = keyframes.size();

  lcmt_robot_plan plan{};
  plan.utime = 0;  // I (sam.creasey) don't think this is used?
  plan.num_states = num_time_steps;
  const lcmt_robot_state default_robot_state{};
  plan.plan.resize(num_time_steps, default_robot_state);
  /// Encode the q_sol returned for each timestep into the vector of
  /// robot states.
  for (int i = 0; i < num_time_steps; i++) {
    DRAKE_DEMAND(keyframes[i].size() == static_cast<int>(joint_names.size()));
    lcmt_robot_state& step = plan.plan[i];
    step.utime = times[i] * 1e6;
    step.num_joints = keyframes[i].size();
    for (int j = 0; j < step.num_joints; j++) {
      step.joint_name.push_back(joint_names[j]);
      step.joint_position.push_back(keyframes[i](j));
      // step.joint_velocity.push_back(0);
      // step.joint_effort.push_back(0);
    }
  }

  return plan;
}

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS((
    &GetJointNames<T>
))

}  // namespace util
}  // namespace manipulation
}  // namespace drake
