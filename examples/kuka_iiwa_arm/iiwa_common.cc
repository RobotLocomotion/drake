#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"

#include <map>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/common/text_logging.h"
#include "drake/manipulation/util/robot_plan_utils.h"

using std::string;
using std::vector;

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

void VerifyIiwaTree(const RigidBodyTree<double>& tree) {
  std::map<std::string, int> name_to_idx = tree.computePositionNameToIndexMap();

  int joint_idx = 0;
  DRAKE_DEMAND(name_to_idx.count("iiwa_joint_1"));
  DRAKE_DEMAND(name_to_idx["iiwa_joint_1"] == joint_idx++);
  DRAKE_DEMAND(name_to_idx.count("iiwa_joint_2"));
  DRAKE_DEMAND(name_to_idx["iiwa_joint_2"] == joint_idx++);
  DRAKE_DEMAND(name_to_idx.count("iiwa_joint_3"));
  DRAKE_DEMAND(name_to_idx["iiwa_joint_3"] == joint_idx++);
  DRAKE_DEMAND(name_to_idx.count("iiwa_joint_4"));
  DRAKE_DEMAND(name_to_idx["iiwa_joint_4"] == joint_idx++);
  DRAKE_DEMAND(name_to_idx.count("iiwa_joint_5"));
  DRAKE_DEMAND(name_to_idx["iiwa_joint_5"] == joint_idx++);
  DRAKE_DEMAND(name_to_idx.count("iiwa_joint_6"));
  DRAKE_DEMAND(name_to_idx["iiwa_joint_6"] == joint_idx++);
  DRAKE_DEMAND(name_to_idx.count("iiwa_joint_7"));
  DRAKE_DEMAND(name_to_idx["iiwa_joint_7"] == joint_idx++);
}

void SetPositionControlledIiwaGains(Eigen::VectorXd* Kp,
                                    Eigen::VectorXd* Ki,
                                    Eigen::VectorXd* Kd) {
  // All the gains are for acceleration, not directly responsible for generating
  // torques. These are set to high values to ensure good tracking. These gains
  // are picked arbitrarily.
  Kp->resize(7);
  *Kp << 100, 100, 100, 100, 100, 100, 100;
  Kd->resize(Kp->size());
  for (int i = 0; i < Kp->size(); i++) {
    // Critical damping gains.
    (*Kd)[i] = 2 * std::sqrt((*Kp)[i]);
  }
  *Ki = Eigen::VectorXd::Zero(7);
}

void SetTorqueControlledIiwaGains(Eigen::VectorXd* stiffness,
                                  Eigen::VectorXd* damping_ratio) {
  // All the gains are for directly generating torques. These gains are set
  // according to the values in the drake-iiwa-driver repository:
  // https://github.com/RobotLocomotion/drake-iiwa-driver/blob/master/kuka-driver/sunrise_1.11/DrakeFRITorqueDriver.java NOLINT

  // The spring stiffness in Nm/rad.
  stiffness->resize(7);
  *stiffness << 1000, 1000, 1000, 500, 500, 500, 500;

  // A dimensionless damping ratio. See KukaTorqueController for details.
  damping_ratio->resize(stiffness->size());
  damping_ratio->setConstant(1.0);
}

void ApplyJointVelocityLimits(const MatrixX<double>& keyframes,
                              std::vector<double>* time) {
  DRAKE_DEMAND(keyframes.cols() == static_cast<int>(time->size()));

  const int num_time_steps = keyframes.cols();

  // Calculate a matrix of velocities between each timestep.  We'll
  // use this later to determine by how much the plan exceeds the
  // joint velocity limits.
  Eigen::MatrixXd velocities(keyframes.rows(), num_time_steps - 1);
  for (int i = 0; i < velocities.rows(); i++) {
    for (int j = 0; j < velocities.cols(); j++) {
      DRAKE_ASSERT((*time)[j + 1] > (*time)[j]);
      velocities(i, j) =
          std::abs((keyframes(i, j + 1) - keyframes(i, j)) /
                   ((*time)[j + 1] - (*time)[j]));
    }
  }

  DRAKE_ASSERT(velocities.rows() == kIiwaArmNumJoints);

  Eigen::VectorXd velocity_ratios(velocities.rows());

  const VectorX<double> iiwa_max_joint_velocities =
      get_iiwa_max_joint_velocities();
  for (int i = 0; i < velocities.rows(); i++) {
    const double max_plan_velocity = velocities.row(i).maxCoeff();
    // Maybe don't try max velocity at first...
    velocity_ratios(i) =
        max_plan_velocity / (iiwa_max_joint_velocities[i] * 0.9);
  }

  const double max_velocity_ratio = velocity_ratios.maxCoeff();
  if (max_velocity_ratio > 1) {
    // The code below slows the entire plan such that the fastest step
    // meets the limits.  If that step is much faster than the others,
    // the whole plan becomes very slow.
    drake::log()->debug("Slowing plan by {}", max_velocity_ratio);
    for (int j = 0; j < num_time_steps; j++) {
      (*time)[j] *= max_velocity_ratio;
    }
  }
}

robotlocomotion::robot_plan_t EncodeKeyFrames(
    const RigidBodyTree<double>& robot,
    const std::vector<double>& time,
    const std::vector<int>& info,
    const MatrixX<double>& keyframes) {
  const int num_positions = robot.get_num_positions();
  DRAKE_DEMAND(keyframes.rows() == num_positions);
  std::vector<std::string> joint_names(num_positions);
  for (int i = 0; i < num_positions; ++i) {
    joint_names[i] = robot.get_position_name(i);
  }

  return EncodeKeyFrames(joint_names, time, info, keyframes);
}

robotlocomotion::robot_plan_t EncodeKeyFrames(
    const std::vector<std::string>& joint_names,
    const std::vector<double>& time,
    const std::vector<int>& info,
    const MatrixX<double>& keyframes) {
  std::vector<Eigen::VectorXd> waypoints;
  for (int i = 0; i < keyframes.cols(); ++i) {
    waypoints.push_back(keyframes.col(i));
  }

  return manipulation::util::EncodeKeyFrames(
      joint_names, time, info, waypoints);
}

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
