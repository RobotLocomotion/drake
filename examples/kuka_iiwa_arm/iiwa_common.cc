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
