#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"

#include <cmath>

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

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
