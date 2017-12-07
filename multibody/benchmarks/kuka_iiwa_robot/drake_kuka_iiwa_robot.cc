#include "drake/multibody/benchmarks/kuka_iiwa_robot/drake_kuka_iiwa_robot.h"

#include "drake/common/eigen_autodiff_types.h"

namespace drake {
namespace multibody {
namespace benchmarks {
namespace kuka_iiwa_robot {

// Explicitly instantiates on the most common scalar types.
template struct KukaRobotJointReactionForces<double>;
template struct KukaRobotJointReactionForces<AutoDiffXd>;

template class DrakeKukaIIwaRobot<double>;
template class DrakeKukaIIwaRobot<AutoDiffXd>;

}  // namespace kuka_iiwa_robot
}  // namespace benchmarks
}  // namespace multibody
}  // namespace drake
