#include "drake/manipulation/planner/rbt_differential_inverse_kinematics.h"

namespace drake {
namespace manipulation {
namespace planner {
namespace rbt {

DifferentialInverseKinematicsResult DoDifferentialInverseKinematics(
    const RigidBodyTree<double>& robot, const KinematicsCache<double>& cache,
    const Isometry3<double>& X_WE_desired,
    const RigidBodyFrame<double>& frame_E,
    const DifferentialInverseKinematicsParameters& parameters) {
  const Isometry3<double> X_WE =
      robot.CalcFramePoseInWorldFrame(cache, frame_E);
  const Vector6<double> V_WE_desired =
      ComputePoseDiffInCommonFrame(X_WE, X_WE_desired) /
      parameters.get_timestep();
  // Call the below function.
  return drake::manipulation::planner::rbt::DoDifferentialInverseKinematics(
      robot, cache, V_WE_desired, frame_E, parameters);
}

DifferentialInverseKinematicsResult DoDifferentialInverseKinematics(
    const RigidBodyTree<double>& robot, const KinematicsCache<double>& cache,
    const Vector6<double>& V_WE_desired, const RigidBodyFrame<double>& frame_E,
    const DifferentialInverseKinematicsParameters& parameters) {
  Isometry3<double> X_WE = robot.CalcFramePoseInWorldFrame(cache, frame_E);
  MatrixX<double> J_WE =
      robot.CalcFrameSpatialVelocityJacobianInWorldFrame(cache, frame_E);
  // Call the (non-attic) helper function.
  return drake::manipulation::planner::detail::DoDifferentialInverseKinematics(
      cache.getQ(), cache.getV(), X_WE, J_WE, V_WE_desired, parameters);
}

}  // namespace rbt
}  // namespace planner
}  // namespace manipulation
}  // namespace drake
