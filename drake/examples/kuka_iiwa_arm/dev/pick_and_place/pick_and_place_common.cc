#include "drake/examples/kuka_iiwa_arm/dev/pick_and_place/pick_and_place_common.h"

#include <list>
#include <memory>
#include <vector>

#include "drake/common/trajectories/piecewise_quaternion.h"
#include "drake/examples/kuka_iiwa_arm/dev/iiwa_ik_planner.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace pick_and_place_demo {

// Determines which table is holding the object based on the object's xy
// distance to the center of each table.
// TODO(siyuan): Implement a better way to determine which table is holding the
// object.
int get_table(const Isometry3<double>& X_WObj,
              const Isometry3<double>& X_WIiiwa) {
  // These need to match iiwa_wsg_simulation.cc's table configuration.
  const Vector3<double> p_IiwaMidTable0 = Vector3<double>(0.8, 0, 0);
  const Vector3<double> p_IiwaMidTable1 = Vector3<double>(0, 0.85, 0);

  Isometry3<double> X_IiwaObj = X_WIiiwa.inverse() * X_WObj;
  double dist_to_table_0 = (X_IiwaObj.translation() - p_IiwaMidTable0).norm();
  double dist_to_table_1 = (X_IiwaObj.translation() - p_IiwaMidTable1).norm();

  int table = 1;
  if (dist_to_table_0 < dist_to_table_1) table = 0;
  return table;
}

// Computes the desired end effector pose in the world frame given the object
// pose in the world frame.
Isometry3<double> ComputeGraspPose(const Isometry3<double>& X_WObj) {
  // Sets desired end effector location to be 12cm behind the object,
  // with the same orientation relative to the object frame. This number
  // dependents on the length of the finger and how the gripper is attached.
  const double kEndEffectorToMidFingerDepth = 0.12;
  Isometry3<double> X_ObjEndEffector_desired;
  X_ObjEndEffector_desired.translation() =
      Vector3<double>(-kEndEffectorToMidFingerDepth, 0, 0);
  X_ObjEndEffector_desired.linear().setIdentity();
  return X_WObj * X_ObjEndEffector_desired;
}

// Generates a sequence (@p num_via_points + 1) of key frames s.t. the end
// effector moves in a straight line between @pX_WEndEffector0 and
// @p X_WEndEffector1. Orientation is interpolated with slerp. Intermediate
// waypoints' tolerance can be adjusted separately.
bool PlanStraightLineMotion(const VectorX<double>& q_current,
                            const int num_via_points, double duration,
                            const Isometry3<double>& X_WEndEffector0,
                            const Isometry3<double>& X_WEndEffector1,
                            const Vector3<double>& via_points_pos_tolerance,
                            const double via_points_rot_tolerance,
                            IiwaIkPlanner* planner, IKResults* ik_res,
                            std::vector<double>* times) {
  DRAKE_DEMAND(duration > 0 && num_via_points >= 0);
  // Makes a slerp trajectory from start to end.
  const eigen_aligned_std_vector<Quaternion<double>> quats = {
      Quaternion<double>(X_WEndEffector0.linear()),
      Quaternion<double>(X_WEndEffector1.linear())};

  const std::vector<MatrixX<double>> pos = {X_WEndEffector0.translation(),
                                            X_WEndEffector1.translation()};
  PiecewiseQuaternionSlerp<double> rot_traj({0, duration}, quats);
  PiecewisePolynomial<double> pos_traj =
      PiecewisePolynomial<double>::FirstOrderHold({0, duration}, pos);

  std::vector<IiwaIkPlanner::IkCartesianWaypoint> waypoints(num_via_points + 1);
  const double dt = duration / (num_via_points + 1);
  double time = 0;
  times->clear();
  times->push_back(time);
  for (int i = 0; i <= num_via_points; ++i) {
    time += dt;
    times->push_back(time);
    waypoints[i].pose.translation() = pos_traj.value(time);
    waypoints[i].pose.linear() = Matrix3<double>(rot_traj.orientation(time));
    if (i != num_via_points) {
      waypoints[i].pos_tol = via_points_pos_tolerance;
      waypoints[i].rot_tol = via_points_rot_tolerance;
    }
    waypoints[i].constrain_orientation = true;
  }
  DRAKE_DEMAND(times->size() == waypoints.size() + 1);
  return planner->PlanSequentialTrajectory(waypoints, q_current, ik_res);
}

}  // namespace pick_and_place_demo
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
