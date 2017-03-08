#pragma once

#include <memory>
#include <vector>

#include "drake/common/trajectories/piecewise_quaternion.h"
#include "drake/examples/kuka_iiwa_arm/dev/iiwa_ik_planner.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace pick_and_place_demo {

// This needs to match the object model file in iiwa_wsg_simulation.cc
const double kHalfBoxHeight = 0.1;

// Desired location to place the object for table0 and table1. Positions are
// specified in the iiwa arm's base frame. Table0 is right in front of the arm's
// base.
const Vector3<double> kPlacePosition0(0.8, 0, kHalfBoxHeight);
const Vector3<double> kPlacePosition1(0, 0.8, kHalfBoxHeight);

// Determines which table is holding the object based on the object's xy
// distance to the center of each table.
// TODO(siyuan): Implement a better way to determine which table is holding the
// object.
int get_table(const Isometry3<double>& X_WObj,
              const Isometry3<double>& X_WIiiwa);

// Computes the desired end effector pose in the world frame given the object
// pose in the world frame.
Isometry3<double> ComputeGraspPose(const Isometry3<double>& X_WObj);

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
                            std::vector<double>* times);

// Different states for the pick and place task.
enum PickAndPlaceState {
  OPEN_GRIPPER,
  APPROACH_PICK_PREGRASP,
  APPROACH_PICK,
  GRASP,
  LIFT_FROM_PICK,
  APPROACH_PLACE_PREGRASP,
  APPROACH_PLACE,
  PLACE,
  LIFT_FROM_PLACE,
  DONE,
};

}  // namespace pick_and_place_demo
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
