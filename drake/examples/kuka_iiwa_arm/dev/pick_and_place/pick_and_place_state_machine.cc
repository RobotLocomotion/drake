/**
 * @file This file implements a state machine that drives the kuka iiwa arm to
 * pick up a block from one table to place it on another repeatedly.
 */

#include <iostream>
#include <list>
#include <memory>

#include <lcm/lcm-cpp.hpp>

#include "bot_core/robot_state_t.hpp"
#include "drake/common/drake_path.h"
#include "drake/common/trajectories/piecewise_quaternion.h"
#include "drake/examples/kuka_iiwa_arm/dev/iiwa_ik_planner.h"
#include "drake/examples/kuka_iiwa_arm/dev/pick_and_place/action.h"
#include "drake/examples/kuka_iiwa_arm/dev/pick_and_place/world_state.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/lcmt_iiwa_status.hpp"

#include "drake/lcmt_schunk_wsg_command.hpp"
#include "drake/lcmt_schunk_wsg_status.hpp"
#include "drake/util/lcmUtil.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace pick_and_place_demo {
namespace {


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

// Makes a state machine that drives the iiwa to pick up a block from one table
// and place it on the other table.
void RunPickAndPlaceDemo() {
  lcm::LCM lcm;

  const std::string iiwa_path =
      GetDrakePath() + "/examples/kuka_iiwa_arm/models/iiwa14/iiwa14.urdf";
  const std::string iiwa_end_effector_name = "iiwa_link_ee";

  // Makes a WorldState, and sets up LCM subscriptions.
  WorldState env_state(iiwa_path, iiwa_end_effector_name, &lcm);
  env_state.SubscribeToWsgStatus("SCHUNK_WSG_STATUS");
  env_state.SubscribeToIiwaStatus("IIWA_STATE_EST");
  env_state.SubscribeToObjectStatus("OBJECT_STATE_EST");

  // Spins until at least one message is received from every LCM channel.
  while (lcm.handleTimeout(10) == 0 || env_state.get_iiwa_time() == -1 ||
         env_state.get_obj_time() == -1 || env_state.get_wsg_time() == -1) {
  }

  // Makes a planner.
  const Isometry3<double> iiwa_base = env_state.get_iiwa_base();
  std::shared_ptr<RigidBodyFrame<double>> iiwa_base_frame =
      std::make_shared<RigidBodyFrame<double>>("world", nullptr, iiwa_base);
  IiwaIkPlanner planner(iiwa_path, iiwa_end_effector_name, iiwa_base);
  IKResults ik_res;
  std::vector<double> times;

  // Makes action handles.
  WsgAction wsg_act("SCHUNK_WSG_COMMAND", &lcm);
  IiwaMove iiwa_move(env_state.get_iiwa(), "COMMITTED_ROBOT_PLAN", &lcm);

  // Desired end effector pose in the world frame for pick and place.
  Isometry3<double> X_WEndEffector0, X_WEndEffector1;

  // Desired object end pose relative to the base of the iiwa arm.
  Isometry3<double> X_IiwaObj_desired;

  // Desired object end pose in the world frame.
  Isometry3<double> X_WObj_desired;

  // Set initial state
  PickAndPlaceState state = OPEN_GRIPPER;

  // Position the gripper 30cm above the object before grasp.
  const double kPreGraspHeightOffset = 0.3;

  // Position and rotation tolerances.
  // These should be adjusted to a tight bound until IK stops reliably giving
  // results.
  const Vector3<double> kTightPosTol(0.005, 0.005, 0.005);
  const double kTightRotTol = 0.05;

  const Vector3<double> kLoosePosTol(0.05, 0.05, 0.05);
  const double kLooseRotTol = 0.5;

  // lcm handle loop
  while (true) {
    // Handles all messages.
    while (lcm.handleTimeout(10) == 0) {
    }

    switch (state) {
      // Opens the gripper.
      case OPEN_GRIPPER:
        if (!wsg_act.ActionStarted()) {
          wsg_act.OpenGripper(env_state);
          std::cout << "OPEN_GRIPPER: " << env_state.get_iiwa_time()
                    << std::endl;
        }

        if (wsg_act.ActionFinished(env_state)) {
          state = APPROACH_PICK_PREGRASP;
          wsg_act.Reset();
        }
        break;

      // Approaches kPreGraspHeightOffset above the center of the object.
      case APPROACH_PICK_PREGRASP:
        if (!iiwa_move.ActionStarted()) {
          // Computes the desired end effector pose in the world frame to be
          // kPreGraspHeightOffset above the object.
          X_WEndEffector0 = env_state.get_iiwa_end_effector_pose();
          X_WEndEffector1 = ComputeGraspPose(env_state.get_object_pose());
          X_WEndEffector1.translation()[2] += kPreGraspHeightOffset;

          // 2 seconds, no via points.
          bool res = PlanStraightLineMotion(
              env_state.get_iiwa_q(), 0, 2, X_WEndEffector0, X_WEndEffector1,
              kLoosePosTol, kLooseRotTol, &planner, &ik_res, &times);
          DRAKE_DEMAND(res);
          iiwa_move.MoveJoints(env_state, times, ik_res.q_sol);

          std::cout << "APPROACH_PICK_PREGRASP: " << env_state.get_iiwa_time()
                    << std::endl;
        }

        if (iiwa_move.ActionFinished(env_state)) {
          state = APPROACH_PICK;
          iiwa_move.Reset();
        }
        break;

      // Moves gripper straight down.
      case APPROACH_PICK:
        if (!iiwa_move.ActionStarted()) {
          X_WEndEffector0 = X_WEndEffector1;
          X_WEndEffector1 = ComputeGraspPose(env_state.get_object_pose());

          // 1 second, 3 via points. More via points to ensure the end effector
          // moves in more or less a straight line.
          bool res = PlanStraightLineMotion(
              env_state.get_iiwa_q(), 3, 1, X_WEndEffector0, X_WEndEffector1,
              kTightPosTol, kTightRotTol, &planner, &ik_res, &times);
          DRAKE_DEMAND(res);

          iiwa_move.MoveJoints(env_state, times, ik_res.q_sol);
          std::cout << "APPROACH_PICK: " << env_state.get_iiwa_time()
                    << std::endl;
        }

        if (iiwa_move.ActionFinished(env_state)) {
          state = GRASP;
          iiwa_move.Reset();
        }
        break;

      // Grasps the object.
      case GRASP:
        if (!wsg_act.ActionStarted()) {
          wsg_act.CloseGripper(env_state);
          std::cout << "GRASP: " << env_state.get_iiwa_time() << std::endl;
        }

        if (wsg_act.ActionFinished(env_state)) {
          state = LIFT_FROM_PICK;
          wsg_act.Reset();
        }
        break;

      // Lifts the object straight up.
      case LIFT_FROM_PICK:
        if (!iiwa_move.ActionStarted()) {
          X_WEndEffector0 = X_WEndEffector1;
          X_WEndEffector1.translation()[2] += kPreGraspHeightOffset;

          // 1 seconds, 3 via points.
          bool res = PlanStraightLineMotion(
              env_state.get_iiwa_q(), 3, 1, X_WEndEffector0, X_WEndEffector1,
              kTightPosTol, kTightRotTol, &planner, &ik_res, &times);
          DRAKE_DEMAND(res);

          iiwa_move.MoveJoints(env_state, times, ik_res.q_sol);
          std::cout << "LIFT_FROM_PICK: " << env_state.get_iiwa_time()
                    << std::endl;
        }

        if (iiwa_move.ActionFinished(env_state)) {
          state = APPROACH_PLACE_PREGRASP;
          iiwa_move.Reset();
        }
        break;

      // Uses 2 seconds to move to right about the target place location.
      case APPROACH_PLACE_PREGRASP:
        if (!iiwa_move.ActionStarted()) {
          int table = get_table(env_state.get_object_pose(), iiwa_base);

          // Sets desired place location based on where we picked up the object.
          // Table 0 is in front of iiwa base, and table 1 is to the left.
          if (table == 0) {
            // Table 0 -> table 1.
            X_IiwaObj_desired.translation() = kPlacePosition1;
            X_IiwaObj_desired.linear() = Matrix3<double>(
                AngleAxis<double>(M_PI / 2., Vector3<double>::UnitZ()));
          } else {
            // Table 1 -> table 0.
            X_IiwaObj_desired.translation() = kPlacePosition0;
            X_IiwaObj_desired.linear().setIdentity();
          }
          X_WObj_desired = iiwa_base * X_IiwaObj_desired;

          // Recomputes gripper's pose relative the object since the object
          // probably moved during transfer.
          const Isometry3<double> X_ObjEndEffector =
              env_state.get_object_pose().inverse() *
              env_state.get_iiwa_end_effector_pose();

          X_WEndEffector0 = X_WEndEffector1;
          X_WEndEffector1 = X_WObj_desired * X_ObjEndEffector;
          X_WEndEffector1.translation()[2] += kPreGraspHeightOffset;

          // 2 seconds, 2 via points. This doesn't have to be a move straight
          // primitive. I did it this way because I have seen the IK gives a
          // wild motion that causes the gripper to lose the object.
          bool res = PlanStraightLineMotion(
              env_state.get_iiwa_q(), 2, 2, X_WEndEffector0, X_WEndEffector1,
              kLoosePosTol, kLooseRotTol, &planner, &ik_res, &times);
          DRAKE_DEMAND(res);

          iiwa_move.MoveJoints(env_state, times, ik_res.q_sol);
          std::cout << "APPROACH_PLACE_PREGRASP: " << env_state.get_iiwa_time()
                    << std::endl;
        }

        if (iiwa_move.ActionFinished(env_state)) {
          state = APPROACH_PLACE;
          iiwa_move.Reset();
        }
        break;

      // Moves straight down.
      case APPROACH_PLACE:
        if (!iiwa_move.ActionStarted()) {
          // Recomputes gripper's pose relative the object since the object
          // probably moved during transfer.
          const Isometry3<double> X_ObjEndEffector =
              env_state.get_object_pose().inverse() *
              env_state.get_iiwa_end_effector_pose();

          // Computes the desired end effector pose in the world frame.
          X_WEndEffector0 = X_WEndEffector1;
          X_WEndEffector1 = X_WObj_desired * X_ObjEndEffector;
          // TODO(siyuan): This hack is to prevent the robot from forcefully
          // pushing the object into the table. Shouldn't be necessary once
          // we have guarded moves supported by the controller side.
          X_WEndEffector1.translation()[2] += 0.02;

          // 1 seconds, 3 via points.
          bool res = PlanStraightLineMotion(
              env_state.get_iiwa_q(), 3, 1, X_WEndEffector0, X_WEndEffector1,
              kTightPosTol, kTightRotTol, &planner, &ik_res, &times);
          DRAKE_DEMAND(res);

          iiwa_move.MoveJoints(env_state, times, ik_res.q_sol);
          std::cout << "APPROACH_PLACE: " << env_state.get_iiwa_time()
                    << std::endl;
        }

        if (iiwa_move.ActionFinished(env_state)) {
          state = PLACE;
          iiwa_move.Reset();
        }
        break;

      // Releases the object.
      case PLACE:
        if (!wsg_act.ActionStarted()) {
          wsg_act.OpenGripper(env_state);
          std::cout << "PLACE: " << env_state.get_iiwa_time() << std::endl;
        }

        if (wsg_act.ActionFinished(env_state)) {
          state = LIFT_FROM_PLACE;
          wsg_act.Reset();
        }
        break;

      // Moves straight up.
      case LIFT_FROM_PLACE:
        if (!iiwa_move.ActionStarted()) {
          X_WEndEffector0 = X_WEndEffector1;
          X_WEndEffector1.translation()[2] += kPreGraspHeightOffset;

          // 1 seconds, 3 via points.
          bool res = PlanStraightLineMotion(
              env_state.get_iiwa_q(), 3, 1, X_WEndEffector0, X_WEndEffector1,
              kTightPosTol, kTightRotTol, &planner, &ik_res, &times);
          DRAKE_DEMAND(res);

          iiwa_move.MoveJoints(env_state, times, ik_res.q_sol);
          std::cout << "LIFT_FROM_PLACE: " << env_state.get_iiwa_time()
                    << std::endl;
        }

        if (iiwa_move.ActionFinished(env_state)) {
          state = OPEN_GRIPPER;
          iiwa_move.Reset();
        }
        break;

      case DONE:
        if (!iiwa_move.ActionStarted()) {
          const std::vector<double> time = {0, 2};
          std::vector<VectorX<double>> q(2, env_state.get_iiwa_q());
          q[1].setZero();
          iiwa_move.MoveJoints(env_state, time, q);
          std::cout << "DONE: " << env_state.get_iiwa_time() << std::endl;
        }

        if (iiwa_move.ActionFinished(env_state)) {
          state = OPEN_GRIPPER;
          iiwa_move.Reset();
        }
        break;
    }
  }
}

}  // namespace
}  // namespace pick_and_place_demo
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

int main(int argc, const char* argv[]) {
  drake::examples::kuka_iiwa_arm::pick_and_place_demo::RunPickAndPlaceDemo();
  return 0;
}
