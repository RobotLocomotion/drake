#include "drake/examples/kuka_iiwa_arm/pick_and_place/pick_and_place_state_machine.h"

#include <algorithm>
#include <limits>

#include "drake/common/drake_assert.h"
#include "drake/common/text_logging.h"
#include "drake/common/trajectories/piecewise_quaternion.h"
#include "drake/math/rotation_matrix.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace pick_and_place {
namespace {

using manipulation::planner::ConstraintRelaxingIk;

// Position the gripper 30cm above the object before grasp.
const double kPreGraspHeightOffset = 0.3;

// Computes the desired end effector pose in the world frame given the object
// pose in the world frame and the dimensions of the objects axis-aligned
// bounding box.
Isometry3<double> ComputeGraspPose(const Isometry3<double>& X_WObj,
                                   Vector3<double> object_dimensions) {
  // A conservative estimate of the fingers' length.
  const double finger_length = 0.07;

  // The grasp frame (G) should be at the center of the object if possible, but
  // no further than finger_length from the back edge of the  object.
  Isometry3<double> X_OG{Isometry3<double>::Identity()};
  X_OG.translation().x() =
      std::min<double>(0, -0.5 * object_dimensions.x() + finger_length);

  // TODO(avalenzu): Add a planning model for the gripper that includes the
  // grasp frame as a named frame.
  // The gripper (and therfore the grasp frame) is rotated relative to the end
  // effector link.
  const double grasp_frame_angular_offset{-M_PI / 8};
  // The grasp frame is located between the fingertips of the gripper which is
  // grasp_frame_translational_offset from the origin of the end-effector link.
  const double grasp_frame_translational_offset{0.19};
  Isometry3<double> X_EG{Isometry3<double>::Identity()};
  X_EG.rotate(
      Eigen::AngleAxisd(grasp_frame_angular_offset, Eigen::Vector3d::UnitX()));
  X_EG.translation().x() = grasp_frame_translational_offset;

  return X_WObj * X_OG * X_EG.inverse();
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
                            ConstraintRelaxingIk* planner, IKResults* ik_res,
                            std::vector<double>* times) {
  DRAKE_DEMAND(duration > 0 && num_via_points >= 0);
  // Makes a slerp trajectory from start to end.
  const eigen_aligned_std_vector<Quaternion<double>> quats = {
      Quaternion<double>(X_WEndEffector0.linear()),
      Quaternion<double>(X_WEndEffector1.linear())};

  const std::vector<MatrixX<double>> pos = {X_WEndEffector0.translation(),
                                            X_WEndEffector1.translation()};
  drake::log()->debug(
      "Planning straight line from {} {} to {} {}", pos[0].transpose(),
      math::rotmat2rpy(X_WEndEffector0.linear()), pos[1].transpose(),
      math::rotmat2rpy(X_WEndEffector1.linear()));

  PiecewiseQuaternionSlerp<double> rot_traj({0, duration}, quats);
  PiecewisePolynomial<double> pos_traj =
      PiecewisePolynomial<double>::FirstOrderHold({0, duration}, pos);

  std::vector<ConstraintRelaxingIk::IkCartesianWaypoint> waypoints(
      num_via_points + 1);
  const double dt = duration / (num_via_points + 1);
  double time = 0;
  times->clear();
  times->push_back(time);
  for (int i = 0; i <= num_via_points; ++i) {
    time += dt;
    times->push_back(time);
    waypoints[i].pose.translation() = pos_traj.value(time);
    waypoints[i].pose.linear() = Matrix3<double>(rot_traj.orientation(time));
    drake::log()->debug(
        "via ({}/{}): {} {}", i, num_via_points,
        waypoints[i].pose.translation().transpose(),
        math::rotmat2rpy(waypoints[i].pose.linear()).transpose());
    if (i != num_via_points) {
      waypoints[i].pos_tol = via_points_pos_tolerance;
      waypoints[i].rot_tol = via_points_rot_tolerance;
    }
    waypoints[i].constrain_orientation = true;
  }
  DRAKE_DEMAND(times->size() == waypoints.size() + 1);
  const bool planner_result =
      planner->PlanSequentialTrajectory(waypoints, q_current, ik_res);
  drake::log()->debug("q initial: {}", q_current.transpose());
  if (!ik_res->q_sol.empty()) {
    drake::log()->debug("q final: {}", ik_res->q_sol.back().transpose());
  }
  drake::log()->debug("result: {}", planner_result);
  return planner_result;
}

bool ComputeInitialAndFinalObjectPoses(const WorldState& env_state,
                                       Isometry3<double>* X_WO_initial,
                                       Isometry3<double>* X_WO_final) {
  // W -- World frame, coincides with robot base frame.
  // S -- fixed Sensor frame. All poses returned by methods of env_state are
  //      expressed relative to this frame.
  // O -- Object frame
  // T -- Table frame

  // Since env_state.get_iiwa_base() returns the pose of the robot's base
  // relative to fixed sensor frame, inverting the returned transform yields the
  // world pose of the fixed sensor frame.
  Isometry3<double> X_WS{env_state.get_iiwa_base().inverse()};
  *X_WO_initial = X_WS * env_state.get_object_pose();

  drake::log()->debug("r_WO_initial = [{}]",
                      X_WO_initial->translation().transpose());
  drake::log()->debug("R_WO_initial = \n{}", X_WO_initial->linear());
  // Check that the object is oriented correctly
  if (X_WO_initial->linear()(2, 2) < std::cos(20 * M_PI / 180)) {
    drake::log()->debug(
        "Improper object orientation relative to robot base. Please reset "
        "object and/or check Optitrack markers.");
    return false;
  }

  // Find the destination table
  const std::vector<Isometry3<double>>& table_poses =
      env_state.get_table_poses();
  int destination_table_index = -1;
  const double kMaxReach = 1.1;
  double min_angle = std::numeric_limits<double>::infinity();
  for (int i = 0; i < static_cast<int>(table_poses.size()); ++i) {
    const Isometry3<double> X_WT = X_WS * table_poses[i];
    Vector3<double> r_WT_in_xy_plane{X_WT.translation()};
    r_WT_in_xy_plane.z() = 0;
    drake::log()->debug("Table {}: Distance: {} m", i, r_WT_in_xy_plane.norm());
    if (r_WT_in_xy_plane.norm() < kMaxReach) {
      Vector3<double> r_WO_in_xy_plane{X_WO_initial->translation()};
      r_WO_in_xy_plane.z() = 0;
      const Vector3<double> dir_WO_in_xy_plane{r_WO_in_xy_plane.normalized()};
      double x = r_WT_in_xy_plane.dot(-dir_WO_in_xy_plane);
      double y = (r_WT_in_xy_plane - x * (-dir_WO_in_xy_plane))
                     .dot(Vector3<double>::UnitZ().cross(-dir_WO_in_xy_plane));
      double angle = std::atan2(y, x) + M_PI;
      drake::log()->debug("Table {}: x = {}, y = {}, Angle: {} degrees", i, x,
                          y, angle * 180 / M_PI);
      if (angle > 20 * M_PI / 180 && angle < min_angle) {
        drake::log()->debug("Table {} is the new destination candidate.", i);
        destination_table_index = i;
        min_angle = angle;
      }
    }
  }
  drake::log()->debug("Destination Table Index: {}", destination_table_index);

  if (destination_table_index < 0) {
    drake::log()->debug("Cannot find a suitable destination table.");
    return false;
  }

  // Pose of destination table in world
  const Isometry3<double> X_WT = X_WS * table_poses.at(destination_table_index);
  const Vector3<double> r_WT = X_WT.translation();
  drake::log()->debug("r_WT = [{}]", X_WT.translation().transpose());
  drake::log()->debug("R_WT = \n{}", X_WT.linear());

  Vector3<double> dir_TO_final = -X_WT.linear().inverse() * r_WT;
  dir_TO_final.z() = 0;
  dir_TO_final.normalize();
  Vector3<double> r_TO_final = Vector3<double>::Zero();
  r_TO_final.z() += 0.5 * env_state.get_object_dimensions().z();
  Matrix3<double> R_TO_final{Matrix3<double>::Identity()};
  R_TO_final.col(0) = -dir_TO_final;
  R_TO_final.col(2) = Vector3<double>::UnitZ();
  R_TO_final.col(1) = R_TO_final.col(2).cross(R_TO_final.col(0));
  Isometry3<double> X_TO_final;
  X_TO_final.translation() = r_TO_final;
  X_TO_final.linear() = R_TO_final;
  *X_WO_final = X_WT * X_TO_final;
  drake::log()->debug("dir_TO_final = [{}]", dir_TO_final.transpose());
  drake::log()->debug("r_TO_final = [{}]",
                      X_WO_final->translation().transpose());
  drake::log()->debug("R_WO_final = \n{}", X_WO_final->linear());
  return true;
}

}  // namespace

PickAndPlaceStateMachine::PickAndPlaceStateMachine(
    const pick_and_place::PlannerConfiguration& configuration, bool single_move)
    : single_move_(single_move),
      state_(kOpenGripper),
      // Position and rotation tolerances.  These were hand-tuned by
      // adjusting to tighter bounds until IK stopped reliably giving
      // results.
      tight_pos_tol_(0.005, 0.005, 0.005),
      tight_rot_tol_(0.05),
      loose_pos_tol_(0.25, 0.25, 0.25),
      loose_rot_tol_(0.5),
      configuration_(configuration) {}

PickAndPlaceStateMachine::~PickAndPlaceStateMachine() {}

void PickAndPlaceStateMachine::Update(const WorldState& env_state,
                                      const IiwaPublishCallback& iiwa_callback,
                                      const WsgPublishCallback& wsg_callback) {
  IKResults ik_res;
  std::vector<double> times;
  robotlocomotion::robot_plan_t stopped_plan{};
  stopped_plan.num_states = 0;

  switch (state_) {
    // Opens the gripper.
    case kOpenGripper: {
      if (!wsg_act_.ActionStarted()) {
        lcmt_schunk_wsg_command msg;
        wsg_act_.OpenGripper(env_state, &msg);
        wsg_callback(&msg);

        drake::log()->info("kOpenGripper at {}", env_state.get_iiwa_time());
        const Isometry3<double>& pose = env_state.get_object_pose();
        drake::log()->info("Object at: {} {}", pose.translation().transpose(),
                           math::rotmat2rpy(pose.rotation()).transpose());
      }

      if (wsg_act_.ActionFinished(env_state)) {
        state_ = kApproachPickPregrasp;
        wsg_act_.Reset();
      }
      break;
    }

    case kApproachPickPregrasp: {
      // Approaches kPreGraspHeightOffset above the center of the object.
      if (!iiwa_move_.ActionStarted()) {
        ComputeInitialAndFinalObjectPoses(env_state, &X_WO_initial_,
                                          &X_WO_final_);

        // Computes the desired end effector pose in the world frame to be
        // kPreGraspHeightOffset above the object.
        const Isometry3<double>& X_WO = X_WO_initial_;
        X_Wend_effector_0_ = env_state.get_iiwa_end_effector_pose();
        X_Wend_effector_1_ =
            ComputeGraspPose(X_WO, configuration_.target_dimensions);
        X_Wend_effector_1_.translation()[2] += kPreGraspHeightOffset;

        // 2 seconds, no via points.
        ConstraintRelaxingIk planner{configuration_.model_path,
                                     configuration_.end_effector_name,
                                     Isometry3<double>::Identity()};
        bool res = PlanStraightLineMotion(
            env_state.get_iiwa_q(), 0, 2, X_Wend_effector_0_,
            X_Wend_effector_1_, loose_pos_tol_, loose_rot_tol_, &planner,
            &ik_res, &times);
        DRAKE_DEMAND(res);

        robotlocomotion::robot_plan_t plan{};
        iiwa_move_.MoveJoints(env_state, planner.get_robot(), times,
                              ik_res.q_sol, &plan);
        iiwa_callback(&plan);

        drake::log()->info("kApproachPickPregrasp at {}",
                           env_state.get_iiwa_time());
      }

      if (iiwa_move_.ActionFinished(env_state)) {
        state_ = kApproachPick;
        iiwa_move_.Reset();
      }
      break;
    }

    case kApproachPick: {
      // Moves gripper straight down.
      if (!iiwa_move_.ActionStarted()) {
        X_Wend_effector_0_ = X_Wend_effector_1_;
        X_Wend_effector_1_.translation()[2] -= kPreGraspHeightOffset;

        // 2 seconds, 3 via points. More via points to ensure the end
        // effector moves in more or less a straight line.
        ConstraintRelaxingIk planner{configuration_.model_path,
                                     configuration_.end_effector_name,
                                     Isometry3<double>::Identity()};
        bool res = PlanStraightLineMotion(
            env_state.get_iiwa_q(), 3, 2, X_Wend_effector_0_,
            X_Wend_effector_1_, tight_pos_tol_, tight_rot_tol_, &planner,
            &ik_res, &times);
        DRAKE_DEMAND(res);

        robotlocomotion::robot_plan_t plan{};
        iiwa_move_.MoveJoints(env_state, planner.get_robot(), times,
                              ik_res.q_sol, &plan);
        iiwa_callback(&plan);

        drake::log()->info("kApproachPick at {}", env_state.get_iiwa_time());
      }

      if (iiwa_move_.ActionFinished(env_state)) {
        state_ = kGrasp;
        iiwa_callback(&stopped_plan);
        iiwa_move_.Reset();
      }
      break;
    }

    case kGrasp: {
      // Grasps the object.
      if (!wsg_act_.ActionStarted()) {
        lcmt_schunk_wsg_command msg;
        wsg_act_.CloseGripper(env_state, &msg);
        wsg_callback(&msg);

        drake::log()->info("kGrasp at {}", env_state.get_iiwa_time());
      }

      if (wsg_act_.ActionFinished(env_state)) {
        state_ = kLiftFromPick;
        wsg_act_.Reset();
      }
      break;
    }

    case kLiftFromPick: {
      // Lifts the object straight up.
      if (!iiwa_move_.ActionStarted()) {
        X_Wend_effector_0_ = X_Wend_effector_1_;
        X_Wend_effector_1_.translation()[2] += kPreGraspHeightOffset;

        // 2 seconds, 3 via points.
        ConstraintRelaxingIk planner{configuration_.model_path,
                                     configuration_.end_effector_name,
                                     Isometry3<double>::Identity()};
        bool res = PlanStraightLineMotion(
            env_state.get_iiwa_q(), 3, 2, X_Wend_effector_0_,
            X_Wend_effector_1_, tight_pos_tol_, tight_rot_tol_, &planner,
            &ik_res, &times);
        DRAKE_DEMAND(res);

        robotlocomotion::robot_plan_t plan{};
        iiwa_move_.MoveJoints(env_state, planner.get_robot(), times,
                              ik_res.q_sol, &plan);
        iiwa_callback(&plan);

        drake::log()->info("kLiftFromPick at {}", env_state.get_iiwa_time());
      }

      if (iiwa_move_.ActionFinished(env_state)) {
        state_ = kApproachPlacePregrasp;
        iiwa_move_.Reset();
      }
      break;
    }

    case kApproachPlacePregrasp: {
      // Uses 2 seconds to move to right about the target place location.
      if (!iiwa_move_.ActionStarted()) {
        const Isometry3<double>& X_WO_desired = X_WO_final_;

        X_Wend_effector_0_ = X_Wend_effector_1_;
        X_Wend_effector_1_ =
            ComputeGraspPose(X_WO_desired, configuration_.target_dimensions);
        X_Wend_effector_1_.translation()[2] += kPreGraspHeightOffset;

        // 2 seconds, no via points.
        ConstraintRelaxingIk planner{configuration_.model_path,
                                     configuration_.end_effector_name,
                                     Isometry3<double>::Identity()};
        bool res = PlanStraightLineMotion(
            env_state.get_iiwa_q(), 0, 2, X_Wend_effector_0_,
            X_Wend_effector_1_, loose_pos_tol_, loose_rot_tol_, &planner,
            &ik_res, &times);
        DRAKE_DEMAND(res);

        robotlocomotion::robot_plan_t plan{};
        iiwa_move_.MoveJoints(env_state, planner.get_robot(), times,
                              ik_res.q_sol, &plan);
        iiwa_callback(&plan);

        drake::log()->info("kApproachPlacePregrasp at {}",
                           env_state.get_iiwa_time());
      }

      if (iiwa_move_.ActionFinished(env_state)) {
        state_ = kApproachPlace;
        iiwa_move_.Reset();
      }
      break;
    }

    case kApproachPlace: {
      // Moves straight down.
      if (!iiwa_move_.ActionStarted()) {
        // Computes the desired end effector pose in the world frame.
        X_Wend_effector_0_ = X_Wend_effector_1_;
        X_Wend_effector_1_.translation()[2] -= kPreGraspHeightOffset;

        // 2 seconds, 3 via points.
        ConstraintRelaxingIk planner{configuration_.model_path,
                                     configuration_.end_effector_name,
                                     Isometry3<double>::Identity()};
        bool res = PlanStraightLineMotion(
            env_state.get_iiwa_q(), 3, 2, X_Wend_effector_0_,
            X_Wend_effector_1_, tight_pos_tol_, tight_rot_tol_, &planner,
            &ik_res, &times);
        DRAKE_DEMAND(res);

        robotlocomotion::robot_plan_t plan{};
        iiwa_move_.MoveJoints(env_state, planner.get_robot(), times,
                              ik_res.q_sol, &plan);
        iiwa_callback(&plan);

        drake::log()->info("kApproachPlace at {}", env_state.get_iiwa_time());
      }

      if (iiwa_move_.ActionFinished(env_state)) {
        state_ = kPlace;
        iiwa_callback(&stopped_plan);
        iiwa_move_.Reset();
      }
      break;
    }

    case kPlace: {
      // Releases the object.
      if (!wsg_act_.ActionStarted()) {
        lcmt_schunk_wsg_command msg;
        wsg_act_.OpenGripper(env_state, &msg);
        wsg_callback(&msg);

        drake::log()->info("kPlace at {}", env_state.get_iiwa_time());
      }

      if (wsg_act_.ActionFinished(env_state)) {
        state_ = kLiftFromPlace;
        wsg_act_.Reset();
      }
      break;
    }

    case kLiftFromPlace: {
      // Moves straight up.
      if (!iiwa_move_.ActionStarted()) {
        X_Wend_effector_0_ = X_Wend_effector_1_;
        X_Wend_effector_1_.translation()[2] += kPreGraspHeightOffset;

        // 2 seconds, 5 via points.
        ConstraintRelaxingIk planner{configuration_.model_path,
                                     configuration_.end_effector_name,
                                     Isometry3<double>::Identity()};
        bool res = PlanStraightLineMotion(
            env_state.get_iiwa_q(), 5, 2.0, X_Wend_effector_0_,
            X_Wend_effector_1_, tight_pos_tol_, tight_rot_tol_, &planner,
            &ik_res, &times);
        DRAKE_DEMAND(res);

        robotlocomotion::robot_plan_t plan{};
        iiwa_move_.MoveJoints(env_state, planner.get_robot(), times,
                              ik_res.q_sol, &plan);
        iiwa_callback(&plan);

        drake::log()->info("kLiftFromPlace at {}", env_state.get_iiwa_time());
      }

      if (iiwa_move_.ActionFinished(env_state)) {
        if (single_move_) {
          state_ = kDone;
          iiwa_callback(&stopped_plan);
          drake::log()->info("kDone at {}", env_state.get_iiwa_time());
        } else {
          state_ = kOpenGripper;
        }
        iiwa_move_.Reset();
      }
      break;
    }

    case kDone: {
      break;
    }
  }
}

}  // namespace pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
