#pragma once

#include <functional>
#include <map>
#include <memory>
#include <random>
#include <utility>
#include <vector>

#include "robotlocomotion/robot_plan_t.hpp"

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/examples/kuka_iiwa_arm/pick_and_place/action.h"
#include "drake/examples/kuka_iiwa_arm/pick_and_place/pick_and_place_configuration.h"
#include "drake/examples/kuka_iiwa_arm/pick_and_place/world_state.h"
#include "drake/lcmt_schunk_wsg_command.hpp"
namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace pick_and_place {


/// Different states for the pick and place task.
enum class PickAndPlaceState {
  kOpenGripper,
  kPlan,
  kApproachPickPregrasp,
  kApproachPick,
  kGrasp,
  kLiftFromPick,
  kApproachPlacePregrasp,
  kApproachPlace,
  kPlace,
  kLiftFromPlace,
  kReset,
  kDone,
};

struct PostureInterpolationRequest {
  // Initial configuration
  MatrixX<double> q_initial;
  // Final configuration
  MatrixX<double> q_final;
  // Knots
  std::vector<double> times;
  // Maximum allowable deviation from straight line end-effector path at knot
  // points
  double position_tolerance;
  // Maximum allowable angular deviation at knot points
  double orientation_tolerance;
  // If true, interpolate in joint space if the planner fails to find an
  // interpolation that provides a
  // straight-line end-effector path
  bool fall_back_to_joint_space_interpolation;
  double max_joint_position_change;
};

struct PostureInterpolationResult {
  // Configuration trajectory
  PiecewisePolynomial<double> q_traj;
  // Success
  bool success;
};

/// A class which controls the pick and place actions for moving a
/// single target in the environment.
class PickAndPlaceStateMachine {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(PickAndPlaceStateMachine)

  typedef std::function<void(
      const robotlocomotion::robot_plan_t*)> IiwaPublishCallback;
  typedef std::function<void(
      const lcmt_schunk_wsg_command*)> WsgPublishCallback;

  /// Construct a pick and place state machine.  The state machine will move the
  /// item counter-clockwise around the tables specified in @p configuration.
  /// If @p single_move is true, the state machine will remain in the kDone
  /// state after moving the object once, otherwise it will loop through the
  /// pick and place.
  PickAndPlaceStateMachine(
      const pick_and_place::PlannerConfiguration& configuration,
      bool single_move);

  ~PickAndPlaceStateMachine();

  /// Update the state machine based on the state of the world in @p
  /// env_state.  When a new robot plan is available, @p iiwa_callback
  /// will be invoked with the new plan.  If the desired gripper state
  /// changes, @p wsg_callback is invoked.
  void Update(const WorldState& env_state,
              const IiwaPublishCallback& iiwa_callback,
              const WsgPublishCallback& wsg_callback);

  PickAndPlaceState state() const { return state_; }

 private:
  bool ComputeNominalConfigurations(const WorldState& env_state,
                                    RigidBodyTree<double>* iiwa);

  bool ComputeDesiredPoses(const WorldState& env_state, double yaw_offset,
                           double pitch_offset);

  bool ComputeTrajectories(const WorldState& env_state,
                           RigidBodyTree<double>* iiwa);

  PostureInterpolationRequest CreatePostureInterpolationRequest(
      const WorldState& env_state, PickAndPlaceState state, double duration,
      bool fall_back_to_joint_space_interpolation = false);

  bool single_move_;

  WsgAction wsg_act_;
  IiwaMove iiwa_move_;

  PickAndPlaceState state_;

  // Poses used for storing end-points of Iiwa trajectories_at various states
  // of the demo.
  Isometry3<double> X_Wend_effector_0_;
  Isometry3<double> X_Wend_effector_1_;

  // Desired object end pose relative to the base of the iiwa arm.
  Isometry3<double> X_IIWAobj_desired_;

  // Desired object end pose in the world frame.
  Isometry3<double> X_Wobj_desired_;

  Vector3<double> tight_pos_tol_;
  double tight_rot_tol_;

  Vector3<double> loose_pos_tol_;
  double loose_rot_tol_;

  pick_and_place::PlannerConfiguration configuration_;

  // Desired grasp frame end-pose for various states
  std::map<PickAndPlaceState, Isometry3<double>> X_WG_desired_;

  // Desired joint configuration for various states
  std::map<PickAndPlaceState, VectorX<double>> nominal_q_map_;

  // Desired interpolation results for various states
  std::map<PickAndPlaceState, PostureInterpolationResult>
      interpolation_result_map_;

  // Measured location of object at planning time
  Isometry3<double> expected_object_pose_;

  // Joint position seed
  VectorX<double> q_seed_;

  // Counter for number of planning failures
  int planning_failure_count_{0};

  std::default_random_engine rand_generator_{1234};
};

}  // namespace pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
