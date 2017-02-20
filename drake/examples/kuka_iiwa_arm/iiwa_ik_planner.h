#pragma once

#include <memory>
#include <random>
#include <string>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/common/trajectories/piecewise_polynomial_trajectory.h"
#include "drake/multibody/rigid_body_tree.h"
#include "robotlocomotion/robot_plan_t.hpp"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

/**
 * A wrapper class around the IK planner. This class improves IK's usability by
 * handling constraint relaxing and multiple initial guesses internally.
 */
class IiwaIkPlanner {
 public:
  /**
   * Struct holding results from the IK planner.
   */
  struct IkResult {
    /// Time steps.
    std::vector<double> time;
    /// Solver information indicating success or failure mode.
    std::vector<int> info;
    /// Resulting generalized position per time step. N by T, where N is the
    /// size of the generalized position, and T is the size of time steps.
    MatrixX<double> q;

    /**
     * Returns a robotlocomotion::robot_plan_t message.
     */
    robotlocomotion::robot_plan_t EncodeMessage(
        const RigidBodyTree<double>& robot) const {
      DRAKE_DEMAND(q.cols() == static_cast<int>(time.size()));
      DRAKE_DEMAND(q.rows() == robot.get_num_positions());
      DRAKE_DEMAND(info.size() == time.size());

      robotlocomotion::robot_plan_t plan;
      plan.utime = 0;
      plan.robot_name = "iiwa";
      plan.num_states = q.cols();
      plan.plan.resize(plan.num_states);
      plan.plan_info.resize(plan.num_states, 0);

      for (int t = 0; t < plan.num_states; t++) {
        bot_core::robot_state_t& step = plan.plan[t];

        step.num_joints = robot.get_num_positions();
        step.joint_name.resize(step.num_joints);
        for (int i = 0; i < step.num_joints; i++) {
          step.joint_name[i] = robot.get_position_name(i);
        }
        step.joint_position.resize(step.num_joints, 0);
        step.joint_velocity.resize(step.num_joints, 0);
        step.joint_effort.resize(step.num_joints, 0);

        step.utime = time[t] * 1e6;
        for (int j = 0; j < step.num_joints; j++) {
          step.joint_position[j] = q(j, t);
        }
        plan.plan_info[t] = info[t];
      }
      plan.num_grasp_transitions = 0;
      plan.left_arm_control_type = plan.POSITION;
      plan.right_arm_control_type = plan.NONE;
      plan.left_leg_control_type = plan.NONE;
      plan.right_leg_control_type = plan.NONE;
      plan.num_bytes = 0;

      return plan;
    }
  };

  /**
   * Cartesian waypoint. Input to the IK solver.
   */
  struct IkCartesianWaypoint {
    /// Time step.
    double time;
    /// Desired end effector pose in the world frame.
    Isometry3<double> pose;
    /// Bounding box for the end effector in the world frame.
    Vector3<double> pos_tol;
    /// Max angle difference between solved end effector's orientation and
    /// the desired.
    double rot_tol;
    /// Signals if orientation constraint is enabled.
    bool constrain_orientation;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /// Default constructor.
    IkCartesianWaypoint() {
      time = 0;
      pose.setIdentity();
      constrain_orientation = false;
      pos_tol = Vector3<double>(0.005, 0.005, 0.005);
      rot_tol = 0.05;
    }
  };

  /**
   * Returns a linear PiecewisePolynomialTrajectory from @p ik_res.
   */
  static std::unique_ptr<PiecewisePolynomialTrajectory>
  GenerateFirstOrderHoldTrajectory(const IkResult& ik_res);

  /**
   * Constructor. Instantiates an internal RigidBodyTree from @p model_path.
   * @param model_path Path to the model file.
   * @param end_effector_link_name Link name of the end effector.
   * @param base_to_world X_WB, transformation from robot's base to the world
   * frame.
   * @param random_seed Seed for the random number generator used to generate
   * random initial guesses.
   */
  IiwaIkPlanner(const std::string& model_path,
                const std::string& end_effector_link_name,
                const Isometry3<double>& base_to_world, int random_seed = 1234);

  /**
   * Constructor. Instantiates an internal RigidBodyTree from @p model_path.
   * @param model_path Path to the model file.
   * @param end_effector_link_name Link name of the end effector.
   * @param base_to_world X_WB, transformation from robot's base to the world
   * frame.
   * @param random_seed Seed for the random number generator used to generate
   * random initial guesses.
   */
  IiwaIkPlanner(const std::string& model_path,
                const std::string& end_effector_link_name,
                std::shared_ptr<RigidBodyFrame<double>> base_to_world,
                int random_seed = 1234);

  /**
   * Sets end effector to @p end_effector_body.
   */
  void SetEndEffector(const RigidBody<double>& end_effector_body) {
    end_effector_body_idx_ = end_effector_body.get_body_index();
  }

  /**
   * Sets end effector to @p link_name.
   */
  void SetEndEffector(const std::string& link_name) {
    end_effector_body_idx_ = robot_->FindBodyIndex(link_name);
  }

  /**
   * Returns constant reference to the robot model.
   */
  const RigidBodyTree<double>& get_robot() const { return *robot_; }

  /**
   * Generates IK solutions for each waypoint sequentially. For waypoint wp_i,
   * the IK tries to solve q_i that satisfies the end effector constraints in
   * wp_i and minimizes the squared difference to q_{i-1}, where q_{i-1} is the
   * solution to the previous wp_{i-1}. q_{i-1} = @p q_current when i = 0. This
   * function internally does constraint relaxing and initial condition
   * guessing if necessary.
   *
   * Note that @p q_current is inserted at the beginning of @p ik_res.
   *
   * @param waypoints A sequence of desired waypoints.
   * @param q_current The initial generalized position.
   * @param[out] ik_res Results.
   * @return True if solved successfully.
   */
  bool PlanSequentialTrajectory(
      const std::vector<IkCartesianWaypoint>& waypoints,
      const VectorX<double>& q_current, IkResult* ik_res);

  /**
   * Returns a linear PiecewisePolynomialTrajectory from the IK solutions
   * that goes through @p way_point_list at @p time_stamps. Only the position
   * of the end effector is constrained, and the tolerance is specified by
   * @p position_tol for all the waypoints.
   */
  std::unique_ptr<PiecewisePolynomialTrajectory>
  GenerateFirstOrderHoldTrajectoryFromCartesianWaypoints(
      const std::vector<double>& time_stamps,
      const std::vector<Vector3<double>>& way_point_list,
      const Vector3<double>& position_tol = Vector3<double>(0.005, 0.005,
                                                            0.005));

 private:
  bool SolveIk(const IkCartesianWaypoint& waypoint, const VectorX<double>& q0,
               const VectorX<double>& q_nom,
               const Vector3<double>& position_tol, double rot_tolerance,
               VectorX<double>* ik_res);

  std::default_random_engine rand_generator_;
  std::unique_ptr<RigidBodyTree<double>> robot_{nullptr};
  int end_effector_body_idx_;
};

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
