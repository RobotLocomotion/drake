#pragma once

#include <functional>
#include <memory>
#include <vector>

#include <Eigen/Core>

#include "drake/multibody/tree/multibody_tree_indexes.h"
#include "planning/robot_diagram.h"

namespace anzu {
namespace planning {

/** Configuration distance takes two configurations of the robot, q1 and q2,
both as Eigen::VectorXd, and returns (potentially weighted) C-space distance
as a double.

To be valid, the function must satisfy the following condition:

 - F(q, q) ≡ 0
*/
using ConfigurationDistanceFunction =
    std::function<double(const Eigen::VectorXd&, const Eigen::VectorXd&)>;

/** Configuration interpolation function takes two configurations of the robot,
q1, and q2, both as Eigen::VectorXd, plus a ratio in [0, 1] and returns the
interpolated configuration.

To be valid, the function must satisfy the following conditions:

 - F(q1, q2, 0) ≡ q1
 - F(q1, q2, 1) ≡ q2
 - F(q, q, for all r in [0, 1]) ≡ q
*/
using ConfigurationInterpolationFunction =
    std::function<Eigen::VectorXd(
        const Eigen::VectorXd&, const Eigen::VectorXd&, double)>;

/** A set of common constructor parameters for a CollisionChecker.
Not all subclasses of CollisionChecker will necessarily support this
configuration struct, but many do so. */
struct CollisionCheckerParams {
  /** A RobotDiagram model of the robot and environment. Must not be
  nullptr. */
  std::unique_ptr<RobotDiagram<double>> model;

  /** A vector of model instance indices that identify which model instances
  belong to the robot. The list must be non-empty and must not include the
  world model instance. */
  std::vector<drake::multibody::ModelInstanceIndex> robot_model_instances;

  /** Configuration (probably weighted) distance function.
  @note the `configuration_distance_function` object will be copied and retained
  by a collision checker, so if the function has any lambda-captured data then
  that data must outlive the collision checker. */
  ConfigurationDistanceFunction configuration_distance_function;

  /** Step size for edge checking; in units compatible with the configuration
   distance function..
  Collision checking of edges q1->q2 is performed by interpolating from q1 to q2
  at edge_step_size steps and checking the interpolated configuration for
  collision. The value must be positive. */
  double edge_step_size{};

  /** Additional padding to apply to all robot-environment collision queries. If
  distance between robot and environment is less than padding, the checker
  reports a collision. */
  double env_collision_padding{};

  /** Additional padding to apply to all robot-robot self collision queries. If
  distance between robot and itself is less than padding, the checker reports a
  collision. */
  double self_collision_padding{};
};

}  // namespace planning
}  // namespace anzu
