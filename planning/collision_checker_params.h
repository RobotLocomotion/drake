#pragma once

#include <functional>
#include <memory>
#include <vector>

#include <Eigen/Core>

#include "drake/common/parallelism.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"
#include "drake/planning/distance_and_interpolation_provider.h"
#include "drake/planning/robot_diagram.h"

namespace drake {
namespace planning {

/** Configuration distance takes two configurations of the robot, q1 and q2,
both as Eigen::VectorXd, and returns (potentially weighted) C-space distance
as a double. The returned distance will be strictly non-negative.

To be valid, the function must satisfy the following condition:

 - dist(q, q) ≡ 0

for values of q that are valid for the CollisionChecker's plant. */
using ConfigurationDistanceFunction =
    std::function<double(const Eigen::VectorXd&, const Eigen::VectorXd&)>;

// TODO(SeanCurtis-TRI): Move the default interpolator into the params.
/** Configuration interpolation function takes two configurations of the robot,
q1, and q2, both as Eigen::VectorXd, plus a ratio, r, in [0, 1] and returns the
interpolated configuration. Behavior of the function for values of r outside of
the range [0,1] is undefined.

To be valid, the function must satisfy the following conditions:

 - interpolate(q1, q2, 0) ≡ q1
 - interpolate(q1, q2, 1) ≡ q2
 - interpolate(q, q, r) ≡ q, for all r in [0, 1]

for values of q, q1, and q2 that are valid for the CollisionChecker's plant. */
using ConfigurationInterpolationFunction = std::function<Eigen::VectorXd(
    const Eigen::VectorXd&, const Eigen::VectorXd&, double)>;

/** A set of common constructor parameters for a CollisionChecker.
Not all subclasses of CollisionChecker will necessarily support this
configuration struct, but many do so.

@ingroup planning_collision_checker */
struct CollisionCheckerParams {
  /** A RobotDiagram model of the robot and environment. Must not be
  nullptr. */
  std::shared_ptr<RobotDiagram<double>> model;

  /** A DistanceAndInterpolationProvider to support configuration distance and
  interpolation operations.
  @note Either a DistanceAndInterpolationProvider OR a
  ConfigurationDistanceFunction may be provided, not both. If neither is
  provided, a LinearDistanceAndInterpolationProvider with default weights is
  used. */
  std::shared_ptr<const DistanceAndInterpolationProvider>
      distance_and_interpolation_provider;

  // TODO(SeanCurtis-TRI): add doc hyperlinks to edge checking doc.
  /** A vector of model instance indices that identify which model instances
  belong to the robot. The list must be non-empty and must not include the
  world model instance. */
  std::vector<drake::multibody::ModelInstanceIndex> robot_model_instances;

  // TODO(SeanCurtis-TRI): add doc hyperlinks to edge checking doc.
  // TODO(calderpg-tri, jwnimmer-tri) Deprecate support for separate distance
  // and interpolation functions.
  /** Configuration (probably weighted) distance function.
  @note Either a DistanceAndInterpolationProvider OR a
  ConfigurationDistanceFunction may be provided, not both. If neither is
  provided, a LinearDistanceAndInterpolationProvider with default weights is
  used.
  @note the `configuration_distance_function` object will be copied and retained
  by a collision checker, so if the function has any lambda-captured data then
  that data must outlive the collision checker. */
  ConfigurationDistanceFunction configuration_distance_function;

  // TODO(SeanCurtis-TRI): add doc hyperlinks to edge checking doc.
  /** Step size for edge checking; in units compatible with the configuration
  distance function.
  Collision checking of edges q1->q2 is performed by interpolating from q1 to q2
  at edge_step_size steps and checking the interpolated configuration for
  collision. The value must be positive. */
  double edge_step_size{};

  // TODO(SeanCurtis-TRI): add doc hyperlinks to edge checking doc.
  /** Additional padding to apply to all robot-environment collision queries. If
  distance between robot and environment is less than padding, the checker
  reports a collision. */
  double env_collision_padding{};

  // TODO(SeanCurtis-TRI): add doc hyperlinks to edge checking doc.
  /** Additional padding to apply to all robot-robot self collision queries. If
  distance between robot and itself is less than padding, the checker reports a
  collision. */
  double self_collision_padding{};

  /** Specify how many contexts should be allocated to support collision checker
  implicit context parallelism. Defaults to the maximum parallelism. If the
  specific collision checker type in use declares that it *does not* support
  parallel queries, then implicit context parallelism is set to None().
  @see @ref ccb_implicit_contexts "Implicit Context Parallelism". */
  Parallelism implicit_context_parallelism = Parallelism::Max();
};

}  // namespace planning
}  // namespace drake
