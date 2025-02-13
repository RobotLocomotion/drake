#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/planning/collision_checker.h"
#include "drake/planning/collision_checker_params.h"
#include "drake/geometry/optimization/convex_set.h"

namespace drake {
namespace planning {

/** An implementation of CollisionChecker that accepts user-specified
configuration-space obstacles in its constructor and respsects collisions
against those configuration-space obstacles.

@ingroup planning_collision_checker
*/
class ConfigurationSpaceObstacleCollisionChecker final : public CollisionChecker {
 public:
  /** @name     Does not allow copy, move, or assignment. */
  /** @{ */
  // N.B. The copy constructor is private for use in implementing Clone().
  void operator=(const ConfigurationSpaceObstacleCollisionChecker&) = delete;
  /** @} */

  /** Creates a new checker with the given params. */
  ConfigurationSpaceObstacleCollisionChecker(
    copyable_unique_ptr<CollisionChecker> checker,
    geometry::optimization::ConvexSets configuration_obstacles);

  void AddConfigurationSpaceObstacles(
    geometry::optimization::ConvexSets new_obstacles);

  void SetConfigurationSpaceObstacles(
    geometry::optimization::ConvexSets new_obstacles);

 private:
  // To support Clone(), allow copying (but not move nor assign).
  explicit ConfigurationSpaceObstacleCollisionChecker(
    const ConfigurationSpaceObstacleCollisionChecker&);

  std::unique_ptr<CollisionChecker> DoClone() const final;

  void DoUpdateContextPositions(CollisionCheckerContext*) const final;

  bool DoCheckContextConfigCollisionFree(
      const CollisionCheckerContext& model_context) const final;

  std::optional<geometry::GeometryId> DoAddCollisionShapeToBody(
      const std::string& group_name, const multibody::RigidBody<double>& bodyA,
      const geometry::Shape& shape,
      const math::RigidTransform<double>& X_AG) final;

  void RemoveAddedGeometries(
      const std::vector<CollisionChecker::AddedShape>& shapes) final;

  void UpdateCollisionFilters() final;

  RobotClearance DoCalcContextRobotClearance(
      const CollisionCheckerContext& model_context,
      double influence_distance) const final;

  std::vector<RobotCollisionType> DoClassifyContextBodyCollisions(
      const CollisionCheckerContext& model_context) const final;

  int DoMaxContextNumDistances(
      const CollisionCheckerContext& model_context) const final;

  const copyable_unique_ptr<CollisionChecker> checker_;
  geometry::optimization::ConvexSets configuration_obstacles_;
};

}  // namespace planning
}  // namespace drake
