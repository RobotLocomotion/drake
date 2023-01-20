#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "drake/planning/collision_checker.h"
#include "drake/planning/collision_checker_params.h"

namespace anzu {
namespace planning {

class SceneGraphCollisionChecker final
    : public drake::planning::CollisionChecker {
 public:
  /** @name     Does not allow copy, move, or assignment. */
  /** @{ */
  // N.B. The copy constructor is private for use in implementing Clone().
  void operator=(const SceneGraphCollisionChecker&) = delete;
  /** @} */

  explicit SceneGraphCollisionChecker(
      drake::planning::CollisionCheckerParams params);

 private:
  // To support Clone(), allow copying (but not move nor assign).
  explicit SceneGraphCollisionChecker(const SceneGraphCollisionChecker&);

  std::unique_ptr<drake::planning::CollisionChecker> DoClone() const final;

  void DoUpdateContextPositions(
      drake::planning::CollisionCheckerContext*) const final;

  bool DoCheckContextConfigCollisionFree(
      const drake::planning::CollisionCheckerContext& model_context)
      const final;

  std::optional<drake::geometry::GeometryId> DoAddCollisionShapeToBody(
      const std::string& group_name,
      const drake::multibody::Body<double>& bodyA,
      const drake::geometry::Shape& shape,
      const drake::math::RigidTransform<double>& X_AG) final;

  void DoRemoveAddedGeometries(
      const std::vector<drake::planning::CollisionChecker::AddedShape>& shapes)
      final;

  drake::planning::RobotClearance DoCalcContextRobotClearance(
      const drake::planning::CollisionCheckerContext& model_context,
      double influence_distance) const final;

  std::vector<drake::planning::RobotCollisionType>
  DoClassifyContextBodyCollisions(
      const drake::planning::CollisionCheckerContext& model_context)
      const final;

  int DoMaxContextNumDistances(
      const drake::planning::CollisionCheckerContext& model_context)
      const final;
};

}  // namespace planning
}  // namespace anzu
