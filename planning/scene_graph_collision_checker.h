#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "drake/planning/collision_checker.h"
#include "drake/planning/collision_checker_params.h"

namespace drake {
namespace planning {

/** An implementation of CollisionChecker that uses SceneGraph for its
calculations. */
class SceneGraphCollisionChecker final : public CollisionChecker {
 public:
  /** @name     Does not allow copy, move, or assignment. */
  /** @{ */
  // N.B. The copy constructor is private for use in implementing Clone().
  void operator=(const SceneGraphCollisionChecker&) = delete;
  /** @} */

  /** Creates a new checker with the given params. */
  explicit SceneGraphCollisionChecker(CollisionCheckerParams params);

 private:
  // To support Clone(), allow copying (but not move nor assign).
  explicit SceneGraphCollisionChecker(const SceneGraphCollisionChecker&);

  std::unique_ptr<CollisionChecker> DoClone() const final;

  void DoUpdateContextPositions(CollisionCheckerContext*) const final;

  bool DoCheckContextConfigCollisionFree(
      const CollisionCheckerContext& model_context) const final;

  std::optional<geometry::GeometryId> DoAddCollisionShapeToBody(
      const std::string& group_name, const multibody::Body<double>& bodyA,
      const geometry::Shape& shape,
      const math::RigidTransform<double>& X_AG) final;

  void DoRemoveAddedGeometries(
      const std::vector<CollisionChecker::AddedShape>& shapes) final;

  RobotClearance DoCalcContextRobotClearance(
      const CollisionCheckerContext& model_context,
      double influence_distance) const final;

  std::vector<RobotCollisionType> DoClassifyContextBodyCollisions(
      const CollisionCheckerContext& model_context) const final;

  int DoMaxContextNumDistances(
      const CollisionCheckerContext& model_context) const final;
};

}  // namespace planning
}  // namespace drake
