#pragma once

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "drake/planning/collision_checker_params.h"
#include "planning/collision_checker.h"

namespace anzu {
namespace planning {

/** A concrete collision checker implementation that throws an exception for
every virtual function hook. This might be useful for unit testing or for
deriving your own collision checker without providing for the full suite of
operations. */
class UnimplementedCollisionChecker : public CollisionChecker {
 public:
  UnimplementedCollisionChecker(drake::planning::CollisionCheckerParams params,
                                bool supports_parallel_checking);

  /** @name Does not allow copy, move, or assignment. */
  /** @{ */
  // N.B. The copy constructor is protected for use in implementing Clone().
  UnimplementedCollisionChecker(UnimplementedCollisionChecker&&) = delete;
  UnimplementedCollisionChecker& operator=(
      const UnimplementedCollisionChecker&) = delete;
  UnimplementedCollisionChecker& operator=(UnimplementedCollisionChecker&&) =
      delete;
  /** @} */

  ~UnimplementedCollisionChecker();

 protected:
  // To support Clone(), allow copying (but not move nor assign).
  UnimplementedCollisionChecker(const UnimplementedCollisionChecker&);

  std::unique_ptr<CollisionChecker> DoClone() const override;

  void DoUpdateContextPositions(
      drake::planning::CollisionCheckerContext* model_context) const override;

  bool DoCheckContextConfigCollisionFree(
      const drake::planning::CollisionCheckerContext&) const override;

  std::optional<drake::geometry::GeometryId> DoAddCollisionShapeToBody(
      const std::string& group_name,
      const drake::multibody::Body<double>& bodyA,
      const drake::geometry::Shape& shape,
      const drake::math::RigidTransform<double>& X_AG) override;

  void DoRemoveAddedGeometries(
      const std::vector<CollisionChecker::AddedShape>& shapes) override;

  drake::planning::RobotClearance DoCalcContextRobotClearance(
      const drake::planning::CollisionCheckerContext&, double) const override;

  std::vector<drake::planning::RobotCollisionType>
  DoClassifyContextBodyCollisions(
      const drake::planning::CollisionCheckerContext&) const override;

  int DoMaxContextNumDistances(
      const drake::planning::CollisionCheckerContext&) const override;
};

}  // namespace planning
}  // namespace anzu
