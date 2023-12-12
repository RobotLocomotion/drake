#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "drake/planning/collision_checker.h"
#include "drake/planning/collision_checker_params.h"

namespace drake {
namespace planning {

/** A concrete collision checker implementation that throws an exception for
every virtual function hook. This might be useful for unit testing or for
deriving your own collision checker without providing for the full suite of
operations. */
class UnimplementedCollisionChecker : public CollisionChecker {
 public:
  /** Constructs a checker.
  @param supports_parallel_checking will serve as the return value of the
  CollisionChecker::SupportsParallelChecking() function. */
  UnimplementedCollisionChecker(CollisionCheckerParams params,
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
      CollisionCheckerContext* model_context) const override;

  bool DoCheckContextConfigCollisionFree(
      const CollisionCheckerContext&) const override;

  std::optional<geometry::GeometryId> DoAddCollisionShapeToBody(
      const std::string& group_name, const multibody::RigidBody<double>& bodyA,
      const geometry::Shape& shape,
      const math::RigidTransform<double>& X_AG) override;

  void RemoveAddedGeometries(
      const std::vector<CollisionChecker::AddedShape>& shapes) override;

  void UpdateCollisionFilters() override;

  RobotClearance DoCalcContextRobotClearance(const CollisionCheckerContext&,
                                             double) const override;

  std::vector<RobotCollisionType> DoClassifyContextBodyCollisions(
      const CollisionCheckerContext&) const override;

  int DoMaxContextNumDistances(const CollisionCheckerContext&) const override;
};

}  // namespace planning
}  // namespace drake
