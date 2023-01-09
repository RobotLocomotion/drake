#include "planning/unimplemented_collision_checker.h"

#include <stdexcept>
#include <utility>

#include <fmt/format.h>

namespace anzu {
namespace planning {

using drake::planning::CollisionCheckerContext;
using drake::planning::CollisionCheckerParams;
using drake::planning::RobotCollisionType;

namespace {

[[noreturn]] void ThrowNotImplemented(const char* func) {
  throw std::runtime_error(fmt::format("{} is not implemented", func));
}

}  // namespace

using drake::planning::RobotClearance;

UnimplementedCollisionChecker::UnimplementedCollisionChecker(
    CollisionCheckerParams params, bool supports_parallel_checking)
    : CollisionChecker(std::move(params), supports_parallel_checking) {}

UnimplementedCollisionChecker::UnimplementedCollisionChecker(
    const UnimplementedCollisionChecker&) = default;

UnimplementedCollisionChecker::~UnimplementedCollisionChecker() {}

std::unique_ptr<CollisionChecker> UnimplementedCollisionChecker::DoClone()
    const {
  ThrowNotImplemented(__func__);
}

void UnimplementedCollisionChecker::DoUpdateContextPositions(
    CollisionCheckerContext*) const {
  ThrowNotImplemented(__func__);
}

bool UnimplementedCollisionChecker::DoCheckContextConfigCollisionFree(
    const CollisionCheckerContext&) const {
  ThrowNotImplemented(__func__);
}

std::optional<drake::geometry::GeometryId>
UnimplementedCollisionChecker::DoAddCollisionShapeToBody(
    const std::string&, const drake::multibody::Body<double>&,
    const drake::geometry::Shape&, const drake::math::RigidTransform<double>&) {
  ThrowNotImplemented(__func__);
}

void UnimplementedCollisionChecker::DoRemoveAddedGeometries(
    const std::vector<CollisionChecker::AddedShape>&) {
  ThrowNotImplemented(__func__);
}

RobotClearance UnimplementedCollisionChecker::DoCalcContextRobotClearance(
    const CollisionCheckerContext&, double) const {
  ThrowNotImplemented(__func__);
}

std::vector<RobotCollisionType>
UnimplementedCollisionChecker::DoClassifyContextBodyCollisions(
    const CollisionCheckerContext&) const {
  ThrowNotImplemented(__func__);
}

int UnimplementedCollisionChecker::DoMaxContextNumDistances(
    const CollisionCheckerContext&) const {
  ThrowNotImplemented(__func__);
}

}  // namespace planning
}  // namespace anzu
