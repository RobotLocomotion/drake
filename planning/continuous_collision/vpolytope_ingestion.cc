#include "drake/planning/continuous_collision/vpolytope_ingestion.h"

#include <stdexcept>

#include <fmt/format.h>

#include "drake/common/drake_throw.h"
#include "drake/geometry/shape_specification.h"
#include "drake/multibody/plant/coulomb_friction.h"

namespace drake {
namespace planning {
namespace continuous_collision {

using drake::geometry::GeometryId;
using drake::math::RigidTransformd;
using drake::multibody::CoulombFriction;
using drake::multibody::MultibodyPlant;

namespace {
// Signed distance never reads friction, but RegisterCollisionGeometry demands
// some proximity properties. Unit friction is Drake's own conventional
// placeholder for "the caller does not care".
constexpr double kDefaultFriction = 1.0;
}  // namespace

GeometryId AddVPolytopeObstacle(MultibodyPlant<double>* plant,
                                const geometry::optimization::VPolytope& vpoly,
                                const RigidTransformd& X_WG,
                                const std::string& name) {
  DRAKE_THROW_UNLESS(plant != nullptr);
  // A non-3D V-polytope is refused by VPolytope::ToShapeConvex() below, and a
  // finalized plant by MultibodyPlant::RegisterCollisionGeometry(); neither
  // needs a check here. An empty vertex set reaches the proximity engine
  // undetected, so it does.
  if (vpoly.vertices().cols() == 0) {
    throw std::runtime_error(fmt::format(
        "AddVPolytopeObstacle(): obstacle '{}' has an empty vertex set.",
        name));
  }

  // Drake's pinned VPolytope -> Convex entry point; it forwards the vertex
  // matrix to Convex(Matrix3X, label, scale=1). The hull is computed lazily
  // by Convex::GetConvexHull() and is the object the proximity engine
  // actually collides.
  const drake::geometry::Convex convex = vpoly.ToShapeConvex(name);

  return plant->RegisterCollisionGeometry(
      plant->world_body(), X_WG, convex, name,
      CoulombFriction<double>(kDefaultFriction, kDefaultFriction));
}

}  // namespace continuous_collision
}  // namespace planning
}  // namespace drake
