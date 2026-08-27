#pragma once

#include <string>

#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/optimization/vpolytope.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace planning {
namespace continuous_collision {

/** Registers a V-polytope as an anchored obstacle with a collision role.

The polytope is converted to `drake::geometry::Convex` through Drake's own
`VPolytope::ToShapeConvex()` entry point, a thin wrapper over the
`Convex(Eigen::Matrix3X<double> points, std::string label, double scale)`
constructor, then registered on the plant's world body. The
result therefore rides the ordinary native narrowphase path end to end: the
proximity engine and the certifier's radius/support code all read the same
`Convex::GetConvexHull()` object, so the certificate stays sound even for
redundant or degenerate vertex sets.

@param plant  The plant to register on. Must be non-null, must already be a
              registered SceneGraph source, and must NOT be finalized.
@param vpoly  The polytope. Its vertices are interpreted in the geometry
              frame G, i.e. the world-frame obstacle is
              `X_WG * conv(vpoly.vertices())`. Must be 3-dimensional with at
              least one vertex.
@param X_WG   Pose of the geometry frame in the world frame.
@param name   Geometry name; also used as the `Convex` shape's label (which
              Drake only uses in its own warning/error messages). Must not
              contain a newline.
@returns The id of the newly registered collision geometry.
@throws std::exception if `plant` is null or already finalized, if
        `vpoly.ambient_dimension() != 3`, if the vertex set is empty, or if
        Drake rejects the resulting hull (e.g. a degenerate vertex set that
        its hull computation cannot inflate).
@ingroup planning_collision_checker */
geometry::GeometryId AddVPolytopeObstacle(
    multibody::MultibodyPlant<double>* plant,
    const geometry::optimization::VPolytope& vpoly,
    const math::RigidTransform<double>& X_WG, const std::string& name);

}  // namespace continuous_collision
}  // namespace planning
}  // namespace drake
