#pragma once

#include <memory>

#include "drake/geometry/proximity/volume_mesh.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {
namespace internal {

/* Returns a volume mesh suitable for deformable body simulation that
 discretizes the shape with the resolution provided at construction. There is a
 seperate function for each concrete shape derived from Shape.
 @pre resolution_hint > 0.
 @throws std::exception if `shape` doesn't support a mesh discretization. */
VolumeMesh<double> MakeDeformableMesh(const Sphere& sphere,
                                      double resolution_hint);
VolumeMesh<double> MakeDeformableMesh(const Cylinder& cylinder,
                                      double resolution_hint);
VolumeMesh<double> MakeDeformableMesh(const HalfSpace& half_space,
                                      double resolution_hint);
VolumeMesh<double> MakeDeformableMesh(const Box& box, double resolution_hint);
VolumeMesh<double> MakeDeformableMesh(const Capsule& capsule,
                                      double resolution_hint);
VolumeMesh<double> MakeDeformableMesh(const Ellipsoid& ellipsoid,
                                      double resolution_hint);
VolumeMesh<double> MakeDeformableMesh(const Mesh& mesh, double resolution_hint);
VolumeMesh<double> MakeDeformableMesh(const Convex& convex,
                                      double resolution_hint);
VolumeMesh<double> MakeDeformableMesh(const MeshcatCone& cone,
                                      double resolution_hint);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
