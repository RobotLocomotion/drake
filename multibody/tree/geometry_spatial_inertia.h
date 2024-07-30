#pragma once

#include "drake/common/drake_deprecated.h"
#include "drake/geometry/proximity/triangle_surface_mesh.h"
#include "drake/geometry/shape_specification.h"
#include "drake/multibody/tree/spatial_inertia.h"

namespace drake {
namespace multibody {

/** Computes the SpatialInertia of a body made up of a homogeneous material
 (of given `density` in kg/m³) uniformly distributed in the volume of the given
 `shape`.
 
 The `shape` is defined in its canonical frame S and the body in frame B. The
 two frames are coincident and aligned (i.e., X_SB = I).

 Most shapes are defined such that their center of mass is coincident with So
 (and, therefore, Bo). These are the shapes that have symmetry across So along
 each of the axes Sx, Sy, Sz (e.g., geometry::Box, geometry::Sphere, etc.) For
 meshes, it depends on how the mesh is defined. For more discussion on the
 nuances of geometry::Mesh and geometry::Convex calculations
 @ref CalcSpatialInertia(const geometry::TriangleSurfaceMesh<double>&,double)
 "see below".

 @retval M_BBo_B The spatial inertia of the hypothetical body implied by the
                 given `shape`.
 @throws std::exception if `shape` is an instance of geometry::HalfSpace or
                        geometry::MeshcatCone.
 @pydrake_mkdoc_identifier{shape} */
SpatialInertia<double> CalcSpatialInertia(const geometry::Shape& shape,
                                          double density);

/** Computes the SpatialInertia of a body made up of a homogeneous material
 (of given `density` in kg/m³) uniformly distributed in the volume of the given
 `mesh`.
 
 The `mesh` is defined in its canonical frame M and the body in frame B. The two
 frames are coincident and aligned (i.e., X_MB = I).

 For the resultant spatial inertia to be meaningful, the `mesh` must satisfy
 certain requirements:

   - The mesh must *fully* enclose a volume (no cracks, no open manifolds,
     etc.)
   - All triangles must be "wound" such that their normals point outward
     (according to the right-hand rule based on vertex winding).

 If these requirements are not met, a value *will* be returned, but its value
 is meaningless.
 @pydrake_mkdoc_identifier{mesh} */
SpatialInertia<double> CalcSpatialInertia(
    const geometry::TriangleSurfaceMesh<double>& mesh, double density);

DRAKE_DEPRECATED(
    "2024-11-01",
    "In the function CalcSpatialInertia(), the density argument's default "
    "value of 1.0 was removed. Provide a sensible density value.")
SpatialInertia<double> CalcSpatialInertia(
    const geometry::TriangleSurfaceMesh<double>& mesh);

// TODO(SeanCurtis-TRI): Add CalcSpatialinertia(VolumeMesh).

}  // namespace multibody
}  // namespace drake
