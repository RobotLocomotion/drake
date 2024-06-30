#pragma once

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

 Note: Spatial inertia calculations for the geometry::Convex type do not
 currently require that the underlying mesh actually be convex. Although certain
 collision calculations involving geometry::Convex may use the mesh's convex
 hull, this function uses the *actual* mesh.

 @retval M_BBo_B The spatial inertia of the hypothetical body implied by the
                 given `shape`.
 @throws std::exception if `shape` is an instance of geometry::HalfSpace or
                        geometry::MeshcatCone.
 @throws std::exception if the resulting spatial inertia computation does not
                        result in a physically meaningful value. See
                        SpatialInertia::IsPhysicallyValid() for more
                        information.
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
    const geometry::TriangleSurfaceMesh<double>& mesh, double density = 1.0);

// TODO(SeanCurtis-TRI): Add CalcSpatialinertia(VolumeMesh).

}  // namespace multibody
}  // namespace drake
