#pragma once

#include "drake/geometry/proximity/triangle_surface_mesh.h"
#include "drake/geometry/shape_specification.h"
#include "drake/multibody/tree/spatial_inertia.h"

namespace drake {
namespace multibody {

/** Computes the SpatialInertia of the given shape. The shape is assumed to have
 uniform density. If unspecified, the density is assumed to be one.

 The shape's center of mass is measured and expressed in the shape's frame S:
 p_SScm_S. The unit inertia is measured around the frame's origin and likewise
 expressed in S: G_SSo_S.

 For geometry::Shape types that are defined symmetrically around So (e.g.,
 geometry::Box, geometry::Sphere, etc.), the shape's center of mass is
 coincident with S's origin. However, this is not guaranteed to be true for all
 Shapes. This has particular significance for meshes. For more discussion on the
 nuances of geometry::Mesh and geometry::Convex calculations
 @ref CalcSpatialInertia(const geometry::TriangleSurfaceMesh<double>&,double)
 "see below".

 @throws std::exception if `shape` is an instance of geometry::HalfSpace or
                        geometry::MeshcatCone.
 @pydrake_mkdoc_identifier{shape} */
SpatialInertia<double> CalcSpatialInertia(const geometry::Shape& shape,
                                          double density = 1.0);

/** Computes the SpatialInertia of the given mesh.

 The shape is assumed to have uniform density. If unspecified, the density is
 assumed to be one.

 Remember that the inertia is measured around the origin of the mesh's frame. If
 we create two mesh files that are identical except that the vertex positions
 have been rigidly displaced, this function will report different SpatialInertia
 values for the two meshes; the poses of the *masses* of the meshes are
 different relative to the meshes' frames.

 For the mass properties to be meaningful, the mesh must satisfy certain
 requirements:

   - The mesh must *fully* enclose a volume (no cracks, no open manifolds,
     etc.)
   - All triangles must be "wound" such that their normals point outward
     (according to the right-hand rule based on vertex winding).

 If these requirements are not met, a value *will* be returned, but its value
 is meaningless.

 A final note on the geometry::Convex type. The geometry referenced by the
 convex specification may not actually be convex. In collision checking, the
 true convexity of the mesh is ignored and its convex hull is used. For this
 function, the *actual* mesh is used.
 @pydrake_mkdoc_identifier{mesh} */
SpatialInertia<double> CalcSpatialInertia(
    const geometry::TriangleSurfaceMesh<double>& mesh, double density = 1.0);

// TODO(SeanCurtis-TRI): Add CalcSpatialinertia(VolumeMesh).

}  // namespace multibody
}  // namespace drake
