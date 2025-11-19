#pragma once

#include <functional>
#include <string>
#include <variant>

#include "drake/common/drake_deprecated.h"
#include "drake/geometry/proximity/triangle_surface_mesh.h"
#include "drake/geometry/shape_specification.h"
#include "drake/multibody/tree/spatial_inertia.h"

namespace drake {
namespace multibody {
namespace internal {

/* Versions of CalcSpatialInertia() that do not throw. If an error occurs, the
 error message is returned instead of a SpatialInertia. */
using CalcSpatialInertiaResult =
    std::variant<SpatialInertia<double>, std::string>;
CalcSpatialInertiaResult CalcSpatialInertiaImpl(
    const geometry::TriangleSurfaceMesh<double>& mesh, double density);
CalcSpatialInertiaResult CalcSpatialInertiaImpl(const geometry::Shape& shape,
                                                double density);

/* Attempts to compute a SpatialInertia related to a given `mesh`. It assumes
the mesh is uniformly filled with a material with the given `density`.

The correctness of the inertia depends on the mesh quality (i.e., a closed
manifold). A malformed mesh may still produce a spatial inertia that _appears_
physically plausible. That physically-plausible inertia is returned, regardless
of how incorrect it may actually be.

If the mesh is sufficiently malformed, such that its spatial inertia isn't even
physically plausible, the mesh's convex hull will be used instead. Furthermore,
if the `warn_for_convex` function is defined, a message will be passed to that
function.

In the event that the mesh is so malformed that it doesn't have a valid convex
hull, an error message is returned instead of a spatial inertia. */
CalcSpatialInertiaResult CalcSpatialInertiaWithFallback(
    const geometry::Mesh& mesh, double density,
    std::function<void(const std::string&)> warn_for_convex = nullptr);

}  // namespace internal

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
     etc.).
   - All triangles must be "wound" such that their normals point outward
     (according to the right-hand rule based on vertex winding).

 Drake currently doesn't validate these requirements on the mesh. Instead, it
 does a best-faith effort to compute a spatial inertia. For some "bad" meshes,
 the SpatialInertia will be objectively physically invalid. For others, the
 SpatialInertia will appear physically valid, but be meaningless because it does
 not accurately represent the mesh.

 @throws std::exception if the resulting spatial inertia is obviously physically
 invalid. See multibody::SpatialInertia::IsPhysicallyValid().
 @pydrake_mkdoc_identifier{mesh} */
SpatialInertia<double> CalcSpatialInertia(
    const geometry::TriangleSurfaceMesh<double>& mesh, double density);

// TODO(SeanCurtis-TRI): Add CalcSpatialinertia(VolumeMesh).

}  // namespace multibody
}  // namespace drake
