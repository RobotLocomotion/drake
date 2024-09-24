#pragma once

#include <filesystem>

#include "drake/geometry/proximity/polygon_surface_mesh.h"

namespace drake {
namespace geometry {
namespace internal {

/* Creates a polygonal mesh representing the convex hull of the vertices
 contained in the named `mesh_file` (scaled with the given `scale` value). The
 mesh is also "inflated" a given margin amount δ. If margin is zero, no
 inflation is applied and the convex hull of the original set of vertices is
 computed.

 With "inflation", we mean the process of moving each of the faces in the convex
 hull an amount δ along the (outwards) normal. This effectively increases or
 "inflates" the convex hull, producing an additional layer of thickness δ all
 around the convex hull of the original set of vertices.

 @note Geometry inflation by a margin δ is meant to support speculative
 constraints (contact constraints before contact actually happens) for
 "compliant" hydroelastic. Since the hydroelastic model is not meant for thin
 objects, inflation of planar (zero thickness) meshes is not implemented. Margin
 is ignored for planar meshes.

 @param mesh_file   A path to a valid mesh file to bound.
 @param scale       All vertices will be multiplied by this value prior to
                    computation.
 @param margin      The margin amount δ, see @ref hydro_margin.

 @throws if `mesh_file` references anything but an .obj, .vtk volume mesh, or
         .gltf.
 @throws if the referenced mesh data is degenerate (insufficient number of
            vertices, co-linear or coincident vertices, etc.) All of the
            vertices lying on a plane is *not* degenerate.
 @throws if there is an unforeseen error in computing the convex hull.
 @throws if `scale` is negative or zero.
 @throws if `margin` is negative. */
PolygonSurfaceMesh<double> MakeConvexHull(
    const std::filesystem::path& mesh_file, double scale, double margin = 0);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
