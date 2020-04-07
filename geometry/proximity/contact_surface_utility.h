#pragma once

/** @file
 There are multiple ways to compute a contact surface depending on the geometry
 representations and compliance types involved. However, they should all produce
 ContactSurface instances that satisfy some basic invariants. These functions
 assist in maintaining those invariants.
 */

#include <vector>

#include "drake/geometry/proximity/surface_mesh.h"

namespace drake {
namespace geometry {
namespace internal {

/* Given a planar, N-sided convex `polygon`, computes its centroid. The
 `polygon` is represented as an ordered list of indices into the given set of
 `vertices_F`. The resulting centroid will be measured and expressed in the same
 Frame F as the provided vertices.

 In debug builds, this method will do _expensive_ validation of its parameters.

 @param polygon
     The planar N-sided convex polygon.
 @param[in] n_F
     A vector that is perpendicular to the `polygon`'s plane, expressed in
     Frame F.
 @param vertices_F
     The set of vertices from which the polygon is defined, each measured and
     expressed in Frame F.
 @retval p_FC, the position of the polygon's centroid C, measured and expressed
     in Frame F.
 @pre `polygon.size()` >= 3.
 @pre `n_F` is  perpendicular to the defined `polygon`'s plane.
 @pre `n_F` has non-trivial length.
 @pre `polygon` is planar.
 @tparam T  The computational scalar type. Only supports double and AutoDiffXd.
 */
template <typename T>
Vector3<T> CalcPolygonCentroid(
    const std::vector<SurfaceVertexIndex>& polygon,
    const Vector3<T>& n_F,
    const std::vector<SurfaceVertex<T>>& vertices_F);

// TODO(SeanCurtis-TRI): Consider creating an overload of this that *computes*
//  the normal and then invokes this one for contexts where they don't have the
//  normal convenient.

/* Adds the planar, N-sided convex `polygon` to the given set of `faces` and
 `vertices` as a set of N triangles. A new vertex is introduced at the
 `polygon`'s centroid and one triangle is added for each edge, formed by the
 edge and the centroid vertex.

 In debug builds, this method will do _expensive_ validation of its parameters.

 @param[in] polygon
     The input polygon is represented by three or more ordered indices into
     `vertices_F`. This polygon is _not_ in `faces` and will not, itself, appear
     in `faces` when done.
 @param[in] n_F
     The vector that is perpendicular to the `polygon`, expressed in frame F.
 @param[in, out] faces
     New triangles are added into `faces`. Each new triangle has the same
     orientation (same normal vector) as the input polygon.
 @param[in ,out] vertices_F
     The set of vertex positions to be extended, each vertex is measured and
     expressed in frame F. It is assumed that `polygon`'s indices all reference
     vertices in this vector. One vertex will be added -- the polygon's
     centroid.
 @pre `faces` and `vertices_F` are not `nullptr`.
 @pre `polygon.size()` >= 3.
 @pre Each index in `polygon` indexes a valid vertex in `vertices_F`.
 @pre `polygon` is planar.
 @pre `n_F` is perpendicular to the defined `polygon`.
 @pre `n_F` has non-trivial length.
 @tparam T  The computational scalar type. Only supports double and AutoDiffXd. 
 */
template <typename T>
void AddPolygonToMeshData(
    const std::vector<SurfaceVertexIndex>& polygon,
    const Vector3<T>& n_F,
    std::vector<SurfaceFace>* faces,
    std::vector<SurfaceVertex<T>>* vertices_F);


/* Determines if the indicated triangle has a face normal that is "in the
 direction" of the given normal.

 The definition of "in the direction" is within a hard-coded tolerance 5π/8,
 which was empirically determined. Note that there is no one value that always
 works (see documentation of IsFaceNormalAlongPressureGradient() in
 mesh_intersection.h for examples).

 @param normal_F    The normal to test against, expressed in Frame F.
 @param surface_M   The mesh from which the triangle is drawn, measured and
                    expressed in Frame M.
 @param tri_index   The index of the triangle in `surface_M` to test.
 @param R_FM        The relative orientation between the bases of M and F.
 @pre `normal_F` is unit length.
 @return `true` if the angle between `normal_F` and the triangle normal lies
          within the hard-coded tolerance.
 @tparam T  The computational scalar type. Only supports double and AutoDiffXd.
 */
template <typename T>
bool IsFaceNormalInNormalDirection(const Vector3<T>& normal_F,
                                   const SurfaceMesh<T>& surface_M,
                                   SurfaceFaceIndex tri_index,
                                   const math::RotationMatrix<T>& R_FM);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
