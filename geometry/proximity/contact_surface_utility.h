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
     The planar N-sided convex polygon defined by three or more ordered indices
     into `vertices_F`.
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
 @tparam_nonsymbolic_scalar
 */
template <typename T>
Vector3<T> CalcPolygonCentroid(const std::vector<int>& polygon,
                               const Vector3<T>& n_F,
                               const std::vector<Vector3<T>>& vertices_F);

// TODO(14579) This overload of CalcPolygonCentroid() is expected to simply go
//  away when we implement the final support for discrete hydroelastics. If it
//  persists (for whatever reason), we'll need to resolve the design between
//  the overloads to eliminate wasteful work.

/* Overload that takes a polygon represented as an ordered list of positional
 vectors `p_FVs` of its vertices, each measured and expressed in frame F.
 @tparam_nonsymbolic_scalar */
template <typename T>
Vector3<T> CalcPolygonCentroid(
    const std::vector<Vector3<T>>& p_FVs,
    const Vector3<T>& n_F);

// TODO(14579) CalcPolygonArea() is expected to simply go away when we
//  implement the final support for discrete hydroelastics. If it
//  persists (for whatever reason), we'll need to consider an alternative
//  design for CalcPolygonArea() to be a part of CalcPolygonCentroid() to
//  reduce code duplication.

/* Calculates area of a planar convex polygon represented as an ordered list
 of positional vectors `p_FVs` of its vertices, each measured and expressed
 in frame F.

 @param[in] p_FVs   Positions of vertices of the polygon.
 @param[in] nhat_F  Unit normal vector of the polygon.
 @pre `p_FVs.size()` >= 3.
 @pre nhat_F is consistent with the winding of the polygon.
 @tparam_nonsymbolic_scalar */
template <typename T>
T CalcPolygonArea(const std::vector<Vector3<T>>& p_FVs,
                  const Vector3<T>& nhat_F);

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
 @tparam_nonsymbolic_scalar */
template <typename T>
void AddPolygonToMeshData(const std::vector<int>& polygon,
                          const Vector3<T>& n_F,
                          std::vector<SurfaceFace>* faces,
                          std::vector<Vector3<T>>* vertices_F);

// Any polygon with area less than this threshold is considered having
// near-zero area in AddPolygonToMeshDataAsOneTriangle() below.
constexpr double kMinimumPolygonArea = 1e-13;

// TODO(14579) The following AddPolygonToMeshDataAsOneTriangle() is expected to
//  simply go away when we implement the final support for discrete
//  hydroelastics. If it persists (for whatever reason), find a better way to
//  deal with a polygon with zero or near-zero area. Perhaps we should
//  allow users to specify criteria to classify such polygons instead of
//  relying on the threshold kMinimumPolygonArea above.

/* Adds a representative triangle of a convex polygon to the given set of
 `faces` and `vertices`. The triangle has the same centroid, area, and normal
 vector as the polygon. The three vertices of the representative triangle are
 introduced into `vertices`.

 The exact choice of the representative triangle is arbitrary subject to the
 constraints in the previous paragraph. If the polygon is already a triangle,
 we will add that original triangle in the output.

 In debug builds, this function will do _expensive_ validation of its
 parameters.

 @param[in] polygon_F
     The input polygon is represented by position vectors of its vertices,
     measured and expressed in frame F. This polygon is _not_ in `faces` or
     `vertices`.
 @param[in] nhat_F
     The unit normal vector to the polygon, expressed in frame F.
 @param[out] faces
     The new triangle is added into `faces` with the same orientation (same
     normal vector) as the input polygon.
 @param[out] vertices_F
     The set of vertex positions to be extended; each vertex is measured and
     expressed in frame F. Three vertices will be added.

 @pre `faces` and `vertices_F` are not `nullptr`.
 @pre The polygon is simple (does not intersect itself and has no holes).
 @pre The winding of `polygon_F` is consistent with the direction of `nhat_F`.
      They must respect the right-hand rule.

 @note There are two reasons for skipping a polygon with zero or near-zero area.
       1. Such a polygon contributes negligibly to contact force and moment.
       2. For the scalar type AutoDiffXd, such a polygon may cause unstable
          calculation of derivatives of the representative triangle.

 @tparam_nonsymbolic_scalar */
template <typename T>
void AddPolygonToMeshDataAsOneTriangle(
    const std::vector<Vector3<T>>& polygon_F, const Vector3<T>& nhat_F,
    std::vector<SurfaceFace>* faces, std::vector<Vector3<T>>* vertices_F);

enum class ContactPolygonRepresentation {
  // Each contact polygon is subdivided into triangles sharing the centroid
  // of the polygon.
  kCentroidSubdivision,

  // Use one representative triangle for each contact polygon.
  kSingleTriangle
};

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
 @tparam_nonsymbolic_scalar */
template <typename T>
bool IsFaceNormalInNormalDirection(const Vector3<T>& normal_F,
                                   const SurfaceMesh<T>& surface_M,
                                   int tri_index,
                                   const math::RotationMatrix<T>& R_FM);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
