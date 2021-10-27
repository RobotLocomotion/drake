#include "drake/multibody/fixed_fem/dev/deformable_contact.h"

#include <array>
#include <utility>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/geometry/proximity/posed_half_space.h"
#include "drake/geometry/utilities.h"

namespace drake {
namespace multibody {
namespace fem {

using geometry::TriangleSurfaceMesh;
using geometry::VolumeMesh;
using geometry::internal::Aabb;
using geometry::internal::Bvh;
using geometry::internal::BvttCallbackResult;
using geometry::internal::convert_to_double;
using geometry::internal::DeformableVolumeMesh;
using geometry::internal::Obb;
using geometry::internal::PosedHalfSpace;
using std::array;
using std::move;
using std::vector;

namespace {
/* %Intersector performs a mesh-intersection algorithm between a triangulated
 surface mesh and a tetrahedral volume mesh. Calculates the mesh and the
 per-polygon quantities defined in ContactPolygonData.

 This is a bunch of modified copypasta from mesh_intersection.{h|cc}.  */
template <typename T>
class Intersector {
 public:
  Intersector() {
    // We know that each contact polygon has at most 7 vertices.
    // Each surface triangle is clipped by four half-spaces of the four
    // triangular faces of a tetrahedron.
    polygon_[0].reserve(7);
    polygon_[1].reserve(7);
  }

  /* Intersects a volume mesh (union of tetrahedra) with a surface mesh (union
   of triangles).

   @param[in] tet_mesh_D   The deformable tet mesh with vertices measured and
                           expressed in the deformable frame D.
   @param[in] surface_R    The surface mesh, whose vertices are measured and
                           expressed in the rigid frame R.
   @param[in] bvh_R        The bounding volume hierarchy for the surface mesh,
                           measured and expressed in frame R.
   @param[in] X_DR  The pose of frame R relative to the world frame D.
   @returns The (possibly empty) contact surface representing the intersection
            between the two meshes.  */
  DeformableContactSurface<T> Intersect(
      const DeformableVolumeMesh<T>& tet_mesh_D,
      const TriangleSurfaceMesh<double>& surface_R,
      const Bvh<Obb, TriangleSurfaceMesh<double>>& bvh_R,
      const math::RigidTransform<T>& X_DR) {
    // The collection of contact data. We will aggregate into this.
    vector<ContactPolygonData<T>> out_poly_data;

    const math::RigidTransformd& X_DR_d = convert_to_double(X_DR);
    for (const auto [tet_index, tri_index] :
         tet_mesh_D.bvh().GetCollisionCandidates(bvh_R, X_DR_d)) {
      const vector<IntersectionVertex<T>>& poly_vertices_D =
          ClipTriangleByTetrahedron(tet_index, tet_mesh_D.mesh(), tri_index,
                                    surface_R, X_DR);
      const int poly_vertex_count = static_cast<int>(poly_vertices_D.size());
      if (poly_vertex_count < 3) continue;

      const Vector3<T>& nhat_D =
          X_DR.rotation() * surface_R.face_normal(tri_index).template cast<T>();

      // TODO(SeanCurtis-TRI): The cost of re-creating these lambdas per element
      //  pair is ridiculous. Move it *outside* the for loop and simply pass
      //  in the polygon normal.

      // Computes the double area of the triangle spanned by the three
      // vertices. This could be more robust by smart selection of the two
      // triangle edges.
      auto calc_double_area =
          [&nhat_D](int v0, int v1, int v2,
                    const vector<IntersectionVertex<T>>& vertices_D) {
            const Vector3<T> p_01_D =
                vertices_D[v1].cartesian - vertices_D[v0].cartesian;
            const Vector3<T> p_02_D =
                vertices_D[v2].cartesian - vertices_D[v0].cartesian;
            return p_01_D.cross(p_02_D).dot(nhat_D);
          };

      // Computes the *scaled* centroid of the triangle spanned by the three
      // vertices in both Cartesian and Barycentric coordinates. The *true*
      // centroid would be found by dividing each quantity by three.
      auto calc_scaled_centroid =
          [](int v0, int v1, int v2,
             const vector<IntersectionVertex<T>>& vertices_D)
          -> IntersectionVertex<T> {
        Vector3<T> centroid_D =
            (vertices_D[v0].cartesian + vertices_D[v1].cartesian +
             vertices_D[v2].cartesian);
        Vector4<T> b_centroid =
            (vertices_D[v0].bary + vertices_D[v1].bary + vertices_D[v2].bary);
        return {centroid_D, b_centroid};
      };

      // We construct a triangle fan from the polygon with vertex 0 as the
      // common vertex: e.g., triangles (0, 1, 2), (0, 2, 3), ... Generally,
      // a triangle from a fan with N vertices is (0, i, i + 1), for
      // i ∈ [1, N - 2]. For each triangle, compute its doubled area and
      // scaled centroid.
      T poly_double_area{0};  // We accumulate into this variable to find 2x
                              // the area of the contact polygon.
      // We accumulate into this variable the sum of
      // 6 x the area of the triangle x the centroid of the triangle
      // over all triangle fans in the polygon.
      IntersectionVertex<T> scaled_centroid{Vector3<T>::Zero(),
                                            Vector4<T>::Zero()};
      for (int i = 1; i < poly_vertex_count - 1; ++i) {
        // 2 x the area of the triangle.
        const T double_area = calc_double_area(0, i, i + 1, poly_vertices_D);
        poly_double_area += double_area;
        // 3 x the centroid of the centroid.
        const IntersectionVertex<T> tri_centroid =
            calc_scaled_centroid(0, i, i + 1, poly_vertices_D);
        // Accumulate
        //   6 x the area of the triangle x the centroid of the triangle.
        scaled_centroid.bary += double_area * tri_centroid.bary;
        scaled_centroid.cartesian += double_area * tri_centroid.cartesian;
      }
      out_poly_data.push_back(
          {poly_double_area / 2, nhat_D,
           scaled_centroid.cartesian / (poly_double_area * 3),
           scaled_centroid.bary / (poly_double_area * 3), tet_index});
    }
    return DeformableContactSurface<T>(move(out_poly_data));
  }

 private:
  /* The vertex of the polygon formed by intersecting a tet with a tri. The
   vertex has a dual representation: as a point in cartesian coordinates
   (measured and expressed in the frame indicated by notation) and the
   barycentric coordinates of the tet.  */
  template <typename U = T>
  struct IntersectionVertex {
    Vector3<U> cartesian;
    Vector4<U> bary;
  };

  /* Calculates the intersection point between an infinite straight line
   spanning points A and B and the bounding plane of the half space H, all
   measured and expressed in the common frame D.

   @param p_DA  The intersection point A.
   @param p_DB  The intersection point B.
   @param H_D   The half space H measured and expressed in frame D.
   @pre         One of A and B is outside the half space, and the other is
                inside or on the boundary of the half space.  */
  static IntersectionVertex<T> CalcIntersection(
      const IntersectionVertex<T>& p_DA, const IntersectionVertex<T>& p_DB,
      const PosedHalfSpace<T>& H_D) {
    const T a = H_D.CalcSignedDistance(p_DA.cartesian);
    const T b = H_D.CalcSignedDistance(p_DB.cartesian);
    // We require that A and B classify in opposite directions (one inside and
    // one outside). Outside has a strictly positive distance, inside is
    // non-positive. We confirm that their product is non-positive and that at
    // least one of the values is positive -- they can't both be zero. This
    // prevents b - a becoming zero and the corresponding division by zero.
    DRAKE_ASSERT(a * b <= 0 && (a > 0 || b > 0));
    const T wa = b / (b - a);
    const T wb = T(1.0) - wa;  // Enforce a + b = 1.
    // Compute the intersection point I in Cartesian and Barycentric coords.
    const Vector3<T> p_DI = wa * p_DA.cartesian + wb * p_DB.cartesian;
    const Vector4<T> b_I = wa * p_DA.bary + wb * p_DB.bary;
    // Empirically we found that numeric_limits<double>::epsilon() 2.2e-16 is
    // too small.
    const T kEps(1e-14);
    // TODO(SeanCurtis-TRI): Consider refactoring this fuzzy test *into*
    //  PosedHalfSpace if it turns out we need to perform this test at other
    //  sites.
    // Verify that the intersection point is on the plane of the half space.
    using std::abs;
    DRAKE_DEMAND(abs(H_D.CalcSignedDistance(p_DI)) < kEps);
    return {p_DI, b_I};
    // Justification.
    // 1. We set up the weights wa and wb such that wa + wb = 1, which
    //    guarantees that the linear combination is on the straight line
    //    through A and B.
    // 2. We show that the H_D.signed_distance(wa * A + wb * B) is zero.
    //    Let H_D.signed_distance be sdf(P) = N.dot(P) + d.
    //      sdf(wa * A + wb * B)
    //      = N.dot(wa * A + wb * B) + d
    //      = wa * N.dot(A) + wb * N.dot(B) + d
    //      = b * N.dot(A)/(b - a) + a * N.dot(B)/(a - b) + d
    //      = b * N.dot(A)/(b - a) - a * N.dot(B)/(b - a) + d
    //      = (b * N.dot(A) - a * N.dot(B) + (b - a) * d) / (b - a)
    //      = (b * (N.dot(A) + d) - a * (N.dot(B) + d)) / (b-a)
    //      = (b * sdf(A) - a * sdf(B)) / (b-a)
    //      = (b * a - a * b) / (b-a)
    //      = 0 when a != b.
  }

  // TODO(SeanCurtis-TRI): This function duplicates functionality implemented in
  //  mesh_half_space_intersection.h. Reconcile the two implementations.
  // TODO(DamrongGuoy): Avoid duplicate vertices mentioned in the note below and
  //  check whether we can have other as yet undocumented degenerate cases.
  /* Intersects a polygon with the half space H. It keeps the part of
   the polygon contained in the half space (signed distance is <= 0).
   The half space `H_F` and vertex positions of `input_vertices_F` are both
   defined in a common frame F.
   @param[in] input_vertices_F
       Input polygon is represented as a sequence of positions of its vertices.
       The input polygon is allowed to have zero area.
   @param[in] H_F
       The clipping half space H in frame F.
   @param[out] output_vertices_F
       Output polygon is represented as a sequence of positions of its vertices.
       It could be an empty sequence if the input polygon is entirely outside
       the half space. It could be the same as the input polygon if the input
       polygon is entirely inside the half space. The output polygon is
       guaranteed to be planar (within floating point tolerance) and, if the
       polygon has area, the normal implied by the winding will be the same
       as the input polygon.
   @pre `input_vertices_F` has at least three vertices.
   @pre the vertices in `input_vertices_F` are all planar.
   @note
       1. For an input polygon P that is parallel to the plane of the half
          space, there are three cases:
          1.1 If P is completely inside the half space, the output polygon
              will be the same as P.
          1.2 If P is completely outside the half space, the output polygon
              will be empty.
          1.3 If P is on the plane of the half space, the output polygon will
              be the same as P.
       2. For an input polygon P outside the half space with one edge on the
          plane of the half space, the output polygon will be a zero-area
          4-gon with two pairs of duplicate vertices.
       3. For an input polygon P outside the half space with one vertex on the
          plane of the half space, the output polygon will be a zero-area
          triangle with three duplicate vertices.
  */
  static void ClipPolygonByHalfSpace(
      const std::vector<IntersectionVertex<T>>& input_vertices_F,
      const PosedHalfSpace<T>& H_F,
      std::vector<IntersectionVertex<T>>* output_vertices_F) {
    DRAKE_ASSERT(output_vertices_F != nullptr);
    // Note: this is the inner loop of a modified Sutherland-Hodgman algorithm
    // for clipping a polygon.
    output_vertices_F->clear();
    // Note: This code is correct for size < 3, but pointless so we make no
    // effort to support it or test it.
    const int size = static_cast<int>(input_vertices_F.size());

    // TODO(SeanCurtis-TRI): If necessary, this can be made more efficient:
    //  eliminating the modulus and eliminating the redundant "inside"
    //  calculation on previous (by pre-determining previous and its
    //  "containedness" and then propagating current -> previous in each loop.
    //  Probably a desirable optimization as we need to make all of this work as
    //  cheap as possible.
    int p = size - 1;
    for (int i = 0; i < size; ++i) {
      const IntersectionVertex<T>& current_F = input_vertices_F[i];
      const IntersectionVertex<T>& previous_F = input_vertices_F[p];
      const bool current_contained =
          H_F.CalcSignedDistance(current_F.cartesian) <= 0;
      const bool previous_contained =
          H_F.CalcSignedDistance(previous_F.cartesian) <= 0;
      if (current_contained) {
        if (!previous_contained) {
          // Current is inside and previous is outside. Compute the point where
          // that edge enters the half space. This is a new vertex in the
          // clipped polygon and must be included before current.
          output_vertices_F->push_back(
              CalcIntersection(current_F, previous_F, H_F));
        }
        output_vertices_F->push_back(current_F);
      } else if (previous_contained) {
        // Current is outside and previous is inside. Compute the point where
        // the edge exits the half space. This is a new vertex in the clipped
        // polygon and is included *instead* of current.
        output_vertices_F->push_back(
            CalcIntersection(current_F, previous_F, H_F));
      }
      p = i;
    }
  }

  /* Remove duplicate vertices from a polygon represented as a cyclical
   sequence of vertex positions. In other words, for a sequence `A,B,B,C,A`, the
   pair of B's is reduced to one B and the first and last A vertices are
   considered duplicates and the result would be `A,B,C`. The polygon might be
   reduced to a pair of points (i.e., `A,A,B,B` becomes `A,B`) or a single point
   (`A,A,A` becomes `A`).
   @param[in,out] polygon
       The input polygon, and the output equivalent polygon with no duplicate
       vertices.
   */
  static void RemoveDuplicateVertices(
      std::vector<IntersectionVertex<T>>* polygon) {
    DRAKE_ASSERT(polygon != nullptr);

    // TODO(SeanCurtis-TRI): The resulting polygon depends on the order of the
    //  inputs. Imagine I have vertices A, A', A'' (such that |X - X'| < eps.
    //  The sequence AA'A'' would be reduced to AA''
    //  The sequence A'A''A would be reduced to A'.
    //  The sequence A''AA' would be reduced to A''A.
    //  In all three cases, the exact same polygon is defined on input, but the
    //  output is different. This should be documented and/or fixed.
    if (polygon->size() <= 1) return;

    auto near = [](const IntersectionVertex<T>& p,
                   const IntersectionVertex<T>& q) -> bool {
      // TODO(SeanCurtis-TRI): This represents 5-6 bits of loss. Confirm that a
      //  tighter epsilon can't be used. This should probably be a function of
      //  the longest edge involved.
      // Empirically we found that numeric_limits<double>::epsilon() 2.2e-16 is
      // too small, especially when the objects are not axis-aligned.
      const double kEpsSquared(1e-14 * 1e-14);
      return (p.cartesian - q.cartesian).squaredNorm() < kEpsSquared;
    };

    // Remove consecutive vertices that are duplicated in the linear order.  It
    // will change "A,B,B,C,C,A" to "A,B,C,A". To close the cyclic order, we
    // will check the first and the last vertices again near the end of the
    // function.
    // TODO(DamrongGuoy): This doesn't strictly satisfy the requirement of
    //  std::unique that the predicate represent an equivalence relation (i.e.,
    //  point A could be "near" points B and C, but that doesn't mean B and C
    //  are near each other). We need to figure out if that matters for this
    //  usage and, if not, document why here.
    auto it = std::unique(polygon->begin(), polygon->end(), near);
    polygon->resize(it - polygon->begin());

    if (polygon->size() >= 3) {
      // Check the first and the last vertices in the sequence. For example,
      // given "A,B,C,A", we want "A,B,C".
      if (near((*polygon)[0], *(polygon->rbegin()))) {
        polygon->pop_back();
      }
    }

    DRAKE_ASSERT(polygon->size() != 2 || !near((*polygon)[0], (*polygon)[1]));
  }

  /* Intersects a triangle with a tetrahedron, returning the portion of the
   triangle with non-zero area contained in the tetrahedron.
   @param tet_index
       Index of the tetrahedron in the volume mesh.
   @param tet_mesh_D
       The volume mesh whose vertex positions are expressed Frame D.
   @param face
       Index of the triangle in the surface mesh.
   @param surface_R
       The surface mesh whose vertex positions are expressed in Frame R.
   @param X_DF
       The pose of the surface frame D in the volume frame R.
   @retval polygon_D
       The output polygon represented by a sequence of positions of its
       vertices, expressed in D's frame. The nature of triangle-tetrahedron
       intersection means that this polygon can have up to seven vertices.
       The following picture shows such an example. The plane of the triangle
       cuts the tetrahedron into a rectangle ABCD with A lying inside the
       triangle. Each of B,C,D is outside the triangle and has two edges that
       intersect an edge of the triangle.

          *
           * *
            *   *
             *     *
              *       *
               *  A------x-----D
                * |         *  |
                 *|            x
                  x            |  *
                  |*           x
                  | *       *  |
                  B--x---x-----C
                      *

   @note
       1. If the triangle is outside the tetrahedron with one vertex on a
          face of the tetrahedron, the output polygon will be empty.
       2. If the triangle is outside the tetrahedron with an edge on a face
          of the tetrahedron, the output polygon will be empty.
       3. If the triangle lies on the plane of a tetrahedron face, the output
          polygon will be that part of the triangle inside the face of the
          tetrahedron (non-zero area restriction still applies).
   */
  const std::vector<IntersectionVertex<T>>& ClipTriangleByTetrahedron(
      int tet_index, const VolumeMesh<T>& tet_mesh_D, int face,
      const TriangleSurfaceMesh<double>& surface_R,
      const math::RigidTransform<T>& X_DR) {
    // Although polygon_D starts out pointing to polygon_[0], that is not an
    // invariant in this function.
    std::vector<IntersectionVertex<T>>* polygon_D = &(polygon_[0]);
    // Initialize output polygon in D's frame from the triangular `face` of
    // surface_R.
    polygon_D->clear();
    for (int i = 0; i < 3; ++i) {
      const int v = surface_R.element(face).vertex(i);
      const Vector3<T> p_DV = X_DR * surface_R.vertex(v).cast<T>();
      polygon_D->push_back({p_DV, tet_mesh_D.CalcBarycentric(p_DV, tet_index)});
    }
    // Get the positions, in Frame D, of the four vertices of the tet.
    Vector3<T> p_DVs[4];
    for (int i = 0; i < 4; ++i) {
      const int v = tet_mesh_D.element(tet_index).vertex(i);
      p_DVs[i] = tet_mesh_D.vertex(v);
    }

    /* Sets up the four half spaces associated with the four triangular faces of
     the tetrahedron.

     A typical tetrahedral element looks like:
          p2 *
             |
             |
          p3 *---* p0
            /
           /
       p1 *
     The index order for a particular tetrahedron has the order [p0, p1, p2,
     p3]. These local indices enumerate each of the tet faces with
     outward-pointing normals with respect to the right-hand rule.  */
    const array<array<int, 3>, 4> faces{
        {{{1, 0, 2}}, {{3, 0, 1}}, {{3, 1, 2}}, {{2, 0, 3}}}};

    // Although this assertion appears trivially true, its presence is
    // protection for the subsequent code, which heavily relies on it being
    // true, from any changes that may be applied to the previous code.
    DRAKE_ASSERT(polygon_D == &(polygon_[0]));
    std::vector<IntersectionVertex<T>>* out_D = &(polygon_[1]);
    for (auto& face_vertex : faces) {
      const Vector3<T>& p_DA = p_DVs[face_vertex[0]];
      const Vector3<T>& p_DB = p_DVs[face_vertex[1]];
      const Vector3<T>& p_DC = p_DVs[face_vertex[2]];
      // We'll allow the PosedHalfSpace to normalize our vector.
      const Vector3<T> normal_D = (p_DB - p_DA).cross(p_DC - p_DA);
      PosedHalfSpace<T> half_space_D(normal_D, p_DA);
      // Intersects the output polygon by the half space of each face of the
      // tetrahedron.
      ClipPolygonByHalfSpace(*polygon_D, half_space_D, out_D);
      std::swap(polygon_D, out_D);
    }

    // TODO(DamrongGuoy): Remove the code below when ClipPolygonByHalfSpace()
    //  stops generating duplicate vertices. See the note in
    //  ClipPolygonByHalfSpace().

    // Remove possible duplicate vertices from ClipPolygonByHalfSpace().
    RemoveDuplicateVertices(polygon_D);
    if (polygon_D->size() < 3) {
      // RemoveDuplicateVertices() may have shrunk the polygon down to one or
      // two vertices, so we empty the polygon.
      polygon_D->clear();
    }

    // TODO(DamrongGuoy): Calculate area of the polygon. If it's too small,
    //  return an empty polygon.

    // The output polygon could be at most a heptagon.
    DRAKE_DEMAND(polygon_D->size() <= 7);
    return *polygon_D;
  }

  /* To avoid heap allocation by std::vector in low-level functions, we use
   these member variables instead of local variables in the functions.
   The two vectors are not guaranteed to have any particular semantic
   interpretation during the execution of this class's main method. This array
   represents a pool of resources; any entry could have arbitrary meaning (or
   none at all) depending where in the algorithm they are inspected.
   Furthermore, any changes to the existing algorithm that make use of these
   pool variables should take care that conflicting use of the resources are
   not introduced.  */
  std::vector<IntersectionVertex<T>> polygon_[2];
};
}  // namespace

template <typename T>
DeformableContactSurface<T> ComputeTetMeshTriMeshContact(
    const DeformableVolumeMesh<T>& tet_mesh_D,
    const TriangleSurfaceMesh<double>& tri_mesh_R,
    const Bvh<Obb, TriangleSurfaceMesh<double>>& bvh_R,
    const math::RigidTransform<T>& X_DR) {
  return Intersector<T>().Intersect(tet_mesh_D, tri_mesh_R, bvh_R, X_DR);
}

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS((
    &ComputeTetMeshTriMeshContact<T>
))

}  // namespace fem
}  // namespace multibody
}  // namespace drake
