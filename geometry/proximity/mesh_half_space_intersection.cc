#include "drake/geometry/proximity/mesh_half_space_intersection.h"

#include <array>
#include <cmath>
#include <limits>
#include <utility>

#include "drake/common/default_scalars.h"
#include "drake/geometry/proximity/contact_surface_utility.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

template <typename T>
int sgn(const T& x) {
  if (x > 0) {
    return 1;
  } else {
    if (x < 0) return -1;
    return 0;
  }
}

/* Utility routine for calculating the intersection point between an edge and
 a plane. Given an edge (defined by its two end-point vertices a and b) and the
 signed distances from a plane (`s_a` and `s_b`, respectively), returns the
 position where the plane splits the edge.

 @pre s_a and s_b must not have the same sign (positive, negative, or zero).
 */
template <typename T>
Vector3<T> CalcEdgePlaneIntersection(
    int a, int b, const T& s_a, const T& s_b,
    const std::vector<Vector3<double>>& vertices_F,
    const math::RigidTransform<T>& X_WF) {
  DRAKE_DEMAND(a != b);
  DRAKE_DEMAND(sgn(s_a) != sgn(s_b));
  using std::abs;
  const T t = abs(s_a) / (abs(s_a) + abs(s_b));
  // We know that the magnitude of the denominator should always be at
  // least as large as the magnitude of the numerator (implying that t should
  // never be greater than unity). Barring an (unlikely) machine epsilon
  // remainder on division, the assertion below should hold.
  DRAKE_DEMAND(t >= 0 && t <= 1);
  const Vector3<T> p_WV =
      X_WF * (vertices_F[a] + t * (vertices_F[b] - vertices_F[a]));
  return p_WV;
}

/* Utility routine for getting the vertex index from the
 `edges_to_newly_created_vertices` hashtable. Given an edge (defined by its two
 end-point vertices a and b) and the signed distances from a plane (`s_a` and
 `s_b`, respectively), returns the index of the vertex that splits the edge at a
 crossing plane. The method creates the vertex if the edge hasn't previously
 been split.

 @pre s_a and s_b must not have the same sign (positive, negative, or zero).
 */
template <typename T>
int GetVertexAddIfNeeded(
    int a, int b, const T& s_a, const T& s_b,
    const std::vector<Vector3<double>>& vertices_F,
    const math::RigidTransform<T>& X_WF,
    std::unordered_map<SortedPair<int>, int>* edges_to_newly_created_vertices,
    std::vector<Vector3<T>>* new_vertices_W) {
  DRAKE_DEMAND(sgn(s_a) != sgn(s_b));

  SortedPair<int> edge_a_b(a, b);
  auto edge_a_b_intersection_iter =
      edges_to_newly_created_vertices->find(edge_a_b);
  if (edge_a_b_intersection_iter == edges_to_newly_created_vertices->end()) {
    bool inserted;
    std::tie(edge_a_b_intersection_iter, inserted) =
        edges_to_newly_created_vertices->insert(
            {edge_a_b, static_cast<int>(new_vertices_W->size())});
    DRAKE_DEMAND(inserted);
    new_vertices_W->emplace_back(
        CalcEdgePlaneIntersection(a, b, s_a, s_b, vertices_F, X_WF));
  }

  return edge_a_b_intersection_iter->second;
}

/* Utility routine for getting the vertex index from the
 `vertices_to_newly_created_vertices` hashtable. Given a vertex `index` from the
 input mesh, returns the corresponding vertex index in `new_vertices_W`. The
 method creates the vertex in `new_vertices_W` if it hasn't already been added.
 */
template <typename T>
int GetVertexAddIfNeeded(
    const std::vector<Vector3<double>>& vertices_F, int index,
    const math::RigidTransform<T>& X_WF,
    std::unordered_map<int, int>* vertices_to_newly_created_vertices,
    std::vector<Vector3<T>>* new_vertices_W) {
  auto v_to_new_v_iter = vertices_to_newly_created_vertices->find(index);
  if (v_to_new_v_iter == vertices_to_newly_created_vertices->end()) {
    bool inserted;
    std::tie(v_to_new_v_iter, inserted) =
        vertices_to_newly_created_vertices->insert(
            {index, static_cast<int>(new_vertices_W->size())});
    DRAKE_DEMAND(inserted);
    // Note: although the vertex is *always* Vector3<double>, we don't support
    // Vector3<AD> = RigidTransform<AD> * Vector3<double>. Therefore, we must
    // cast the vector to T to account for when it is AutoDiffXd.
    const Vector3<T> p_WV = X_WF * vertices_F[index].cast<T>();
    new_vertices_W->emplace_back(p_WV);
  }
  return v_to_new_v_iter->second;
}

}  // namespace

template <typename T>
void ConstructTriangleHalfspaceIntersectionPolygon(
    const SurfaceMesh<double>& mesh_F, int tri_index,
    const PosedHalfSpace<T>& half_space_F, const math::RigidTransform<T>& X_WF,
    ContactPolygonRepresentation representation,
    std::vector<Vector3<T>>* new_vertices_W,
    std::vector<SurfaceFace>* new_faces,
    std::unordered_map<int, int>* vertices_to_newly_created_vertices,
    std::unordered_map<SortedPair<int>, int>* edges_to_newly_created_vertices) {
  // TODO(SeanCurtis-TRI): This needs to support the "backface" culling that is
  //  implemented in mesh-mesh intersection. See the
  //  IsFaceNormalAlongPressureGradient() function in mesh_intersection.h.
  DRAKE_DEMAND(new_vertices_W != nullptr);
  DRAKE_DEMAND(new_faces != nullptr);
  DRAKE_DEMAND(vertices_to_newly_created_vertices != nullptr);
  DRAKE_DEMAND(edges_to_newly_created_vertices != nullptr);

  const std::vector<Vector3<double>>& vertices_F = mesh_F.vertices();
  const SurfaceFace& triangle = mesh_F.element(tri_index);

  // TODO(SeanCurtis-TRI): This code _might_ look cleaner if it used the same
  //  pattern as plane-mesh intersection. I.e., use a bit-encoding of the sign
  //  of each vertex's signed distance φ(v). There are three vertices so 8
  //  possible outcomes, where 0 → φ(v) ≤ 0, 1 → φ(v) > 0. We assume that the
  //  triangles winding is (v0, v1, v2). If we split an edge bridging vertices
  //  vᵢ and vⱼ, it is split at the vertex eᵢⱼ.
  //
  //     v2 | v1 | v0 | outcome
  //    -----------------------
  //      0 | 0  | 0  | Whole triangle is contained in the half space.
  //      0 | 0  | 1  | Quad: v₁ v₂ e₀₂ e₀₁
  //      0 | 1  | 0  | Quad: v₀ e₀₁ e₁₂ v₂
  //      0 | 1  | 1  | Tri: v₂ e₀₂ e₁₂
  //      1 | 0  | 0  | Quad: v₀ v₁ e₁₂ e₀₂
  //      1 | 0  | 1  | Tri: v₁ v₁₂ v₀₂
  //      1 | 1  | 0  | Tri: v₀ e₀₁ e₀₂
  //      1 | 1  | 1  | None
  //
  //  The outcome polygon is a sequence of split vertices that can be thought of
  //  as coming from an "edge". In the case where a vertex is directly included
  //  in the outcome polygon, we can say the edge consists of that vertex twice
  //  leading to the following table:
  //
  //    split vertex | tri "edge" | "edge" index
  //         v₀      |  (v₀, v₀)  |    0
  //         v₁      |  (v₁, v₁)  |    1
  //         v₂      |  (v₂, v₂)  |    2
  //         e₀₁     |  (v₀, v₁)  |    3
  //         e₀₂     |  (v₀, v₂)  |    4
  //         e₁₂     |  (v₁, v₂)  |    5
  //
  //  In code, the first table would be encoded as follows, such that the lists
  //  are indices into the "edges" table shown above. And the decoder of the
  //  edges would know that the edge (i, i) simply refers to vertex i.
  //
  //     000 | (0, 1, 2)
  //     001 | (1, 2, 4, 3)
  //     010 | (0, 3, 5, 2)
  //     011 | (2, 4, 5)
  //     100 | (0, 1, 5, 4)
  //     101 | (1, 5, 4)
  //     110 | (0, 3, 5)
  //     111 | ()
  //
  //  This alternative approach would unify the two mesh-"plane" intersection
  //  algorithms into a common paradigm and simplify the code (i.e., the
  //  variation of outcomes would be encoded in compile-time tables instead of
  //  run-time code paths).

  // NOLINTNEXTLINE(whitespace/line_length)
  // This code was inspired from
  // https://www.geometrictools.com/GTEngine/Include/Mathematics/GteIntrHalfspace3Triangle3.h
  //
  // Compute the signed distance of each triangle vertex from the half space.
  // The table of possibilities is listed next with p = num_positive.
  //
  //     p  intersection
  //     ---------------------------------
  // 1.  0  triangle (original)
  // 2.  1  quad (2 edges clipped)
  // 3.  2  triangle (2 edges clipped)
  // 4.  3  none

  // Compute the signed distance of each triangle vertex from the half space.
  T s[3];
  int num_positive = 0;
  for (int i = 0; i < 3; ++i) {
    s[i] = half_space_F.CalcSignedDistance(vertices_F[triangle.vertex(i)]);
    if (s[i] > 0) ++num_positive;
  }

  if (num_positive == 3) return;

  // Note: although the face normal is *always* Vector3<double>, we don't
  // support Vector3<AD> = RotationMatrix<AD> * Vector3<double>. Therefore, we
  // must cast the vector to T to account for AutoDiffXd.
  const Vector3<T> nhat_W =
      X_WF.rotation() * mesh_F.face_normal(tri_index).cast<T>();

  // Case 1: triangle lies completely within the half space. Preserve
  // the ordering of the triangle vertices.
  if (num_positive == 0) {
    switch (representation) {
      case ContactPolygonRepresentation::kCentroidSubdivision: {
        const int v0_new_index = GetVertexAddIfNeeded(
            vertices_F, triangle.vertex(0), X_WF,
            vertices_to_newly_created_vertices, new_vertices_W);
        const int v1_new_index = GetVertexAddIfNeeded(
            vertices_F, triangle.vertex(1), X_WF,
            vertices_to_newly_created_vertices, new_vertices_W);
        const int v2_new_index = GetVertexAddIfNeeded(
            vertices_F, triangle.vertex(2), X_WF,
            vertices_to_newly_created_vertices, new_vertices_W);

        AddPolygonToMeshData({v0_new_index, v1_new_index, v2_new_index},
                             nhat_W, new_faces, new_vertices_W);
        break;
      }
      case ContactPolygonRepresentation::kSingleTriangle: {
        AddPolygonToMeshDataAsOneTriangle(
            {
                X_WF * vertices_F[triangle.vertex(0)].cast<T>(),
                X_WF * vertices_F[triangle.vertex(1)].cast<T>(),
                X_WF * vertices_F[triangle.vertex(2)].cast<T>(),
            },
            nhat_W, new_faces, new_vertices_W);
        break;
      }
    }
    return;
  }

  // Case 2: The portion of the triangle in the half space is a quadrilateral.
  if (num_positive == 1) {
    for (int i0 = 0; i0 < 3; ++i0) {
      if (s[i0] >= 0) {
        const int i1 = (i0 + 1) % 3;
        const int i2 = (i0 + 2) % 3;
        const int v0 = triangle.vertex(i0);
        const int v1 = triangle.vertex(i1);
        const int v2 = triangle.vertex(i2);

        switch (representation) {
          case ContactPolygonRepresentation::kCentroidSubdivision: {
            // Get the vertices that result from intersecting edge i0/i1 and
            // i0/i2.
            const int edge_i0_i1_intersection_index = GetVertexAddIfNeeded(
                v0, v1, s[i0], s[i1], vertices_F, X_WF,
                edges_to_newly_created_vertices, new_vertices_W);
            const int edge_i0_i2_intersection_index = GetVertexAddIfNeeded(
                v0, v2, s[i0], s[i2], vertices_F, X_WF,
                edges_to_newly_created_vertices, new_vertices_W);

            // Get the indices of the new vertices, adding them if needed.
            const int i1_new_index = GetVertexAddIfNeeded(
                vertices_F, v1, X_WF, vertices_to_newly_created_vertices,
                new_vertices_W);
            const int i2_new_index = GetVertexAddIfNeeded(
                vertices_F, v2, X_WF, vertices_to_newly_created_vertices,
                new_vertices_W);

            // Add the polygon (i1, i2, e02, e01)
            //
            //             i0
            //            ╱╲
            //       e01 ╱  ╲ e02
            //    ______╱____╲___
            //         ╱      ╲
            //        ╱________╲
            //      i1          i2
            //
            AddPolygonToMeshData(
                {i1_new_index, i2_new_index, edge_i0_i2_intersection_index,
                 edge_i0_i1_intersection_index},
                nhat_W, new_faces, new_vertices_W);
            break;
          }
          case ContactPolygonRepresentation::kSingleTriangle: {
            AddPolygonToMeshDataAsOneTriangle(
                {X_WF * vertices_F[v1].cast<T>(),
                 X_WF * vertices_F[v2].cast<T>(),
                 CalcEdgePlaneIntersection(v0, v2, s[i0], s[i2], vertices_F,
                                           X_WF),
                 CalcEdgePlaneIntersection(v0, v1, s[i0], s[i1], vertices_F,
                                           X_WF)},
                nhat_W, new_faces, new_vertices_W);
            break;
          }
        }
        return;
      }
    }

    DRAKE_UNREACHABLE();
  }

  // Case 3: The portion of the triangle in the half space is a triangle.
  if (num_positive == 2) {
    for (int i0 = 0; i0 < 3; ++i0) {
      if (s[i0] <= 0) {
        const int i1 = (i0 + 1) % 3;
        const int i2 = (i0 + 2) % 3;
        const int v0 = triangle.vertex(i0);
        const int v1 = triangle.vertex(i1);
        const int v2 = triangle.vertex(i2);

        switch (representation) {
          case ContactPolygonRepresentation::kCentroidSubdivision: {
            // Get the vertex that corresponds to i0.
            const int i0_new_index = GetVertexAddIfNeeded(
                vertices_F, v0, X_WF, vertices_to_newly_created_vertices,
                new_vertices_W);

            // Get the vertex that results from intersecting edge i0/i1.
            const int edge_i0_i1_intersection_index = GetVertexAddIfNeeded(
                v0, v1, s[i0], s[i1], vertices_F, X_WF,
                edges_to_newly_created_vertices, new_vertices_W);

            // Get the vertex that results from intersecting edge i0/i2.
            const int edge_i0_i2_intersection_index = GetVertexAddIfNeeded(
                v0, v2, s[i0], s[i2], vertices_F, X_WF,
                edges_to_newly_created_vertices, new_vertices_W);

            AddPolygonToMeshData({i0_new_index, edge_i0_i1_intersection_index,
                                  edge_i0_i2_intersection_index},
                                 nhat_W, new_faces, new_vertices_W);
            break;
          }
          case ContactPolygonRepresentation::kSingleTriangle: {
            AddPolygonToMeshDataAsOneTriangle(
                {X_WF * vertices_F[v0].cast<T>(),
                 CalcEdgePlaneIntersection(v0, v1, s[i0], s[i1], vertices_F,
                                           X_WF),
                 CalcEdgePlaneIntersection(v0, v2, s[i0], s[i2], vertices_F,
                                           X_WF)},
                nhat_W, new_faces, new_vertices_W);
            break;
          }
        }
        return;
      }
    }

    DRAKE_UNREACHABLE();
  }
}

template <typename T>
std::unique_ptr<SurfaceMesh<T>>
ConstructSurfaceMeshFromMeshHalfspaceIntersection(
    const SurfaceMesh<double>& input_mesh_F,
    const PosedHalfSpace<T>& half_space_F,
    const std::vector<int>& tri_indices,
    const math::RigidTransform<T>& X_WF,
    ContactPolygonRepresentation representation) {
  std::vector<Vector3<T>> new_vertices_W;
  std::vector<SurfaceFace> new_faces;
  std::unordered_map<int, int> vertices_to_newly_created_vertices;
  std::unordered_map<SortedPair<int>, int> edges_to_newly_created_vertices;

  for (const auto& tri_index : tri_indices) {
    ConstructTriangleHalfspaceIntersectionPolygon(
        input_mesh_F, tri_index, half_space_F, X_WF, representation,
        &new_vertices_W, &new_faces, &vertices_to_newly_created_vertices,
        &edges_to_newly_created_vertices);
  }

  if (new_faces.size() == 0) return nullptr;

  // TODO(SeanCurtis-TRI): This forces SurfaceMesh to recompute the normals for
  //  every triangle in the set of faces. But we know that they should be the
  //  normals drawn from the input mesh. We should accumulate the normals
  //  during accumulation and explicitly provide them here (with a reasonable
  //  check that the winding and the normals are consistent).
  return std::make_unique<SurfaceMesh<T>>(std::move(new_faces),
                                          std::move(new_vertices_W));
}

template <typename T>
std::unique_ptr<ContactSurface<T>>
ComputeContactSurfaceFromSoftHalfSpaceRigidMesh(
    GeometryId id_S, const math::RigidTransform<T>& X_WS, double pressure_scale,
    GeometryId id_R, const SurfaceMesh<double>& mesh_R,
    const Bvh<Obb, SurfaceMesh<double>>& bvh_R,
    const math::RigidTransform<T>& X_WR,
    ContactPolygonRepresentation representation) {
  std::vector<int> tri_indices;
  tri_indices.reserve(mesh_R.num_elements());
  auto bvh_callback = [&tri_indices, &mesh_R,
                       R_WS = convert_to_double(X_WS).rotation(),
                       R_WR =
                           convert_to_double(X_WR).rotation()](int tri_index) {
    // The gradient of the half space pressure field lies in the _opposite_
    // direction as its normal. Its normal is Sz. So, unit_grad_p_W = -Sz_W.
    const Eigen::Vector3d& unit_grad_p_W = -R_WS.col(2);
    if (IsFaceNormalInNormalDirection(unit_grad_p_W, mesh_R, tri_index, R_WR)) {
      tri_indices.push_back(tri_index);
    }
    return BvttCallbackResult::Continue;
  };
  const math::RigidTransform<T> X_RS = X_WR.InvertAndCompose(X_WS);
  // To collide half space with _BVH_ we need (as documented):
  //   - HalfSpace and
  //   - X_PH - pose of the hierarchy in the primitive frame or, in this
  //     context, the hierarchy of the _rigid_ mesh_R in the half space
  //     primitive: X_SR.
  bvh_R.Collide(HalfSpace{}, convert_to_double(X_RS).inverse(), bvh_callback);

  if (tri_indices.size() == 0) return nullptr;

  // In contrast, to collide the half space with the _mesh_, we need the half
  // space measured and expressed in the mesh's frame.
  const Vector3<T>& Sz_R = X_RS.rotation().col(2);
  const Vector3<T>& p_RSo = X_RS.translation();
  // NOTE: We don't need the PosedHalfSpace constructor to normalize Sz_R. It's
  // sufficiently unit-length for our purposes.
  const PosedHalfSpace<T> hs_R{Sz_R, p_RSo, false /* needs_normalization */};

  // TODO(SeanCurtis-TRI): Modify this to return the signed distances of all the
  //  vertices. It will be cheaper than re-evaluating the signed distance for
  //  every vertex just so I can compute the pressure. Alternatively, these
  //  should take a functor for evaluating the pressure or some such thing.
  //  This is a recurrent them in _all_ of the contact surface generating
  //  methods.
  std::unique_ptr<SurfaceMesh<T>> mesh_W =
      ConstructSurfaceMeshFromMeshHalfspaceIntersection(mesh_R, hs_R,
                                                        tri_indices, X_WR,
                                                        representation);

  if (mesh_W == nullptr) return nullptr;

  // Compute the pressure field
  using std::min;
  std::vector<T> vertex_pressures;
  const PosedHalfSpace<T> hs_W{X_WR.rotation() * hs_R.normal(), X_WR * p_RSo};
  vertex_pressures.reserve(mesh_W->num_vertices());
  for (int v = 0; v < mesh_W->num_vertices(); ++v) {
    const Vector3<T> p_WV = mesh_W->vertex(v);
    // The signed distance of the point is the negative of the penetration
    // depth. We can use the pressure_scale to directly compute pressure at the
    // point.
    const T phi_V = hs_W.CalcSignedDistance(p_WV);
    vertex_pressures.push_back(-phi_V * pressure_scale);
  }
  auto field_W = std::make_unique<SurfaceMeshFieldLinear<T, T>>(
      std::move(vertex_pressures), mesh_W.get(), false /* calc_gradient */);

  // TODO(SeanCurtis-TRI) In this case, the gradient across the contact surface
  //  is a constant. It would be good if we could exploit this and *not* copy
  //  the same vector value once per triangle.
  const Vector3<T>& unit_grad_p_W = X_WS.rotation().col(2);
  auto grad_eS_W = std::make_unique<std::vector<Vector3<T>>>(
      mesh_W->num_elements(), -pressure_scale * unit_grad_p_W);

  // ConstructSurfaceMeshFromMeshHalfspaceIntersection() promises to make the
  // face normals point out of the rigid surface and into the soft half space.
  return std::make_unique<ContactSurface<T>>(id_S, id_R, std::move(mesh_W),
                                             std::move(field_W),
                                             std::move(grad_eS_W), nullptr);
}

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS((
    &ConstructTriangleHalfspaceIntersectionPolygon<T>,
    &ConstructSurfaceMeshFromMeshHalfspaceIntersection<T>,
    &ComputeContactSurfaceFromSoftHalfSpaceRigidMesh<T>
))

}  // namespace internal
}  // namespace geometry
}  // namespace drake
