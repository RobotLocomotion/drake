#include "drake/geometry/proximity/mesh_plane_intersection.h"

#include <array>
#include <utility>

#include "drake/common/default_scalars.h"
#include "drake/geometry/proximity/contact_surface_utility.h"
#include "drake/geometry/proximity/surface_mesh_field.h"
#include "drake/geometry/proximity/volume_mesh.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

/* This table essentially assigns an index to each edge in the tetrahedron.
 Each edge is represented by its pair of vertex indexes. */
using TetrahedronEdge = std::pair<int, int>;
constexpr std::array<std::pair<int, int>, 6> kTetEdges = {
    // base formed by vertices 0, 1, 2.
    TetrahedronEdge{0, 1}, TetrahedronEdge{1, 2}, TetrahedronEdge{2, 0},
    // pyramid with top at node 3.
    TetrahedronEdge{0, 3}, TetrahedronEdge{1, 3}, TetrahedronEdge{2, 3}};

/* Marching tetrahedra table. Each entry in this table has an index value
 based on a binary encoding of the signs of the plane's signed distance
 function evaluated at all tetrahedron vertices. Therefore, with four
 vertices and two possible signs, we have a total of 16 entries. We encode
 the table indexes in binary so that a "1" and "0" correspond to a vertex
 with positive or negative signed distance, respectively. The least
 significant bit (0) corresponds to vertex 0 in the tetrahedron, and the
 most significant bit (3) is vertex 3. Each entry stores a vector of edges.
 Based on the signed distance values, these edges are the ones that
 intersect the plane. Edges are numbered according to the table kTetEdges.
 The edges have been ordered such that a polygon formed by visiting the
 listed edge's intersection vertices in the array order has a right-handed
 normal pointing in the direction of the plane's normal. The accompanying
 unit tests verify this.

 A -1 is a sentinel value indicating no edge encoding. The number of
 intersecting edges is equal to the index of the *first* -1 (with an implicit
 logical -1 at index 4).  */
constexpr std::array<std::array<int, 4>, 16> kMarchingTetsTable = {
                                /* bits    3210 */
    std::array<int, 4>{-1, -1, -1, -1}, /* 0000 */
    std::array<int, 4>{0, 3, 2, -1},    /* 0001 */
    std::array<int, 4>{0, 1, 4, -1},    /* 0010 */
    std::array<int, 4>{4, 3, 2, 1},     /* 0011 */
    std::array<int, 4>{1, 2, 5, -1},    /* 0100 */
    std::array<int, 4>{0, 3, 5, 1},     /* 0101 */
    std::array<int, 4>{0, 2, 5, 4},     /* 0110 */
    std::array<int, 4>{3, 5, 4, -1},    /* 0111 */
    std::array<int, 4>{3, 4, 5, -1},    /* 1000 */
    std::array<int, 4>{4, 5, 2, 0},     /* 1001 */
    std::array<int, 4>{1, 5, 3, 0},     /* 1010 */
    std::array<int, 4>{1, 5, 2, -1},    /* 1011 */
    std::array<int, 4>{1, 2, 3, 4},     /* 1100 */
    std::array<int, 4>{0, 4, 1, -1},    /* 1101 */
    std::array<int, 4>{0, 2, 3, -1},    /* 1110 */
    std::array<int, 4>{-1, -1, -1, -1}  /* 1111 */};

}  // namespace

template <typename T>
void SliceTetWithPlane(int tet_index,
                       const VolumeMeshFieldLinear<double, double>& field_M,
                       const Plane<T>& plane_M,
                       const math::RigidTransform<T>& X_WM,
                       ContactPolygonRepresentation representation,
                       std::vector<SurfaceFace>* faces,
                       std::vector<Vector3<T>>* vertices_W,
                       std::vector<T>* surface_e,
                       std::unordered_map<SortedPair<int>, int>* cut_edges) {
  const VolumeMesh<double>& mesh_M = field_M.mesh();

  T distance[4];
  // Bit encoding of the sign of signed-distance: v0, v1, v2, v3.
  int intersection_code = 0;
  for (int i = 0; i < 4; ++i) {
    const int v = mesh_M.element(tet_index).vertex(i);
    distance[i] = plane_M.CalcHeight(mesh_M.vertex(v));
    if (distance[i] > T(0)) intersection_code |= 1 << i;
  }

  const std::array<int, 4>& intersected_edges =
      kMarchingTetsTable[intersection_code];

  // No intersecting edges --> no intersection.
  if (intersected_edges[0] == -1) return;

  int num_intersections = 0;
  // Indices of the new polygon's vertices in vertices_W. There can be, at
  // most, four due to the intersection with the plane.
  // Used for ContactPolygonRepresentation::kCentroidSubdivision.
  std::vector<int> face_verts(4);
  // Positions of the new polygon's vertices.
  // Used for ContactPolygonRepresentation::kSingleTriangle.
  std::vector<Vector3<T>> polygon_W(4);
  for (int e = 0; e < 4; ++e) {
    const int edge_index = intersected_edges[e];
    if (edge_index == -1) break;
    ++num_intersections;
    const TetrahedronEdge& tet_edge = kTetEdges[edge_index];
    const int v0 = mesh_M.element(tet_index).vertex(tet_edge.first);
    const int v1 = mesh_M.element(tet_index).vertex(tet_edge.second);
    const SortedPair<int> mesh_edge{v0, v1};

    auto iter = cut_edges->find(mesh_edge);
    if (representation == ContactPolygonRepresentation::kCentroidSubdivision &&
        iter != cut_edges->end()) {
      // Result has already been computed.
      face_verts[e] = iter->second;
    } else {
      // Need to compute the result; but we already know that this edge
      // intersects the plane based on the signed distances of its two
      // vertices.
      const Vector3<double>& p_MV0 = mesh_M.vertex(v0);
      const Vector3<double>& p_MV1 = mesh_M.vertex(v1);
      const T d_v0 = distance[tet_edge.first];
      const T d_v1 = distance[tet_edge.second];
      // Note: It should be impossible for the denominator to be zero. By
      // definition, this is an edge that is split by the plane; they can't
      // both have the same value. More particularly, one must be strictly
      // positive the other must be strictly non-positive.
      const T t = d_v0 / (d_v0 - d_v1);
      const Vector3<T> p_MC = p_MV0 + t * (p_MV1 - p_MV0);
      const Vector3<T> p_WC = X_WM * p_MC;

      switch (representation) {
        case ContactPolygonRepresentation::kCentroidSubdivision: {
          int new_index = static_cast<int>(vertices_W->size());
          vertices_W->emplace_back(p_WC);
          const double e0 = field_M.EvaluateAtVertex(v0);
          const double e1 = field_M.EvaluateAtVertex(v1);
          surface_e->emplace_back(e0 + t * (e1 - e0));
          (*cut_edges)[mesh_edge] = new_index;
          face_verts[e] = new_index;
          break;
        }
        case ContactPolygonRepresentation::kSingleTriangle: {
          polygon_W[e] = p_WC;
          break;
        }
      }
    }
  }

  // Because the vertices are measured and expressed in the world frame, we
  // need to provide a normal expressed in the same.
  const Vector3<T> nhat_W = X_WM.rotation() * plane_M.normal();
  const size_t before = vertices_W->size();
  switch (representation) {
    case ContactPolygonRepresentation::kCentroidSubdivision: {
      face_verts.resize(num_intersections);
      AddPolygonToMeshData(face_verts, nhat_W, faces, vertices_W);
      break;
    }
    case ContactPolygonRepresentation::kSingleTriangle: {
      polygon_W.resize(num_intersections);
      AddPolygonToMeshDataAsOneTriangle(polygon_W, nhat_W, faces, vertices_W);
      break;
    }
  }
  for (size_t v = before; v < vertices_W->size(); ++v) {
    const Vector3<T> p_MV = X_WM.inverse() * vertices_W->at(v);
    surface_e->emplace_back(field_M.EvaluateCartesian(tet_index, p_MV));
  }
}

template <typename T>
std::unique_ptr<ContactSurface<T>> ComputeContactSurface(
    GeometryId mesh_id,
    const VolumeMeshFieldLinear<double, double>& mesh_field_M,
    GeometryId plane_id, const Plane<T>& plane_M,
    const std::vector<int>& tet_indices,
    const math::RigidTransform<T>& X_WM,
    ContactPolygonRepresentation representation) {
  if (tet_indices.size() == 0) return nullptr;

  std::vector<SurfaceFace> faces;
  std::vector<Vector3<T>> vertices_W;
  std::vector<T> surface_e;
  std::unordered_map<SortedPair<int>, int> cut_edges;

  auto grad_eM_W = std::make_unique<std::vector<Vector3<T>>>();
  size_t old_face_count = 0;
  for (const auto& tet_index : tet_indices) {
    const Vector3<T>& grad_eMi_W =
        X_WM.rotation() * mesh_field_M.EvaluateGradient(tet_index).cast<T>();
    SliceTetWithPlane(tet_index, mesh_field_M, plane_M, X_WM, representation,
                      &faces, &vertices_W, &surface_e, &cut_edges);
    // The gradient of every triangle that arises from slicing a tet with a
    // plane is the *constant* gradient inside that tet.
    for (size_t i = old_face_count; i < faces.size(); ++i) {
      grad_eM_W->push_back(grad_eMi_W);
    }
    old_face_count = faces.size();
  }

  // Construct the contact surface from the components.
  DRAKE_DEMAND(vertices_W.size() == surface_e.size());
  if (faces.empty()) return nullptr;

  auto mesh_W =
      std::make_unique<SurfaceMesh<T>>(std::move(faces), std::move(vertices_W));
  auto field_W = std::make_unique<SurfaceMeshFieldLinear<T, T>>(
      std::move(surface_e), mesh_W.get(), false /* calculate_gradient */);
  // SliceTetWithPlane promises to make the surface normals point in the plane
  // normal direction (i.e., out of the plane and into the mesh).
  return std::make_unique<ContactSurface<T>>(
      mesh_id, plane_id, std::move(mesh_W), std::move(field_W),
      std::move(grad_eM_W), nullptr);
}

template <typename T>
std::unique_ptr<ContactSurface<T>>
ComputeContactSurfaceFromSoftVolumeRigidHalfSpace(
    const GeometryId id_S, const VolumeMeshFieldLinear<double, double>& field_S,
    const Bvh<Obb, VolumeMesh<double>>& bvh_S,
    const math::RigidTransform<T>& X_WS, const GeometryId id_R,
    const math::RigidTransform<T>& X_WR,
    ContactPolygonRepresentation representation) {
  std::vector<int> tet_indices;
  tet_indices.reserve(field_S.mesh().num_elements());
  auto callback = [&tet_indices](int tet_index) {
    tet_indices.push_back(tet_index);
    return BvttCallbackResult::Continue;
  };

  const math::RigidTransform<T> X_SR = X_WS.inverse() * X_WR;
  const Vector3<T>& Rz_S = X_SR.rotation().col(2);
  const Vector3<T>& p_SRo = X_SR.translation();
  // NOTE: We don't need the Plane constructor to normalize normal Pz_S. It's
  // sufficiently unit-length for our purposes (and even if it wasn't, we're
  // really only looking for zero *crossings* and that doesn't move with scale.
  Plane<T> plane_S{Rz_S, p_SRo, true /* already_normalized */};
  // We need a double-valued plane for BVH culling.
  Plane<double> plane_S_double{convert_to_double(Rz_S),
                               convert_to_double(p_SRo),
                               true /* already_normalized */};
  // Plane is already defined in the mesh's frame; transform from plane to
  // BVH hierarchy is the identity.
  bvh_S.Collide(plane_S_double, math::RigidTransformd{}, callback);

  // Build the contact surface from the plane and the list of tetrahedron
  // indices.
  return ComputeContactSurface(id_S, field_S, id_R, plane_S, tet_indices, X_WS,
                               representation);
}

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS((
    &ComputeContactSurface<T>,
    &ComputeContactSurfaceFromSoftVolumeRigidHalfSpace<T>,
    &SliceTetWithPlane<T>
))

}  // namespace internal
}  // namespace geometry
}  // namespace drake
