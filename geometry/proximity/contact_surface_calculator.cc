#include "drake/geometry/proximity/contact_surface_calculator.h"

#include "drake/common/default_scalars.h"

namespace drake {
namespace geometry {
namespace internal {

// This table essentially assigns an index to each edge in the tetrahedra. Each
// edge is represented by its pair of vertex indexes.
using Edge = ContactSurfaceCalculatorConstants::Edge;
const std::array<Edge, 6> ContactSurfaceCalculatorConstants::kEdges = {
    Edge{0, 1}, Edge{1, 2}, Edge{2, 0},   // base formed by vertices 0, 1, 2.
    Edge{0, 3}, Edge{1, 3}, Edge{2, 3}};  // pyramid with top at node 3.

// Marching tetrahedra table. Each entry in this table has an index assigned by
// encoding the sign of each vertex in binary. Therefore, with four vertices and
// two possible signs, we have a total of 16 entries.
// Each entry stores a vector of edges. Based on the sign of the level set,
// these edges are the ones with a zero level set crossing. Edges are numbered
// according to table ContactSurfaceCalculatorConstants::kEdges.
// See Figure 3 in [Bloomenthal, 1994] cited in the documentation for
// CalcZeroLevelSetInMeshDomain() for details on the entries in this table.
// We changed the order of the edges here so that they always form a closed
// boundary oriented according to the right-hand rule around a vector from the
// negative side to the positive side. The accompanying unit tests verify this.
using EdgeIndex = ContactSurfaceCalculatorConstants::EdgeIndex;
const std::array<std::vector<EdgeIndex>, 16>
    ContactSurfaceCalculatorConstants::kMarchingTetsTable = {
        {{},           /* 0000 */
         {0, 2, 3},    /* 0001 */
         {0, 4, 1},    /* 0010 */
         {1, 2, 3, 4}, /* 0011 */
         {1, 5, 2},    /* 0100 */
         {1, 5, 3, 0}, /* 0101 */
         {4, 5, 2, 0}, /* 0110 */
         {3, 4, 5},    /* 0111 */
         {3, 5, 4},    /* 1000 */
         {0, 2, 5, 4}, /* 1001 */
         {0, 3, 5, 1}, /* 1010 */
         {1, 2, 5},    /* 1011 */
         {4, 3, 2, 1}, /* 1100 */
         {0, 1, 4},    /* 1101 */
         {0, 3, 2},    /* 1110 */
         {} /* 1111 */}};

template <typename T>
int ContactSurfaceCalculator<T>::IntersectTetWithLevelSet(
    const std::vector<Vector3<T>>& tet_vertices_N, const Vector4<T>& phi_N,
    std::vector<SurfaceVertex<T>>* vertices, std::vector<SurfaceFace>* faces) {
  DRAKE_ASSERT(tet_vertices_N.size() == 4);
  DRAKE_ASSERT(phi_N.size() == 4);
  DRAKE_ASSERT(vertices != nullptr);
  DRAKE_ASSERT(faces != nullptr);

  using Vector3i = Vector3<int>;
  const Vector3i kOnes3 = Vector3i::Ones();

  // The current number of vertices before new are added.
  const int num_vertices = vertices->size();

  // Find out the marching tetrahedra case. We encode the case number in binary.
  // If a vertex is positive, we assign a "1", otherwise "0". We then from the
  // four bits number "binary_code" which in decimal leads to the index entry in
  // the marching tetrahedra table (from 0 to 15).
  using Array4i = Eigen::Array<int, 4, 1>;
  const Array4i binary_code = (phi_N.array() > 0.0).template cast<int>();
  const Array4i powers_of_two = Vector4<int>(1, 2, 4, 8).array();
  const int case_index = (binary_code * powers_of_two).sum();

  const std::vector<int>& intersected_edges =
      ContactSurfaceCalculatorConstants::kMarchingTetsTable[case_index];
  const int num_intersections = intersected_edges.size();

  if (num_intersections == 0) return num_intersections;  // no new triangles.

  // Compute intersections vertices by linear interpolation of the level-set
  // to the zero-crossing.
  Vector3<T> pc_N = Vector3<T>::Zero();  // Geometric center.
  for (int edge_index : intersected_edges) {
    const Edge& edge = ContactSurfaceCalculatorConstants::kEdges[edge_index];
    const Vector3<T>& p1_N = tet_vertices_N[edge.first];
    const Vector3<T>& p2_N = tet_vertices_N[edge.second];
    const T& phi1 = phi_N[edge.first];
    const T& phi2 = phi_N[edge.second];
    using std::abs;
    const T w2 = abs(phi1) / (abs(phi1) + abs(phi2));
    const T w1 = 1.0 - w2;
    const Vector3<T> pz_N = w1 * p1_N + w2 * p2_N;
    vertices->emplace_back(pz_N);
    // The geometric center is only needed for Case II.
    if (num_intersections == 4) pc_N += pz_N;
  }

  // Case I: A single vertex has different sign from the other three. A single
  // triangle is formed. We form a triangle so that its right handed normal
  // points in the direction of the positive side of the volume.
  if (num_intersections == 3) {
    faces->emplace_back(Vector3i(0, 1, 2) + num_vertices * kOnes3);
    return num_intersections;
  }

  // Case II: Two pairs of vertices with the same sign. We form four new
  // triangles by placing an additional vertex in the geometry center of the
  // intersected vertices. The new triangles are oriented such that their
  // normals point towards the positive side, in accordance to our convention.
  if (num_intersections == 4) {
    // Add the geometric center.
    pc_N /= 4.0;
    vertices->emplace_back(pc_N);

    // Make four triangles sharing the geometric center. All oriented such
    // that their right-handed normal points towards the positive side.
    faces->emplace_back(Vector3i(0, 1, 4) + num_vertices * kOnes3);
    faces->emplace_back(Vector3i(1, 2, 4) + num_vertices * kOnes3);
    faces->emplace_back(Vector3i(2, 3, 4) + num_vertices * kOnes3);
    faces->emplace_back(Vector3i(3, 0, 4) + num_vertices * kOnes3);
  }

  // The number of vertices equals 5.
  return num_intersections + 1;
}

template <typename T>
SurfaceMesh<T> ContactSurfaceCalculator<T>::CalcZeroLevelSetInMeshDomain(
    const VolumeMesh<T>& mesh_M,
    std::function<T(const Vector3<T>&)> level_set_N,
    const math::RigidTransform<T>& X_NM) {
  std::vector<SurfaceVertex<T>> vertices;
  std::vector<SurfaceFace> faces;

  // We scan each tetrahedra in the mesh and compute the zero level set with
  // IntersectTetWithLevelSet().
  std::vector<Vector3<T>> tet_vertices_N(4);
  Vector4<T> phi_N;
  for (const auto& tet_indexes : mesh_M.tetrahedra()) {
    // Collect data for each vertex of the tetrahedron.
    for (int i = 0; i < 4; ++i) {
      const auto& p_MV = mesh_M.vertex(tet_indexes.vertex(i)).r_MV();
      tet_vertices_N[i] = X_NM * p_MV;
      phi_N[i] = level_set_N(tet_vertices_N[i]);
    }
    // IntersectTetWithLevelSet() uses a different convention than VolumeMesh
    // to index the vertices of a tetrahedra and therefore we swap vertexes 1
    // and 2.
    // TODO(amcastro-tri): If this becomes a performance issue, re-write
    // IntersectTetWithLevelSet() to use the convention in VolumeMesh.
    std::swap(tet_vertices_N[1], tet_vertices_N[2]);
    std::swap(phi_N[1], phi_N[2]);
    IntersectTetWithLevelSet(tet_vertices_N, phi_N, &vertices, &faces);
  }

  return SurfaceMesh<T>(std::move(faces), std::move(vertices));
}

}  // namespace internal.
}  // namespace geometry.
}  // namespace drake.

// TODO(amcastro-tri): enable for symbolic.
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::geometry::internal::ContactSurfaceCalculator)
