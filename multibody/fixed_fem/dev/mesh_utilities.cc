#include "drake/multibody/fixed_fem/dev/mesh_utilities.h"

#include <memory>
#include <unordered_set>
#include <utility>
#include <vector>

#include "drake/geometry/proximity/make_box_mesh.h"
#include "drake/geometry/proximity/volume_to_surface_mesh.h"
#include "drake/geometry/query_object.h"
#include "drake/geometry/scene_graph.h"

namespace drake {
namespace multibody {
namespace fem {
using geometry::Box;
using geometry::internal::CollectUniqueVertices;
using geometry::internal::IdentifyBoundaryFaces;
using geometry::VolumeElement;
using geometry::VolumeMesh;
using geometry::VolumeMeshFieldLinear;

namespace {

/* Returns true if the given tetrahedron has all four vertices on the
 boundary, which is described by the given set of boundary vertices.
 */
bool IsBoundaryTetrahedron(
    const VolumeElement& tetrahedron,
    const std::unordered_set<int>& boundary_vertex_set) {
  for (int j = 0; j < 4; ++j) {
    if (boundary_vertex_set.count(tetrahedron.vertex(j)) == 0) {
      // The trahedron has a non-boundary vertex.
      return false;
    }
  }
  return true;
}

/* Refines a tetrahedron by inserting a star vertex at its centroid and
 connects the star vertex to all faces of the tetrahedron. The original
 tetrahedron becomes four tetrahedra.
 */
template <typename T>
void StarRefineTetrahedron(int e, std::vector<VolumeElement>* elements,
                           std::vector<Vector3<T>>* vertices) {
  DRAKE_DEMAND(elements != nullptr);
  DRAKE_DEMAND(vertices != nullptr);
  const int num_elements_before = elements->size();
  DRAKE_DEMAND(e < num_elements_before);

  VolumeElement& tetrahedron = (*elements)[e];
  int v0 = tetrahedron.vertex(0);
  int v1 = tetrahedron.vertex(1);
  int v2 = tetrahedron.vertex(2);
  int v3 = tetrahedron.vertex(3);

  const int num_vertices_before = vertices->size();
  DRAKE_DEMAND(v0 < num_vertices_before);
  DRAKE_DEMAND(v1 < num_vertices_before);
  DRAKE_DEMAND(v2 < num_vertices_before);
  DRAKE_DEMAND(v3 < num_vertices_before);

  Vector3<T> star{
      ((*vertices)[v0] + (*vertices)[v1] + (*vertices)[v2] + (*vertices)[v3]) /
      4.};
  vertices->push_back(std::move(star));
  int star_vertex(num_vertices_before);

  //
  // You can use this picture to verify the connectivity with your
  // right hand.
  //
  //     +Z
  //      |
  //      v3
  //      |
  //      v0---v2---+Y
  //     /
  //    v1
  //   /
  // +X
  //
  (*elements)[e] = VolumeElement(v0, v1, v2, star_vertex);  // replace v3
  elements->emplace_back(v1, v3, v2, star_vertex);  // replace v0
  elements->emplace_back(v2, v3, v0, star_vertex);  // replace v1
  elements->emplace_back(v3, v1, v0, star_vertex);  // replace v2
}

}  // namespace

template <typename T>
VolumeMesh<T> StarRefineBoundaryTetrahedra(
    const VolumeMesh<T>& in) {
  std::vector<Vector3<T>> out_vertices(in.vertices());
  std::vector<VolumeElement> out_elements(in.tetrahedra());

  const std::vector<int> boundary_vertices =
      CollectUniqueVertices(IdentifyBoundaryFaces(out_elements));
  const std::unordered_set<int> boundary_vertex_set(
      boundary_vertices.begin(), boundary_vertices.end());

  const int num_input_tetrahedra = in.num_elements();
  for (int i=0; i < num_input_tetrahedra; ++i) {
    if (IsBoundaryTetrahedron(out_elements[i], boundary_vertex_set)) {
      StarRefineTetrahedron(i, &out_elements, &out_vertices);
    }
  }

  return {std::move(out_elements), std::move(out_vertices)};
}

/* Generates connectivity for the tetrahedral elements of the mesh by splitting
 each cube into five tetrahedra.
 @param[in] num_vertices
     Number of vertices in each of x-, y-, and z-directions.
 @return
     A sequence of tetrahedral elements that share unique vertices. */
std::vector<VolumeElement> GenerateDiamondCubicElements(
    const Vector3<int>& num_vertices) {
  std::vector<VolumeElement> elements;
  const Vector3<int> num_cells = num_vertices - Vector3<int>(1, 1, 1);
  /* The number of vertices in each direction. */
  for (int i = 0; i < num_cells(0); ++i) {
    for (int j = 0; j < num_cells(1); ++j) {
      for (int k = 0; k < num_cells(2); ++k) {
        /* The following picture shows where vertex vₛ (for `v[s]` below)
         locates in the rectangular cell.  The I-, J-, K-axes show the
         direction of increasing i, j, k indices.

                       v₁     v₃
                       ●------●
                      /|     /|
                     / |  v₇/ |
                 v₅ ●------●  |
                    |  |   |  |
                    |  ●---|--● v₂
                    | /v₀  | /
                    |/     |/
            +K   v₄ ●------● v₆
             |
             |
             o------+J
            /
           /
         +I                                                        */
        int v[8];
        int s = 0;
        for (int l = 0; l < 2; ++l) {
          for (int m = 0; m < 2; ++m) {
            for (int n = 0; n < 2; ++n) {
              v[s++] = geometry::internal::CalcSequentialIndex(
                  i + l, j + m, k + n, num_vertices);
            }
          }
        }

        /* The following table subdivides the rectangular cell into five
         tetrahedra. Refer to the picture above to determine which four vertices
         form a tetrahedron. Refer to splitting No. 13 in:
         http://www.baumanneduard.ch/Splitting%20a%20cube%20in%20tetrahedras2.htm
         The splitting alternates depending on the positions of the cube to
         ensure that the mesh is conforming. */
        if ((i + j + k) % 2 == 1) {
          elements.emplace_back(v[6], v[4], v[7], v[2]);
          elements.emplace_back(v[7], v[2], v[1], v[3]);
          elements.emplace_back(v[1], v[4], v[7], v[5]);
          elements.emplace_back(v[2], v[4], v[1], v[0]);
          elements.emplace_back(v[7], v[4], v[1], v[2]);
        } else {
          elements.emplace_back(v[0], v[6], v[5], v[4]);
          elements.emplace_back(v[3], v[6], v[0], v[2]);
          elements.emplace_back(v[5], v[6], v[3], v[7]);
          elements.emplace_back(v[3], v[0], v[5], v[1]);
          elements.emplace_back(v[0], v[6], v[3], v[5]);
        }
      }
    }
  }
  return elements;
}

template <typename T>
internal::ReferenceDeformableGeometry<T> MakeDiamondCubicBoxDeformableGeometry(
    const Box& box, double resolution_hint,
    const math::RigidTransform<T>& X_WB) {
  DRAKE_DEMAND(resolution_hint > 0.);
  /* Number of vertices in x-, y-, and z- directions.  In each direction,
   there is one more vertices than cells. */
  const Vector3<int> num_vertices{
      1 + static_cast<int>(ceil(box.width() / resolution_hint)),
      1 + static_cast<int>(ceil(box.depth() / resolution_hint)),
      1 + static_cast<int>(ceil(box.height() / resolution_hint))};

  /* Initially generate vertices in box's frame B. */
  std::vector<Vector3<T>> vertices =
      geometry::internal::GenerateVertices<T>(box, num_vertices);

  // TODO(xuchenhan-tri): This is an expedient but expensive way to calculate
  //  the signed distance of the vertices. When moving out of dev/, come up with
  //  a better solution.
  /* Generate the vertex distances to the shape. */
  geometry::SceneGraph<T> scene_graph;
  const auto& source_id = scene_graph.RegisterSource();
  const auto& geometry_id = scene_graph.RegisterGeometry(
      source_id, scene_graph.world_frame_id(),
      std::make_unique<geometry::GeometryInstance>(math::RigidTransformd(),
                                                   box.Clone(), "box"));
  scene_graph.AssignRole(source_id, geometry_id,
                         geometry::ProximityProperties());
  const auto& context = scene_graph.CreateDefaultContext();
  const auto& query_object =
      scene_graph.get_query_output_port()
          .template Eval<geometry::QueryObject<T>>(*context);
  std::vector<T> signed_distances;
  for (const Vector3<T>& vertex : vertices) {
    const auto& d = query_object.ComputeSignedDistanceToPoint(vertex);
    DRAKE_DEMAND(d.size() == 1);
    signed_distances.emplace_back(d[0].distance);
  }

  for (Vector3<T>& vertex : vertices) {
    // Transform to World frame.
    vertex = Vector3<T>(X_WB * vertex);
  }

  std::vector<VolumeElement> elements =
      GenerateDiamondCubicElements(num_vertices);
  auto mesh =
      std::make_unique<VolumeMesh<T>>(std::move(elements), std::move(vertices));
  auto mesh_field = std::make_unique<VolumeMeshFieldLinear<T, T>>(
      std::move(signed_distances), mesh.get(), false);
  return {std::move(mesh), std::move(mesh_field)};
}

template <typename T>
VolumeMesh<T> MakeOctahedronVolumeMesh() {
  // Eight tetrahedra in the octahedron mesh. Order the vertices to follow the
  // convention in geometry::VolumeMesh. See geometry::VolumeMesh for more
  // details.
  const int element_data[8][4] = {
      // The top four tetrahedrons share the top vertex v5.
      {0, 1, 2, 5},
      {0, 2, 3, 5},
      {0, 3, 4, 5},
      {0, 4, 1, 5},
      // The bottom four tetrahedrons share the bottom vertex v6.
      {0, 2, 1, 6},
      {0, 3, 2, 6},
      {0, 4, 3, 6},
      {0, 1, 4, 6}};
  std::vector<VolumeElement> elements;
  for (const auto& element : element_data) {
    elements.emplace_back(element);
  }
  // clang-format off
  const Vector3<T> vertex_data[7] = {
      { 0,  0,  0},
      { 1,  0,  0},
      { 0,  1,  0},
      {-1,  0,  0},
      { 0, -1,  0},
      { 0,  0,  1},
      { 0,  0, -1}};
  // clang-format on
  std::vector<Vector3<T>> vertices;
  for (const auto& vertex : vertex_data) {
    vertices.emplace_back(vertex);
  }
  return VolumeMesh<T>(std::move(elements), std::move(vertices));
}

template <typename T>
internal::ReferenceDeformableGeometry<T> MakeOctahedronDeformableGeometry() {
  auto mesh = std::make_unique<VolumeMesh<T>>(MakeOctahedronVolumeMesh<T>());
  /* The distance to surface of the octahedron is zero for all vertices except
   for v0 that has signed distance to the surface of -1/√3. */
  std::vector<T> signed_distances(7, 0.0);
  signed_distances[0] = -1.0 / std::sqrt(3);
  auto mesh_field = std::make_unique<VolumeMeshFieldLinear<T, T>>(
      std::move(signed_distances), mesh.get(), false);
  return {std::move(mesh), std::move(mesh_field)};
}

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    (&MakeDiamondCubicBoxDeformableGeometry<T>, &MakeOctahedronVolumeMesh<T>,
     &MakeOctahedronDeformableGeometry<T>, &StarRefineBoundaryTetrahedra<T>))

}  // namespace fem
}  // namespace multibody
}  // namespace drake
