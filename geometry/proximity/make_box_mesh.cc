#include "drake/geometry/proximity/make_box_mesh.h"

#include <algorithm>
#include <array>

#include "drake/geometry/proximity/proximity_utilities.h"

namespace drake {
namespace geometry {
namespace internal {

using Eigen::Vector3d;

/* Subdivides a virtual cube to have up to six tetrahedra such that they
 share the diagonal v0v6 (see ordering below). We allow repeated vertices in
 the virtual cube and will skip tetrahedra with repeated vertices.

 We assume the input vertex indices (represented as VolumeVertexIndex) are in
 this ordering:
   1. Four vertices v0,v1,v2,v3 of the "bottom" face in counterclockwise
      order when look from "above" the cube.
   2. Four vertices v4,v5,v6,v7 of the "top" face matching v0,v1,v2,v3,
      respectively.
 The following picture shows such a cube. Here, the terms "bottom", "above",
 and "top" are relative to the view of users who look at the virtual cube from
 outside, not necessarily a certain axis of a coordinates frame. The term
 "bottom" face means the further face from the user. The term "top" face means
 the closer face to the user. The "top" face is "above" the "bottom" face.

                  User's point of view
               (looking "above" the cube)
                            .
                           /\
                          /o\

                       v4-----------v7
                      /|           /|
                     / |          / |
                    /  |         /  |
                   /   |        /   |
                  /    v0------/----v3
                 /    /       /    /
                v5-----------v6   /
                |   /        |   /
                |  /         |  /
                | /          | /
                |/           |/
                v1-----------v2

 Examples of cases with repeated vertices.
 - If v0 == v3 and v1 == v2, we will have a prism instead of a cube and will
   get 3 tetrahedra instead of 6 tetrahedra.
 - If v0 == v1 == v2 == v3, we will have a pyramid instead of a cube and will
   get 2 tetrahedra instead of 6 tetrahedra.

 @note   This function is purely topological because it takes only
 VolumeVertexIndex into account without attempting to access coordinates of
 the vertices.
 */
std::vector<VolumeElement> SplitToTetrahedra(
    VolumeVertexIndex v0, VolumeVertexIndex v1, VolumeVertexIndex v2,
    VolumeVertexIndex v3, VolumeVertexIndex v4, VolumeVertexIndex v5,
    VolumeVertexIndex v6, VolumeVertexIndex v7) {
  std::vector<VolumeElement> elements;
  elements.reserve(6);
  auto has_repeat_index = [](std::array<VolumeVertexIndex, 4> v) -> bool {
    std::sort(v.begin(), v.end());
    return std::adjacent_find(v.begin(), v.end()) != v.end();
  };
  VolumeVertexIndex previous = v1;
  for (const VolumeVertexIndex& next : {v2, v3, v7, v4, v5, v1}) {
    if (!has_repeat_index({previous, next, v0, v6})) {
      elements.emplace_back(previous, next, v0, v6);
    }
    previous = next;
  }
  return elements;
}

template <typename T>
VolumeMesh<T> MakeBoxVolumeMeshWithMa(const Box& box) {
  // Begin: Notes For Developers.
  //
  // <h1> Illustration in two dimensions </h1>
  //
  // To illustrate the idea of the algorithm, first we describe an analogy in
  // two dimensions. Given a rectangle V₀V₁V₂V₃ in ℝ², we can imagine the
  // process of growing four trapezoids, each of which is from an edge of the
  // rectangle, as shown in these three pictures:
  //
  //     V₀                         V₁     V₀                         V₁
  //     +--------------------------+      +--------------------------+
  //     | ↘----------------------↙ |      | ↘                      ↙ |
  //     | |M₀                  M₁| |      |   ↘------------------↙   |
  //     | |                      | |      |   |                  |   |
  //     | |M₂                  M₃| |      |   ↗------------------↖   |
  //     | ↗----------------------↖ |      | ↗                      ↖ |
  //     +--------------------------+      +--------------------------+
  //     V₂                         V₃     V₂                         V₃
  //
  //     V₀                         V₁
  //     +--------------------------+
  //     | ↘                      ↙ |
  //     |   ↘                  ↙   |      ↙↘↖↗ border medial lines
  //     |    M₀==============M₁    |      ==== central medial lines
  //     |   ↗                  ↖   |
  //     | ↗                      ↖ |
  //     +--------------------------+
  //     V₂                         V₃
  //
  // Notice that:
  //
  // 1. For each vertex Vᵢ of the rectangle, its virtual copy (vertex Mᵢ in the
  //    pictures) moves along an MA's line from Vᵢ towards an MA's vertex. We
  //    call this line a border medial line (drawn with arrows in the picture).
  // 2. For each edge VᵢVⱼ, its virtual copy (edge MᵢMⱼ in the pictures)
  //    shrinks and moves towards the MA's line connecting two MA's vertices
  //    in the middle of the rectangle. We call this line a central medial
  //    line (drawn with "==" in the picture). In some cases, MᵢMⱼ ends up to
  //    coincide with the medial line. In other cases, MᵢMⱼ shrink to a single
  //    point, which is an MA's vertex on the medial line.
  // 3. For each trapezoid VᵢVⱼMᵢMⱼ, it grows until its edge MᵢMⱼ reaches the
  //    central medial line. Notice that two of the four trapezoids degenerate
  //    to triangles. The four (possibly degenerated) trapezoids partition the
  //    rectangle V₀V₁V₂V₃.
  //
  // In two dimensions, the algorithm would partition the rectangle into four
  // blocks, each of which is a (possibly degenerated) trapezoid. Then, we
  // can subdivide each block into triangles for the output mesh.
  //
  // To handle both the normal trapezoids and the degenerated trapezoids
  // seamlessly, we can use two duplicated vertices of a trapezoid to
  // represent a triangle. In other words, the MA-partitioning algorithm
  // always gives virtual trapezoids VᵢVⱼMᵢMⱼ, and the triangle-subdivision
  // subroutine generates triangles VᵢVⱼMᵢ and VⱼMᵢMⱼ, and filters out the
  // triangle VⱼMᵢMⱼ when Mᵢ=Mⱼ.
  //
  // In summary, this is the algoritm in two dimensions:
  //
  //   1. For each edge VᵢVⱼ of the rectangle {
  //   2.   Create a virtual trapezoid VᵢVⱼMᵢMⱼ, where Mᵢ and Mⱼ are the
  //        matching MA's vertices of Vᵢ and Vⱼ, respectively.
  //   3.   Subdivide the virtual trapezoid VᵢVⱼMᵢMⱼ into one or two triangles.
  //   4. }
  //
  // Notice that if we are given a square instead of a rectangle, all the
  // virtual trapezoids would degenerate to triangles. Therefore, the
  // triangle-subdivision subroutine will generate one triangle for each
  // virtual trapezoid.
  //
  // This algorithm in two dimensions generate four and six triangles
  // in the output meshes for the input square and rectangle, respectively.
  //
  //
  // <h1> Algorithm in three dimensions </h1>
  //
  // This is the main idea of our implementation in three dimensions analogous
  // to the example in two dimensions above.
  //
  //   1. For each face VᵢVⱼVₖVₗ of the input box {
  //   2.   Create a virtual frustum VᵢVⱼVₖVₗMᵢMⱼMₖMₗ, where Mᵢ, Mⱼ, Mₖ, Mₗ
  //        are the matching MA's vertices of Vᵢ, Vⱼ, Vₖ, Vₗ, respectively.
  //   3.   Subdivide each virtual frustum (topological cube) into six
  //        tetrahedra and filter out tetrahedra with duplicated vertices.
  //   4  }
  //
  // In this discussion, a frustum is a three-dimensional polytope with two
  // parallel rectangular faces (one face is smaller than the other) and four
  // trapezoidal faces connecting the two rectangular faces. The following
  // pictures show the top views of a 6-face frustum, a frustum that
  // degenerates to a 5-face (topological) "triangular prism", and a frustum
  // that degenerates to a 5-face right square pyramid.
  //
  //     Vᵢ                         Vⱼ     Vᵢ                         Vⱼ
  //     +--------------------------+      +--------------------------+
  //     | ↘                      ↙ |      | ↘                      ↙ |
  //     |   ↘------------------↙   |      |   ↘                  ↙   |
  //     |   |                  |   |      |     x--------------x     |
  //     |   ↗------------------↖   |      |   ↗                  ↖   |
  //     | ↗                      ↖ |      | ↗                      ↖ |
  //     +--------------------------+      +--------------------------+
  //     Vₖ                         Vₗ     Vₖ                         Vₗ
  //
  //             Vᵢ          Vⱼ
  //             +-----------+          These pictures are intended to be
  //             | ↘       ↙ |          three dimensional with the axis of
  //             |   ↘   ↙   |          the third dimension perpendicular
  //             |     x     |          to the screen. The arrows go from
  //             |   ↗   ↖   |          lower altitude to higher altitude.
  //             | ↗       ↖ |          All arrows have the same slope 1.
  //             +-----------+
  //             Vₖ          Vₗ
  //
  // In all cases, we allow the virtual frustum to have duplicated vertices
  // in order to represent the degenerated ones (prisms and pyramids)
  // seamlessly. (A prism is a virtual frustum with Mᵢ = Mⱼ and Mₖ = Mₗ. A
  // pyramid is a virtual frustum with  Mᵢ = Mⱼ = Mₖ = Mₗ.)
  //
  // End: Notes for Developers.

  // The mesh vertices comprise the eight box vertices and up to four unique
  // MA's vertices inside the box. Later each virtual frustum will comprise
  // eight (possibly with duplication) indices into mesh_vertices.
  std::vector<VolumeVertex<T>> mesh_vertices;
  mesh_vertices.reserve(12);

  // The eight mesh vertices of the box are indexed by the 2x2x2 array `v`,
  // where the j,j,k indices for v[i][j][k] correspond to the x,y,z
  // directions of the box's frame, respectively.
  //
  //         v001------v011
  //        /|        /|           Z
  //       / |       / |           |
  //      /  v000---/--v010        |
  //     /  /      /  /            o------Y  The box's frame's origin is
  //    /  /      /  /            /          actually at the center of the box.
  //   v101------v111            /
  //   | /       | /            X
  //   |/        |/
  //   v100------v110
  //
  VolumeVertexIndex v[2][2][2];
  const Vector3d half = box.size() / 2.;
  for (const int i : {0, 1}) {
    const double x(i == 0 ? -half.x() : half.x());
    for (const int j : {0, 1}) {
      const double y(j == 0 ? -half.y() : half.y());
      for (const int k : {0, 1}) {
        const double z(k == 0 ? -half.z() : half.z());
        v[i][j][k] = VolumeVertexIndex(mesh_vertices.size());
        mesh_vertices.emplace_back(x, y, z);
      }
    }
  }
  // Virtually we pretend that there are 8 MA's vertices--corresponding
  // to the 8 box vertices--even though actually there are at most 4 unique
  // MA's vertices (and could be as low as only 1) inside the box. The 8
  // virtual MA's vertices are represented by the 2x2x2 array `m`, where the
  // virtual MA's vertex m[i][j][k] corresponds to the box's vertex
  // v[i][j][k]. Some of the m[i][j][k]'s are duplicated by design.
  VolumeVertexIndex m[2][2][2];
  const double min_half_dimension = half.minCoeff();
  Vector3d shrink = half - Vector3d::Constant(min_half_dimension);
  const double tolerance = DistanceToPointRelativeTolerance(min_half_dimension);
  shrink = (shrink.array() > tolerance).select(shrink, 0.);
  for (const int i : {0, 1}) {
    const double x(i == 0 ? -shrink.x() : shrink.x());
    for (const int j : {0, 1}) {
      const double y(j == 0 ? -shrink.y() : shrink.y());
      for (const int k : {0, 1}) {
        const double z(k == 0 ? -shrink.z() : shrink.z());
        m[i][j][k] = (i == 1 && shrink.x() == 0)
                         ? m[0][j][k]
                         : (j == 1 && shrink.y() == 0)
                               ? m[i][0][k]
                               : (k == 1 && shrink.z() == 0)
                                     ? m[i][j][0]
                                     : VolumeVertexIndex(mesh_vertices.size());
        if ((i == 0 || shrink.x() != 0) && (j == 0 || shrink.y() != 0) &&
            (k == 0 || shrink.z() != 0)) {
          mesh_vertices.emplace_back(x, y, z);
        }
      }
    }
  }
  DRAKE_DEMAND(mesh_vertices.size() <= 12);

  std::vector<VolumeElement> mesh_elements;
  mesh_elements.reserve(24);
  auto append =
      [&mesh_elements](const std::vector<VolumeElement>& new_elements) {
        mesh_elements.insert(mesh_elements.end(), new_elements.begin(),
                             new_elements.end());
      };

  // We design the ordering of the 8 vertices of each virtual frustum, so
  // that all SplitToTetrahedra() of all virtual frusta are conforming.
  // Here conforming means a tetrahedron of one virtual frustum and another
  // tetrahedron of another virtual frustum intersect in their shared face,
  // or shared edge, or shared vertex, i.e., there is no partial overlap.
  //
  // This picture is useful for visualizing construction of the six virtual
  // frustra. The virtual vertices m[i][j][k] are supposed to be inside the
  // box, but we draw them outside the box for clarity. Here we draw the
  // m[i][j][k] as 8 separated vertices, even though some of them are
  // actually duplicated.
  //
  //         v001------v011
  //        /|        /|                m001--m011           Z
  //       / |       / |               /|    /|              |
  //      /  v000---/--v010           / m000/-m010           |
  //     /  /      /  /              / /   / /               o------Y
  //    /  /      /  /              m101--m111              /
  //   v101------v111               |/    |/               /
  //   | /       | /                m100--m110            X
  //   |/        |/
  //   v100------v110
  //

  // front face (+X)
  append(SplitToTetrahedra(m[1][0][0], m[1][1][0], m[1][1][1], m[1][0][1],
                           v[1][0][0], v[1][1][0], v[1][1][1], v[1][0][1]));
  // back face (-X)
  append(SplitToTetrahedra(m[0][0][0], m[0][0][1], m[0][1][1], m[0][1][0],
                           v[0][0][0], v[0][0][1], v[0][1][1], v[0][1][0]));
  // right face (+Y)
  append(SplitToTetrahedra(m[0][1][0], m[0][1][1], m[1][1][1], m[1][1][0],
                           v[0][1][0], v[0][1][1], v[1][1][1], v[1][1][0]));
  // left face (-Y)
  append(SplitToTetrahedra(m[0][0][0], m[1][0][0], m[1][0][1], m[0][0][1],
                           v[0][0][0], v[1][0][0], v[1][0][1], v[0][0][1]));
  // top face (+Z)
  append(SplitToTetrahedra(m[0][0][1], m[1][0][1], m[1][1][1], m[0][1][1],
                           v[0][0][1], v[1][0][1], v[1][1][1], v[0][1][1]));
  // bottom face (-Z)
  append(SplitToTetrahedra(m[0][0][0], m[0][1][0], m[1][1][0], m[1][0][0],
                           v[0][0][0], v[0][1][0], v[1][1][0], v[1][0][0]));
  return {std::move(mesh_elements), std::move(mesh_vertices)};
}

int CalcSequentialIndex(int i, int j, int k, const Vector3<int>& num_vertices) {
  DRAKE_DEMAND(0 <= i && i < num_vertices.x());
  DRAKE_DEMAND(0 <= j && j < num_vertices.y());
  DRAKE_DEMAND(0 <= k && k < num_vertices.z());
  return i * num_vertices.y() * num_vertices.z() + j * num_vertices.z() + k;
}

template <typename T>
std::vector<VolumeVertex<T>> GenerateVertices(
    const Box& box, const Vector3<int>& num_vertices) {
  const T half_x = box.width() / T(2);
  const T half_y = box.depth() / T(2);
  const T half_z = box.height() / T(2);
  const auto x_coords =
      VectorX<T>::LinSpaced(num_vertices.x(), -half_x, half_x);
  const auto y_coords =
      VectorX<T>::LinSpaced(num_vertices.y(), -half_y, half_y);
  const auto z_coords =
      VectorX<T>::LinSpaced(num_vertices.z(), -half_z, half_z);

  std::vector<VolumeVertex<T>> vertices;
  vertices.reserve(num_vertices.x() * num_vertices.y() * num_vertices.z());
  // The order of nested i-loop, j-loop, then k-loop makes the sequence of
  // vertices consistent with CalcSequentialIndex.
  for (int i = 0; i < num_vertices.x(); ++i) {
    for (int j = 0; j < num_vertices.y(); ++j) {
      for (int k = 0; k < num_vertices.z(); ++k) {
        vertices.emplace_back(x_coords[i], y_coords[j], z_coords[k]);
      }
    }
  }
  return vertices;
}

void AddSixTetrahedraOfCell(const Vector3<int>& lowest,
                            const Vector3<int>& num_vertices,
                            std::vector<VolumeElement>* elements) {
  const int i = lowest.x();
  const int j = lowest.y();
  const int k = lowest.z();
  // Get the sequential indices of the eight vertices of the rectangular cell.
  int v[8];
  int s = 0;
  for (int l = 0; l < 2; ++l)
    for (int m = 0; m < 2; ++m)
      for (int n = 0; n < 2; ++n)
        v[s++] = CalcSequentialIndex(i + l, j + m, k + n, num_vertices);
  // The following picture shows where vertex vₛ (for `v[s]` above) locates
  // in the rectangular cell.  The I-, J-, K-axes show the direction of
  // increasing i, j, k indices.
  //
  //               v₁     v₃
  //               ●------●
  //              /|     /|
  //             / |  v₇/ |
  //         v₅ ●------●  |
  //            |  |   |  |
  //            |  ●---|--● v₂
  //            | /v₀  | /
  //            |/     |/
  //    +K   v₄ ●------● v₆
  //     |
  //     |
  //     o------+J
  //    /
  //   /
  // +I
  //
  // The following table subdivides the rectangular cell into six tetrahedra.
  // Refer to the picture above to determine which four vertices form a
  // tetrahedron. The six tetrahedra form a cycle around the diagonal v₀v₇
  // of the cell. Refer to:
  // http://www.baumanneduard.ch/Splitting%20a%20cube%20in%20tetrahedras2.htm
  const int tetrahedron[6][4]{
      // clang-format off
      {v[0], v[7], v[4], v[6]},
      {v[0], v[7], v[6], v[2]},
      {v[0], v[7], v[2], v[3]},
      {v[0], v[7], v[3], v[1]},
      {v[0], v[7], v[1], v[5]},
      {v[0], v[7], v[5], v[4]}};
  // clang-format on
  // The above table guarantees that adjacent rectangular cells will be
  // subdivided in a consistent way, i.e., both cells will pick the same
  // diagonal of their shared rectangular face.
  for (int t = 0; t < 6; ++t) elements->emplace_back(tetrahedron[t]);
}

std::vector<VolumeElement> GenerateElements(const Vector3<int>& num_vertices) {
  std::vector<VolumeElement> elements;
  const Vector3<int> num_cell = num_vertices - Vector3<int>(1, 1, 1);
  elements.reserve(6 * num_cell.x() * num_cell.y() * num_cell.z());
  for (int i = 0; i < num_cell.x(); ++i) {
    for (int j = 0; j < num_cell.y(); ++j) {
      for (int k = 0; k < num_cell.z(); ++k) {
        AddSixTetrahedraOfCell(Vector3<int>(i, j, k), num_vertices, &elements);
      }
    }
  }
  return elements;
}

template <typename T>
VolumeMesh<T> MakeBoxVolumeMesh(const Box& box, double resolution_hint) {
  // TODO(DamrongGuoy): Generate the mesh with rigid core at medial axis or
  //  offset surface (issue #11906) and remove the "@note" above.
  DRAKE_DEMAND(resolution_hint > 0.);
  // Number of vertices in x-, y-, and z- directions.  In each direction,
  // there is one more vertices than cells.
  const Vector3<int> num_vertices{
      1 + static_cast<int>(ceil(box.width() / resolution_hint)),
      1 + static_cast<int>(ceil(box.depth() / resolution_hint)),
      1 + static_cast<int>(ceil(box.height() / resolution_hint))};

  std::vector<VolumeVertex<T>> vertices =
      GenerateVertices<T>(box, num_vertices);

  std::vector<VolumeElement> elements = GenerateElements(num_vertices);

  return VolumeMesh<T>(std::move(elements), std::move(vertices));
}

template VolumeMesh<double> MakeBoxVolumeMeshWithMa(const Box&);
template VolumeMesh<AutoDiffXd> MakeBoxVolumeMeshWithMa(const Box&);

// Explicit instantiations for double and AutoDiffXd to support the
// documentation.
template VolumeMesh<double> MakeBoxVolumeMesh(const Box& box,
                                              double resolution_hint);
template VolumeMesh<AutoDiffXd> MakeBoxVolumeMesh(const Box& box,
                                                  double resolution_hint);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
