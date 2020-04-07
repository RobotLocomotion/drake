#include "drake/geometry/proximity/make_box_mesh.h"

namespace drake {
namespace geometry {
namespace internal {

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

// Explicit instantiations for double and AutoDiffXd to support the
// documentation.
template VolumeMesh<double> MakeBoxVolumeMesh(const Box& box,
                                              double resolution_hint);
template VolumeMesh<AutoDiffXd> MakeBoxVolumeMesh(const Box& box,
                                                  double resolution_hint);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
