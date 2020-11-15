#include "drake/geometry/proximity/make_cylinder_mesh.h"

#include <cmath>

#include "drake/common/eigen_types.h"
#include "drake/geometry/proximity/meshing_utilities.h"
#include "drake/geometry/proximity/proximity_utilities.h"

namespace drake {
namespace geometry {
namespace internal {

template <typename T>
std::vector<VolumeElement> CalcLongCylinderVolumeMeshWithMa(
    const Cylinder& cylinder,
    const int num_vertices_per_circle,
    const VolumeVertexIndex cv[2],
    const std::vector<VolumeVertexIndex> v[2],
    std::vector<VolumeVertex<T>>* mesh_vertices) {
  const double top_z = cylinder.length() / 2.;
  const double tolerance =
      DistanceToPointRelativeTolerance(std::min(top_z, cylinder.radius()));
  using std::abs;
  DRAKE_DEMAND(top_z - cylinder.radius() > tolerance);
  DRAKE_DEMAND(num_vertices_per_circle == static_cast<int>(v[0].size()));
  DRAKE_DEMAND(num_vertices_per_circle == static_cast<int>(v[1].size()));

  // These two mesh vertices will be at the two end points of the central
  // medial line segment.
  VolumeVertexIndex cm[2];
  const double offset_distance = cylinder.radius();
  const double offset_top_z = top_z - offset_distance;
  const double offset_bottom_z = -offset_top_z;
  cm[0] = VolumeVertexIndex(mesh_vertices->size());
  mesh_vertices->emplace_back(0, 0, offset_bottom_z);
  cm[1] = VolumeVertexIndex(mesh_vertices->size());
  mesh_vertices->emplace_back(0, 0, offset_top_z);

  std::vector<VolumeElement> mesh_elements;
  // n = num_vertices_per_circle.
  // bottom cone: n tetrahedra, top cone: n tetrahedra.
  // main bore: n (topological) triangular prisms => 3n tetrahedra.
  mesh_elements.reserve(5 * num_vertices_per_circle);
  auto append =
      [&mesh_elements](const std::vector<VolumeElement>& new_elements) {
        mesh_elements.insert(mesh_elements.end(), new_elements.begin(),
                             new_elements.end());
      };
  // Add tetrahedra as we go around the circle.
  // i = (n-1) 0 1 2 ... (n-2)
  // j =   0   1 2 3 ... (n-1)
  int i = num_vertices_per_circle - 1;
  for (int j = 0; j < num_vertices_per_circle; ++j) {
    // Bottom cone.
    mesh_elements.emplace_back(cv[0], v[0][i], v[0][j], cm[0]);
    // Top cone.
    mesh_elements.emplace_back(cv[1], v[1][j], v[1][i], cm[1]);
    // Main bore consists of n triangular prisms.
    append(SplitTriangularPrismToTetrahedra(cm[0], v[0][i], v[0][j],  // bottom
                                            cm[1], v[1][i], v[1][j]));  // top
    i = j;
  }

  return mesh_elements;
}

template <typename T>
std::vector<VolumeElement> CalcShortCylinderVolumeMeshWithMa(
    const Cylinder& cylinder, const int num_vertices_per_circle,
    const VolumeVertexIndex cv[2], const std::vector<VolumeVertexIndex> v[2],
    std::vector<VolumeVertex<T>>* mesh_vertices) {
  const double top_z = cylinder.length() / 2.;
  const double tolerance =
      DistanceToPointRelativeTolerance(std::min(top_z, cylinder.radius()));
  using std::abs;
  DRAKE_DEMAND(cylinder.radius() - top_z > tolerance);

  DRAKE_DEMAND(num_vertices_per_circle == static_cast<int>(v[0].size()));
  DRAKE_DEMAND(num_vertices_per_circle == static_cast<int>(v[1].size()));

  // For convenience, we add one vertex at the center of the cylinder, which
  // is also the center of the central medial circle.
  VolumeVertexIndex cm = VolumeVertexIndex(mesh_vertices->size());
  mesh_vertices->emplace_back(0, 0, 0);

  // Vertices along the medial circle.
  std::vector<VolumeVertexIndex> m(num_vertices_per_circle);
  const double offset_radius = cylinder.radius() - cylinder.length() / 2.;
  const double scale_cylinder_radius_to_medial_circle =
      offset_radius / cylinder.radius();
  for (int i = 0; i < num_vertices_per_circle; ++i) {
    const double x =
        ExtractDoubleOrThrow(mesh_vertices->at(v[0][i]).r_MV().x()) *
        scale_cylinder_radius_to_medial_circle;
    const double y =
        ExtractDoubleOrThrow(mesh_vertices->at(v[0][i]).r_MV().y()) *
        scale_cylinder_radius_to_medial_circle;
    m[i] = VolumeVertexIndex(mesh_vertices->size());
    mesh_vertices->emplace_back(x, y, 0);
  }

  std::vector<VolumeElement> mesh_elements;
  // n = num_vertices_per_circle.
  // bottom frustum: n triangular prisms => 3n tetrahedra.
  // top frustum: n triangular prisms => 3n tetrahedra.
  // side co-frustum: ring of n triangular prisms => 3n tetrahedra.
  mesh_elements.reserve(9 * num_vertices_per_circle);
  auto append =
      [&mesh_elements](const std::vector<VolumeElement>& new_elements) {
        mesh_elements.insert(mesh_elements.end(), new_elements.begin(),
                             new_elements.end());
      };
  // Add tetrahedra as we go around the circle.
  // i = (n-1) 0 1 2 ... (n-2)
  // j =   0   1 2 3 ... (n-1)
  int i = num_vertices_per_circle - 1;
  for (int j = 0; j < num_vertices_per_circle; ++j) {
    // Bottom frustum.
    append(SplitTriangularPrismToTetrahedra(cv[0], v[0][i], v[0][j],  // bottom
                                            cm, m[i], m[j]));         // medial
    // Top frustum.
    append(SplitTriangularPrismToTetrahedra(cm, m[i], m[j],  // medial
                                            cv[1], v[1][i], v[1][j]));  // top
    // Side anti-frustum.
    append(SplitTriangularPrismToTetrahedra(v[0][i], m[i], v[1][i],    // i-tri
                                            v[0][j], m[j], v[1][j]));  // j-tri
    i = j;
  }

  return mesh_elements;
}

template <typename T>
std::vector<VolumeElement> CalcMediumCylinderVolumeMeshWithMa(
    const Cylinder& cylinder, const int num_vertices_per_circle,
    const VolumeVertexIndex cv[2], const std::vector<VolumeVertexIndex> v[2],
    std::vector<VolumeVertex<T>>* mesh_vertices) {
  const double top_z = cylinder.length() / 2.;
  const double tolerance =
      DistanceToPointRelativeTolerance(std::min(top_z, cylinder.radius()));
  using std::abs;
  DRAKE_DEMAND(abs(top_z - cylinder.radius()) <= tolerance);
  DRAKE_DEMAND(num_vertices_per_circle == static_cast<int>(v[0].size()));
  DRAKE_DEMAND(num_vertices_per_circle == static_cast<int>(v[1].size()));

  // The central medial vertex is at the center of the medium cylinder.
  VolumeVertexIndex cm = VolumeVertexIndex(mesh_vertices->size());
  mesh_vertices->emplace_back(0, 0, 0);

  std::vector<VolumeElement> mesh_elements;
  // n = num_vertices_per_circle.
  // bottom cone: n tetrahedra, top cone: n tetrahedra.
  // side co-cone: ring of n pyramids => 2n tetrahedra.
  mesh_elements.reserve(4 * num_vertices_per_circle);
  auto append =
      [&mesh_elements](const std::vector<VolumeElement>& new_elements) {
        mesh_elements.insert(mesh_elements.end(), new_elements.begin(),
                             new_elements.end());
      };
  // Add tetrahedra as we go around the circle.
  // i = (n-1) 0 1 2 ... (n-2)
  // j =   0   1 2 3 ... (n-1)
  int i = num_vertices_per_circle - 1;
  for (int j = 0; j < num_vertices_per_circle; ++j) {
    // Bottom cone.
    mesh_elements.emplace_back(cv[0], v[0][i], v[0][j], cm);
    // Top cone.
    mesh_elements.emplace_back(cv[1], v[1][j], v[1][i], cm);
    // Side co-cone.
    append(SplitPyramidToTetrahedra(v[1][i], v[1][j], v[0][j], v[0][i], cm));
    i = j;
  }
  return mesh_elements;
}

template <typename T>
VolumeMesh<T> MakeCylinderVolumeMeshWithMa(const Cylinder& cylinder,
                                           const double resolution_hint) {
  const double top_z = cylinder.length() / 2.;
  const double bottom_z = -top_z;
  enum class CylinderClass {
    kLong,
    kMedium,
    kShort
  };
  const double shorter_half = std::min(top_z, cylinder.radius());
  const double tolerance = DistanceToPointRelativeTolerance(shorter_half);
  const CylinderClass cylinder_class =
      (top_z - cylinder.radius() > tolerance)
          ? CylinderClass::kLong
          : (cylinder.radius() - top_z > tolerance) ? CylinderClass::kShort
                                                    : CylinderClass::kMedium;
  const int num_vertices_per_circle =
      static_cast<int>(ceil(2. * M_PI * cylinder.radius() / resolution_hint));

  std::vector<VolumeVertex<T>> mesh_vertices;
  switch (cylinder_class) {
    case CylinderClass::kLong:
      mesh_vertices.reserve(2 * num_vertices_per_circle + 4);
      break;
    case CylinderClass::kShort:
      mesh_vertices.reserve(3 * num_vertices_per_circle + 3);
      break;
    case CylinderClass::kMedium:
      mesh_vertices.reserve(2 * num_vertices_per_circle + 3);
      break;
  }

  // For convenience, we always add one vertex at the center of each of the
  // bottom and the top circular caps. Our convention, which is used in
  // connecting vertices together to make tetrahedra, is to add the vertex at
  // the bottom first and then the top.
  VolumeVertexIndex cv[2];
  cv[0] = VolumeVertexIndex(mesh_vertices.size());
  mesh_vertices.emplace_back(0, 0, bottom_z);
  cv[1] = VolumeVertexIndex(mesh_vertices.size());
  mesh_vertices.emplace_back(0, 0, top_z);

  // Vertices on the two circular rims. Our convention, which is used in
  // generateing tetrahedra, is to add bottom vertices before top vertices.
  // v[0][i] is for the bottom circular rim.
  // v[1][i] is for the top circular rim.
  std::vector<VolumeVertexIndex> v[2];
  v[0].resize(num_vertices_per_circle);
  v[1].resize(num_vertices_per_circle);
  const double angle_step = 2. * M_PI / num_vertices_per_circle;
  for (int i = 0; i < num_vertices_per_circle; ++i) {
    const double x = cylinder.radius() * cos(angle_step * i);
    const double y = cylinder.radius() * sin(angle_step * i);
    v[0][i] = VolumeVertexIndex(mesh_vertices.size());
    mesh_vertices.emplace_back(x, y, bottom_z);
    v[1][i] = VolumeVertexIndex(mesh_vertices.size());
    mesh_vertices.emplace_back(x, y, top_z);
  }

  std::vector<VolumeElement> mesh_elements;
  switch (cylinder_class) {
    case CylinderClass::kLong:
      mesh_elements = CalcLongCylinderVolumeMeshWithMa<T>(
          cylinder, num_vertices_per_circle, cv, v, &mesh_vertices);
      break;
    case CylinderClass::kShort:
      mesh_elements = CalcShortCylinderVolumeMeshWithMa<T>(
          cylinder, num_vertices_per_circle, cv, v, &mesh_vertices);
      break;
    case CylinderClass::kMedium:
      mesh_elements = CalcMediumCylinderVolumeMeshWithMa<T>(
          cylinder, num_vertices_per_circle, cv, v, &mesh_vertices);
      break;
  }

  return {std::move(mesh_elements), std::move(mesh_vertices)};
}

template VolumeMesh<double> MakeCylinderVolumeMeshWithMa(
    const Cylinder&, double resolution_hint);
template VolumeMesh<AutoDiffXd> MakeCylinderVolumeMeshWithMa(
    const Cylinder&, double resolution_hint);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
