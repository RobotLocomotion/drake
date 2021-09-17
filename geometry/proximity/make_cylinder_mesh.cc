#include "drake/geometry/proximity/make_cylinder_mesh.h"

#include <cmath>

#include "drake/common/default_scalars.h"
#include "drake/geometry/proximity/meshing_utilities.h"
#include "drake/geometry/proximity/proximity_utilities.h"

namespace drake {
namespace geometry {
namespace internal {

namespace {

/* Generates tetrahedral elements of a long cylinder conforming to its medial
 axis. It assumes the mesh vertices on the surface of the cylinder was already
 generated on the bottom and the top circular rims of the cylinder together
 with the centers of the two circles. After adding two medial vertices inside
 the cylinder, it connects mesh vertices to form tetrahedral elements.

 @param[in] bottom_center  Index into mesh_vertices of the center of the
                           bottom circular rim of the cylinder.
 @param[in] bottom         A sequence of indices into mesh_vertices of the
                           points on the bottom circular rim of the cylinder
                           in the counterclockwise order around +Z axis of
                           rotation of the cylinder. The first index can be
                           anywhere on the circular rim.
 @param[in] top_center     Index into mesh_vertices of the center of the top
                           circular rim of the cylinder.
 @param[in] top            A sequence of indices into mesh_vertices of the
                           points on the top circular rim of the cylinder in
                           the same order as `bottom`.
 @param[in,out] mesh_vertices  As input, it contain vertices on the surface of
                               the cylinder. The two medial vertices of the
                               cylinder will be added to mesh_vertices as
                               output.
 @return  The generated tetrahedral elements. There is no guarantee how the
          elements are ordered.
*/
template <typename T>
std::vector<VolumeElement> CalcLongCylinderVolumeMeshWithMa(
    const Cylinder& cylinder, const int num_vertices_per_circle,
    const int bottom_center, const std::vector<int>& bottom,
    const int top_center, const std::vector<int>& top,
    std::vector<Vector3<T>>* mesh_vertices) {
  const double top_z = cylinder.length() / 2.;
  const double tolerance =
      DistanceToPointRelativeTolerance(std::min(top_z, cylinder.radius()));
  DRAKE_DEMAND(top_z - cylinder.radius() > tolerance);
  DRAKE_DEMAND(num_vertices_per_circle == static_cast<int>(bottom.size()));
  DRAKE_DEMAND(num_vertices_per_circle == static_cast<int>(top.size()));

  // These two mesh vertices will be at the two end points of the central
  // medial line segment.
  int medial[2];
  const double offset_distance = cylinder.radius();
  const double offset_top_z = top_z - offset_distance;
  const double offset_bottom_z = -offset_top_z;
  medial[0] = static_cast<int>(mesh_vertices->size());
  mesh_vertices->emplace_back(0, 0, offset_bottom_z);
  medial[1] = static_cast<int>(mesh_vertices->size());
  mesh_vertices->emplace_back(0, 0, offset_top_z);

  std::vector<VolumeElement> mesh_elements;
  // n = num_vertices_per_circle.
  // bottom cone: n tetrahedra.
  // top cone: n tetrahedra.
  // remaining volume: n (topological) triangular prisms => 3n tetrahedra.
  mesh_elements.reserve(5 * num_vertices_per_circle);

  //                +Z (axis of rotation)
  //                 ↑
  //                 |  ●t2
  //                 | /
  //                 |/
  //      t3●--------●tc------●t1
  //                /|                  t  = top
  //               / |                  tc = top_center
  //              ●t0|                  m  = medial vertices
  //                 |
  //                 ●m1
  //                 |
  //                 |
  //                 +--------------------+Y
  //                /|
  //               / |
  //              /  ●m0
  //             /   |
  //            /    |  ●b2             b  = bottom
  //          +X     | /                bc = bottom_center
  //                 |/
  //      b3●--------●bc------●b1
  //                /
  //               /
  //              ●b0

  // Add tetrahedra as we go around the circle.
  // i = (n-1) 0 1 2 ... (n-2)
  // j =   0   1 2 3 ... (n-1)
  int i = num_vertices_per_circle - 1;
  for (int j = 0; j < num_vertices_per_circle; ++j) {
    // The bottom cone (covering b0,b1,b2,b3,m0 in the picture above) is
    // discretized by a series of tetrahedra around the axis of rotation (at
    // bc,m0 in the picture above).
    mesh_elements.emplace_back(bottom_center, bottom[i], bottom[j], medial[0]);
    // The top cone (covering t0,t1,t2,t3,m0 in the picture above) is
    // discretized by a series of tetrahedra around the axis of rotation (at
    // tc,m1 in the picture above).
    mesh_elements.emplace_back(top_center, top[j], top[i], medial[1]);
    // The remaining volume is discretized by a series of topological
    // triangular prisms (e.g., topological prism m0,b0,b1,m1,t0,t1 in the
    // picture above) around the axis of rotation (at m0,m1 in the picture
    // above).
    Append(SplitTriangularPrismToTetrahedra(medial[0], bottom[i], bottom[j],
                                            medial[1], top[i], top[j]),
           &mesh_elements);
    i = j;
  }

  return mesh_elements;
}

/* Generates tetrahedral elements of a medium cylinder conforming to its medial
 axis. It assumes the mesh vertices on the surface of the cylinder were already
 generated on the bottom and the top circular rims of the cylinder together
 with the centers of the two circles. After adding the medial vertex at the
 center of the cylinder, it connects mesh vertices to form tetrahedral elements.

 @param[in] bottom_center  Index into mesh_vertices of the center of the
                           bottom circular rim of the cylinder.
 @param[in] bottom         A sequence of indices into mesh_vertices of the
                           points on the bottom circular rim of the cylinder
                           in the counterclockwise order around +Z axis of
                           rotation of the cylinder. The first index can be
                           anywhere on the circular rim.
 @param[in] top_center     Index into mesh_vertices of the center of the top
                           circular rim of the cylinder.
 @param[in] top            A sequence of indices into mesh_vertices of the
                           points on the top circular rim of the cylinder in
                           the same order as `bottom`.
 @param[in,out] mesh_vertices  As input, it contains vertices on the surface of
                               the cylinder. The medial vertex of the
                               cylinder will be added to mesh_vertices as
                               output.
 @return  The generated tetrahedral elements. There is no guarantee how the
          elements are ordered.
*/
template <typename T>
std::vector<VolumeElement> CalcMediumCylinderVolumeMeshWithMa(
    const Cylinder& cylinder, const int num_vertices_per_circle,
    const int bottom_center, const std::vector<int>& bottom,
    const int top_center, const std::vector<int>& top,
    std::vector<Vector3<T>>* mesh_vertices) {
  const double top_z = cylinder.length() / 2.;
  const double tolerance =
      DistanceToPointRelativeTolerance(std::min(top_z, cylinder.radius()));
  DRAKE_DEMAND(std::abs(top_z - cylinder.radius()) <= tolerance);
  DRAKE_DEMAND(num_vertices_per_circle == static_cast<int>(bottom.size()));
  DRAKE_DEMAND(num_vertices_per_circle == static_cast<int>(top.size()));

  // The central medial vertex is at the center of the medium cylinder.
  int medial = static_cast<int>(mesh_vertices->size());
  mesh_vertices->emplace_back(0, 0, 0);

  std::vector<VolumeElement> mesh_elements;
  // n = num_vertices_per_circle.
  // bottom cone: n tetrahedra.
  // top cone: n tetrahedra.
  // remaining volume: ring of n pyramids => 2n tetrahedra.
  mesh_elements.reserve(4 * num_vertices_per_circle);

  //
  //                +Z (axis of rotation)
  //                 ↑
  //                 |  ●t2
  //                 | /
  //                 |/
  //      t3●--------●tc------●t1
  //                /|                  t  = top
  //               / |                  tc = top_center
  //              ●t0|                  m  = medial vertex at origin.
  //                 |
  //                 ●m---------------+Y
  //                /|
  //               / |  ●b2             b  = bottom
  //              /  | /                bc = bottom_center
  //            +X   |/
  //      b3●--------●bc------●b1
  //                /
  //               /
  //              ●b0

  // Add tetrahedra as we go around the circle.
  // i = (n-1) 0 1 2 ... (n-2)
  // j =   0   1 2 3 ... (n-1)
  int i = num_vertices_per_circle - 1;
  for (int j = 0; j < num_vertices_per_circle; ++j) {
    // The bottom cone (covering b0,b1,b2,b3,m in the picture above) is
    // discretized by a series of tetrahedra (e.g., tetrahedron bc,b0,b1,m in
    // the picture) around the axis of rotation (at bc,m in the picture).
    mesh_elements.emplace_back(bottom_center, bottom[i], bottom[j], medial);
    // The top cone (covering t0,t1,t2,t3,m in the picture above) is
    // discretized by a series of tetrahedra (e.g., tetrahedron tc,t1,t0,m in
    // the picture) around the axis of rotation (at tc,m in the picture).
    mesh_elements.emplace_back(top_center, top[j], top[i], medial);
    // The remaining volume is discretized by a series of rectangular
    // pyramids (e.g., pyramid t0,t1,b1,b0,m in the picture above) around the
    // axis of rotation. These pyramids share the apex at the medial vertex
    // (m in the picture).
    Append(
        SplitPyramidToTetrahedra(top[i], top[j], bottom[j], bottom[i], medial),
        &mesh_elements);
    i = j;
  }
  return mesh_elements;
}

/* Generates tetrahedral elements of a short cylinder conforming to its medial
 axis. It assumes the mesh vertices on the surface of the cylinder was already
 generated on the bottom and the top circular rims of the cylinder together
 with the centers of the two circles. After adding vertices on the medial
 circular disk of the cylinder, it connects mesh vertices to form
 tetrahedral elements.

 @param[in] bottom_center  Index into mesh_vertices of the center of the
                           bottom circular rim of the cylinder.
 @param[in] bottom         A sequence of indices into mesh_vertices of the
                           points on the bottom circular rim of the cylinder
                           in the counterclockwise order around +Z axis of
                           rotation of the cylinder. The first index can be
                           anywhere on the circular rim.
 @param[in] top_center     Index into mesh_vertices of the center of the top
                           circular rim of the cylinder.
 @param[in] top            A sequence of indices into mesh_vertices of the
                           points on the top circular rim of the cylinder in
                           the same order as `bottom`.
 @param[in,out] mesh_vertices  As input, it contains vertices on the surface of
                               the cylinder. The vertices on the medial
                               circular disk of the cylinder will be added to
                               mesh_vertices as output.
 @return  The generated tetrahedral elements. There is no guarantee how the
          elements are ordered.
*/
template <typename T>
std::vector<VolumeElement> CalcShortCylinderVolumeMeshWithMa(
    const Cylinder& cylinder, const int num_vertices_per_circle,
    const int bottom_center, const std::vector<int>& bottom,
    const int top_center, const std::vector<int>& top,
    std::vector<Vector3<T>>* mesh_vertices) {
  const double top_z = cylinder.length() / 2.;
  const double tolerance =
      DistanceToPointRelativeTolerance(std::min(top_z, cylinder.radius()));
  DRAKE_DEMAND(cylinder.radius() - top_z > tolerance);
  DRAKE_DEMAND(num_vertices_per_circle == static_cast<int>(bottom.size()));
  DRAKE_DEMAND(num_vertices_per_circle == static_cast<int>(top.size()));

  // For convenience in generating the mesh, we add one vertex at the center
  // of the cylinder, which is also the center of the medial circular disk.
  // This vertex helps us make the mesh symmetric around the rotation axis of
  // the cylinder. It is possible to skip this center point, and we
  // will decrease the number of vertices and tetrahedra slightly, but the
  // mesh will not be symmetric around the rotation axis, and the code will
  // become more complicated.
  int center = static_cast<int>(mesh_vertices->size());
  mesh_vertices->emplace_back(0, 0, 0);

  // Vertices along the medial circle.
  std::vector<int> medial(num_vertices_per_circle);
  const double medial_radius = cylinder.radius() - cylinder.length() / 2.;
  const double scale_cylinder_radius_to_medial_circle =
      medial_radius / cylinder.radius();
  for (int i = 0; i < num_vertices_per_circle; ++i) {
    const double x =
        ExtractDoubleOrThrow(mesh_vertices->at(bottom[i]).x()) *
        scale_cylinder_radius_to_medial_circle;
    const double y =
        ExtractDoubleOrThrow(mesh_vertices->at(bottom[i]).y()) *
        scale_cylinder_radius_to_medial_circle;
    medial[i] = static_cast<int>(mesh_vertices->size());
    mesh_vertices->emplace_back(x, y, 0);
  }

  std::vector<VolumeElement> mesh_elements;
  // n = num_vertices_per_circle.
  // bottom frustum: n triangular prisms => 3n tetrahedra.
  // top frustum: n triangular prisms => 3n tetrahedra.
  // remaining volume: ring of n triangular prisms => 3n tetrahedra.
  mesh_elements.reserve(9 * num_vertices_per_circle);

  //                  +Z (axis of rotation)
  //                   ↑
  //                   |       ●t2
  //                   |     ↗
  //                   |   ↗                t  = top
  //                   | ↗                  tc = top_center
  //   t3● ←  ←  ←  ←  ●tc→  →  →  → ●t1    m  = vertices on medial circle
  //                 ↙ |
  //               ↙   |                   +Z
  //             ↙     |   ●m2              ↑
  //         t0●       | ↗                  + → +Y
  //            m3● ←  ●c → ●m1           ↙
  //                 ↙ |       ●b2     +X   c  = `center` is at the origin.
  //               ●m0 |     ↗
  //                   |   ↗                b  = bottom
  //                   | ↗                  bc = bottom_center
  //   b3● ←  ←  ←  ←  ●bc→  →  →  → ●b1
  //                 ↙
  //               ↙
  //             ↙
  //         b0●
  //

  // Add tetrahedra as we go around the circle.
  // i = (n-1) 0 1 2 ... (n-2)
  // j =   0   1 2 3 ... (n-1)
  int i = num_vertices_per_circle - 1;
  for (int j = 0; j < num_vertices_per_circle; ++j) {
    // The bottom circular frustum (covering b0,b1,b2,b3,m0,m1,m2,m3 in the
    // picture above) is discretized by a series of topological triangular
    // prisms around the axis of rotation (at c,bc in the picture).
    Append(SplitTriangularPrismToTetrahedra(bottom_center, bottom[i], bottom[j],
                                            center, medial[i], medial[j]),
           &mesh_elements);
    // The top circular frustum (covering t0,t1,t2,t3,m0,m1,m2,m3 in the
    // picture above) is discretized by a series of topological triangular
    // prisms around the axis of rotation (at c,tc in the picture).
    Append(SplitTriangularPrismToTetrahedra(center, medial[i], medial[j],
                                            top_center, top[i], top[j]),
           &mesh_elements);
    // The remaining volume of the cylinder is discretized by a series of
    // topological triangular prisms with their triangular faces (b_i,m_i,t_i,
    // i=0,1,2,3 in the picture above) circling around the axis of rotation.
    Append(SplitTriangularPrismToTetrahedra(bottom[i], medial[i], top[i],
                                            bottom[j], medial[j], top[j]),
           &mesh_elements);
    i = j;
  }

  return mesh_elements;
}

}  // namespace

template <typename T>
VolumeMesh<T> MakeCylinderVolumeMeshWithMa(const Cylinder& cylinder,
                                           const double resolution_hint) {
  DRAKE_DEMAND(resolution_hint > 0.0);
  const double top_z = cylinder.length() / 2.;
  const double bottom_z = -top_z;
  enum class CylinderClass { kLong, kMedium, kShort };
  // This is just a heuristic to make the numerical decision below.
  const double smaller_measure = std::min(top_z, cylinder.radius());
  const double tolerance = DistanceToPointRelativeTolerance(smaller_measure);
  CylinderClass cylinder_class = CylinderClass::kMedium;
  if (top_z - cylinder.radius() > tolerance)
    cylinder_class = CylinderClass::kLong;
  else if (cylinder.radius() - top_z > tolerance)
    cylinder_class = CylinderClass::kShort;

  // At the minimum 3 vertices per circular rims of the cylinder, the mesh
  // will cover a triangular prism inside the cylinder. The larger value
  // of num_vertices_per_circle will make the mesh approximate the cylinder
  // better.
  const int num_vertices_per_circle = std::max(
      3,
      static_cast<int>(ceil(2. * M_PI * cylinder.radius() / resolution_hint)));

  std::vector<Vector3<T>> mesh_vertices;
  switch (cylinder_class) {
    case CylinderClass::kLong:
      mesh_vertices.reserve(2 * num_vertices_per_circle + 4);
      break;
    case CylinderClass::kMedium:
      mesh_vertices.reserve(2 * num_vertices_per_circle + 3);
      break;
    case CylinderClass::kShort:
      mesh_vertices.reserve(3 * num_vertices_per_circle + 3);
      break;
  }

  // For convenience in generating the mesh, we always add one vertex at the
  // center of each of the bottom and the top circular caps of the cylinder.
  // These two vertices help us make the mesh symmetric around the rotation
  // axis of the cylinder. It is possible to skip these two points, and we
  // will decrease the number of vertices and tetrahedra slightly, but the
  // mesh will not be symmetric around the rotation axis, and the code will
  // become more complicated.
  int bottom_center = static_cast<int>(mesh_vertices.size());
  mesh_vertices.emplace_back(0, 0, bottom_z);
  int top_center = static_cast<int>(mesh_vertices.size());
  mesh_vertices.emplace_back(0, 0, top_z);

  // Vertices on the two circular rims of the cylinder.
  std::vector<int> bottom(num_vertices_per_circle);
  std::vector<int> top(num_vertices_per_circle);
  const double angle_step = 2. * M_PI / num_vertices_per_circle;
  for (int i = 0; i < num_vertices_per_circle; ++i) {
    const double x = cylinder.radius() * cos(angle_step * i);
    const double y = cylinder.radius() * sin(angle_step * i);
    bottom[i] = static_cast<int>(mesh_vertices.size());
    mesh_vertices.emplace_back(x, y, bottom_z);
    top[i] = static_cast<int>(mesh_vertices.size());
    mesh_vertices.emplace_back(x, y, top_z);
  }

  std::vector<VolumeElement> mesh_elements;
  switch (cylinder_class) {
    case CylinderClass::kLong:
      mesh_elements = CalcLongCylinderVolumeMeshWithMa<T>(
          cylinder, num_vertices_per_circle, bottom_center, bottom, top_center,
          top, &mesh_vertices);
      break;
    case CylinderClass::kMedium:
      mesh_elements = CalcMediumCylinderVolumeMeshWithMa<T>(
          cylinder, num_vertices_per_circle, bottom_center, bottom, top_center,
          top, &mesh_vertices);
      break;
    case CylinderClass::kShort:
      mesh_elements = CalcShortCylinderVolumeMeshWithMa<T>(
          cylinder, num_vertices_per_circle, bottom_center, bottom, top_center,
          top, &mesh_vertices);
      break;
  }

  return {std::move(mesh_elements), std::move(mesh_vertices)};
}

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS((
    &MakeCylinderVolumeMeshWithMa<T>
))

}  // namespace internal
}  // namespace geometry
}  // namespace drake
