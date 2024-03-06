#include "drake/geometry/proximity/polygon_to_triangle_mesh.h"

#include <utility>
#include <vector>

#include "drake/common/eigen_types.h"

namespace drake {
namespace geometry {
namespace internal {

using Eigen::Vector3d;

TriangleSurfaceMesh<double> MakeTriangleFromPolygonMesh(
    const PolygonSurfaceMesh<double>& poly_mesh) {
  std::vector<Vector3<double>> vertices;
  vertices.reserve(poly_mesh.num_vertices());
  for (int v = 0; v < poly_mesh.num_vertices(); ++v) {
    vertices.push_back(poly_mesh.vertex(v));
  }

  std::vector<SurfaceTriangle> tris;
  // According to Euler's formula, the maximum number of possible triangles.
  tris.reserve(2 * poly_mesh.num_vertices() - 4);
  for (int p = 0; p < poly_mesh.num_faces(); ++p) {
    // Create a triangle fan around vertex 0.
    const SurfacePolygon& poly = poly_mesh.element(p);
    const int v0 = poly.vertex(0);
    int v1 = poly.vertex(1);
    for (int v = 2; v < poly.num_vertices(); ++v) {
      const int v2 = poly.vertex(v);
      tris.emplace_back(v0, v1, v2);
      v1 = v2;
    }
  }

  return TriangleSurfaceMesh<double>(std::move(tris), std::move(vertices));
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
