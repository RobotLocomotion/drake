#include "drake/geometry/proximity/meshing_utilities.h"

#include <unordered_set>

namespace drake {
namespace geometry {
namespace internal {

using Eigen::Vector3d;

void Append(const std::vector<VolumeElement>& new_elements,
            std::vector<VolumeElement>* mesh_elements) {
  mesh_elements->insert(mesh_elements->end(), new_elements.begin(),
                        new_elements.end());
}

using std::unordered_set;

std::vector<VolumeElement> SplitTriangularPrismToTetrahedra(int v0, int v1,
                                                            int v2, int v3,
                                                            int v4, int v5) {
  std::vector<VolumeElement> elements;
  elements.reserve(3);
  int previous = v3;
  for (int next : {v4, v1, v2}) {
    elements.emplace_back(previous, next, v0, v5);
    previous = next;
  }
  return elements;
}

std::vector<VolumeElement> SplitPyramidToTetrahedra(int v0, int v1, int v2,
                                                    int v3, int v4) {
  std::vector<VolumeElement> elements;
  elements.reserve(2);
  int previous = v3;
  for (int next : {v4, v1}) {
    elements.emplace_back(previous, next, v0, v2);
    previous = next;
  }
  return elements;
}

double CalcEnclosedVolume(const TriangleSurfaceMesh<double>& surface_mesh) {
  double six_total_volume = 0;

  // For convenience we tetrahedralize each triangle (p,q,r) about the origin o
  // = (0,0,0) in the mesh's frame. We assume that p,q,r is wound with outward
  // facing normals. The choice of o is arbitrary and need not be on the
  // interior of surface_mesh. For efficiency we compute 6*Vi for the signed
  // volume Vi of the ith element and 6*V for the signed volume V of the
  // region.
  for (const auto& tri : surface_mesh.triangles()) {
    const Vector3d& p = surface_mesh.vertex(tri.vertex(0));
    const Vector3d& q = surface_mesh.vertex(tri.vertex(1));
    const Vector3d& r = surface_mesh.vertex(tri.vertex(2));

    // 6 * signed volume of (p,q,r,o)
    const double six_volume = (p).cross(q).dot(r);
    six_total_volume += six_volume;
  }

  return six_total_volume / 6;
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
