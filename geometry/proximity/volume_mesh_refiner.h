#pragma once

#include <vector>

#include "drake/common/sorted_pair.h"
#include "drake/geometry/proximity/sorted_triplet.h"
#include "drake/geometry/proximity/volume_mesh.h"

namespace drake {
namespace geometry {
namespace internal {

class VolumeMeshRefiner {
 public:
  explicit VolumeMeshRefiner(const VolumeMesh<double>& input_mesh);

  VolumeMesh<double> refine();

 private:
  void RefineTetrahedron(int tetrahedron);
  void RefineTriangle(const SortedTriplet<int>& triangle);
  void RefineEdge(const SortedPair<int>& edge);

  void ReplaceOneTetrahedronWithFour(int tetrahedron, int new_vertex);
  void ReplaceOneTetrahedronWithThree(int tetrahedron,
                                      const SortedTriplet<int>& triangle,
                                      int new_vertex);
  void ReplaceOneTetrahedronWithTwo(int tetrahedron,
                                    const SortedPair<int>& edge,
                                    int new_vertex);

  // @returns tetrahedron indices
  std::vector<int> GetTetrahedraOnTriangle(int v0, int v1, int v2) const;
  std::vector<int> GetTetrahedraOnEdge(int v0, int v1) const;

  bool IsTriangleInTetrahedron(int v0, int v1, int v2, int tetrahedron) const;
  bool IsEdgeInTetrahedron(int v0, int v1, int tetrahedron) const;

  VolumeMesh<double> CreateMesh(
      const std::vector<VolumeElement>& elements,
      const std::vector<Vector3<double>>& vertices) const;

  // As we refine the mesh, we collect tetrahedra and vertices in these two
  // members. They are initialized as the input mesh.
  std::vector<VolumeElement> tetrahedra_;
  std::vector<Vector3<double>> vertices_;

  const VolumeMesh<double>& input_mesh_;
};

}  // namespace internal
}  // namespace geometry
}  // namespace drake
