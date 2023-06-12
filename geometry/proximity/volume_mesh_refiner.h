#pragma once

#include <vector>

#include "drake/common/sorted_pair.h"
#include "drake/geometry/proximity/sorted_triplet.h"
#include "drake/geometry/proximity/volume_mesh.h"

namespace drake {
namespace geometry {
namespace internal {

// (Advanced) %VolumeMeshRefiner creates a refined version of the input
// tetrahedral mesh to eliminate problematic simplices that evaluate to zero
// signed-distance in its interior.  See detect_null_simplex.h for the
// definition of a problematic simplex. */
class VolumeMeshRefiner {
 public:
  // @note Assume the lifetime of input_mesh covers the lifetime of the
  // instance of this class.
  //
  // @note If the input_mesh changes its content, you can call Refine()
  // again, and it will start from the new content of input_mesh.
  explicit VolumeMeshRefiner(const VolumeMesh<double>& input_mesh);

  // Performs mesh-refinement algorithm and returns the refined mesh.
  // A problematic tetrahedra is replaced by four new tetrahedra (1:4 ratio).
  // Two tetrahedra sharing a problematic triangle are replaced by six new
  // tetrahedra (1:3 ratio).
  // The n tetrahedra sharing a problematic edge are repalced by 2n new
  // tetrahedra (1:2 ratio).
  //
  // In the future, we will also consider topological flips in addition to
  // refinement-based algorithms above. It will give more economical output.
  // For example, flipping two tetrahedra sharing a triangle to three
  // tetrahedra that share an edge (2:3 ratio) or the opposite operation of
  // flipping three tetrahedra sharing an edge to two tetrahedra that share a
  // triangle (3:2 ratio), which will reduce the number of tetrahedra.
  //
  // @note Depending on how many problematic simplices the input mesh has,
  // the refined mesh can have as many as 4 times tetrahedra of the input
  // mesh.
  VolumeMesh<double> Refine();

 private:
  // @section Refine a simplex and its incident simplices in the mesh.

  void RefineTetrahedron(int tetrahedron);
  void RefineTriangle(const SortedTriplet<int>& triangle);
  void RefineEdge(const SortedPair<int>& edge);

  // @section Helpers of Refine* functions.

  void ReplaceOneTetrahedronWithFour(int tetrahedron, int new_vertex);
  void ReplaceOneTetrahedronWithThree(int tetrahedron,
                                      const SortedTriplet<int>& triangle,
                                      int new_vertex);
  void ReplaceOneTetrahedronWithTwo(int tetrahedron,
                                    const SortedPair<int>& edge,
                                    int new_vertex);

  // @section Global incident query in the mesh

  // TODO(DamrongGuoy): Consider a more efficient data structure for these
  //  queries and update the @note.

  // @returns indices of tetrahedra sharing the triangle v0v1v2.
  //
  // @note It has linear complexity on the number of tetrahedra.
  std::vector<int> GetTetrahedraOnTriangle(int v0, int v1, int v2) const;

  // @returns indices of tetrahedra sharing the edge v0v1.
  //
  // @note It has linear complexity on the number of tetrahedra.
  std::vector<int> GetTetrahedraOnEdge(int v0, int v1) const;

  // @section Local incident query in a tetrahedron

  // @returns true if the triangle v0v1v2 is a face of the tetrahedron.
  bool IsTriangleInTetrahedron(int v0, int v1, int v2, int tetrahedron) const;

  // @returns true if the edge v0,v1 is an edge of the tetrahedron.
  bool IsEdgeInTetrahedron(int v0, int v1, int tetrahedron) const;

  // @section Aggregrated data

  // As we incrementally Refine the mesh, we collect tetrahedra and
  // vertices into these variables. They are initialized by the input mesh.
  // They grow as we Refine the mesh steps by steps.
  std::vector<VolumeElement> tetrahedra_{};
  std::vector<Vector3<double>> vertices_{};

  // @section Reference to the input mesh

  // It must be valid during the lifetime of this class.
  const VolumeMesh<double>& input_mesh_;
};

}  // namespace internal
}  // namespace geometry
}  // namespace drake
