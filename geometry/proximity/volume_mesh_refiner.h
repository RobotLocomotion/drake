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
// signed-distance in its interior.  See detect_zero_simplex.h for the
// definition of a problematic simplex.
class VolumeMeshRefiner {
 public:
  // @note Assume input_mesh is valid during the lifetime of this class.
  explicit VolumeMeshRefiner(const VolumeMesh<double>& input_mesh);

  // Performs mesh-refinement algorithm and returns the new refined mesh.
  //
  // @note Depending on how many problematic simplices the input mesh has,
  // the refined mesh can have as many as 4 times tetrahedra of the input
  // mesh.
  VolumeMesh<double> Refine();

 private:
  // @section Refine A Simplex and Incident Tetrahedra

  // Refines a tetrahedron into four tetrahedra. The ratio of number of
  // old tetrahedron and new tetrahedra is 1:4.
  void RefineTetrahedron(int tetrahedron);
  // Refines an interior triangle and its two incident tetrahedra into six
  // tetrahedra. The ratio of number of old tetrahedra and new tetrahedra is
  // 1:3.
  void RefineTriangle(const SortedTriplet<int>& triangle);
  // Refines an interior edge and its n incident tetrahedra into 2n tetrahedra.
  // The ratio of number of old tetrahedra and new tetrahedra is 1:2.
  void RefineEdge(const SortedPair<int>& edge);

  // Helper of RefineTetrahedron(), RefineTriangle(), and RefineEdge().
  // It cuts a tetrahedron into smaller ones by inserting a new vertex
  // into one of its sub-simplex (edge, triangle, or the tetrahedron itself).
  //
  // @note This operation is local to the specified tetrahedron. It does not
  // guarantee conformity with neighboring tetrahedra. Callers are
  // responsible, if needed, to call this function on adjacent tetrahedra to
  // maintain conformity.
  //
  // @pre 2 <= sub_simplex.size() <= 4
  // @pre 0 <= sub_simplex[i] < vertices_.size()
  // @pre sub_simplex[i] != sub_simplex[j] for i != j
  // @pre sub_simplex[i] is a vertex of the tetrahedron's vertices.
  void CutTetrahedron(int tetrahedron, std::vector<int> sub_simplex,
                      int new_vertex);

  // @section Global Incident Query

  // TODO(DamrongGuoy): Consider more efficient data structures for these
  //  global incident queries and update their @note.

  // @returns indices of tetrahedra sharing the triangle v0v1v2.
  //
  // @note It has linear complexity on the number of tetrahedra.
  //
  // @pre 0 <= v0, v1, v2 < vertices_.size()
  std::vector<int> GetTetrahedraOnTriangle(int v0, int v1, int v2) const;

  // @returns indices of tetrahedra sharing the edge v0v1.
  //
  // @note It has linear complexity on the number of tetrahedra.
  //
  // @pre 0 <= v0, v1 < vertices_.size()
  std::vector<int> GetTetrahedraOnEdge(int v0, int v1) const;

  // @section Local Incident Query in a Tetrahedron

  // @returns true if the triangle v0v1v2 is a face of the tetrahedron.
  //
  // @pre 0 <= v0, v1, v2 < vertices_.size()
  bool IsTriangleInTetrahedron(int v0, int v1, int v2, int tetrahedron) const;

  // @returns true if the edge v0v1 is an edge of the tetrahedron.
  //
  // @pre 0 <= v0, v1 < vertices_.size()
  bool IsEdgeInTetrahedron(int v0, int v1, int tetrahedron) const;

  // @section Aggregated Data

  // As we incrementally Refine the mesh, we collect tetrahedra and
  // vertices into these variables. They are initialized by the input mesh.
  // They grow as we refine the mesh steps by steps.
  std::vector<VolumeElement> tetrahedra_{};
  std::vector<Vector3<double>> vertices_{};

  // @section Reference to the Input Mesh

  // It must be valid during the lifetime of this class.
  const VolumeMesh<double>& input_mesh_;
};

}  // namespace internal
}  // namespace geometry
}  // namespace drake
