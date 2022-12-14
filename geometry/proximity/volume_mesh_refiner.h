#pragma once

#include <vector>

#include "drake/common/sorted_pair.h"
#include "drake/geometry/proximity/sorted_triplet.h"
#include "drake/geometry/proximity/volume_mesh.h"

namespace drake {
namespace geometry {
namespace internal {

// (Advanced) %VolumeMeshRefiner refines the input tetrahedral mesh to
// eliminate problematic simplices that evaluate to zero signed-distance in
// its interior.  See detect_zero_simplex.h for the definition of a
// problematic simplex.
class VolumeMeshRefiner {
 public:
  // @pre Assume input_mesh is valid during the lifetime of this class.
  explicit VolumeMeshRefiner(const VolumeMesh<double>& input_mesh)
      : input_mesh_(input_mesh) {}

  // Performs mesh-refinement algorithm and returns the new refined mesh.
  //
  // @note Depending on how many problematic simplices the input mesh has,
  // the output may have four or five times tetrahedra of the input
  // mesh.
  //
  // @note If the input mesh has no problematic simplices, the returned mesh
  // is a copy of the input.
  VolumeMesh<double> Refine();

 private:
  // Refines a tetrahedron into four tetrahedra.
  void RefineTetrahedron(int tetrahedron);

  // Refines an interior triangle and its two incident tetrahedra into six
  // tetrahedra. One old tetrahedron becomes three new tetrahedra.
  void RefineTriangle(const SortedTriplet<int>& triangle);

  // Refines an interior edge and its n incident tetrahedra into 2n tetrahedra.
  // One old tetrahedron becomes two new tetrahedra.
  void RefineEdge(const SortedPair<int>& edge);

  // Helper of RefineTetrahedron(), RefineTriangle(), and RefineEdge().
  // It cuts a tetrahedron into smaller ones by inserting `new_vertex`
  // in the middle of one of its sub-simplex, which could be its edge,
  // its triangle, or the tetrahedron itself.
  //
  // @param tetrahedron  Tetrahedron index in the mesh.
  //
  // @param sub_simplex  A list of two integers for an edge, three
  // integers for a triangle, or four integers for a tetrahedron. Each integer
  // is a vertex index into the mesh.
  //
  // @param new_vertex  Index of the new vertex for insertion into the
  //                    sub-simplex.
  //
  // @note This operation is local to the specified tetrahedron. It does not
  // guarantee conformity with neighboring tetrahedra. Callers are responsible
  // to call this function on adjacent tetrahedra to maintain conformity.
  //
  // @pre 2 <= sub_simplex.size() <= 4
  // @pre 0 <= sub_simplex[i] < vertices_.size()
  // @pre sub_simplex[i] != sub_simplex[j] for i != j
  // @pre sub_simplex[i] is a vertex of the tetrahedron.
  void CutTetrahedron(int tetrahedron, const std::vector<int>& sub_simplex,
                      int new_vertex);

  // TODO(DamrongGuoy): Consider more efficient data structures for these
  //  global incident queries.

  // @returns indices of tetrahedra sharing the triangle v0,v1,v2.
  // @note It has linear complexity on the number of tetrahedra.
  // @pre 0 <= v0, v1, v2 < vertices_.size()
  std::vector<int> GetTetrahedraOnTriangle(int v0, int v1, int v2) const;

  // @returns indices of tetrahedra sharing the edge v0,v1.
  // @note It has linear complexity on the number of tetrahedra.
  // @pre 0 <= v0, v1 < vertices_.size()
  std::vector<int> GetTetrahedraOnEdge(int v0, int v1) const;

  // As we incrementally refine the mesh, we collect tetrahedra and
  // vertices into these variables.
  std::vector<VolumeElement> tetrahedra_{};
  std::vector<Vector3<double>> vertices_{};
  // Reference to the input mesh must be valid during the lifetime of this
  // object.
  const VolumeMesh<double>& input_mesh_;
};

}  // namespace internal
}  // namespace geometry
}  // namespace drake
