#pragma once

#include <string>
#include <vector>

#include "drake/common/sorted_pair.h"
#include "drake/geometry/mesh_source.h"
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
  // the output may have four or five times the number of tetrahedra of
  // the input mesh.
  //
  // @note If the input mesh has no problematic simplices, the returned mesh
  // is a copy of the input.
  VolumeMesh<double> Refine();

 private:
  // Refines a tetrahedron into four tetrahedra.
  //
  // @param tetrahedron  Tetrahedron index into the mesh.
  void RefineTetrahedron(int tetrahedron);

  // Refines an interior triangle and its two incident tetrahedra into six
  // tetrahedra. Each of the two old tetrahedra becomes three new tetrahedra.
  //
  // @param triangle   Three vertex indices of the triangle.
  void RefineTriangle(const SortedTriplet<int>& triangle);

  // Refines an interior edge and its n incident tetrahedra into 2n tetrahedra.
  // Each of the n old tetrahedron becomes two new tetrahedra.
  //
  // @param edge  Two vertex indices of the edge.
  void RefineEdge(const SortedPair<int>& edge);

  // Helper of RefineTetrahedron(), RefineTriangle(), and RefineEdge().
  // It cuts the given `tetrahedron` into smaller ones by inserting the given
  // `new_vertex` into the interior of its `sub_simplex`, which could be its
  // edge, its triangle, or the tetrahedron itself.
  //
  // @param tetrahedron  Tetrahedron index into the mesh.
  // @pre 0 <= tetrahedron < tetrahedra_.size()
  //
  // @param sub_simplex  A list of two integers for an edge, three
  // integers for a triangle, or four integers for a tetrahedron. Each integer
  // is a vertex index into the mesh.
  // @pre 2 <= sub_simplex.size() <= 4
  // @pre 0 <= sub_simplex[i] < vertices_.size()
  // @pre sub_simplex[i] != sub_simplex[j] for i != j
  // @pre sub_simplex[i] is a vertex of the tetrahedron.
  //
  // @param new_vertex  Index of the new vertex for insertion into the
  //                    sub-simplex of the tetrahedron.
  // @pre 0 <= new_vertex < vertices_.size()
  // @pre The new vertex is in the relative interior of the sub-simplex.
  //
  // This function preserves tetrahedron orientation. The sign of the signed
  // volume of each of the new tetrahedra is the same as the sign of the
  // signed volume of the old tetrahedron.
  //
  // @note This operation is local to the specified tetrahedron. It does not
  // guarantee conformity with neighboring tetrahedra. Callers are responsible
  // to call this function on adjacent tetrahedra to maintain conformity.
  void CutTetrahedron(int tetrahedron, const std::vector<int>& sub_simplex,
                      int new_vertex);

  // TODO(DamrongGuoy): Consider more efficient data structures for these
  //  two queries.

  // Returns indices of tetrahedra sharing the given triangle. If there is no
  // such tetrahedra, return an empty list.
  //
  // @param v0,v1,v2   Three unique vertex indices of the triangle.
  // @pre 0 <= v0, v1, v2 < vertices_.size()
  // @pre v0 != v1 && v1 != v2 && v2 != v0
  //
  // @note  This function has linear complexity in the number of tetrahedra.
  std::vector<int> GetTetrahedraOnTriangle(int v0, int v1, int v2) const;

  // Returns indices of tetrahedra sharing the given edge. If there is no
  // such tetrahedra, return an empty list.
  //
  // @param v0,v1   Two unique vertex indices of the edge.
  // @pre 0 <= v0, v1 < vertices_.size()
  // @pre v0 != v1
  //
  // @note This function has linear complexity on the number of tetrahedra.
  std::vector<int> GetTetrahedraOnEdge(int v0, int v1) const;

  // Initializes vertex_to_tetrahedra_ from tetrahedra_;
  void ResetVertexToTetrahedra();

  // As we incrementally refine the mesh, we collect tetrahedra and
  // vertices into these variables.
  std::vector<VolumeElement> tetrahedra_{};
  std::vector<Vector3<double>> vertices_{};
  // For quick local searches in GetTetrahedraOnTriangle() and
  // GetTetrahedraOnEdge() (as opposed to global search the entire mesh),
  // we maintain vertex_to_tetrahedra_, which is the reverse of tetrahedra_:
  //   tetrahedra_: tetrahedron -> 4 vertices,
  //   vertex_to_tetrahedra_: vertex -> incident tetrahedra.
  // Invariants:
  // 1. vertex_to_tetrahedra_.size() == vertices_.size().
  // 2. sum vertex_to_tetrahedra_[i].size() == 4 * tetrahedra_.size()
  std::vector<std::vector<int>> vertex_to_tetrahedra_{};

  // Reference to the input mesh must be valid during the lifetime of this
  // object.
  const VolumeMesh<double>& input_mesh_;
};

}  // namespace internal

/** Refines a tetrahedral mesh to eliminate problematic simplices.
@param mesh  The mesh to refine.
@return      The refined mesh, or a copy of the input mesh if no refinement
             was needed.
@throws std::exception if `mesh` is not a valid tetrahedral mesh. */
VolumeMesh<double> RefineVolumeMesh(const VolumeMesh<double>& mesh);

/** Refines a tetrahedral mesh to eliminate problematic simplices.
@param mesh_source  The mesh to refine.
@return             A valid, ASCII VTK file defining the refined tetrahedral
                    mesh. Represents the input mesh if no refinement was needed.
@throws std::exception if `mesh_source` does not reference a valid VTK-formatted
                       tetrahedral mesh. */
std::string RefineVolumeMeshIntoVtkFileContents(const MeshSource& mesh_source);

}  // namespace geometry
}  // namespace drake
