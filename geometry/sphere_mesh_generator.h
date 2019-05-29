#pragma once

#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace geometry {
namespace internal {

/// This struct stores the minimum amount of information to represent a volume
/// of tetrahedra.
// TODO(amcastro-tri): move this to its own file so it can be used by mesh
// generators for other primitive shapes.
template <typename T>
struct TetrahedraMesh {
  /// The vertices of the entire volume mesh, expressed in a frame M of the
  /// mesh.
  std::vector<Vector3<T>> vertices;

  /// Entry `tetrahedra[t]` contains the four indexes into `vertices`
  /// forming the t-th tetrahedron.
  /// The convention is that the first three vertices (0, 1, 2) define the
  /// triangular "base" of the tetrahedra with its right-handed normal pointing
  /// towards the outside of the volume. The fourth vertex is on the negative
  /// side of the plane for on which triangle 0-1-2 lies.
  std::vector<Vector4<int>> tetrahedra;
};

/// This class is meant to provide a number of methods to make meshes of
/// tetrahedra for the unit radius sphere.
/// We encapsulate these methods in a class to easily instantate this
/// functionality on different scalar types.
template <typename T>
class SphereMeshGenerator {
 public:
  /// This method implements a variant of the generator described in
  /// [Everett, 1997]. It is based on a recursive refinement of an initial mesh
  /// (refinement_level = 0) for an octahedron six vertices on the surface of
  /// the unit sphere and a seventh vertex at the origin to form a tessellation
  /// consisting of eight tetrahedra. At each refinement level each tetrahedron
  /// is split into eight new tetrahedra by splitting each edge in half.
  /// This implementation deals with each tetrahedron independently of its
  /// neighbors and therefore common vertices are not shared among adjacent
  /// tetrahedra but are instead repeated.
  /// In the implementation we use the notation in [Jakub Velímský, 2010].
  ///
  /// [Everett, 1997]  Everett, M.E., 1997. A three-dimensional spherical mesh
  /// generator. Geophysical Journal International, 130(1), pp.193-200.
  /// [Jakub Velímský, 2010] GESTIKULATOR Generator of a tetrahedral mesh on a
  /// sphere. Department of Geophysics, Charles University in Prague.
  static TetrahedraMesh<T> MakeSphereMesh(int refinement_level);

 private:
  // Helper method to split a tetrahedron defined by vertices A, B, C, and D
  // into 8 new tetrahedra. Vector is_boundary stores `true` at entries 0, 1, 2,
  // and 3 (corresponding to A, B, C, and D respectively) for those vertices
  // that lie on the surface of the unit sphere. If an edge is formed by two
  // vertices that lie on the sphere, the new vertex created by splitting this
  // edge in half is projected so that it lies on the surface of the sphere as
  // well.
  // The return is a new TetrahedraMesh with all the vertices of the newly
  // tesselated volume of the original tetrahedron (with projection) and a
  // vector that, similarly to is_boundary, stores `true` iff the corresponding
  // vertex on the new mesh lies on the surface of the sphere.
  static std::pair<TetrahedraMesh<T>, std::vector<bool>> SplitTetrahedra(
      const Vector3<T>& A, const Vector3<T>& B, const Vector3<T>& C,
      const Vector3<T>& D, const std::vector<bool>& is_boundary);

  // Helper method to make the initial mesh for refinement_level = 0.
  // It creates an octahedron by placing its six vertices on the surface of the
  // unit sphere and an addition vertex at the origin. The volume is then
  // tessellated into eight tetrahedra.
  static TetrahedraMesh<T> MakeSphereMeshLevel0();

  // This helper methods calls SplitTetrahedra() on each tetrahedron of `mesh`.
  // `is_boundary` is a vector with as many vertices in the mesh indicating if
  // the vertex lies on the boundary of the unit sphere.
  static std::pair<TetrahedraMesh<T>, std::vector<bool>> SplitMesh(
      const TetrahedraMesh<T>& mesh, const std::vector<bool>& is_boundary);
};

}  // namespace internal
}  // namespace geometry
}  // namespace drake
