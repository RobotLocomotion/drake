#pragma once

#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/proximity/volume_mesh.h"

namespace drake {
namespace geometry {
namespace internal {

/// This class is meant to provide a number of methods to make meshes of
/// tetrahedra for the unit radius sphere.
/// We encapsulate these methods in a class to easily instantate this
/// functionality on different scalar types.
template <typename T>
class SphereMeshGenerator {
 public:
  /// This method implements a variant of the generator described in
  /// [Everett, 1997]. It is based on a recursive refinement of an initial
  /// (refinement_level = 0) coarse mesh representation of the unit sphere. The
  /// initial mesh discretizes an octahedron with its six vertices on the
  /// surface of the unit sphere and a seventh vertex at the origin to form a
  /// volume tessellation consisting of eight tetrahedra. At each refinement
  /// level each tetrahedron is split into eight new tetrahedra by splitting
  /// each edge in half. When splitting an edge formed by vertices on the
  /// surface of the sphere, the newly created vertex is projected back onto the
  /// surface of the sphere.
  /// See [Jakub Velímský, 2010] for additional implementation details and a
  /// series of very useful schematics.
  ///
  /// @throws std::exception if refinement_level is negative.
  ///
  /// [Everett, 1997]  Everett, M.E., 1997. A three-dimensional spherical mesh
  /// generator. Geophysical Journal International, 130(1), pp.193-200.
  /// [Jakub Velímský, 2010] GESTIKULATOR Generator of a tetrahedral mesh on a
  /// sphere. Department of Geophysics, Charles University in Prague.
  static VolumeMesh<T> MakeSphereMesh(int refinement_level);

 private:
  // Splits a tetrahedron defined by vertices A, B, C, and D
  // into 8 new tetrahedra. Vector is_boundary stores `true` at entries 0, 1, 2,
  // and 3 (corresponding to A, B, C, and D respectively) for those vertices
  // that lie on the surface of the unit sphere. If an edge is formed by two
  // vertices that lie on the sphere, the new vertex created by splitting this
  // edge in half is projected so that it lies on the surface of the sphere as
  // well.
  // The return is a pair with: 1) a VolumeMesh including the original and
  // new vertices, and 2) a vector of booleans that, similar to is_boundary,
  // stores `true` iff the corresponding vertex on the new mesh lies on the
  // surface of the sphere.
  static std::pair<VolumeMesh<T>, std::vector<bool>> RefineTetrahdron(
      const Vector3<T>& A, const Vector3<T>& B, const Vector3<T>& C,
      const Vector3<T>& D, const std::vector<bool>& is_boundary);

  // Makes the initial mesh for refinement_level = 0.
  // It creates an octahedron by placing its six vertices on the surface of the
  // unit sphere and an additional vertex at the origin. The volume is then
  // tessellated into eight tetrahedra.
  // The additional vector of booleans indicates `true` if the corresponding
  // vertex lies on the surface of the sphere.
  static std::pair<VolumeMesh<T>, std::vector<bool>> MakeSphereMeshLevel0();

  // Splits a mesh by calling RefineTetrahdron() on each tetrahedron of `mesh`.
  // `is_boundary` is a vector with as many entries as vertices in the mesh
  // indicating if the vertex at the same index lies on the boundary of the
  // sphere.
  static std::pair<VolumeMesh<T>, std::vector<bool>> RefineMesh(
      const VolumeMesh<T>& mesh, const std::vector<bool>& is_boundary);
};

}  // namespace internal
}  // namespace geometry
}  // namespace drake
