#pragma once

#include <array>
#include <memory>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/type_safe_index.h"
#include "drake/geometry/proximity/surface_mesh.h"
#include "drake/geometry/proximity/volume_mesh.h"
#include "drake/geometry/query_results/contact_surface.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
namespace internal {

// We place static constants in a separate struct to avoid having them templated
// on <T>.
struct ContactSurfaceCalculatorConstants {
  using Edge = std::pair<int, int>;
  using EdgeIndex = int;

  // Marching tetrahedra table.
  // We encode the table indexes in binary so that a "1" corresponds to a
  // positive value at that vertex and conversely "0" corresponds to a
  // negative value.
  // As we have only 4 points, we have 16 cases to consider (positive or
  // negative values at each vertex).
  static const std::array<std::vector<EdgeIndex>, 16> kMarchingTetsTable;

  // The six edges of a tetrahedra
  static const std::array<Edge, 6> kEdges;
};

/// This class provides a number of methods to compute contact surfaces between
/// two solids to use with the Hydroelastic model.
/// We place them within a class to aid the explicit instantiation for different
/// scalar types.
template <typename T>
class ContactSurfaceCalculator {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ContactSurfaceCalculator)

  ContactSurfaceCalculator() = default;

  /// Given a level set function `φ(V)` and a volume defined by `mesh_M`, this
  /// method computes a triangulation of the zero level set `φ(V)` in the volume
  /// defined by the `mesh_M`.
  /// @param mesh_M
  ///   Defines the volume M within which the zero level of `φ(V)` is found.
  /// @param phi_N
  ///   The level set function `φ(V)` represented as a `std::function` phi_N,
  ///   which takes the position of point V expressed in a frame N. That is, in
  ///   code, φ(V) ≡ phi_N(p_NV).
  /// @param X_NM
  ///   The pose of M in N.
  ///
  /// @note The resolution of the zero level set triangulation depends on the
  /// resolution of the initial volume mesh. That is, if we denote with h the
  /// typical length scale of a tetrahedral element, then the surface
  /// triangulation will also have triangular elements of size in the order of
  /// h. Thus, features in the level set function with a length scale smaller
  /// than h will not be properly captured by the triangulation.
  ///
  /// @note This implementation uses the marching tetrahedra algorithm as
  /// described in [Bloomenthal, 1994].
  ///
  /// @returns A triangulation of the zero level set of `φ(V)` in the volume
  /// defined by the `mesh_M`.  The triangulation is expressed in frame N.
  ///
  /// @note  The SurfaceMesh may have duplicated vertices.
  ///
  /// Bloomenthal, J., 1994. An Implicit Surface Polygonizer. Graphics Gems IV,
  /// pp. 324-349.
  static SurfaceMesh<T> CalcZeroLevelSetInMeshDomain(
      const VolumeMesh<T>& mesh_M, std::function<T(const Vector3<T>&)> phi_N,
      const math::RigidTransform<T>& X_NM);

 private:
  using Edge = std::pair<int, int>;
  using EdgeIndex = int;

  // Friend declaration so that the internals of this class can be confirmed in
  // unit tests.
  template <class U>
  friend class ContactSurfaceCalculatorTester;

  // The first three "vertices" define the first face of the tetrahedra, with
  // its right-handed normal pointing towards the outside.
  // The last vertex is on the "negative side" of the first face.
  // On return this method adds the new vertices and faces into `vertices` and
  // `faces` respectively and returns the number of vertices added.
  //
  // @param tet_vertices_N Vertices defining the tetrahedron. The first three
  // vertices define the first face of the tetrahedra, with its right-handed
  // normal pointing towards the outside. The last vertex is on the "negative
  // side" of the first face.
  // @param phi_N The level set function evaluated at each of the four vertices
  // in `tet_vertices_N`, in the same order.
  // @retval vertices Adds the new vertices into `vertices`.
  // @retval faces Adds the new faces into `faces`.
  // @return The number of vertices added.
  // @note   The convention used by this private method is different from the
  // one used in VolumeMesh.
  static int IntersectTetWithLevelSet(
      const std::vector<Vector3<T>>& tet_vertices_N, const Vector4<T>& phi_N,
      std::vector<SurfaceVertex<T>>* vertices, std::vector<SurfaceFace>* faces);
};

}  // namespace internal.
}  // namespace geometry.
}  // namespace drake.
