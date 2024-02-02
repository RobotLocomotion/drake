#pragma once

#include <utility>
#include <variant>
#include <vector>

#include "drake/common/ssize.h"
#include "drake/geometry/deformable_mesh.h"
#include "drake/geometry/proximity/triangle_surface_mesh.h"
#include "drake/geometry/proximity/volume_mesh.h"

namespace drake {
namespace geometry {
namespace internal {

/* Given a volume mesh (the control mesh) and a list of (passively driven)
 points embedded in the mesh, BarycentricInterpolator uses the vertex positions
 of the control mesh to compute the positions of the embedded points based on
 their barycentric coordinates in the tetrahedron containing them. */
class BarycentricInterpolator {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(BarycentricInterpolator)

  /* Constructs a BarycentricInterpolator.
   @param[in] positions     The positions of the passively driven points
                            embedded in the control mesh in the mesh frame.
   @param[in] control mesh  The volume mesh driving the embedded points.
   @throws std::exception if any passively driven point is outside of the
   control mesh. */
  BarycentricInterpolator(const std::vector<Vector3<double>>& positions,
                          const VolumeMesh<double>& control_mesh);

  /* Given the positions q of the vertices of the control mesh in some frame F,
   returns the positions of the passively driven points in the same frame F.
   Both the input and the returned value are ordered as flat Eigen vectors
   composing of (x₀, y₀, z₀, x₁, y₁, z₁, ...). The input is ordered the same as
   the vertices in the input control mesh. The output is ordered the same as the
   order of the positions of the points at construction.
   @pre q.size() is 3 times the number of vertices of the control mesh given at
   construction. */
  Eigen::VectorXd operator()(const Eigen::VectorXd& q) const;

  /* Returns the number of vertices in the control mesh. */
  int num_control_vertices() const { return num_control_vertices_; }
  /* Returns the number of driven vertices. */
  int num_driven_vertices() const { return ssize(vertex_indices_); }

 private:
  std::vector<Eigen::Vector4i> vertex_indices_;
  std::vector<Eigen::Vector4d> barycentric_coordinates_;
  int num_control_vertices_{};
};

/* Given an Eigen vector representing the positions of a control mesh, computes
 the positions of a subset of vertices of the control mesh in the same frame. */
class VertexSelector {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(VertexSelector)

  /* Constructs a VertexSelector.
   @pre `selected_vertices` is non-empty, sorted, doesn't contain duplicates,
   and its values fall in [0, control_mesh.num_vertices()). */
  VertexSelector(std::vector<int> selected_vertices,
                 const VolumeMesh<double>& control_mesh)
      : selected_vertices_(std::move(selected_vertices)),
        num_control_vertices_(control_mesh.num_vertices()) {
    DRAKE_THROW_UNLESS(!selected_vertices_.empty());
    DRAKE_THROW_UNLESS(selected_vertices_[0] >= 0);
    // No duplicates.
    DRAKE_THROW_UNLESS(std::adjacent_find(selected_vertices_.begin(),
                                          selected_vertices_.end()) ==
                       selected_vertices_.end());
    DRAKE_THROW_UNLESS(
        std::is_sorted(selected_vertices_.begin(), selected_vertices_.end()));
    DRAKE_THROW_UNLESS(num_control_vertices_ > selected_vertices_.back());
  }

  /* Returns the number of vertices in the control mesh. */
  int num_control_vertices() const { return num_control_vertices_; }
  /* Returns the number of selected vertices */
  int num_driven_vertices() const { return ssize(selected_vertices_); }

  /* Given the positions q of the vertices of the control mesh in some frame F,
   returns the positions of the passively driven points in the same frame F.
   Both the input and the returned value are ordered as flat Eigen vectors
   composing of (x₀, y₀, z₀, x₁, y₁, z₁, ...). The input is ordered the same as
   the vertices in the input control mesh. The output is ordered the same as the
   order of the selected vertices at construction.
   @pre q.size() is 3 times the number of vertices of the control mesh
   given at construction. */
  Eigen::VectorXd operator()(const Eigen::VectorXd& q) const;

 private:
  std::vector<int> selected_vertices_;
  int num_control_vertices_{};
};

/* DrivenTriangleSurfaceMesh represents a triangle surface mesh that is driven
 by a control volume mesh. The control volume mesh completely encloses the
 driven surface mesh and the vertex positions of the surface mesh is
 interpolated from the vertex positions of the control mesh. The vertex normals
 of the driven surface mesh can be queried, and their values are computed as
 area-weighted averages of the normals of all faces the vertices are incident
 to. */
class DrivenTriangleSurfaceMesh {
 public:
  /* Constructs a DrivenTriangleSurfaceMesh with the given `surface_mesh` as a
   the driven mesh and the given `control_mesh` as the control mesh. Both meshes
   are measured and expressed in a common mesh frame M.
   @throws std::exception if any vertex of the surface mesh is not enclosed by
   any element of the control mesh. */
  DrivenTriangleSurfaceMesh(TriangleSurfaceMesh<double> surface_mesh,
                            const VolumeMesh<double>& control_mesh);

  /* Constructs a DrivenTriangleSurfaceMesh with the given
   `deformable_surface_mesh` whose deformation is interpolated with the
   explicitly supplied `interpolator`.
   @pre interpolator.num_driven_vertices() ==
   deformable_surface_mesh.num_vertices() */
  DrivenTriangleSurfaceMesh(
      std::variant<BarycentricInterpolator, VertexSelector> interpolator,
      DeformableTriangleMesh<double> deformable_surface_mesh);

  const TriangleSurfaceMesh<double>& get_triangle_surface_mesh() const {
    return deformable_surface_mesh_.mesh();
  }

  /* Returns the number of vertices in the control mesh. */
  int num_control_vertices() const;

  /* Updates the vertex positions of the control volume mesh.
   @param q_M  A vector of 3N values (where the control mesh has N vertices).
   The iᵗʰ vertex gets values <q_M(3i), q_M(3i + 1), q_M(3i + 2)>. Each vertex
   is assumed to be measured and expressed in the mesh's frame M.
   @pre q_M.size() == 3 * num_control_vertices(). */
  void SetControlMeshPositions(const VectorX<double>& q_M);

  /* Returns q, the vertex positions of the driven surface mesh as an Eigen
   vector of 3N values (where the driven mesh has N vertices). The iᵗʰ vertex
   gets values <q(3i), q(3i + 1), q(3i + 2)>. Each vertex position is measured
   and expressed in the mesh's frame M. The vertex positions of the control mesh
   set in the last call to SetControlMeshPositions() are used to compute vertex
   positions of the driven mesh. If SetControlMeshPositions() has never been
   called, the vertex positions of the driven mesh at construction is reported.
  */
  VectorX<double> GetDrivenVertexPositions() const;

  /* Computes n, the vertex normals of the driven surface mesh as an Eigen
   vector of 3N values (where the driven mesh has N vertices). The iᵗʰ vertex
   gets values <n(3i), n(3i + 1), n(3i + 2)>. Each vertex normal is computed as
   area-weighted averages of the normals of all faces the vertices are incident
   to. The vertex normal is normalized and is measured and expressed in the
   mesh's frame M. The vertex positions of the control mesh set in the last call
   to SetControlMeshPositions() are used to compute vertex normals of the driven
   mesh. If SetControlMeshPositions() has never been called, the vertex normals
   of the driven mesh at construction is reported. */
  VectorX<double> GetDrivenVertexNormals() const;

 private:
  std::variant<BarycentricInterpolator, VertexSelector> interpolator_;
  DeformableTriangleMesh<double> deformable_surface_mesh_;
};

/* Given a volume control mesh, returns a its surface mesh whose deformation is
 driven by the volume control mesh. The vertex indices of the returned surface
 mesh respects the original vertex indices of the volume mesh: if vertex i and
 vertex j are vertices of the resulting surface mesh and their indices in the
 control volume mesh are f(i) and f(j) respectively, then i < j iff f(i) < f(j).
 */
DrivenTriangleSurfaceMesh MakeDrivenSurfaceMesh(
    const VolumeMesh<double>& control_mesh);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
