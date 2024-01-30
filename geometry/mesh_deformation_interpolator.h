#pragma once

#include <utility>
#include <variant>
#include <vector>

#include "drake/common/ssize.h"
#include "drake/geometry/proximity/triangle_surface_mesh.h"
#include "drake/geometry/proximity/volume_mesh.h"

namespace drake {
namespace geometry {
namespace internal {

/* Note, BarycentricInterpolator and VertexSampler defined below are included
 in a common std::variant. */

/* Given a volume mesh (the control mesh) and a list of (passively driven)
 points embedded in the mesh, BarycentricInterpolator uses the vertex positions
 of the control mesh to compute the positions of the embedded points based on
 their barycentric coordinates in the tetrahedron containing them. */
class BarycentricInterpolator {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(BarycentricInterpolator)

  /* Constructs a BarycentricInterpolator.
   @param[in] positions_M     The positions of the passively driven points
                              embedded in the control mesh, measured and
                              expressed in the control mesh's frame M.
   @param[in] control_mesh_M  The volume mesh driving the embedded points,
                              measured and expressed its own frame M.
   @throws std::exception if any passively driven point is outside of the
   control mesh. */
  BarycentricInterpolator(const std::vector<Vector3<double>>& positions_M,
                          const VolumeMesh<double>& control_mesh_M);

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

/* Serves as a mask on a vector with 3N triples. The mask is defined by a set
 of indices in the range [0, N). It can be used to extract a subset of triples
 from a single source vector of triples. In particular, it can be used to
 extract a subset of vertex positions from the vertex positions of of a mesh. */
class VertexSampler {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(VertexSampler)

  /* Constructs a VertexSampler.
   @pre `sampled_vertices` is non-empty, sorted, doesn't contain duplicates,
   and its values fall in [0, control_mesh.num_vertices()). */
  VertexSampler(std::vector<int> sampled_vertices,
                const VolumeMesh<double>& control_mesh);

  /* Returns the number of vertices in the control mesh. */
  int num_control_vertices() const { return num_control_vertices_; }

  /* Returns the number of selected vertices */
  int num_driven_vertices() const { return ssize(sampled_vertices_); }

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
  std::vector<int> sampled_vertices_;
  int num_control_vertices_{};
};

/* %DrivenTriangleSurfaceMesh represents a triangle surface mesh that is driven
 by a control volume mesh. The control volume mesh completely encloses the
 driven surface mesh and the vertex positions of the surface mesh is
 interpolated from the vertex positions of the control mesh. The vertex normals
 of the driven surface mesh can be queried, and their values are computed as
 area-weighted averages of the normals of all faces the vertices are incident
 to. */
class DrivenTriangleSurfaceMesh {
 public:
  /* Constructs a %DrivenTriangleSurfaceMesh such that the given `triangle_mesh`
   is driven by the given `control_mesh`. Both meshes are measured and expressed
   in a common mesh frame M.
   @throws std::exception if any vertex of the surface mesh is not enclosed by
   any element of the control mesh. */
  DrivenTriangleSurfaceMesh(TriangleSurfaceMesh<double> triangle_mesh,
                            const VolumeMesh<double>& control_mesh);

  /* Constructs a %DrivenTriangleSurfaceMesh with the given `triangle_mesh`
   whose deformation is interpolated with the explicitly supplied
   `interpolator`. This constructor differs from the alternative constructor in
   a few ways:

    1. The other constructor automatically constructs a barycentric interpolator
       to drive the mesh positions.
    2. The other constructor requires the triangle surface mesh to be completely
       contained within the control mesh.
    3. This constructor is equivalent to the other one when a barycentric
       interpolator is passed, but this one is cheaper to construct.
    4. This constructor can accept a vertex selector.

   @pre interpolator.num_driven_vertices() == triangle_mesh.num_vertices() */
  DrivenTriangleSurfaceMesh(
      std::variant<BarycentricInterpolator, VertexSampler> interpolator,
      TriangleSurfaceMesh<double> triangle_mesh);

  const TriangleSurfaceMesh<double>& triangle_surface_mesh() const {
    return triangle_mesh_;
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
   vector of 3V values (where the driven mesh has V vertices). The iᵗʰ vertex
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
   area-weighted averages of the normals of all triangles containing the vertex
   as one of its vertices. The vertex normal is normalized and is measured and
   expressed in the mesh's frame M. The vertex positions of the control mesh set
   in the last call to SetControlMeshPositions() are used to compute vertex
   normals of the driven mesh. If SetControlMeshPositions() has never been
   called, the vertex normals of the driven mesh at construction is reported. */
  VectorX<double> GetDrivenVertexNormals() const;

 private:
  std::variant<BarycentricInterpolator, VertexSampler> interpolator_;
  TriangleSurfaceMesh<double> triangle_mesh_;
};

/* Makes a driven mesh based on the surface mesh of the given control_mesh. */
DrivenTriangleSurfaceMesh MakeDrivenSurfaceMesh(
    const VolumeMesh<double>& control_mesh);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
