#include "drake/geometry/proximity/inflate_mesh.h"

#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include "drake/geometry/proximity/volume_to_surface_mesh.h"
#include "drake/solvers/clarabel_solver.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

/* Implements the QP program to inflate a mesh by a given margin amount.
 This can be written as:
   min 1/2‖u‖|²
   s.t. uᵢ⋅n̂ₐ ≥ δ
 where for each i-th surface we add a linear constraint involving
 all adjacent faces to vertex i with normal n̂ₐ. */
class InflateProgram {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(InflateProgram);

  // Create an optimization program to inflate `mesh` and amount `margin.`
  // @note `mesh` is aliased and must remain valid for the lifetime of this
  // class.
  // @pre mesh is not nullptr.
  InflateProgram(const VolumeMesh<double>* mesh, double margin);

  VolumeMesh<double> Solve() const;

 private:
  // Makes a dimensionless version of program described in the class's
  // documentation to minimize round-off errors. We use the margin δ to define a
  // dimensionless displacement ũ = u/δ, and write the dimensionless program as:
  //   min 1/2‖ũ‖|²
  //   s.t. ũᵢ⋅n̂ₐ ≥ 1
  //
  // N.B. The program is "separable", i.e. we could solve the displacement for
  // each surface vertex separately, independently of all other vertices. That
  // strategy could be considered for instance for thread parallelization.
  void MakeProgram();

  const VolumeMesh<double>& mesh_;
  double margin_{0.0};
  std::unique_ptr<solvers::MathematicalProgram> prog_;
  VectorX<symbolic::Variable> u_;  // displacements
  // Map from surface indexes (i.e. indexes into u) to vertices in the original
  // volume mesh, i.e. mesh_.
  std::vector<int> surface_to_volume_vertices_;
};

InflateProgram::InflateProgram(const VolumeMesh<double>* mesh, double margin)
    : mesh_{*mesh}, margin_(margin) {
  DRAKE_DEMAND(margin >= 0);
  if (margin > 0) MakeProgram();
}

void InflateProgram::MakeProgram() {
  prog_ = std::make_unique<solvers::MathematicalProgram>();
  const TriangleSurfaceMesh<double> mesh_surface =
      ConvertVolumeToSurfaceMeshWithBoundaryVertices(
          mesh_, &surface_to_volume_vertices_);
  const int num_surface_vertices = mesh_surface.num_vertices();
  DRAKE_DEMAND(ssize(surface_to_volume_vertices_) == num_surface_vertices);

  const int num_vars = 3 * num_surface_vertices;
  u_ = prog_->NewContinuousVariables(num_vars, "u");

  prog_->AddQuadraticCost(MatrixXd::Identity(num_vars, num_vars),
                          VectorXd::Zero(num_vars), u_,
                          true /* it is convex */);

  // Determine adjacent faces to each vertex on the surface.
  std::vector<std::vector<int>> adjacent_faces(
      num_surface_vertices);  // indexed by surface vertex.
  for (int e = 0; e < mesh_surface.num_elements(); ++e) {
    const SurfaceTriangle& triangle = mesh_surface.element(e);
    for (int i = 0; i < 3; ++i) {
      const int surface_vertex = triangle.vertex(i);
      adjacent_faces[surface_vertex].push_back(e);
    }
  }

  // For each surface vertex we add as many linear constraints as adjacent
  // faces to that vertex. These constraints enforce that the mesh actually
  // inflates (avoiding deflation regions).
  for (int v = 0; v < num_surface_vertices; ++v) {
    const std::vector<int>& faces = adjacent_faces[v];

    // One linear constraint per face.
    const int num_faces = faces.size();
    const VectorXd lb = VectorXd::Ones(num_faces);
    const VectorXd ub =
        VectorXd::Constant(num_faces, std::numeric_limits<double>::infinity());
    MatrixXd A(num_faces, 3);
    for (int f = 0; f < num_faces; ++f) {
      const Vector3d& normal = mesh_surface.face_normal(faces[f]);
      A.row(f) = normal.transpose();
    }
    prog_->AddLinearConstraint(A, lb, ub, u_.segment<3>(3 * v));
  }
}

VolumeMesh<double> InflateProgram::Solve() const {
  if (margin_ == 0) return mesh_;

  // N.B. By experimentation with meshes of different complexity, we determined
  // that Clarabel performed best in terms of both accuracy and computational
  // performance using solver default parameters.
  solvers::ClarabelSolver solver;
  const solvers::MathematicalProgramResult result = solver.Solve(*prog_);

  if (!result.is_success()) {
    throw std::runtime_error(
        "Failure to inflate mesh. Unless there is a bug, the procedure to "
        "apply margins to non-convex meshes is guaranteed to succeed. You "
        "might also want to check your volume mesh is not somehow degenerate. "
        "Otherwise, please open a Drake issue.");
  }

  // The solution corresponds to the dimensionless displacements ũ for each
  // vertex of the input volume mesh. Scaling by the margin, gives us the
  // displacements u.
  const VectorXd u = margin_ * result.get_x_val();

  // First copy all vertices.
  std::vector<Vector3d> vertices = mesh_.vertices();

  // Apply displacement to each surface vertex.
  for (int s = 0; s < ssize(surface_to_volume_vertices_); ++s) {
    const int v = surface_to_volume_vertices_[s];
    vertices[v] += u.segment<3>(3 * s);
  }

  std::vector<VolumeElement> tetrahedra = mesh_.tetrahedra();
  return VolumeMesh<double>(std::move(tetrahedra), std::move(vertices));
}

}  // namespace

VolumeMesh<double> MakeInflatedMesh(const VolumeMesh<double>& mesh,
                                    double margin) {
  DRAKE_THROW_UNLESS(margin >= 0);
  InflateProgram program(&mesh, margin);
  return program.Solve();
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
