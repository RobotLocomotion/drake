#include "drake/geometry/proximity/inflate_mesh.h"

#include <limits>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "drake/geometry/proximity/volume_to_surface_mesh.h"
#include "drake/solvers/choose_best_solver.h"
#include "drake/solvers/clarabel_solver.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/osqp_solver.h"

namespace drake {
namespace geometry {
namespace internal {
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;
using solvers::ClarabelSolver;
using solvers::CommonSolverOption;
using solvers::MakeFirstAvailableSolver;
using solvers::OsqpSolver;
using solvers::SolverInterface;
using solvers::SolverOptions;

namespace {

/* Given a surface and a collection `incident_faces` to a vertex (not given
 explicitly), makes the program to "inflate" the vertex outwards.

 To minimize round-off errors, we make the problem dimensionless by definining
 the dimensionless displacement ũ = u/δ where δ is the margin. The dimensionless
 program is:

   min 1/2‖ũ‖|²
   s.t. ũ⋅n̂ₐ ≥ 1, ∀ n̂ₐ ∈ normals(incident_faces)

 where n̂ₐ is the normal of a face in `incident_faces`.

 Note: `incident_faces` will not contain _all_ triangles incident to the implied
 vertex. This function is helper for resolving displacement for infeasible
 vertex displacement. They are infeasible because the _full_ set of incident
 faces define a set of infeasible constraints.

 @pre incident_faces.size() > 1. If there is only a single incident face, the
 solution of the optimization is trivial: δ⋅n̂ₐ. Calling this is overkill. */
std::unique_ptr<solvers::MathematicalProgram> MakeVertexProgram(
    const TriangleSurfaceMesh<double>& surface,
    const std::vector<int>& incident_faces) {
  DRAKE_DEMAND(incident_faces.size() > 1);

  auto prog = std::make_unique<solvers::MathematicalProgram>();
  const int nv = 3;
  auto u = prog->NewContinuousVariables(nv);
  prog->AddQuadraticCost(MatrixXd::Identity(nv, nv), VectorXd::Zero(nv), u,
                         true /* it is convex */);
  /* Add one constraint per incident face, inflating w.r.t. all incident faces
   simultaneously. */
  const int num_faces = incident_faces.size();
  const VectorXd lb = VectorXd::Ones(num_faces);
  const VectorXd ub =
      VectorXd::Constant(num_faces, std::numeric_limits<double>::infinity());
  MatrixXd A(num_faces, 3);
  for (int f = 0; f < num_faces; ++f) {
    const Vector3d& normal = surface.face_normal(incident_faces[f]);
    A.row(f) = normal.transpose();
  }
  prog->AddLinearConstraint(A, lb, ub, u);

  return prog;
}

/* Given a set of `faces` that lie on the surface of a volume mesh, all incident
 to a common vertex, partitions the faces based on the tetrahedra the faces
 come from.

 @returns The partitioning of incident faces. Each entry of the return value
 contains those incident faces which are faces of a common tetrahedron. (Looking
 up the tet index from `tri_to_tet` should confirm this.)

 If the volume mesh is a valid hydroelastic or deformable volume mesh, there
 should be no more than one face in each entry. */
std::vector<std::vector<int>> PartitionIncidentFaces(
    const std::vector<int>& faces, const std::vector<TetFace>& tri_to_tet) {
  /* Builds a map from the faces' original tets to the face indices. We're using
   a map so that the partitions are well-ordered on all platforms, otherwise
   the changes to the tetrahedral mesh will vary from platform to platform. */
  std::map<int, std::vector<int>> tet_groups;
  for (int f : faces) {
    tet_groups[tri_to_tet[f].tet_index].push_back(f);
  }

  /* Extract groups. */
  std::vector<std::vector<int>> groups;
  groups.reserve(tet_groups.size());
  for (auto& [_, fg] : tet_groups) {
    groups.push_back(std::move(fg));
  }

  return groups;
}

/* Computes the displacement for an *implied* vertex. It is a vertex that is
 incident to all of the faces enumerated in `face_group`. The displacement
 satisfies the optimization program documented for MakeVertexProgram() and
 only depends on the face normals and `margin` value. */
Vector3d CalculateDisplacement(const TriangleSurfaceMesh<double>& mesh_surface,
                               const std::vector<int>& face_group,
                               double margin, const SolverInterface& solver) {
  if (face_group.size() == 1) {
    /* A single face has a trivial solution. */
    return margin * mesh_surface.face_normal(face_group[0]);
  } else {
    std::unique_ptr<solvers::MathematicalProgram> prog =
        MakeVertexProgram(mesh_surface, face_group);
    std::optional<SolverOptions> solver_options;
    solver_options.emplace();
    solver_options->SetOption(CommonSolverOption::kMaxThreads, 1);
    solvers::MathematicalProgramResult result;
    solver.Solve(*prog, /* initial_guess = */ std::nullopt, solver_options,
                 &result);
    if (!result.is_success()) {
      /* At this point, it isn't clear what circumstances would cause failure.
       Most likely an extremely degenerate case (e.g., a flat tetrahedron with
       two anti-parallel faces on the surface). As such, it's difficult to
       know if there's a more helpful message that could be given. */
      const std::string status =
          result.get_solver_id() == ClarabelSolver::id()
              ? result.get_solver_details<ClarabelSolver>().status
              : fmt::to_string(result.get_solution_result());
      throw std::logic_error(
          fmt::format("Program fails with status: {}.", status));
    }

    return margin * result.get_x_val();
  }
}

/* Creates a new VolumeElement (tetrahedron) from an old tet. The new tet has
 all the same vertices except for one. The kth vertex uses the value
 `new_index`, for k ∈ [0, 3]. */
VolumeElement SwapTetVertex(const VolumeElement& tet, int k, int new_index) {
  // clang-format off
  return VolumeElement(
      k == 0 ? new_index : tet.vertex(0),
      k == 1 ? new_index : tet.vertex(1),
      k == 2 ? new_index : tet.vertex(2),
      k == 3 ? new_index : tet.vertex(3));
  // clang-format on
}

/* We were unable to compute a displacement for the indicated `surface_vertex`.
 This is because the faces incident to that vertex defined a set of infeasible
 constraints.

 The solution is to partition the set of faces into smaller groups and to
 duplicate the problematic vertex so each group has its own copy. The result of
 this duplication is:

   - New vertex values get appended to `vertices`.
   - New displacement values appended to `u`.
   - The definitions of *existing* tetrahedra in `tetrahedra` get changed to
     reference duplicated vertices.
   - Mapping from surface vertex to volume vertex is extended in
    `surface_to_volume_vertices`.
   - Mapping from split vertices to original vertices is stored in
    `split_vertex_to_original` */
void ProcessUnfeasibleVertex(const VolumeMesh<double>& mesh,
                             const TriangleSurfaceMesh<double>& mesh_surface,
                             double margin, int surface_vertex,
                             const std::vector<int>& incident_faces,
                             const std::vector<TetFace>& tri_to_tet,
                             const SolverInterface& solver,
                             std::vector<Vector3d>* u,
                             std::vector<Vector3d>* vertices,
                             std::vector<VolumeElement>* tetrahedra,
                             std::vector<int>* surface_to_volume_vertices,
                             std::map<int, int>* split_vertex_to_original) {
  /* Find v's local index in its tetrahedron. */
  auto find_tet_local_index = [&](int tet_index, int vertex_index) {
    const VolumeElement& tet = mesh.element(tet_index);
    for (int k = 0; k < 4; ++k) {
      if (vertex_index == tet.vertex(k)) return k;
    }
    DRAKE_UNREACHABLE();
  };

  /* The index of the volume vertex to which `surface_vertex` corresponds. */
  const int volume_vertex = (*surface_to_volume_vertices)[surface_vertex];

  const std::vector<std::vector<int>> face_groups =
      PartitionIncidentFaces(incident_faces, tri_to_tet);
  /* The only way to resolve the infeasible displacement is to partition the
   faces into _multiple_ groups. */
  DRAKE_DEMAND(face_groups.size() > 1);

  /* For each partition, we will have _one_ vertex in the resulting mesh; this
   will require copying the original vertex. However, we only need to make N-1
   copies; the first group will use the original vertex. */
  (*u)[surface_vertex] =
      CalculateDisplacement(mesh_surface, face_groups[0], margin, solver);

  /* Intentional copy -- we'll be pushing to `vertices` and don't want to risk
   an invalidated reference. */
  const Vector3d p = (*vertices)[volume_vertex];
  for (int i = 1; i < ssize(face_groups); ++i) {
    const std::vector<int>& face_group = face_groups[i];

    /* Duplicate the vertex value. */
    const int dupe_volume_index = ssize(*vertices);
    surface_to_volume_vertices->push_back(dupe_volume_index);
    split_vertex_to_original->emplace(dupe_volume_index, volume_vertex);
    vertices->push_back(p);
    u->push_back(
        CalculateDisplacement(mesh_surface, face_group, margin, solver));

    /* Update volume elements to point to the newly added duplicate. */
    const int tet_index = tri_to_tet[face_group[0]].tet_index;
    const int k = find_tet_local_index(tet_index, volume_vertex);
    (*tetrahedra)[tet_index] =
        SwapTetVertex((*tetrahedra)[tet_index], k, dupe_volume_index);
  }
}

}  // namespace

VolumeMesh<double> MakeInflatedMesh(
    const VolumeMesh<double>& mesh, double margin,
    std::map<int, int>* split_vertex_to_original) {
  DRAKE_THROW_UNLESS(margin >= 0);
  DRAKE_THROW_UNLESS(split_vertex_to_original != nullptr);

  // Surface mesh and map to volume vertices.
  std::vector<int> surface_to_volume_vertices;
  std::vector<TetFace> tri_to_tet;
  const TriangleSurfaceMesh<double> mesh_surface =
      ConvertVolumeToSurfaceMeshWithBoundaryVertices(
          mesh, &surface_to_volume_vertices, &tri_to_tet);
  const int num_surface_vertices = mesh_surface.num_vertices();
  DRAKE_DEMAND(ssize(surface_to_volume_vertices) == num_surface_vertices);

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

  std::vector<Vector3d> vertices = mesh.vertices();
  std::vector<VolumeElement> tetrahedra = mesh.tetrahedra();
  std::vector<Vector3d> u(num_surface_vertices, Vector3d::Zero());

  // Choose a solver to solver the MakeVertexProgram QPs. Clarabel is more
  // accurate, but might not be available (e.g., if Rust is disabled).
  auto solver =
      MakeFirstAvailableSolver({ClarabelSolver::id(), OsqpSolver::id()});

  /* Process each surface vertex separately. In the case where a vertex is
   infeasible, we will end up appending new vertices to `vertices`. It is
   essential that this for loop limit itself to the *original* vertices. */
  for (int s = 0; s < num_surface_vertices; ++s) {
    const std::vector<int>& faces = adjacent_faces[s];

    std::unique_ptr<solvers::MathematicalProgram> prog =
        MakeVertexProgram(mesh_surface, faces);
    std::optional<SolverOptions> solver_options;
    solver_options.emplace();
    solver_options->SetOption(CommonSolverOption::kMaxThreads, 1);
    solvers::MathematicalProgramResult result;
    solver->Solve(*prog, /* initial_guess = */ std::nullopt, solver_options,
                  &result);
    if (result.is_success()) {
      u[s] = margin * result.get_x_val();
    } else {
      ProcessUnfeasibleVertex(mesh, mesh_surface, margin, s, faces, tri_to_tet,
                              *solver, &u, &vertices, &tetrahedra,
                              &surface_to_volume_vertices,
                              split_vertex_to_original);
    }
  }

  DRAKE_DEMAND(surface_to_volume_vertices.size() == u.size());

  // Apply displacement to each surface vertex.
  for (int s = 0; s < ssize(u); ++s) {
    int v = surface_to_volume_vertices[s];
    vertices[v] += u[s];
  }

  return VolumeMesh<double>(std::move(tetrahedra), std::move(vertices));
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
