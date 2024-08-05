#include "drake/geometry/proximity/inflate_mesh.h"

#include <iostream>
#include <limits>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include "drake/common/fmt_eigen.h"
#include "drake/geometry/proximity/sorted_triplet.h"
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

// For a set of `incident_faces` incident to a vertex, this function makes the
// program to "inflate" the vertex outwards. To minimize round-off errors, we
// make the problem dimensionless by definining the dimensionless displacement
// ũ = u/δ where δ is the margin. The dimensionless program is:
//   min 1/2‖ũ‖|²
//   s.t. ũᵢ⋅n̂ₐ ≥ 1
// where n̂ₐ are the normals of each face in `incident_faces`.
std::unique_ptr<solvers::MathematicalProgram> MakeVertexProgram(
    const TriangleSurfaceMesh<double>& surface,
    const std::vector<int>& incident_faces) {
  auto prog = std::make_unique<solvers::MathematicalProgram>();
  const int nv = 3;
  auto u = prog->NewContinuousVariables(nv);
  prog->AddQuadraticCost(MatrixXd::Identity(nv, nv), VectorXd::Zero(nv), u,
                         true /* it is convex */);
  // For each surface vertex we add as many linear constraints as incident
  // faces to that vertex. These constraints enforce that the mesh actually
  // inflates (avoiding deflation regions).
  // One linear constraint per face.
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

// Given that multiple faces can belong to a same element, in general we have
// elements_group.size() <= faces_group.size().
struct VertexGroup {
  std::vector<int> elements_group;
  std::vector<int> faces_group;
};

// Given a set of `faces`, this function splits them into groups exclusive
// groups such that each group corresponds to faces that belong to the same
// volumetric element. That is, the e-th group is defined as
//   Gₑ = {f ∈ faces | tri_to_tet[f] == e}.
std::vector<VertexGroup> SplitFacesIntoElementGroups(
    const std::vector<int>& faces, const std::vector<int>& tri_to_tet) {
  // We'll make a map from an element e into subsets of `faces` that
  // belong to element e.
  std::unordered_map<int, std::vector<int>> element_to_group;
  for (int f : faces) {
    const int e = tri_to_tet[f];
    element_to_group[e].push_back(f);
  }

  // Extract groups.
  std::vector<VertexGroup> groups;
  groups.reserve(element_to_group.size());
  for (const auto& [e, fg] : element_to_group) {
    groups.push_back(VertexGroup{{e}, fg});
  }

  return groups;
}

void ProcessUnfeasibleVertex(const VolumeMesh<double>& mesh,
                             const TriangleSurfaceMesh<double>& mesh_surface,
                             double margin, int surface_vertex,
                             const std::vector<int>& incident_faces,
                             const std::vector<int>& tri_to_tet,
                             std::vector<Vector3d>* u,
                             std::vector<Vector3d>* vertices,
                             std::vector<VolumeElement>* tetrahedra,
                             std::vector<int>* surface_to_volume_vertices,
                             std::vector<int>* new_vertices) {
  // find v's local index in e.
  auto find_tet_local_index = [&](int element_index, int vertex_index) {
    const VolumeElement& tet = mesh.element(element_index);
    for (int k = 0; k < 4; ++k) {
      if (vertex_index == tet.vertex(k)) return k;
    }
    // Something went wrong if v is not in e.
    throw std::logic_error("Vertex not found in element.");
  };

  const int v = (*surface_to_volume_vertices)[surface_vertex];

  const std::vector<VertexGroup> groups =
      SplitFacesIntoElementGroups(incident_faces, tri_to_tet);
  const Vector3d p = (*vertices)[v];
  for (int i = 0; i < ssize(groups); ++i) {
    const auto& faces = groups[i].faces_group;
    std::unique_ptr<solvers::MathematicalProgram> prog =
        MakeVertexProgram(mesh_surface, faces);
    solvers::ClarabelSolver solver;
    const solvers::MathematicalProgramResult result = solver.Solve(*prog);
    if (!result.is_success()) {
      // TODO(amcastro-tri): Either throw or fall back to the most conservative
      // strategy (always succeeds) in SplitFacesIntoElementGroups().
      const solvers::ClarabelSolver::Details& details =
          result.get_solver_details<solvers::ClarabelSolver>();
      throw std::logic_error(
          fmt::format("Program fails with status: {}.", details.status));
    }

    // Assign the original vertex v to the first group and duplicate for all
    // other groups.
    const Vector3d& displacement = margin * result.get_x_val();
    if (i > 0) {
      new_vertices->push_back(v);  // Remember original vertex.

      // Duplicate.
      const int v_duplicate = ssize(*vertices);
      vertices->push_back(p);
      surface_to_volume_vertices->push_back(v_duplicate);
      u->push_back(displacement);

      // Update volume elements to point to the newly added duplicate.
      for (int e : groups[i].elements_group) {
        const int k = find_tet_local_index(e, v);
        (*tetrahedra)[e].set_vertex(k, v_duplicate);
      }
    } else {
      (*u)[surface_vertex] = displacement;
    }
  }
}

}  // namespace

VolumeMesh<double> MakeInflatedMesh(const VolumeMesh<double>& mesh,
                                    double margin,
                                    std::vector<int>* new_vertices) {
  DRAKE_THROW_UNLESS(margin >= 0);

  // Surface mesh and map to volume vertices.
  std::vector<int> surface_to_volume_vertices;
  std::vector<int> tri_to_tet;
  const TriangleSurfaceMesh<double> mesh_surface =
      ConvertVolumeToSurfaceMeshWithBoundaryVerticesAndElementMap(
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

  // Process each surface vertex separately.
  for (int s = 0; s < num_surface_vertices; ++s) {
    const std::vector<int>& faces = adjacent_faces[s];

    std::unique_ptr<solvers::MathematicalProgram> prog =
        MakeVertexProgram(mesh_surface, faces);
    solvers::ClarabelSolver solver;
    const solvers::MathematicalProgramResult result = solver.Solve(*prog);

    if (result.is_success()) {
      u[s] = margin * result.get_x_val();
    } else {
      ProcessUnfeasibleVertex(mesh, mesh_surface, margin, s, faces, tri_to_tet,
                              &u, &vertices, &tetrahedra,
                              &surface_to_volume_vertices, new_vertices);
    }
  }

  // DRAKE_DEMAND(vertices.size() == u.size());
  DRAKE_DEMAND(surface_to_volume_vertices.size() == u.size());

#if 0
  const int num_vertices = vertices.size();
  const int new_num_vertices = vertices.size();
  std::cout << fmt::format("nv: {}. nv(new): {}\n", num_vertices,
                           new_num_vertices);
#endif

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
