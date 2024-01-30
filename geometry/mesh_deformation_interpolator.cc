#include "drake/geometry/mesh_deformation_interpolator.h"

#include <array>
#include <map>
#include <set>

#include "drake/common/fmt_eigen.h"
#include "drake/geometry/proximity/sorted_triplet.h"

namespace drake {
namespace geometry {
namespace internal {

using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::Vector4i;
using Eigen::VectorXd;
using std::array;

BarycentricInterpolator::BarycentricInterpolator(
    const std::vector<Vector3<double>>& positions_M,
    const VolumeMesh<double>& control_mesh_M)
    : num_control_vertices_(control_mesh_M.num_vertices()) {
  for (int v = 0; v < ssize(positions_M); ++v) {
    const Vector3d& p_MV = positions_M[v];
    bool matched = false;
    for (int e = 0; e < control_mesh_M.num_elements(); ++e) {
      const Vector4d bary = control_mesh_M.CalcBarycentric(p_MV, e);
      if ((bary.array() >= 0.0).all()) {
        barycentric_coordinates_.push_back(bary);
        vertex_indices_.emplace_back(control_mesh_M.element(e).vertex(0),
                                     control_mesh_M.element(e).vertex(1),
                                     control_mesh_M.element(e).vertex(2),
                                     control_mesh_M.element(e).vertex(3));
        matched = true;
        break;
      }
    }
    if (!matched) {
      throw std::runtime_error(fmt::format(
          "The {}-th passively driven point is outside of the control mesh. "
          "Its position in the control mesh's frame is {}",
          v, fmt_eigen(p_MV.transpose())));
    }
  }
  DRAKE_DEMAND(vertex_indices_.size() == barycentric_coordinates_.size());
}

VectorXd BarycentricInterpolator::operator()(const VectorXd& q) const {
  DRAKE_THROW_UNLESS(q.size() == 3 * num_control_vertices_);
  VectorXd result(3 * vertex_indices_.size());
  for (int i = 0; i < ssize(vertex_indices_); ++i) {
    Vector3d p_FV = Vector3d::Zero();
    const Vector4d& bary = barycentric_coordinates_[i];
    const Vector4i& indices = vertex_indices_[i];
    for (int j = 0; j < 4; ++j) {
      p_FV += bary[j] * q.segment<3>(3 * indices[j]);
    }
    result.segment<3>(3 * i) = p_FV;
  }
  return result;
}

VertexSampler::VertexSampler(std::vector<int> sampled_vertices,
                             const VolumeMesh<double>& control_mesh)
    : sampled_vertices_(std::move(sampled_vertices)),
      num_control_vertices_(control_mesh.num_vertices()) {
  DRAKE_THROW_UNLESS(!sampled_vertices_.empty());
  DRAKE_THROW_UNLESS(sampled_vertices_[0] >= 0);
  // No duplicates.
  DRAKE_THROW_UNLESS(
      std::adjacent_find(sampled_vertices_.begin(), sampled_vertices_.end()) ==
      sampled_vertices_.end());
  DRAKE_THROW_UNLESS(
      std::is_sorted(sampled_vertices_.begin(), sampled_vertices_.end()));
  DRAKE_THROW_UNLESS(num_control_vertices_ > sampled_vertices_.back());
}

VectorXd VertexSampler::operator()(const VectorXd& q) const {
  DRAKE_THROW_UNLESS(q.size() == 3 * num_control_vertices_);
  VectorXd result(3 * sampled_vertices_.size());
  for (int i = 0; i < ssize(sampled_vertices_); ++i) {
    result.segment<3>(3 * i) = q.segment<3>(3 * sampled_vertices_[i]);
  }
  return result;
}

DrivenTriangleSurfaceMesh::DrivenTriangleSurfaceMesh(
    TriangleSurfaceMesh<double> triangle_mesh,
    const VolumeMesh<double>& control_mesh)
    : interpolator_(
          BarycentricInterpolator(triangle_mesh.vertices(), control_mesh)),
      triangle_mesh_(std::move(triangle_mesh)) {}

DrivenTriangleSurfaceMesh::DrivenTriangleSurfaceMesh(
    std::variant<BarycentricInterpolator, VertexSampler> interpolator,
    TriangleSurfaceMesh<double> triangle_surface_mesh)
    : interpolator_(std::move(interpolator)),
      triangle_mesh_(std::move(triangle_surface_mesh)) {}

int DrivenTriangleSurfaceMesh::num_control_vertices() const {
  return std::visit(
      [](const auto& interpolator) {
        return interpolator.num_control_vertices();
      },
      interpolator_);
}

void DrivenTriangleSurfaceMesh::SetControlMeshPositions(
    const VectorX<double>& q_M) {
  triangle_mesh_.SetAllPositions(std::visit(
      [&q_M](const auto& interpolator) {
        return interpolator(q_M);
      },
      interpolator_));
}

VectorX<double> DrivenTriangleSurfaceMesh::GetDrivenVertexPositions() const {
  VectorX<double> q_M(3 * triangle_mesh_.num_vertices());
  for (int v = 0; v < triangle_mesh_.num_vertices(); ++v) {
    q_M.segment<3>(3 * v) = triangle_mesh_.vertices()[v];
  }
  return q_M;
}

VectorX<double> DrivenTriangleSurfaceMesh::GetDrivenVertexNormals() const {
  VectorX<double> nhats_M =
      VectorX<double>::Zero(3 * triangle_mesh_.num_vertices());
  for (int f = 0; f < triangle_mesh_.num_triangles(); ++f) {
    const Vector3<double> area_weighted_normal =
        triangle_mesh_.area(f) * triangle_mesh_.face_normal(f);
    for (int v = 0; v < 3; ++v) {
      const int vertex = triangle_mesh_.element(f).vertex(v);
      nhats_M.segment<3>(3 * vertex) += area_weighted_normal;
    }
  }
  for (int v = 0; v < triangle_mesh_.num_vertices(); ++v) {
    nhats_M.segment<3>(3 * v).normalize();
  }
  return nhats_M;
}

DrivenTriangleSurfaceMesh MakeDrivenSurfaceMesh(
    const VolumeMesh<double>& control_mesh) {
  /* For each tet mesh, extract all the border triangles. Those are the
   triangles that are only referenced by a single tet. So, for every tet, we
   examine its four constituent triangles and determine if any other tet
   shares it. Any triangle that is only referenced once is a border triangle.
   Each triangle has a unique key: a SortedTriplet (so the ordering of the
   triangle vertex indices won't matter). The first time we see a triangle, we
   add it to a map. The second time we see the triangle, we remove it. When
   we're done, the keys in the map will be those triangles referenced only
   once. The values in the map represent the triangle, with the vertex indices
   ordered so that they point *out* of the tetrahedron. Therefore,
   they will also point outside of the mesh. A typical tetrahedral element
   looks like:

       p2 *
          |
          |
       p3 *---* p0
         /
        /
    p1 *

   The index order for a particular tetrahedron has the order [p0, p1, p2,
   p3]. These local indices enumerate each of the tet triangles with
   outward-pointing normals with respect to the right-hand rule. */
  const array<array<int, 3>, 4> local_indices{
      {{{1, 0, 2}}, {{3, 0, 1}}, {{3, 1, 2}}, {{2, 0, 3}}}};

  std::map<SortedTriplet<int>, array<int, 3>> border_triangles;
  for (const VolumeElement& tet : control_mesh.tetrahedra()) {
    for (const array<int, 3>& tet_triangle : local_indices) {
      const array<int, 3> tri{tet.vertex(tet_triangle[0]),
                              tet.vertex(tet_triangle[1]),
                              tet.vertex(tet_triangle[2])};
      const SortedTriplet triangle_key(tri[0], tri[1], tri[2]);
      // Here we rely on the fact that at most two tets would share a common
      // triangle.
      if (auto itr = border_triangles.find(triangle_key);
          itr != border_triangles.end()) {
        border_triangles.erase(itr);
      } else {
        border_triangles[triangle_key] = tri;
      }
    }
  }

  const int volume_vertex_count = control_mesh.num_vertices();

  /* Using a set because the vertices will be nicely ordered as required by
   the contract of this function. */
  std::set<int> unique_vertices;
  for (const auto& [triangle_key, triangle] : border_triangles) {
    unused(triangle_key);
    for (const int index : triangle) unique_vertices.insert(index);
  }

  /* Populate the mapping from surface to volume so that we can efficiently
   *extract the surface* vertex positions_M from the *volume* vertex input. */
  const std::vector<int> surface_to_volume_vertices(unique_vertices.begin(),
                                                    unique_vertices.end());

  /* The border triangles all include indices into the volume vertices. To
   turn them into surface triangles, they need to include indices into the
   surface vertices. Create the volume index --> surface map to facilitate the
   transformation. */
  const int surface_vertex_count =
      static_cast<int>(surface_to_volume_vertices.size());
  std::map<int, int> volume_to_surface;
  for (int j = 0; j < surface_vertex_count; ++j) {
    volume_to_surface[surface_to_volume_vertices[j]] = j;
  }

  /* Create the topology of the surface triangle mesh for each volume mesh.
   Each triangle consists of three indices into the set of *surface* vertex
   positions_M. */
  std::vector<SurfaceTriangle> surface_triangles;
  surface_triangles.reserve(border_triangles.size());
  for (auto& [triangle_key, face] : border_triangles) {
    unused(triangle_key);
    surface_triangles.emplace_back(volume_to_surface[face[0]],
                                   volume_to_surface[face[1]],
                                   volume_to_surface[face[2]]);
  }

  VectorXd q(3 * volume_vertex_count);
  for (int v = 0; v < volume_vertex_count; ++v) {
    q.segment<3>(3 * v) = control_mesh.vertex(v);
  }
  VertexSampler sampler{std::move(surface_to_volume_vertices), control_mesh};
  const VectorXd driven_qs = sampler(q);
  std::vector<Vector3<double>> vertex_positions_M(driven_qs.size() / 3);
  for (int i = 0; i < ssize(vertex_positions_M); ++i) {
    vertex_positions_M[i] = driven_qs.segment<3>(3 * i);
  }
  TriangleSurfaceMesh<double> triangle_mesh(std::move(surface_triangles),
                                            std::move(vertex_positions_M));

  return DrivenTriangleSurfaceMesh(
      std::variant<BarycentricInterpolator, VertexSampler>(std::move(sampler)),
      TriangleSurfaceMesh<double>(std::move(triangle_mesh)));
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
