#include "drake/geometry/mesh_deformation_interpolator.h"

#include "drake/common/fmt_eigen.h"
#include "drake/geometry/proximity/volume_to_surface_mesh.h"

namespace drake {
namespace geometry {
namespace internal {

using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::Vector4i;
using Eigen::VectorXd;

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
          "A passively driven point lies outside the control mesh: ({}): {}", v,
          fmt_eigen(p_MV.transpose())));
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

DrivenTriangleMesh::DrivenTriangleMesh(
    TriangleSurfaceMesh<double> triangle_mesh,
    const VolumeMesh<double>& control_mesh)
    : interpolator_(
          BarycentricInterpolator(triangle_mesh.vertices(), control_mesh)),
      triangle_mesh_(std::move(triangle_mesh)) {}

DrivenTriangleMesh::DrivenTriangleMesh(
    std::variant<BarycentricInterpolator, VertexSampler> interpolator,
    TriangleSurfaceMesh<double> triangle_surface_mesh)
    : interpolator_(std::move(interpolator)),
      triangle_mesh_(std::move(triangle_surface_mesh)) {}

int DrivenTriangleMesh::num_control_vertices() const {
  return std::visit(
      [](const auto& interpolator) {
        return interpolator.num_control_vertices();
      },
      interpolator_);
}

void DrivenTriangleMesh::SetControlMeshPositions(const VectorX<double>& q_M) {
  triangle_mesh_.SetAllPositions(std::visit(
      [&q_M](const auto& interpolator) {
        return interpolator(q_M);
      },
      interpolator_));
}

VectorX<double> DrivenTriangleMesh::GetDrivenVertexPositions() const {
  VectorX<double> q_M(3 * triangle_mesh_.num_vertices());
  for (int v = 0; v < triangle_mesh_.num_vertices(); ++v) {
    q_M.segment<3>(3 * v) = triangle_mesh_.vertices()[v];
  }
  return q_M;
}

VectorX<double> DrivenTriangleMesh::GetDrivenVertexNormals() const {
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

DrivenTriangleMesh MakeDrivenSurfaceMesh(
    const VolumeMesh<double>& control_mesh) {
  std::vector<int> surface_to_volume_vertices;
  TriangleSurfaceMesh<double> triangle_mesh =
      ConvertVolumeToSurfaceMeshWithBoundaryVertices(
          control_mesh, &surface_to_volume_vertices);
  VertexSampler sampler{std::move(surface_to_volume_vertices), control_mesh};
  return DrivenTriangleMesh(
      std::variant<BarycentricInterpolator, VertexSampler>(std::move(sampler)),
      std::move(triangle_mesh));
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
