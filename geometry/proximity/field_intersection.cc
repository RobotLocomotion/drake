#include "drake/geometry/proximity/field_intersection.h"

#include <memory>
#include <unordered_map>
#include <utility>

#include "drake/common/default_scalars.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/proximity/mesh_intersection.h"
#include "drake/geometry/proximity/mesh_plane_intersection.h"
#include "drake/geometry/proximity/plane.h"
#include "drake/geometry/proximity/posed_half_space.h"
#include "drake/geometry/proximity/triangle_surface_mesh.h"
#include "drake/geometry/proximity/triangle_surface_mesh_field.h"
#include "drake/geometry/proximity/volume_mesh_field.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
namespace internal {

using Eigen::Vector3d;

template <typename T>
bool CalcEquilibriumPlane(int element0,
                          const VolumeMeshFieldLinear<double, double>& field0_M,
                          int element1,
                          const VolumeMeshFieldLinear<double, double>& field1_N,
                          const math::RigidTransform<T>& X_MN,
                          Plane<T>* plane_M) {
  const Vector3d grad_f0_M = field0_M.EvaluateGradient(element0);
  const Vector3d p_MMo = Vector3d::Zero();
  // Value of f₀ at the origin of frame M.
  const double f0_Mo = field0_M.EvaluateCartesian(element0, p_MMo);

  const Vector3d grad_f1_N = field1_N.EvaluateGradient(element1);
  const Vector3<T> grad_f1_M = X_MN.rotation() * grad_f1_N.cast<T>();
  const Vector3<T> p_NMo = X_MN.inverse() * p_MMo.cast<T>();
  // Value of f₁ at the origin of frame M, which is the frame of f₀.
  const T f1_Mo = field1_N.EvaluateCartesian(element1, p_NMo);

  // In frame M, the two linear functions are:
  //      f₀(p_MQ) = grad_f0_M.dot(p_MQ) + f0_Mo.
  //      f₁(p_MQ) = grad_f1_M.dot(p_MQ) + f1_Mo.
  // Their equilibrium plane is:
  //   (grad_f0_M - grad_f1_M).dot(p_MQ) + (f0_Mo - f1_Mo) = 0.   (1)
  // Its perpendicular (but not necessarily unit-length) vector is:
  //           n_M = grad_f0_M - grad_f1_M,
  // which is in the direction of increasing f₀ and decreasing f₁.
  const Vector3<T> n_M = grad_f0_M - grad_f1_M;
  const T magnitude = n_M.norm();
  // TODO(DamrongGuoy): Change the threshold according to some
  //  experiments with respect to use cases, or make it a parameter.
  //  It is not clear to me what is the appropriate tolerance since the
  //  `magnitude` is in unit-field-value per meter. The other idea is to use
  //  non-unit-length normal vector in the Plane class; however, it will change
  //  the Plane API contract (the CalcHeight() will have different meaning).
  if (magnitude <= 0.0) {
    return false;
  }
  const Vector3<T> nhat_M = n_M / magnitude;

  // Using the unit normal vector nhat_M, the plane equation (1) becomes:
  //
  //          nhat_M.dot(p_MQ) + Δ = 0,
  //
  // where Δ = (f0_Mo - f1_Mo)/‖n_M‖. One such p_MQ is:
  //
  //                          p_MQ = -Δ * nhat_M
  //
  const Vector3<T> p_MQ = -((f0_Mo - f1_Mo) / magnitude) * nhat_M;

  *plane_M = Plane<T>(nhat_M, p_MQ, /*already_normalized = */ true);
  return true;
}

template <typename T>
void IntersectPlaneTetrahedron(
    int tetrahedron, const VolumeMesh<double>& mesh_M,
    const Plane<T>& plane_M, std::vector<Vector3<T>>* polygon_M) {
  polygon_M->clear();
  SliceTetrahedronWithPlane<T>(tetrahedron, mesh_M, plane_M, polygon_M);
}

// TODO(DamrongGuoy): Refactor ClipPolygonByHalfSpace() to share between
//  mesh_intersection and field_intersection instead of this hack.
template<typename MeshType>
class SurfaceVolumeIntersectorTester {
 public:
  using T = typename MeshType::ScalarType;

  void ClipPolygonByHalfSpace(const std::vector<Vector3<T>>& polygon_vertices_F,
                              const PosedHalfSpace<double>& H_F,
                              std::vector<Vector3<T>>* output_vertices_F) {
    intersect_.ClipPolygonByHalfSpace(polygon_vertices_F, H_F,
                                      output_vertices_F);
  }
 private:
  SurfaceVolumeIntersector<MeshType> intersect_;
};

template <typename T>
std::vector<Vector3<T>> IntersectTetrahedra(
    int element0, const VolumeMeshFieldLinear<double, double>& field0_M,
    int element1, const VolumeMeshFieldLinear<double, double>& field1_N,
    const math::RigidTransform<T>& X_MN, const Plane<T>& equilibrium_plane_M) {
  // TODO(DamrongGuoy): Refactor this buffer from being a local variable to
  //  being a class variable to decrease heap allocations. Then, return
  //  the const reference ("const std::vector<Vector3<T>>&").
  std::vector<Vector3<T>> polygon_buffer[2];

  // Intersects the equilibrium plane with the tetrahedron element0.
  std::vector<Vector3<T>>* polygon_M = &(polygon_buffer[0]);
  IntersectPlaneTetrahedron(element0, field0_M.mesh(), equilibrium_plane_M,
                            polygon_M);

  // Intersection of the above polygon with the four halfspaces of the four
  // faces of the tetrahedron element1. Expressed in frame M.
  Vector3<double> p_MVs[4];
  for (int i = 0; i < 4; ++i) {
    Vector3d p_NVi =
        field1_N.mesh().vertex(field1_N.mesh().element(element1).vertex(i));
    Vector3<T> p_MV = X_MN * p_NVi.cast<T>();
    p_MVs[i] = Vector3<double>(ExtractDoubleOrThrow(p_MV.x()),
                               ExtractDoubleOrThrow(p_MV.y()),
                               ExtractDoubleOrThrow(p_MV.z()));
  }
  const int kFaceVertexLocalIndex[4][3] = {
      {1, 2, 3}, {0, 3, 2}, {0, 1, 3}, {0, 2, 1}};
  DRAKE_ASSERT(polygon_M == &(polygon_buffer[0]));
  std::vector<Vector3<T>>* in_M = polygon_M;
  std::vector<Vector3<T>>* out_M = &(polygon_buffer[1]);
  for (const auto& face_vertex : kFaceVertexLocalIndex) {
    const Vector3<double>& p_MA = p_MVs[face_vertex[0]];
    const Vector3<double>& p_MB = p_MVs[face_vertex[1]];
    const Vector3<double>& p_MC = p_MVs[face_vertex[2]];
    const Vector3<double> triangle_normal_M = (p_MB - p_MA).cross(p_MC - p_MA);
    PosedHalfSpace<double> half_space_M(triangle_normal_M, p_MA);
    SurfaceVolumeIntersectorTester<TriangleSurfaceMesh<T>>()
        .ClipPolygonByHalfSpace(*in_M, half_space_M, out_M);
    std::swap(in_M, out_M);
  }
  polygon_M = in_M;

  // TODO(DamrongGuoy): Take care of duplicated vertices or vertices that are
  //  so closed together. Use RemoveDuplicateVertices() of
  //  SurfaceVolumeIntersector.

  return *polygon_M;
}

bool IsPlaneNormalAlongPressureGradient(
    const Vector3<double>& nhat_M, int tetrahedron,
    const VolumeMeshFieldLinear<double, double>& field_M) {
  const Vector3<double> grad_p_M = field_M.EvaluateGradient(tetrahedron);
  const double cos_theta = nhat_M.dot(grad_p_M.normalized());
  // We pick 5π/8 empirically to be the threshold angle, alpha.
  constexpr double kAlpha = 5. * M_PI / 8.;
  static const double kCosAlpha = std::cos(kAlpha);
  // cos(θ) > cos(α) → θ < α → condition met.
  return cos_theta > kCosAlpha;
}

template <class MeshType, class MeshBuilder, typename T, class FieldType>
void FieldIntersection(const VolumeMeshFieldLinear<double, double>& field0_M,
                       const Bvh<Obb, VolumeMesh<double>>& bvh0_M,
                       const VolumeMeshFieldLinear<double, double>& field1_N,
                       const Bvh<Obb, VolumeMesh<double>>& bvh1_N,
                       const math::RigidTransform<double>& X_MN,
                       MeshBuilder builder,
                       std::unique_ptr<MeshType>* surface_MN_M,
                       std::unique_ptr<FieldType>* e_01_M,
                       std::vector<Vector3<double>>* grad_e0_Ms,
                       std::vector<Vector3<double>>* grad_e1_Ms) {
  DRAKE_DEMAND(surface_MN_M != nullptr);
  DRAKE_DEMAND(e_01_M != nullptr);
  DRAKE_DEMAND(grad_e0_Ms != nullptr);
  DRAKE_DEMAND(grad_e1_Ms != nullptr);
  grad_e0_Ms->clear();
  grad_e1_Ms->clear();

  std::vector<std::pair<int, int>> candidate_tetrahedra;
  auto callback = [&candidate_tetrahedra](int tet0,
                                          int tet1) -> BvttCallbackResult {
    candidate_tetrahedra.emplace_back(tet0, tet1);
    return BvttCallbackResult::Continue;
  };
  bvh0_M.Collide(bvh1_N, X_MN, callback);

  std::vector<SurfaceTriangle> surface_faces;
  std::vector<Vector3<double>> surface_vertices_M;
  std::vector<double> surface_field_values;
  // Here the contact polygon is represented as a list of vertex indices.
  std::vector<int> contact_polygon;
  // Each contact polygon has at most 8 vertices because it is the
  // intersection of the pressure-equilibrium plane and the two tetrahedra.
  // The plane intersects a tetrahedron into a convex polygon with at most four
  // vertices. That convex polygon intersects a tetrahedron into at most four
  // more vertices.
  contact_polygon.reserve(8);
  for (const auto& [tet0, tet1] : candidate_tetrahedra) {
    // Initialize the plane with a non-zero-length normal vector
    // and an arbitrary point.
    Plane<double> equilibrium_plane_M{Vector3d::UnitZ(), Vector3d::Zero()};
    if (!CalcEquilibriumPlane(tet0, field0_M, tet1, field1_N, X_MN,
                              &equilibrium_plane_M)) {
      continue;
    }
    Vector3<double> polygon_nhat_M = equilibrium_plane_M.normal();
    if (!IsPlaneNormalAlongPressureGradient(polygon_nhat_M, tet0, field0_M)) {
      continue;
    }
    const math::RotationMatrixd R_NM = X_MN.rotation().inverse();
    Vector3<double> reverse_polygon_nhat_N = R_NM * (-polygon_nhat_M);
    if (!IsPlaneNormalAlongPressureGradient(reverse_polygon_nhat_N, tet1,
                                            field1_N)) {
      continue;
    }
    const std::vector<Vector3<double>>& polygon_vertices_M =
        IntersectTetrahedra(tet0, field0_M, tet1, field1_N, X_MN,
                            equilibrium_plane_M);

    if (polygon_vertices_M.size() < 3) continue;

    // Add the vertices to the builder (with corresponding pressure values)
    // and construct index-based polygon representation.
    std::vector<int> polygon_vertex_indices;
    polygon_vertex_indices.reserve(polygon_vertices_M.size());
    for (const auto& p_MV : polygon_vertices_M) {
      polygon_vertex_indices.push_back(
          builder.AddVertex(p_MV, field0_M.EvaluateCartesian(tet0, p_MV)));
    }

    const Vector3<double>& grad_field0_M = field0_M.EvaluateGradient(tet0);
    const int num_new_faces = builder.AddPolygon(polygon_vertex_indices,
                                                 polygon_nhat_M, grad_field0_M);

    const Vector3<double>& grad_field1_N = field1_N.EvaluateGradient(tet1);
    const Vector3<double>& grad_field1_M = X_MN.rotation() * grad_field1_N;
    for (int i = 0; i < num_new_faces; ++i) {
      grad_e0_Ms->push_back(grad_field0_M);
      grad_e1_Ms->push_back(grad_field1_M);
    }
  }

  if (builder.num_faces() == 0) return;

  std::tie(*surface_MN_M, *e_01_M) = builder.MakeMeshAndField();
}

template <class MeshType, class MeshBuilder, typename T, class FieldType>
std::unique_ptr<ContactSurface<double>> IntersectCompliantVolumes(
    GeometryId id0, const VolumeMeshFieldLinear<double, double>& field0_F,
    const Bvh<Obb, VolumeMesh<double>>& bvh0_F,
    const math::RigidTransform<double>& X_WF, GeometryId id1,
    const VolumeMeshFieldLinear<double, double>& field1_G,
    const Bvh<Obb, VolumeMesh<double>>& bvh1_G,
    const math::RigidTransform<double>& X_WG,
    MeshBuilder builder) {
  const math::RigidTransformd X_FG = X_WF.InvertAndCompose(X_WG);

  // The computation will be in Frame F and then transformed to the world frame.
  std::unique_ptr<MeshType> surface01;
  std::unique_ptr<FieldType> field01;
  std::vector<Vector3<double>> grad_field0_Fs;
  std::vector<Vector3<double>> grad_field1_Fs;
  FieldIntersection(field0_F, bvh0_F, field1_G, bvh1_G, X_FG, builder,
                    &surface01, &field01, &grad_field0_Fs, &grad_field1_Fs);

  if (surface01 == nullptr) return nullptr;

  // TODO(DamrongGuoy): Compute the mesh and field with the quantities
  //  expressed in World frame by construction so that we can delete these
  //  transforming methods.
  surface01->TransformVertices(X_WF);
  field01->Transform(X_WF);
  auto grad_field0_W = std::make_unique<std::vector<Vector3<double>>>();
  grad_field0_W->reserve(grad_field0_Fs.size());
  for (const Vector3<double>& grad_field0_F : grad_field0_Fs) {
    grad_field0_W->emplace_back(X_WF.rotation() * grad_field0_F);
  }
  auto grad_field1_W = std::make_unique<std::vector<Vector3<double>>>();
  grad_field1_W->reserve(grad_field1_Fs.size());
  for (const Vector3<double>& grad_field1_F : grad_field1_Fs) {
    grad_field1_W->emplace_back(X_WF.rotation() * grad_field1_F);
  }

  // The contact surface is documented as having the normals pointing *out* of
  // the second geometry and *into* the first geometry. This code creates a
  // surface mesh with normals pointing out of field1's geometry into field0's
  // geometry, so we make sure the ids are ordered so that the field1 is
  // the second id.
  return std::make_unique<ContactSurface<double>>(
      id0, id1, std::move(surface01), std::move(field01),
      std::move(grad_field0_W), std::move(grad_field1_W));
}

std::unique_ptr<ContactSurface<double>>
ComputeContactSurfaceFromCompliantVolumes(
    GeometryId id0, const VolumeMeshFieldLinear<double, double>& field0_F,
    const Bvh<Obb, VolumeMesh<double>>& bvh0_F,
    const math::RigidTransform<double>& X_WF,
    GeometryId id1, const VolumeMeshFieldLinear<double, double>& field1_G,
    const Bvh<Obb, VolumeMesh<double>>& bvh1_G,
    const math::RigidTransform<double>& X_WG,
    HydroelasticContactRepresentation representation) {
  if (representation == HydroelasticContactRepresentation::kTriangle) {
    return IntersectCompliantVolumes<TriangleSurfaceMesh<double>>(
        id0, field0_F, bvh0_F, X_WF, id1, field1_G, bvh1_G, X_WG,
        TriMeshBuilder<double>());
  } else {
    return IntersectCompliantVolumes<PolygonSurfaceMesh<double>>(
        id0, field0_F, bvh0_F, X_WF, id1, field1_G, bvh1_G, X_WG,
        PolyMeshBuilder<double>());
  }
}

std::unique_ptr<ContactSurface<AutoDiffXd>>
ComputeContactSurfaceFromCompliantVolumes(
    GeometryId, const VolumeMeshFieldLinear<double, double>&,
    const Bvh<Obb, VolumeMesh<double>>&,
    const math::RigidTransform<AutoDiffXd>&, GeometryId,
    const VolumeMeshFieldLinear<double, double>& ,
    const Bvh<Obb, VolumeMesh<double>>& ,
    const math::RigidTransform<AutoDiffXd>&,
    HydroelasticContactRepresentation) {
  throw std::logic_error(
      "ComputeContactSurfaceFromCompliantVolumes() does not "
      "support RigidTransform<AutoDiffXd> yet.");
}

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS((
  &IntersectTetrahedra<T>,
  &IntersectPlaneTetrahedron<T>
))

}  // namespace internal
}  // namespace geometry
}  // namespace drake
