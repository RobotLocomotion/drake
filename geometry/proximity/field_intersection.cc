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
std::vector<Vector3<T>> IntersectTetrahedra(
    int element0, const VolumeMesh<double>& mesh0_M, int element1,
    const VolumeMesh<double>& mesh1_N, const math::RigidTransform<T>& X_MN,
    const Plane<T>& equilibrium_plane_M) {
  // TODO(DamrongGuoy): Refactor this buffer from being a function-local
  //  variable to a class member variable to reduce heap allocations. Then,
  //  return the const reference. I cannot make them static function-local
  //  because it will create race condition in multithreading environment.

  // We use two alternating buffers to reduce heap allocations.
  std::vector<Vector3<T>> polygon_buffer[2];

  // Intersects the equilibrium plane with the tetrahedron element0.
  std::vector<Vector3<T>>* polygon_M = &(polygon_buffer[0]);
  SliceTetrahedronWithPlane(element0, mesh0_M, equilibrium_plane_M, polygon_M);
  RemoveNearlyDuplicateVertices(polygon_M);
  // Null polygon
  if (polygon_M->size() < 3) return {};

  // Positions of vertices of tetrahedral element1 in mesh1_N expressed in
  // frame M.
  Vector3<T> p_MVs[4];
  for (int i = 0; i < 4; ++i) {
    p_MVs[i] =
        X_MN * mesh1_N.vertex(mesh1_N.element(element1).vertex(i)).cast<T>();
  }
  // Each tuple of three vertex indices are oriented so that their normal
  // vector points outward from the tetrahedron.
  constexpr int kFaceVertexLocalIndex[4][3] = {
      {1, 2, 3}, {0, 3, 2}, {0, 1, 3}, {0, 2, 1}};
  std::vector<Vector3<T>>* in_M = polygon_M;
  std::vector<Vector3<T>>* out_M = &(polygon_buffer[1]);
  // Intersects the polygon with the four halfspaces of the four triangles
  // of the tetrahedral element1.
  for (const auto& face_vertices : kFaceVertexLocalIndex) {
    const Vector3<T>& p_MA = p_MVs[face_vertices[0]];
    const Vector3<T>& p_MB = p_MVs[face_vertices[1]];
    const Vector3<T>& p_MC = p_MVs[face_vertices[2]];
    const Vector3<T> triangle_outward_normal_M =
        (p_MB - p_MA).cross(p_MC - p_MA);
    PosedHalfSpace<T> half_space_M(triangle_outward_normal_M, p_MA);
    ClipPolygonByHalfSpace(*in_M, half_space_M, out_M);
    RemoveNearlyDuplicateVertices(out_M);
    if (out_M->size() < 3) {
      return {};  // Empty intersection; no contact here.
    }
    std::swap(in_M, out_M);
  }
  polygon_M = in_M;

  return *polygon_M;
}

template <typename T>
bool IsPlaneNormalAlongPressureGradient(
    const Vector3<T>& nhat_M, int tetrahedron,
    const VolumeMeshFieldLinear<double, double>& field_M) {
  const Vector3<double> grad_p_M = field_M.EvaluateGradient(tetrahedron);
  const T cos_theta = nhat_M.dot(grad_p_M.normalized());

  // TODO(DamrongGuoy): Consider exposing the threshold kAlpha to users.
  //  It should coordinate with IsFaceNormalInNormalDirection() for surface
  //  triangles.

  // We pick 5π/8 empirically to be the threshold angle, alpha.
  constexpr double kAlpha = 5. * M_PI / 8.;
  const double kCosAlpha = std::cos(kAlpha);
  // cos(θ) > cos(α) → θ < α → condition met.
  return cos_theta > kCosAlpha;
}

template <class MeshType, class MeshBuilder, typename T, class FieldType>
void IntersectFields(const VolumeMeshFieldLinear<double, double>& field0_M,
                     const Bvh<Obb, VolumeMesh<double>>& bvh0_M,
                     const VolumeMeshFieldLinear<double, double>& field1_N,
                     const Bvh<Obb, VolumeMesh<double>>& bvh1_N,
                     const math::RigidTransform<T>& X_MN,
                     std::unique_ptr<MeshType>* surface_01_M,
                     std::unique_ptr<FieldType>* e_01_M,
                     std::vector<Vector3<T>>* grad_e0_Ms,
                     std::vector<Vector3<T>>* grad_e1_Ms) {
  DRAKE_DEMAND(surface_01_M != nullptr);
  DRAKE_DEMAND(e_01_M != nullptr);
  DRAKE_DEMAND(grad_e0_Ms != nullptr);
  DRAKE_DEMAND(grad_e1_Ms != nullptr);
  surface_01_M->reset();
  e_01_M->reset();
  grad_e0_Ms->clear();
  grad_e1_Ms->clear();

  std::vector<std::pair<int, int>> candidate_tetrahedra;
  auto callback = [&candidate_tetrahedra](int tet0,
                                          int tet1) -> BvttCallbackResult {
    candidate_tetrahedra.emplace_back(tet0, tet1);
    return BvttCallbackResult::Continue;
  };
  bvh0_M.Collide(bvh1_N, convert_to_double(X_MN), callback);

  MeshBuilder builder;
  std::vector<SurfaceTriangle> surface_faces;
  std::vector<Vector3<T>> surface_vertices_M;
  std::vector<T> surface_field_values;
  // Here the contact polygon is represented as a list of vertex indices.
  std::vector<int> contact_polygon;
  // Each contact polygon has at most 8 vertices because it is the
  // intersection of the pressure-equilibrium plane and the two tetrahedra.
  // The plane intersects a tetrahedron into a convex polygon with at most four
  // vertices. That convex polygon intersects a tetrahedron into at most four
  // more vertices.
  contact_polygon.reserve(8);
  const math::RotationMatrix<T> R_NM = X_MN.rotation().inverse();
  for (const auto& [tet0, tet1] : candidate_tetrahedra) {
    // Initialize the plane with a non-zero-length normal vector
    // and an arbitrary point.
    Plane<T> equilibrium_plane_M{Vector3d::UnitZ(), Vector3d::Zero()};
    if (!CalcEquilibriumPlane(tet0, field0_M, tet1, field1_N, X_MN,
                              &equilibrium_plane_M)) {
      continue;
    }
    Vector3<T> polygon_nhat_M = equilibrium_plane_M.normal();
    if (!IsPlaneNormalAlongPressureGradient(polygon_nhat_M, tet0, field0_M)) {
      continue;
    }
    Vector3<T> reverse_polygon_nhat_N = R_NM * (-polygon_nhat_M);
    if (!IsPlaneNormalAlongPressureGradient(reverse_polygon_nhat_N, tet1,
                                            field1_N)) {
      continue;
    }
    const std::vector<Vector3<T>>& polygon_vertices_M =
        IntersectTetrahedra(tet0, field0_M.mesh(), tet1, field1_N.mesh(), X_MN,
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

    const Vector3<T>& grad_field0_M = field0_M.EvaluateGradient(tet0);
    const int num_new_faces = builder.AddPolygon(polygon_vertex_indices,
                                                 polygon_nhat_M, grad_field0_M);

    const Vector3<T>& grad_field1_N = field1_N.EvaluateGradient(tet1);
    const Vector3<T>& grad_field1_M = X_MN.rotation() * grad_field1_N;
    for (int i = 0; i < num_new_faces; ++i) {
      grad_e0_Ms->push_back(grad_field0_M);
      grad_e1_Ms->push_back(grad_field1_M);
    }
  }

  if (builder.num_faces() == 0) return;

  std::tie(*surface_01_M, *e_01_M) = builder.MakeMeshAndField();
}

template <class MeshType, class MeshBuilder, typename T, class FieldType>
std::unique_ptr<ContactSurface<T>> IntersectCompliantVolumes(
    GeometryId id0, const VolumeMeshFieldLinear<double, double>& field0_F,
    const Bvh<Obb, VolumeMesh<double>>& bvh0_F,
    const math::RigidTransform<T>& X_WF, GeometryId id1,
    const VolumeMeshFieldLinear<double, double>& field1_G,
    const Bvh<Obb, VolumeMesh<double>>& bvh1_G,
    const math::RigidTransform<T>& X_WG) {
  const math::RigidTransform<T> X_FG = X_WF.InvertAndCompose(X_WG);

  // The computation will be in Frame F and then transformed to the world frame.
  std::unique_ptr<MeshType> surface01_F;
  std::unique_ptr<FieldType> field01_F;
  std::vector<Vector3<T>> grad_field0_Fs;
  std::vector<Vector3<T>> grad_field1_Fs;
  IntersectFields<MeshType, MeshBuilder>(field0_F, bvh0_F, field1_G, bvh1_G,
                                         X_FG, &surface01_F, &field01_F,
                                         &grad_field0_Fs, &grad_field1_Fs);

  if (surface01_F == nullptr) return nullptr;

  // TODO(DamrongGuoy): Compute the mesh and field with the quantities
  //  expressed in World frame by construction so that we can delete these
  //  transforming methods.
  surface01_F->TransformVertices(X_WF);
  field01_F->Transform(X_WF);
  auto grad_field0_W = std::make_unique<std::vector<Vector3<T>>>();
  grad_field0_W->reserve(grad_field0_Fs.size());
  for (const Vector3<T>& grad_field0_F : grad_field0_Fs) {
    grad_field0_W->emplace_back(X_WF.rotation() * grad_field0_F);
  }
  auto grad_field1_W = std::make_unique<std::vector<Vector3<T>>>();
  grad_field1_W->reserve(grad_field1_Fs.size());
  for (const Vector3<T>& grad_field1_F : grad_field1_Fs) {
    grad_field1_W->emplace_back(X_WF.rotation() * grad_field1_F);
  }

  // The contact surface is documented as having the normals pointing *out* of
  // the second geometry and *into* the first geometry. This code creates a
  // surface mesh with normals pointing out of field1's geometry into field0's
  // geometry, so we make sure the ids are ordered so that the field1 is
  // the second id.
  return std::make_unique<ContactSurface<T>>(
      id0, id1, std::move(surface01_F), std::move(field01_F),
      std::move(grad_field0_W), std::move(grad_field1_W));
}

template <typename T>
std::unique_ptr<ContactSurface<T>> ComputeContactSurfaceFromCompliantVolumes(
    GeometryId id0, const VolumeMeshFieldLinear<double, double>& field0_F,
    const Bvh<Obb, VolumeMesh<double>>& bvh0_F,
    const math::RigidTransform<T>& X_WF, GeometryId id1,
    const VolumeMeshFieldLinear<double, double>& field1_G,
    const Bvh<Obb, VolumeMesh<double>>& bvh1_G,
    const math::RigidTransform<T>& X_WG,
    HydroelasticContactRepresentation representation) {
  if (representation == HydroelasticContactRepresentation::kTriangle) {
    return IntersectCompliantVolumes<TriangleSurfaceMesh<T>, TriMeshBuilder<T>>(
        id0, field0_F, bvh0_F, X_WF, id1, field1_G, bvh1_G, X_WG);
  } else {
    return IntersectCompliantVolumes<PolygonSurfaceMesh<T>, PolyMeshBuilder<T>>(
        id0, field0_F, bvh0_F, X_WF, id1, field1_G, bvh1_G, X_WG);
  }
}

//----------------------------------------------------------
// Template instantiations
//----------------------------------------------------------

// Triangle, double
template void
IntersectFields<TriangleSurfaceMesh<double>, TriMeshBuilder<double>>(
    const VolumeMeshFieldLinear<double, double>& field0_M,
    const Bvh<Obb, VolumeMesh<double>>& bvh0_M,
    const VolumeMeshFieldLinear<double, double>& field1_N,
    const Bvh<Obb, VolumeMesh<double>>& bvh1_N,
    const math::RigidTransform<double>& X_MN,
    std::unique_ptr<TriangleSurfaceMesh<double>>* surface_01_M,
    std::unique_ptr<TriangleSurfaceMeshFieldLinear<double, double>>* e_01_M,
    std::vector<Vector3<double>>* grad_e0_Ms,
    std::vector<Vector3<double>>* grad_e1_Ms);
// Polygon, double
template void
IntersectFields<PolygonSurfaceMesh<double>, PolyMeshBuilder<double>>(
    const VolumeMeshFieldLinear<double, double>& field0_M,
    const Bvh<Obb, VolumeMesh<double>>& bvh0_M,
    const VolumeMeshFieldLinear<double, double>& field1_N,
    const Bvh<Obb, VolumeMesh<double>>& bvh1_N,
    const math::RigidTransform<double>& X_MN,
    std::unique_ptr<PolygonSurfaceMesh<double>>* surface_01_M,
    std::unique_ptr<PolygonSurfaceMeshFieldLinear<double, double>>* e_01_M,
    std::vector<Vector3<double>>* grad_e0_Ms,
    std::vector<Vector3<double>>* grad_e1_Ms);
// Triangle, AutoDiffXd
template void
IntersectFields<TriangleSurfaceMesh<AutoDiffXd>, TriMeshBuilder<AutoDiffXd>>(
    const VolumeMeshFieldLinear<double, double>& field0_M,
    const Bvh<Obb, VolumeMesh<double>>& bvh0_M,
    const VolumeMeshFieldLinear<double, double>& field1_N,
    const Bvh<Obb, VolumeMesh<double>>& bvh1_N,
    const math::RigidTransform<AutoDiffXd>& X_MN,
    std::unique_ptr<TriangleSurfaceMesh<AutoDiffXd>>* surface_01_M,
    std::unique_ptr<TriangleSurfaceMeshFieldLinear<AutoDiffXd, AutoDiffXd>>*
        e_01_M,
    std::vector<Vector3<AutoDiffXd>>* grad_e0_Ms,
    std::vector<Vector3<AutoDiffXd>>* grad_e1_Ms);
// Polygon, AutoDiffXd
template void
IntersectFields<PolygonSurfaceMesh<AutoDiffXd>, PolyMeshBuilder<AutoDiffXd>>(
    const VolumeMeshFieldLinear<double, double>& field0_M,
    const Bvh<Obb, VolumeMesh<double>>& bvh0_M,
    const VolumeMeshFieldLinear<double, double>& field1_N,
    const Bvh<Obb, VolumeMesh<double>>& bvh1_N,
    const math::RigidTransform<AutoDiffXd>& X_MN,
    std::unique_ptr<PolygonSurfaceMesh<AutoDiffXd>>* surface_01_M,
    std::unique_ptr<PolygonSurfaceMeshFieldLinear<AutoDiffXd, AutoDiffXd>>*
        e_01_M,
    std::vector<Vector3<AutoDiffXd>>* grad_e0_Ms,
    std::vector<Vector3<AutoDiffXd>>* grad_e1_Ms);

// Triangle, double
template std::unique_ptr<ContactSurface<double>>
IntersectCompliantVolumes<TriangleSurfaceMesh<double>, TriMeshBuilder<double>>(
    GeometryId id0, const VolumeMeshFieldLinear<double, double>& field0_F,
    const Bvh<Obb, VolumeMesh<double>>& bvh0_F,
    const math::RigidTransform<double>& X_WF, GeometryId id1,
    const VolumeMeshFieldLinear<double, double>& field1_G,
    const Bvh<Obb, VolumeMesh<double>>& bvh1_G,
    const math::RigidTransform<double>& X_WG);
// Polygon, double
template std::unique_ptr<ContactSurface<double>>
IntersectCompliantVolumes<PolygonSurfaceMesh<double>, PolyMeshBuilder<double>>(
    GeometryId id0, const VolumeMeshFieldLinear<double, double>& field0_F,
    const Bvh<Obb, VolumeMesh<double>>& bvh0_F,
    const math::RigidTransform<double>& X_WF, GeometryId id1,
    const VolumeMeshFieldLinear<double, double>& field1_G,
    const Bvh<Obb, VolumeMesh<double>>& bvh1_G,
    const math::RigidTransform<double>& X_WG);
// Triangle, AutoDiffXd
template std::unique_ptr<ContactSurface<AutoDiffXd>> IntersectCompliantVolumes<
    TriangleSurfaceMesh<AutoDiffXd>, TriMeshBuilder<AutoDiffXd>>(
    GeometryId id0, const VolumeMeshFieldLinear<double, double>& field0_F,
    const Bvh<Obb, VolumeMesh<double>>& bvh0_F,
    const math::RigidTransform<AutoDiffXd>& X_WF, GeometryId id1,
    const VolumeMeshFieldLinear<double, double>& field1_G,
    const Bvh<Obb, VolumeMesh<double>>& bvh1_G,
    const math::RigidTransform<AutoDiffXd>& X_WG);
// Polygon, AutoDiffXd
template std::unique_ptr<ContactSurface<AutoDiffXd>> IntersectCompliantVolumes<
    PolygonSurfaceMesh<AutoDiffXd>, PolyMeshBuilder<AutoDiffXd>>(
    GeometryId id0, const VolumeMeshFieldLinear<double, double>& field0_F,
    const Bvh<Obb, VolumeMesh<double>>& bvh0_F,
    const math::RigidTransform<AutoDiffXd>& X_WF, GeometryId id1,
    const VolumeMeshFieldLinear<double, double>& field1_G,
    const Bvh<Obb, VolumeMesh<double>>& bvh1_G,
    const math::RigidTransform<AutoDiffXd>& X_WG);

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    (&CalcEquilibriumPlane<T>, &IntersectTetrahedra<T>,
     &IsPlaneNormalAlongPressureGradient<T>,
     &ComputeContactSurfaceFromCompliantVolumes<T>))

}  // namespace internal
}  // namespace geometry
}  // namespace drake
