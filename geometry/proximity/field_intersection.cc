#include "drake/geometry/proximity/field_intersection.h"

#include <memory>
#include <unordered_map>
#include <utility>

#include "drake/common/default_scalars.h"
#include "drake/geometry/proximity/mesh_intersection.h"
#include "drake/geometry/proximity/mesh_plane_intersection.h"
#include "drake/geometry/proximity/posed_half_space.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
namespace internal {

using Eigen::Vector3d;

template <typename T>
bool CalcEquilibriumPlane(VolumeElementIndex element0,
                          const VolumeMeshFieldLinear<double, double>& field0_M,
                          VolumeElementIndex element1,
                          const VolumeMeshFieldLinear<double, double>& field1_N,
                          const math::RigidTransform<T>& X_MN,
                          Plane<T>* plane_M) {
  const Vector3d df0_M = field0_M.EvaluateGradient(element0);
  const Vector3d p_MMo = Vector3d::Zero();
  const double f0_Mo = field0_M.EvaluateCartesian(element0, p_MMo);

  const Vector3d df1_N = field1_N.EvaluateGradient(element1);
  const Vector3<T> df1_M = X_MN.rotation() * df1_N.cast<T>();
  const Vector3<T> p_NMo = X_MN.inverse() * p_MMo.cast<T>();
  const T f1_Mo = field1_N.EvaluateCartesian(element1, p_NMo);

  // Extend the linear field field0 within tetrahedron element0 to the entire
  // space as:
  //      f0(p_MQ) = df0_M.dot(p_MQ) + f0_Mo.             (1)
  // Extend the linear field field1 within tetrahedron element1 to the entire
  // space as:
  //      f1(p_MQ) = df1_M.dot(p_MQ) + f1_Mo.             (2)
  // This is an equation of the equilibrium plane, where the two functions
  // have the same value:
  //   (df0_M - df1_M).dot(p_MQ) + (f0_Mo - f1_Mo) = 0.   (3)
  // Therefore, a vector n_M perpendicular to the equilibrium plane is:
  //             n_M = df0_M - df1_M.                     (4)
  const Vector3<T> n_M = df0_M - df1_M;
  if (n_M.norm() == 0) {
    return false;
  }

  // Using n_M in the plane equation (3), we have the plane equation:
  //   n_M.dot(p_MQ) = f1_Mo - f0_Mo,                     (5)
  // and a possible point on that plane is:
  //            p_MQ = (f1_Mo - f0_Mo)*n_M/n_M.dot(n_M).  (6)
  const Vector3<T> p_MQ = (f1_Mo - f0_Mo) * n_M / n_M.dot(n_M);

  *plane_M = Plane<T>{n_M, p_MQ};
  return true;
}

// TODO(DamrongGuoy): Refactor ClipPolygonByHalfSpace() to share between
//  mesh_intersection and field_intersection instead of this hack.
template<typename T>
class SurfaceVolumeIntersectorTester {
 public:
  void ClipPolygonByHalfSpace(const std::vector<Vector3<T>>& polygon_vertices_F,
                              const PosedHalfSpace<double>& H_F,
                              std::vector<Vector3<T>>* output_vertices_F) {
    intersect_.ClipPolygonByHalfSpace(polygon_vertices_F, H_F,
                                      output_vertices_F);
  }
 private:
  SurfaceVolumeIntersector<T> intersect_;
};

template <typename T>
void IntersectPlaneTetrahedron(
    VolumeElementIndex tetrahedron,
    const VolumeMeshFieldLinear<double, double>& field_M,
    const Plane<T>& plane_M, std::vector<Vector3<T>>* polygon_M) {
  std::vector<SurfaceFace> unused_faces;
  std::vector<SurfaceVertex<T>> vertices_with_centroid_M;
  std::vector<T> unused_surface_e;
  // TODO(DamrongGuoy): Remove #include <unordered_map> when we remove
  //  unused_cut_edges.
  std::unordered_map<SortedPair<VolumeVertexIndex>, SurfaceVertexIndex>
      unused_cut_edges;

  SliceTetWithPlane(
      tetrahedron, field_M, plane_M, math::RigidTransform<T>::Identity(),
      ContactPolygonRepresentation::kCentroidSubdivision, &unused_faces,
      &vertices_with_centroid_M, &unused_surface_e, &unused_cut_edges);

  int num_vertices_exclude_centroid = vertices_with_centroid_M.size() - 1;
  for (int i = 0; i < num_vertices_exclude_centroid; ++i) {
    polygon_M->emplace_back(vertices_with_centroid_M[i].r_MV());
  }
}

template <typename T>
std::vector<Vector3<T>> IntersectTetrahedra(
    VolumeElementIndex element0,
    const VolumeMeshFieldLinear<double, double>& field0_M,
    VolumeElementIndex element1,
    const VolumeMeshFieldLinear<double, double>& field1_N,
    const math::RigidTransform<T>& X_MN,
    const Plane<T>& equilibrium_plane_M) {
  // TODO(DamrongGuoy): Refactor this buffer from being a local variable to
  //  being a class variable to decrease heap allocations. Then, return
  //  the const reference ("const std::vector<Vector3<T>>&").
  std::vector<Vector3<T>> polygon_buffer[2];

  // Intersects the equilibrium plane with the tetrahedron element0.
  std::vector<Vector3<T>>* polygon_M = &(polygon_buffer[0]);
  IntersectPlaneTetrahedron(element0, field0_M, equilibrium_plane_M, polygon_M);

  // Intersection of the above polygon with the four halfspaces of the four
  // faces of the tetrahedron element1. Expressed in frame M.
  Vector3<double> p_MVs[4];
  for (int i = 0; i < 4; ++i) {
    Vector3d p_NVi = field1_N.mesh()
                         .vertex(field1_N.mesh().element(element1).vertex(i))
                         .r_MV();
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
    SurfaceVolumeIntersectorTester<T>().ClipPolygonByHalfSpace(
        *in_M, half_space_M, out_M);
    std::swap(in_M, out_M);
  }
  polygon_M = in_M;

  // TODO(DamrongGuoy): Take care of duplicated vertices or vertices that are
  //  so closed together. Use RemoveDuplicateVertices() of
  //  SurfaceVolumeIntersector.

  return *polygon_M;
}

bool IsPlaneNormalAlongPressureGradient(
    const Vector3<double>& nhat_M, VolumeElementIndex tetrahedron,
    const VolumeMeshFieldLinear<double, double>& field_M) {
  const Vector3<double> grad_p_M = field_M.EvaluateGradient(tetrahedron);
  const double cos_theta = nhat_M.dot(grad_p_M.normalized());
  // We pick 5π/8 empirically to be the threshold angle, alpha.
  constexpr double kAlpha = 5. * M_PI / 8.;
  static const double kCosAlpha = std::cos(kAlpha);
  // cos(θ) > cos(α) → θ < α → condition met.
  return cos_theta > kCosAlpha;
}

void FieldIntersection(
    const VolumeMeshFieldLinear<double, double>& field0_M,
    const Bvh<Obb, VolumeMesh<double>>& bvh0_M,
    const VolumeMeshFieldLinear<double, double>& field1_N,
    const Bvh<Obb, VolumeMesh<double>>& bvh1_N,
    const math::RigidTransform<double>& X_MN,
    ContactPolygonRepresentation representation,
    std::unique_ptr<SurfaceMesh<double>>* surface_MN_M,
    std::unique_ptr<SurfaceMeshFieldLinear<double, double>>* e_01_M,
    std::vector<Vector3<double>>* grad_e0_Ms,
    std::vector<Vector3<double>>* grad_e1_Ms) {
  DRAKE_DEMAND(surface_MN_M != nullptr);
  DRAKE_DEMAND(e_01_M != nullptr);
  DRAKE_DEMAND(grad_e0_Ms != nullptr);
  DRAKE_DEMAND(grad_e1_Ms != nullptr);
  grad_e0_Ms->clear();
  grad_e1_Ms->clear();

  std::vector<std::pair<VolumeElementIndex, VolumeElementIndex>> candidates;
  auto callback = [&candidates](VolumeElementIndex tet0,
                                VolumeElementIndex tet1) -> BvttCallbackResult {
    candidates.emplace_back(tet0, tet1);
    return BvttCallbackResult::Continue;
  };
  bvh0_M.Collide(bvh1_N, X_MN, callback);

  std::vector<SurfaceFace> surface_faces;
  std::vector<SurfaceVertex<double>> surface_vertices_M;
  std::vector<double> surface_field_values;
  // We know that each contact polygon has at most 8 vertices because it is
  // the intersection of the pressure-equilibrium plane and the two tetrahedra.
  // The plane intersects a tetrahedron into a convex polygon with at most four
  // vertices. That convex polygon intersects a tetrahedron into at most four
  // more vertices.
  std::vector<SurfaceVertexIndex> contact_polygon;
  contact_polygon.reserve(8);
  for (const auto [tet0, tet1] : candidates) {
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

    const size_t num_polygon_vertices = polygon_vertices_M.size();
    if (num_polygon_vertices < 3) continue;

    const size_t num_previous_vertices = surface_vertices_M.size();
    if (representation == ContactPolygonRepresentation::kCentroidSubdivision) {
      contact_polygon.clear();
      for (size_t i = 0; i < num_polygon_vertices; ++i) {
        contact_polygon.emplace_back(surface_vertices_M.size());
        surface_vertices_M.emplace_back(polygon_vertices_M[i]);
      }
    }
    const size_t num_previous_faces = surface_faces.size();
    switch (representation) {
      case ContactPolygonRepresentation::kCentroidSubdivision:
        AddPolygonToMeshData(contact_polygon, polygon_nhat_M, &surface_faces,
                             &surface_vertices_M);
        break;
      case ContactPolygonRepresentation::kSingleTriangle:
        AddPolygonToMeshDataAsOneTriangle(polygon_vertices_M, polygon_nhat_M,
                                          &surface_faces, &surface_vertices_M);
        break;
    }
    // Assign field values at the new vertices.
    for (size_t v = num_previous_vertices; v < surface_vertices_M.size();
         ++v) {
      surface_field_values.push_back(
          field0_M.EvaluateCartesian(tet0, surface_vertices_M[v].r_MV()));
    }
    // Assign per-face gradient at the new faces.
    const Vector3<double>& grad_field0_M = field0_M.EvaluateGradient(tet0);
    const Vector3<double>& grad_field1_N = field1_N.EvaluateGradient(tet1);
    const Vector3<double>& grad_field1_M = X_MN.rotation() * grad_field1_N;
    for (size_t i = num_previous_faces; i < surface_faces.size(); ++i) {
      grad_e0_Ms->push_back(grad_field0_M);
      grad_e1_Ms->push_back(grad_field1_M);
    }
  }

  DRAKE_DEMAND(surface_vertices_M.size() == surface_field_values.size());
  if (surface_faces.empty()) return;

  *surface_MN_M = std::make_unique<SurfaceMesh<double>>(
      std::move(surface_faces), std::move(surface_vertices_M));
  const bool calculate_gradient = false;
  *e_01_M = std::make_unique<SurfaceMeshFieldLinear<double, double>>(
      field0_M.name(), std::move(surface_field_values), surface_MN_M->get(),
      calculate_gradient);
}

std::unique_ptr<ContactSurface<double>>
ComputeContactSurfaceFromCompliantVolumes(
    GeometryId id0, const VolumeMeshFieldLinear<double, double>& field0_F,
    const Bvh<Obb, VolumeMesh<double>>& bvh0_F,
    const math::RigidTransform<double>& X_WF,
    GeometryId id1, const VolumeMeshFieldLinear<double, double>& field1_G,
    const Bvh<Obb, VolumeMesh<double>>& bvh1_G,
    const math::RigidTransform<double>& X_WG,
    ContactPolygonRepresentation representation) {
  const math::RigidTransformd X_FG = X_WF.InvertAndCompose(X_WG);

  // The computation will be in Frame F and then transformed to the world frame.
  std::unique_ptr<SurfaceMesh<double>> surface01;
  std::unique_ptr<SurfaceMeshFieldLinear<double, double>> field01;
  std::vector<Vector3<double>> grad_field0_Fs;
  std::vector<Vector3<double>> grad_field1_Fs;
  FieldIntersection(field0_F, bvh0_F, field1_G, bvh1_G, X_FG, representation,
                    &surface01, &field01, &grad_field0_Fs, &grad_field1_Fs);

  if (surface01 == nullptr) return nullptr;

  // TODO(DamrongGuoy): Compute the mesh and field with the quantities
  //  expressed in World frame by construction so that we can delete these
  //  transforming methods.
  surface01->TransformVertices(X_WF);
  field01->TransformGradients(X_WF);
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

std::unique_ptr<ContactSurface<AutoDiffXd>>
ComputeContactSurfaceFromCompliantVolumes(
    GeometryId, const VolumeMeshFieldLinear<double, double>&,
    const Bvh<Obb, VolumeMesh<double>>&,
    const math::RigidTransform<AutoDiffXd>&, GeometryId,
    const VolumeMeshFieldLinear<double, double>& ,
    const Bvh<Obb, VolumeMesh<double>>& ,
    const math::RigidTransform<AutoDiffXd>& ,
    ContactPolygonRepresentation ) {
  throw std::logic_error(
      "ComputeContactSurfaceFromCompliantVolumes() does not "
      "support RigidTransform<AutoDiffXd> yet.");
}

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS((
  &IntersectTetrahedra<T>
))


}  // namespace internal
}  // namespace geometry
}  // namespace drake
