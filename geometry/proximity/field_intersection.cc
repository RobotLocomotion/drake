#include "drake/geometry/proximity/field_intersection.h"

#include <array>
#include <queue>
#include <unordered_map>
#include <unordered_set>

#include "drake/common/default_scalars.h"
#include "drake/geometry/proximity/contact_surface_utility.h"
#include "drake/geometry/proximity/mesh_intersection.h"
#include "drake/geometry/proximity/mesh_plane_intersection.h"
#include "drake/geometry/proximity/posed_half_space.h"

namespace drake {
namespace geometry {
namespace internal {

using Eigen::Vector3d;

namespace {

struct TetPairHasher final {
  std::size_t operator()(const std::pair<int, int>& p) const {
    return std::hash<int>()(p.first) ^ std::hash<int>()(p.second);
  }
};

}  // namespace

template <typename T>
bool CalcEquilibriumPlane(int element0,
                          const VolumeMeshFieldLinear<double, double>& field0_M,
                          int element1,
                          const VolumeMeshFieldLinear<double, double>& field1_N,
                          const math::RigidTransform<T>& X_MN,
                          Plane<T>* plane_M) {
  const Vector3d grad_f0_M = field0_M.EvaluateGradient(element0);
  // Value of f₀ at the origin of frame M.
  const double f0_Mo = field0_M.EvaluateAtMo(element0);

  const Vector3d grad_f1_N = field1_N.EvaluateGradient(element1);
  const Vector3<T> grad_f1_M = X_MN.rotation() * grad_f1_N.cast<T>();
  // p_NoMo_N = R_NM * p_NoMo_M = R_MN⁻¹ * (-p_MoNo_M)
  const Vector3<T> p_NMo =
      -(X_MN.rotation().matrix().transpose() * X_MN.translation());

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
  const Vector3<T> p_MQ = ((f1_Mo - f0_Mo) / magnitude) * nhat_M;

  *plane_M = Plane<T>(nhat_M, p_MQ, /*already_normalized = */ true);
  return true;
}

template <typename T>
std::pair<std::vector<Vector3<T>>, std::vector<int>> IntersectTetrahedra(
    int element0, const VolumeMesh<double>& mesh0_M, int element1,
    const VolumeMesh<double>& mesh1_N, const math::RigidTransform<T>& X_MN,
    const Plane<T>& equilibrium_plane_M) {
  // TODO(DamrongGuoy): Refactor this buffer from being a function-local
  //  variable to a class member variable to reduce heap allocations. Then,
  //  return the const reference. I cannot make them static function-local
  //  because it will create race condition in multithreading environment.

  // We use two alternating buffers to reduce heap allocations.
  std::vector<Vector3<T>> polygon_buffer[2];
  std::vector<int> face_buffer[2];
  std::array<T, 8> distances;

  // Each contact polygon has at most 8 vertices because it is the
  // intersection of the pressure-equilibrium plane and the two tetrahedra.
  // The plane intersects a tetrahedron into a convex polygon with at most four
  // vertices. That convex polygon intersects a tetrahedron into at most four
  // more vertices.
  polygon_buffer[0].reserve(8);
  polygon_buffer[1].reserve(8);
  face_buffer[0].reserve(8);
  face_buffer[1].reserve(8);

  // Intersects the equilibrium plane with the tetrahedron element0.
  SliceTetrahedronWithPlane(element0, mesh0_M, equilibrium_plane_M,
                            &polygon_buffer[0], nullptr /* cut_edges */,
                            &face_buffer[0]);

  RemoveNearlyDuplicateVertices(&polygon_buffer[0]);
  if (polygon_buffer[0].size() < 3) {
    return {};
  }

  // Positions of vertices of tetrahedral element1 in mesh1_N expressed in
  // frame M.
  Vector3<T> p_MVs[4];
  for (int i = 0; i < 4; ++i) {
    p_MVs[i] =
        X_MN * mesh1_N.vertex(mesh1_N.element(element1).vertex(i)).cast<T>();
  }
  std::vector<Vector3<T>>* current_polygon = &polygon_buffer[0];
  std::vector<Vector3<T>>* clipped_polygon = &polygon_buffer[1];
  std::vector<int>* current_faces = &face_buffer[0];
  std::vector<int>* clipped_faces = &face_buffer[1];

  // Intersects the polygon with the four halfspaces of the four triangles
  // of the tetrahedral element1.
  for (int face = 0; face < 4; ++face) {
    clipped_polygon->clear();
    clipped_faces->clear();

    // 'face' corresponds to the triangle formed by {0, 1, 2, 3} - {face}
    // so any of (face+1)%4, (face+2)%4, (face+3)%4 are candidates for a
    // point on the face's plane. We arbitrarily choose (face + 1) % 4.
    const Vector3<T>& p_MA = p_MVs[(face + 1) % 4];
    const Vector3<T> triangle_outward_normal_M =
        X_MN.rotation() * -mesh1_N.inward_normal(element1, face).cast<T>();
    PosedHalfSpace<T> half_space_M(triangle_outward_normal_M, p_MA);

    const int size = ssize(*current_polygon);

    for (int i = 0; i < size; ++i) {
      distances[i] = half_space_M.CalcSignedDistance((*current_polygon)[i]);
    }

    // Walk the vertices checking for intersecting edges.
    for (int i = 0; i < size; ++i) {
      int j = (i + 1) % size;
      // If the edge out of vertex i is at least partially inside, include
      // vertex i and face i.
      if (distances[i] <= 0) {
        clipped_polygon->push_back((*current_polygon)[i]);
        clipped_faces->push_back((*current_faces)[i]);
        // If vertex j is outside, we've discovered an intersection. Add the
        // intersection point as well as "face" to the list of faces. (For faces
        // from element1 we insert face + 4, to indicate which element the edge
        // came from).
        if (distances[j] > 0) {
          const T wa = distances[j] / (distances[j] - distances[i]);
          const T wb = T(1.0) - wa;
          clipped_polygon->push_back(wa * (*current_polygon)[i] +
                                     wb * (*current_polygon)[j]);
          clipped_faces->push_back(face + 4);
        }
      } else if (distances[j] <= 0) {
        // Vertex i is outside, but vertex j is at least partially inside. We've
        // discovered another intersection. Add the intersection point, this
        // point is still on the edge created by current_faces[i], so include
        // that face for the edge starting at the intersection point.
        const T wa = distances[j] / (distances[j] - distances[i]);
        const T wb = T(1.0) - wa;
        clipped_polygon->push_back(wa * (*current_polygon)[i] +
                                   wb * (*current_polygon)[j]);
        clipped_faces->push_back((*current_faces)[i]);
      }
    }
    std::swap(current_polygon, clipped_polygon);
    std::swap(current_faces, clipped_faces);

    // If we've clipped off the entire polygon, return empty.
    if (ssize(*current_polygon) == 0) {
      return {};
    }
  }

  RemoveNearlyDuplicateVertices(current_polygon);
  if (current_polygon->size() < 3) {
    return {};
  }

  return std::make_pair(*current_polygon, *current_faces);
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

template <class MeshBuilder, class BvType>
void VolumeIntersector<MeshBuilder, BvType>::IntersectFields(
    const VolumeMeshFieldLinear<double, double>& field0_M,
    const Bvh<BvType, VolumeMesh<double>>& bvh0_M,
    const VolumeMeshFieldLinear<double, double>& field1_N,
    const Bvh<BvType, VolumeMesh<double>>& bvh1_N,
    const math::RigidTransform<T>& X_MN,
    std::unique_ptr<MeshType>* surface_01_M,
    std::unique_ptr<FieldType>* e_01_M) {
  DRAKE_DEMAND(surface_01_M != nullptr);
  DRAKE_DEMAND(e_01_M != nullptr);
  surface_01_M->reset();
  e_01_M->reset();
  tet0_of_contact_polygon_.clear();
  tet1_of_contact_polygon_.clear();

  std::vector<std::pair<int, int>> candidate_tetrahedra;
  auto callback = [&candidate_tetrahedra](int tet0,
                                          int tet1) -> BvttCallbackResult {
    candidate_tetrahedra.emplace_back(tet0, tet1);
    return BvttCallbackResult::Continue;
  };

  bvh0_M.Collide(bvh1_N, convert_to_double(X_MN), callback);

  MeshBuilder builder_M;
  const math::RotationMatrix<T> R_NM = X_MN.rotation().inverse();
  for (const auto& [tet0, tet1] : candidate_tetrahedra) {
    CalcContactPolygon(field0_M, field1_N, X_MN, R_NM, tet0, tet1, &builder_M);
  }

  if (builder_M.num_faces() == 0) return;

  std::tie(*surface_01_M, *e_01_M) = builder_M.MakeMeshAndField();
}

template <class MeshBuilder, class BvType>
void VolumeIntersector<MeshBuilder, BvType>::IntersectFields(
    const VolumeMeshFieldLinear<double, double>& field0_M,
    const Bvh<Obb, TriangleSurfaceMesh<double>>& bvh0_M,
    const std::vector<TetFace>& tri_to_tet_M,
    const VolumeMeshTopology& mesh_topology_M,
    const VolumeMeshFieldLinear<double, double>& field1_N,
    const Bvh<Obb, TriangleSurfaceMesh<double>>& bvh1_N,
    const std::vector<TetFace>& tri_to_tet_N,
    const VolumeMeshTopology& mesh_topology_N,
    const math::RigidTransform<T>& X_MN,
    std::unique_ptr<MeshType>* surface_01_M,
    std::unique_ptr<FieldType>* e_01_M) {
  DRAKE_DEMAND(surface_01_M != nullptr);
  DRAKE_DEMAND(e_01_M != nullptr);
  surface_01_M->reset();
  e_01_M->reset();
  tet0_of_contact_polygon_.clear();
  tet1_of_contact_polygon_.clear();

  const math::RigidTransform<double> X_MNd = convert_to_double(X_MN);

  // Keep track of pairs of tet indices that have already been inserted into the
  // queue.
  std::unordered_set<std::pair<int, int>, TetPairHasher> visited_pairs;

  std::queue<std::pair<int, int>> candidate_tetrahedra;
  auto callback = [&candidate_tetrahedra, &visited_pairs, &tri_to_tet_M,
                   &tri_to_tet_N](int tri0, int tri1) -> BvttCallbackResult {
    DRAKE_ASSERT(0 <= tri0 && tri0 < ssize(tri_to_tet_M));
    DRAKE_ASSERT(0 <= tri1 && tri1 < ssize(tri_to_tet_N));
    std::pair<int, int> tet_pair(tri_to_tet_M[tri0].tet_index,
                                 tri_to_tet_N[tri1].tet_index);
    if (!visited_pairs.contains(tet_pair)) {
      candidate_tetrahedra.push(tet_pair);
      visited_pairs.insert(tet_pair);
    }
    return BvttCallbackResult::Continue;
  };

  // Collide the BVHs of the surface meshes and enqueue the candidate surface
  // tetrahedra. The pressure range of surface tetrahedra are guaranteed to
  // contain 0 (even if the surface vertex values are not 0 due to margin),
  // thus if the tetrahedra overlap spatially they are guaranteed to produce
  // a non-empty contact polygon.
  bvh0_M.Collide(bvh1_N, X_MNd, callback);

  if (candidate_tetrahedra.empty()) return;

  MeshBuilder builder_M;
  const math::RotationMatrix<T> R_NM = X_MN.rotation().inverse();

  // The intersected faces of tetM and tetN with the contact polygon.
  std::vector<int> faces;

  // Traverse all overlapping tet pairs and generate contact polygons.
  while (!candidate_tetrahedra.empty()) {
    const std::pair<int, int> pair = candidate_tetrahedra.front();
    candidate_tetrahedra.pop();

    const auto& [tetM, tetN] = pair;
    // If a pair makes it into the queue, their pressure ranges intersect. But,
    // they may or may not overlap. If they do not overlap and produce a contact
    // surface, we do not have to process any neighbors. Note: the initial set
    // of candidate tets are all boundary tets, so their pressure ranges
    // necessarily intersect because they all contain 0 and are thus inserted
    // without checking range intersection.
    faces = CalcContactPolygon(field0_M, field1_N, X_MN, R_NM, tetM, tetN,
                               &builder_M);

    if (ssize(faces) == 0) {
      continue;
    }

    // Loop over all intersected faces.
    for (int face : faces) {
      // If face ∈ [0, 3], then it is an index of a face of tetM.
      if (face < 4) {
        // The index of the neighbor of tetM sharing the face with index face.
        int neighborM = mesh_topology_M.neighbor(tetM, face);
        // If there is no neighbor, continue.
        if (neighborM < 0) continue;

        // neighbor and tetN's pressure ranges intersect necessarily (the equal
        // pressure plane intersects a face of neighbor). They may overlap
        // spatially as well, so try to insert them into the queue.
        std::pair<int, int> neighbor_pair(neighborM, tetN);

        // If this candidate pair has already been checked, continue.
        if (visited_pairs.contains(neighbor_pair)) {
          continue;
        }

        candidate_tetrahedra.push(neighbor_pair);
        visited_pairs.insert(neighbor_pair);

      } else {
        // face ∈ [4, 7], thus face - 4 is a face of tetN.
        int neighborN = mesh_topology_N.neighbor(tetN, face - 4);
        // If there is no neighbor, continue.
        if (neighborN < 0) continue;

        // neighbor and tetM's pressure ranges intersect necessarily (the equal
        // pressure plane intersects a face of neighbor). They may overlap
        // spatially as well, so try to insert them into the queue.
        std::pair<int, int> neighbor_pair(tetM, neighborN);

        // If this candidate pair has already been checked, continue.
        if (visited_pairs.contains(neighbor_pair)) {
          continue;
        }

        candidate_tetrahedra.push(neighbor_pair);
        visited_pairs.insert(neighbor_pair);
      }
    }
  }

  if (builder_M.num_faces() == 0) return;

  std::tie(*surface_01_M, *e_01_M) = builder_M.MakeMeshAndField();
}

template <class MeshBuilder, class BvType>
std::vector<int> VolumeIntersector<MeshBuilder, BvType>::CalcContactPolygon(
    const VolumeMeshFieldLinear<double, double>& field0_M,
    const VolumeMeshFieldLinear<double, double>& field1_N,
    const math::RigidTransform<T>& X_MN, const math::RotationMatrix<T>& R_NM,
    const int tet0, const int tet1, MeshBuilder* builder_M) {
  // Initialize the plane with a non-zero-length normal vector
  // and an arbitrary point.
  Plane<T> equilibrium_plane_M{Vector3d::UnitZ(), Vector3d::Zero()};
  if (!CalcEquilibriumPlane(tet0, field0_M, tet1, field1_N, X_MN,
                            &equilibrium_plane_M)) {
    return {};
  }
  // The normal points in the direction of increasing field_0 and decreasing
  // field_1.
  Vector3<T> polygon_nhat_M = equilibrium_plane_M.unit_normal();
  if (!IsPlaneNormalAlongPressureGradient(polygon_nhat_M, tet0, field0_M)) {
    return {};
  }
  Vector3<T> reverse_polygon_nhat_N = R_NM * (-polygon_nhat_M);
  if (!IsPlaneNormalAlongPressureGradient(reverse_polygon_nhat_N, tet1,
                                          field1_N)) {
    return {};
  }
  const auto [polygon_vertices_M, faces] = IntersectTetrahedra(
      tet0, field0_M.mesh(), tet1, field1_N.mesh(), X_MN, equilibrium_plane_M);

  if (polygon_vertices_M.size() < 3) return {};

  // Add the vertices to the builder_M (with corresponding pressure values)
  // and construct index-based polygon representation.
  std::vector<int> polygon_vertex_indices;
  polygon_vertex_indices.reserve(polygon_vertices_M.size());
  for (const auto& p_MV : polygon_vertices_M) {
    polygon_vertex_indices.push_back(
        builder_M->AddVertex(p_MV, field0_M.EvaluateCartesian(tet0, p_MV)));
  }
  // TODO(DamrongGuoy): Right now we pass the gradient of the volumetric
  //  field for the gradient tangent to the polygon. Consider passing only
  //  the tangential component without the normal component.
  const int num_new_faces = builder_M->AddPolygon(
      polygon_vertex_indices, polygon_nhat_M, field0_M.EvaluateGradient(tet0));

  // PolyMeshBuilder makes one new polygonal face. TriMeshBuilder makes multiple
  // new triangular faces.
  for (int i = 0; i < num_new_faces; ++i) {
    tet0_of_contact_polygon_.push_back(tet0);
    tet1_of_contact_polygon_.push_back(tet1);
  }

  return faces;
}

template <class MeshBuilder>
void HydroelasticVolumeIntersector<MeshBuilder>::IntersectCompliantVolumes(
    GeometryId id_M, const hydroelastic::SoftMesh& compliant_M,
    const math::RigidTransform<T>& X_WM, GeometryId id_N,
    const hydroelastic::SoftMesh& compliant_N,
    const math::RigidTransform<T>& X_WN,
    std::unique_ptr<ContactSurface<T>>* contact_surface_W,
    const bool use_surfaces) {
  const math::RigidTransform<T> X_MN = X_WM.InvertAndCompose(X_WN);

  // The computation will be in Frame M and then transformed to the world frame.
  std::unique_ptr<typename MeshBuilder::MeshType> surface01_M;
  std::unique_ptr<typename MeshBuilder::FieldType> field01_M;
  VolumeIntersector<MeshBuilder, Obb> volume_intersector;

  if (use_surfaces) {
    volume_intersector.IntersectFields(
        compliant_M.pressure(), compliant_M.surface_mesh_bvh(),
        compliant_M.tri_to_tet(), compliant_M.mesh_topology(),
        compliant_N.pressure(), compliant_N.surface_mesh_bvh(),
        compliant_N.tri_to_tet(), compliant_N.mesh_topology(), X_MN,
        &surface01_M, &field01_M);
  } else {
    volume_intersector.IntersectFields(
        compliant_M.pressure(), compliant_M.bvh(), compliant_N.pressure(),
        compliant_N.bvh(), X_MN, &surface01_M, &field01_M);
  }

  if (surface01_M == nullptr) return;

  const int num_contact_polygons = surface01_M->num_elements();

  // TODO(DamrongGuoy): Compute the mesh and field with the quantities
  //  expressed in World frame by construction so that we can delete these
  //  transforming methods.

  // N.B. After this transformation, surface01_M and field01_M are expressed
  // in World frame. We will violate the frame notations briefly.
  surface01_M->TransformVertices(X_WM);
  field01_M->Transform(X_WM);

  auto grad_field0_W = std::make_unique<std::vector<Vector3<T>>>();
  grad_field0_W->reserve(num_contact_polygons);
  for (int i = 0; i < num_contact_polygons; ++i) {
    const Vector3<T>& grad_field0_M = compliant_M.pressure().EvaluateGradient(
        volume_intersector.tet0_of_polygon(i));
    grad_field0_W->emplace_back(X_WM.rotation() * grad_field0_M);
  }
  auto grad_field1_W = std::make_unique<std::vector<Vector3<T>>>();
  grad_field1_W->reserve(num_contact_polygons);
  for (int i = 0; i < num_contact_polygons; ++i) {
    const Vector3<T>& grad_field1_N = compliant_N.pressure().EvaluateGradient(
        volume_intersector.tet1_of_polygon(i));
    grad_field1_W->emplace_back(X_WN.rotation() * grad_field1_N);
  }

  // ContactSurface(id_first, id_second, mesh_W,...) requires that the face
  // normals in `mesh_W` are:
  //     - *out of* the second geometry and
  //     - *into* the first geometry.
  // This is the same convention that IntersectFields() create a mesh
  // with normals in the direction of
  //     - increasing pressure field0 (going *into* the first geometry) and
  //     - decreasing pressure field1 (going *out* of the second geometry),
  // so we create ContactSurface with the ids in the order of (id_M, id_N).
  *contact_surface_W = std::make_unique<ContactSurface<T>>(
      id_M, id_N, std::move(surface01_M), std::move(field01_M),
      std::move(grad_field0_W), std::move(grad_field1_W));
}

template <typename T>
std::unique_ptr<ContactSurface<T>> ComputeContactSurfaceFromCompliantVolumes(
    GeometryId id_M, const hydroelastic::SoftMesh& compliant_M,
    const math::RigidTransform<T>& X_WM, GeometryId id_N,
    const hydroelastic::SoftMesh& compliant_N,
    const math::RigidTransform<T>& X_WN,
    HydroelasticContactRepresentation representation) {
  std::unique_ptr<ContactSurface<T>> contact_surface_W;
  if (representation == HydroelasticContactRepresentation::kTriangle) {
    HydroelasticVolumeIntersector<TriMeshBuilder<T>>()
        .IntersectCompliantVolumes(id_M, compliant_M, X_WM, id_N, compliant_N,
                                   X_WN, &contact_surface_W,
                                   false /* use_surfaces */);
  } else {
    HydroelasticVolumeIntersector<PolyMeshBuilder<T>>()
        .IntersectCompliantVolumes(id_M, compliant_M, X_WM, id_N, compliant_N,
                                   X_WN, &contact_surface_W,
                                   false /* use_surfaces */);
  }
  return contact_surface_W;
}

//----------------------------------------------------------
// Template instantiations
//----------------------------------------------------------
// These instantiations are for Hydroelastics.
template class HydroelasticVolumeIntersector<PolyMeshBuilder<double>>;
template class HydroelasticVolumeIntersector<TriMeshBuilder<double>>;
template class HydroelasticVolumeIntersector<PolyMeshBuilder<AutoDiffXd>>;
template class HydroelasticVolumeIntersector<TriMeshBuilder<AutoDiffXd>>;
// This instantiation is for Deformables.
template class VolumeIntersector<PolyMeshBuilder<double>, Aabb>;

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    (&CalcEquilibriumPlane<T>, &IntersectTetrahedra<T>,
     &IsPlaneNormalAlongPressureGradient<T>,
     &ComputeContactSurfaceFromCompliantVolumes<T>));

}  // namespace internal
}  // namespace geometry
}  // namespace drake
