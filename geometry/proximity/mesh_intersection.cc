#include "drake/geometry/proximity/mesh_intersection.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/proximity/bvh.h"
#include "drake/geometry/proximity/contact_surface_utility.h"
#include "drake/geometry/proximity/mesh_field_linear.h"
#include "drake/geometry/proximity/posed_half_space.h"
#include "drake/geometry/proximity/surface_mesh.h"
#include "drake/geometry/proximity/volume_mesh.h"
#include "drake/geometry/proximity/volume_mesh_field.h"
#include "drake/geometry/query_results/contact_surface.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
namespace internal {

// TODO(DamrongGuoy): Take care of double counting problem when a triangle of
//  a surface mesh overlaps a triangular face shared by two tetrahedrons of a
//  volume mesh. The fix will require several functions in the code to work
//  together. These ideas should help:
//  - Use Simulation of Simplicity with floating-point filter.
//  - Book keeping the shared faces of the tetrahedrons in the volume mesh.
//  - Require unique vertices in the surface mesh and in the volume mesh.

// TODO(DamrongGuoy): Take care of special cases when `p` lies on the
//  plane of the half space to help with the double counting problem. Right
//  now it is taken as being inside the half space.
//    Instead of fcl::Halfspace(normal, distance), we might want to specify
//  the half space using a pair (face, element), i.e., a triangular face of a
//  tetrahedral element. It will identify the half space bounded by the plane
//  of the triangular `face` that contains the tetrahedral `element`, and will
//  enable symbolic perturbation in such a way that a point `p` on a shared
//  face of two tetrahedrons will be considered inside one tetrahedron and
//  outside another tetrahedron. Right now it will be considered inside both
//  tetrahedrons.

// TODO(DamrongGuoy): Handle the case that the line is parallel to the plane.
template <typename T>
Vector3<T> SurfaceVolumeIntersector<T>::CalcIntersection(
    const Vector3<T>& p_FA, const Vector3<T>& p_FB,
    const PosedHalfSpace<T>& H_F) {
  const T a = H_F.CalcSignedDistance(p_FA);
  const T b = H_F.CalcSignedDistance(p_FB);
  // We require that A and B classify in opposite directions (one inside and one
  // outside). Outside has a strictly positive distance, inside is non-positive.
  // We confirm that their product is non-positive and that at least one of the
  // values is positive -- they can't both be zero. This prevents b - a becoming
  // zero and the corresponding division by zero.
  DRAKE_ASSERT(a * b <= 0 && (a > 0 || b > 0));
  const T wa = b / (b - a);
  const T wb = T(1.0) - wa;  // Enforce a + b = 1.
  const Vector3<T> intersection = wa * p_FA + wb * p_FB;
  // Empirically we found that numeric_limits<double>::epsilon() 2.2e-16 is
  // too small.
  const T kEps(1e-14);
  // TODO(SeanCurtis-TRI): Consider refactoring this fuzzy test *into*
  //  PosedHalfSpace if it turns out we need to perform this test at other
  //  sites.
  // Verify that the intersection point is on the plane of the half space.
  using std::abs;
  DRAKE_DEMAND(abs(H_F.CalcSignedDistance(intersection)) < kEps);
  return intersection;
  // Justification.
  // 1. We set up the weights wa and wb such that wa + wb = 1, which
  //    guarantees that the linear combination is on the straight line
  //    through A and B.
  // 2. We show that the H_F.signed_distance(wa * A + wb * B) is zero.
  //    Let H_F.signed_distance be sdf(P) = N.dot(P) + d.
  //      sdf(wa * A + wb * B)
  //      = N.dot(wa * A + wb * B) + d
  //      = wa * N.dot(A) + wb * N.dot(B) + d
  //      = b * N.dot(A)/(b - a) + a * N.dot(B)/(a - b) + d
  //      = b * N.dot(A)/(b - a) - a * N.dot(B)/(b - a) + d
  //      = (b * N.dot(A) - a * N.dot(B) + (b - a) * d) / (b - a)
  //      = (b * (N.dot(A) + d) - a * (N.dot(B) + d)) / (b-a)
  //      = (b * sdf(A) - a * sdf(B)) / (b-a)
  //      = (b * a - a * b) / (b-a)
  //      = 0 when a != b.
}

template <typename T>
void SurfaceVolumeIntersector<T>::ClipPolygonByHalfSpace(
    const std::vector<Vector3<T>>& input_vertices_F,
    const PosedHalfSpace<T>& H_F, std::vector<Vector3<T>>* output_vertices_F) {
  DRAKE_ASSERT(output_vertices_F != nullptr);
  // Note: this is the inner loop of a modified Sutherland-Hodgman algorithm for
  // clipping a polygon.
  output_vertices_F->clear();
  // Note: This code is correct for size < 3, but pointless so we make no effort
  // to support it or test it.
  const int size = static_cast<int>(input_vertices_F.size());

  // TODO(SeanCurtis-TRI): If necessary, this can be made more efficient:
  //  eliminating the modulus and eliminating the redundant "inside" calculation
  //  on previous (by pre-determining previous and its "containedness" and then
  //  propagating current -> previous in each loop. Probably a desirable
  //  optimization as we need to make all of this work as cheap as possible.
  for (int i = 0; i < size; ++i) {
    const Vector3<T>& current = input_vertices_F[i];
    const Vector3<T>& previous = input_vertices_F[(i - 1 + size) % size];
    const bool current_contained = H_F.CalcSignedDistance(current) <= 0;
    const bool previous_contained = H_F.CalcSignedDistance(previous) <= 0;
    if (current_contained) {
      if (!previous_contained) {
        // Current is inside and previous is outside. Compute the point where
        // that edge enters the half space. This is a new vertex in the clipped
        // polygon and must be included before current.
        output_vertices_F->push_back(CalcIntersection(current, previous, H_F));
      }
      output_vertices_F->push_back(current);
    } else if (previous_contained) {
      // Current is outside and previous is inside. Compute the point where
      // the edge exits the half space. This is a new vertex in the clipped
      // polygon and is included *instead* of current.
      output_vertices_F->push_back(CalcIntersection(current, previous, H_F));
    }
  }
}

template <typename T>
void SurfaceVolumeIntersector<T>::RemoveDuplicateVertices(
    std::vector<Vector3<T>>* polygon) {
  DRAKE_ASSERT(polygon != nullptr);

  // TODO(SeanCurtis-TRI): The resulting polygon depends on the order of the
  //  inputs. Imagine I have vertices A, A', A'' (such that |X - X'| < eps.
  //  The sequence AA'A'' would be reduced to AA''
  //  The sequence A'A''A would be reduced to A'.
  //  The sequence A''AA' would be reduced to A''A.
  //  In all three cases, the exact same polygon is defined on input, but the
  //  output is different. This should be documented and/or fixed.
  if (polygon->size() <= 1)
    return;

  auto near = [](const Vector3<T>& p, const Vector3<T>& q) -> bool {
    // TODO(SeanCurtis-TRI): This represents 5-6 bits of loss. Confirm that a
    //  tighter epsilon can't be used. This should probably be a function of the
    //  longest edge involved.
    // Empirically we found that numeric_limits<double>::epsilon() 2.2e-16 is
    // too small, especially when the objects are not axis-aligned.
    const double kEpsSquared(1e-14 * 1e-14);
    return (p - q).squaredNorm() < kEpsSquared;
  };

  // Remove consecutive vertices that are duplicated in the linear order.  It
  // will change "A,B,B,C,C,A" to "A,B,C,A". To close the cyclic order, we
  // will check the first and the last vertices again near the end of the
  // function.
  // TODO(DamrongGuoy): This doesn't strictly satisfy the requirement of
  //  std::unique that the predicate represent an equivalence relation (i.e.,
  //  point A could be "near" points B and C, but that doesn't mean B and C are
  //  near each other). We need to figure out if that matters for this usage
  //  and, if not, document why here.
  auto it = std::unique(polygon->begin(), polygon->end(), near);
  polygon->resize(it - polygon->begin());

  if (polygon->size() >= 3) {
    // Check the first and the last vertices in the sequence. For example, given
    // "A,B,C,A", we want "A,B,C".
    if (near((*polygon)[0], *(polygon->rbegin()))) {
      polygon->pop_back();
    }
  }

  DRAKE_ASSERT(polygon->size() != 2 || !near((*polygon)[0], (*polygon)[1]));
}

template <typename T>
const std::vector<Vector3<T>>&
SurfaceVolumeIntersector<T>::ClipTriangleByTetrahedron(
    VolumeElementIndex element, const VolumeMesh<T>& volume_M,
    SurfaceFaceIndex face, const SurfaceMesh<T>& surface_N,
    const math::RigidTransform<T>& X_MN) {
  // Although polygon_M starts out pointing to polygon_[0] that is not an
  // invariant in this function.
  std::vector<Vector3<T>>* polygon_M = &(polygon_[0]);
  // Initialize output polygon in M's frame from the triangular `face` of
  // surface_N.
  polygon_M->clear();
  for (int i = 0; i < 3; ++i) {
    SurfaceVertexIndex v = surface_N.element(face).vertex(i);
    // TODO(SeanCurtis-TRI): The `M` in `r_MV()` is different from the M in this
    //  function. More evidence that the `vertex(v).r_MV()` notation is *bad*.
    const Vector3<T>& p_NV = surface_N.vertex(v).r_MV();
    polygon_M->push_back(X_MN * p_NV);
  }
  // Get the positions, in M's frame, of the four vertices of the tetrahedral
  // `element` of volume_M.
  Vector3<T> p_MVs[4];
  for (int i = 0; i < 4; ++i) {
    VolumeVertexIndex v = volume_M.element(element).vertex(i);
    p_MVs[i] = volume_M.vertex(v).r_MV();
  }
  // Sets up the four half spaces associated with the four triangular faces of
  // the tetrahedron. Assume the tetrahedron has the fourth vertex seeing the
  // first three vertices in CCW order; for example, a tetrahedron of (Zero(),
  // UnitX(), UnitY(), UnitZ()) (see the picture below) has this orientation.
  //
  //      +Z
  //       |
  //       v3
  //       |
  //       |
  //     v0+------v2---+Y
  //      /
  //     /
  //   v1
  //   /
  // +X
  //
  // This table encodes the four triangular faces of the tetrahedron in such
  // a way that each right-handed face normal points outward from the
  // tetrahedron, which is suitable for setting up the half space. Refer to
  // the above picture.
  const int faces[4][3] = {{1, 2, 3}, {0, 3, 2}, {0, 1, 3}, {0, 2, 1}};
  // Although this assertion appears trivially true, its presence is protection
  // for the subsequent code, which heavily relies on it being true, from any
  // changes that may be applied to the previous code.
  DRAKE_ASSERT(polygon_M == &(polygon_[0]));
  std::vector<Vector3<T>>* in_M = polygon_M;
  std::vector<Vector3<T>>* out_M = &(polygon_[1]);
  for (auto& face_vertex : faces) {
    const Vector3<T>& p_MA = p_MVs[face_vertex[0]];
    const Vector3<T>& p_MB = p_MVs[face_vertex[1]];
    const Vector3<T>& p_MC = p_MVs[face_vertex[2]];
    // We'll allow the PosedHalfSpace to normalize our vector.
    const Vector3<T> normal_M = (p_MB - p_MA).cross(p_MC - p_MA);
    PosedHalfSpace<T> half_space_M(normal_M, p_MA);
    // Intersects the output polygon by the half space of each face of the
    // tetrahedron.
    ClipPolygonByHalfSpace(*in_M, half_space_M, out_M);
    std::swap(in_M, out_M);
  }
  polygon_M = in_M;

  // TODO(DamrongGuoy): Remove the code below when ClipPolygonByHalfSpace()
  //  stops generating duplicate vertices. See the note in
  //  ClipPolygonByHalfSpace().

  // Remove possible duplicate vertices from ClipPolygonByHalfSpace().
  RemoveDuplicateVertices(polygon_M);
  if (polygon_M->size() < 3) {
    // RemoveDuplicateVertices() may have shrunk the polygon down to one or
    // two vertices, so we empty the polygon.
    polygon_M->clear();
  }

  // TODO(DamrongGuoy): Calculate area of the polygon. If it's too small,
  //  return an empty polygon.

  // The output polygon could be at most a heptagon.
  DRAKE_DEMAND(polygon_M->size() <= 7);
  return *polygon_M;
}

template <typename T>
bool SurfaceVolumeIntersector<T>::IsFaceNormalAlongPressureGradient(
    const VolumeMeshField<T, T>& volume_field_M,
    const SurfaceMesh<T>& surface_N, const math::RigidTransform<T>& X_MN,
    const VolumeElementIndex& tet_index, const SurfaceFaceIndex& tri_index) {
  // TODO(DamrongGuoy): Change the type of volume_field_M from
  //  VolumeMeshField to VolumeMeshFieldLinear and remove this cast.
  // Evaluate the gradient vector on the tetrahedron.
  const auto field_M =
      dynamic_cast<const VolumeMeshFieldLinear<T, T>*>(&volume_field_M);
  DRAKE_DEMAND(field_M);
  const Vector3<T> grad_p_M = field_M->EvaluateGradient(tet_index);
  return IsFaceNormalInNormalDirection(grad_p_M.normalized(), surface_N,
                                       tri_index, X_MN.rotation());
}

template <typename T>
void SurfaceVolumeIntersector<T>::SampleVolumeFieldOnSurface(
    const VolumeMeshField<T, T>& volume_field_M,
    const Bvh<VolumeMesh<T>>& bvh_M, const SurfaceMesh<T>& surface_N,
    const Bvh<SurfaceMesh<T>>& bvh_N, const math::RigidTransform<T>& X_MN,
    std::unique_ptr<SurfaceMesh<T>>* surface_MN_M,
    std::unique_ptr<SurfaceMeshFieldLinear<T, T>>* e_MN,
    std::vector<Vector3<T>>* grad_eM_Ms) {
  DRAKE_DEMAND(surface_MN_M);
  DRAKE_DEMAND(e_MN);
  DRAKE_DEMAND(grad_eM_Ms);
  grad_eM_Ms->clear();

  // In order to get the gradient, we need the concrete linear field underneath.
  const VolumeMeshFieldLinear<T, T>& volume_linear_field_M =
      dynamic_cast<const VolumeMeshFieldLinear<T, T>&>(volume_field_M);
  std::vector<SurfaceFace> surface_faces;
  std::vector<SurfaceVertex<T>> surface_vertices_M;
  std::vector<T> surface_e;
  const auto& mesh_M = volume_field_M.mesh();
  // We know that each contact polygon has at most 7 vertices because
  // each surface triangle is clipped by four half-spaces of the four
  // triangular faces of a tetrahedron.
  std::vector<SurfaceVertexIndex> contact_polygon;
  contact_polygon.reserve(7);

  auto callback = [&volume_linear_field_M, &surface_N, &surface_faces,
                   &surface_vertices_M, &surface_e, &mesh_M, &X_MN,
                   &contact_polygon, grad_eM_Ms,
                   this](VolumeElementIndex tet_index,
                         SurfaceFaceIndex tri_index) -> BvttCallbackResult {
    if (!this->IsFaceNormalAlongPressureGradient(
            volume_linear_field_M, surface_N, X_MN, tet_index, tri_index)) {
      return BvttCallbackResult::Continue;
    }

    // TODO(SeanCurtis-TRI): This redundantly transforms surface mesh vertex
    //  positions. Specifically, each vertex will be transformed M times (once
    //  per tetrahedron). Even with broadphase culling, this vertex will get
    //  transformed once for each tet-tri pair where the tri is incidental
    //  to the vertex and the tet-tri pair can't be conservatively culled.
    //  This is O(mn), where m is the number of faces incident to the vertex
    //  and n is the number of tet BVs that overlap this triangle BV. However,
    //  if the broadphase culling determines the surface and volume are
    //  disjoint regions, *no* vertices will be transformed. Unclear what the
    //  best balance for best average performance.
    const std::vector<Vector3<T>>& polygon_vertices_M =
        this->ClipTriangleByTetrahedron(tet_index, mesh_M, tri_index, surface_N,
                                        X_MN);

    const int poly_vertex_count = static_cast<int>(polygon_vertices_M.size());
    if (poly_vertex_count < 3) return BvttCallbackResult::Continue;

    const int num_previous_vertices = surface_vertices_M.size();
    // Add the new polygon vertices to the mesh vertices and construct a
    // polygon from the vertex indices.
    contact_polygon.clear();
    for (int i = 0; i < poly_vertex_count; ++i) {
      contact_polygon.emplace_back(surface_vertices_M.size());
      surface_vertices_M.emplace_back(polygon_vertices_M[i]);
    }
    const Vector3<T>& nhat_M =
        X_MN.rotation() * surface_N.face_normal(tri_index);

    size_t old_count = surface_faces.size();
    AddPolygonToMeshData(contact_polygon, nhat_M, &surface_faces,
                         &surface_vertices_M);

    // TODO(SeanCurtis-TRI) Consider rolling this operation into
    // AddPolygonToMeshData to eliminate the extra pass through triangles.
    const Vector3<T>& grad_eMi_M =
        volume_linear_field_M.EvaluateGradient(tet_index);
    for (size_t i = old_count; i < surface_faces.size(); ++i) {
      grad_eM_Ms->push_back(grad_eMi_M);
    }

    const int num_current_vertices = surface_vertices_M.size();
    // Calculate values of the pressure field and the normal field at the
    // new vertices.
    for (int v = num_previous_vertices; v < num_current_vertices; ++v) {
      const Vector3<T>& r_MV = surface_vertices_M[v].r_MV();
      const T pressure =
          volume_linear_field_M.EvaluateCartesian(tet_index, r_MV);
      surface_e.push_back(pressure);
    }
    return BvttCallbackResult::Continue;
  };
  bvh_M.Collide(bvh_N, convert_to_double(X_MN), callback);

  DRAKE_DEMAND(surface_vertices_M.size() == surface_e.size());
  if (surface_faces.empty()) return;

  *surface_MN_M = std::make_unique<SurfaceMesh<T>>(
      std::move(surface_faces), std::move(surface_vertices_M));
  const bool calculate_gradient = false;
  *e_MN = std::make_unique<SurfaceMeshFieldLinear<T, T>>(
      "e", std::move(surface_e), surface_MN_M->get(), calculate_gradient);
}

template <typename T>
std::unique_ptr<ContactSurface<T>>
ComputeContactSurfaceFromSoftVolumeRigidSurface(
    const GeometryId id_S, const VolumeMeshField<T, T>& field_S,
    const Bvh<VolumeMesh<T>>& bvh_S, const math::RigidTransform<T>& X_WS,
    const GeometryId id_R, const SurfaceMesh<T>& mesh_R,
    const Bvh<SurfaceMesh<T>>& bvh_R, const math::RigidTransform<T>& X_WR) {
  // TODO(SeanCurtis-TRI): This function is insufficiently templated. Generally,
  //  there are three types of scalars: the pose scalar, the mesh field *value*
  //  scalar, and the mesh vertex-position scalar. However, short term, it is
  //  probably unnecessary to distinguish all three here. If we assume that this
  //  function is *only* called to find the contact surface between two declared
  //  geometries, then the volume mesh values and vertex positions will always
  //  be in double. However, even dividing between pose and mesh scalars has
  //  *huge* downstream implications on how the meshes and mesh fields are
  //  defined. We're deferring this work by simply disallowing invocation of
  //  this method on AutoDiffXd (see below). It compiles, but throws.
  //  See issue #14136.

  // Compute the transformation from the rigid frame to the soft frame.
  const math::RigidTransform<T> X_SR = X_WS.inverse() * X_WR;

  // The mesh will be computed in Frame S and then transformed to the world
  // frame.
  std::unique_ptr<SurfaceMesh<T>> surface_SR;
  std::unique_ptr<SurfaceMeshFieldLinear<T, T>> e_SR;
  std::vector<Vector3<T>> grad_eS_S;

  SurfaceVolumeIntersector<T>().SampleVolumeFieldOnSurface(
      field_S, bvh_S, mesh_R, bvh_R, X_SR, &surface_SR, &e_SR, &grad_eS_S);

  if (surface_SR == nullptr) return nullptr;

  // TODO(DamrongGuoy): Compute the mesh and field with the quantities
  //  expressed in World frame by construction so that we can delete these two
  //  transforming methods.

  // Transform the mesh from the S frame to the world frame. This entails:
  //  1. Transforming the surface's vertices.
  //  2. Allowing the LinearField e_SR a chance to re-express its cached
  //     gradient values.
  //  3. Re-expressing the gradients of the soft-mesh's pressure field
  //    (grad_eS_S) in the world frame (grad_eS_W).
  surface_SR->TransformVertices(X_WS);
  e_SR->TransformGradients(X_WS);
  std::unique_ptr<std::vector<Vector3<T>>> grad_eS_W;
  grad_eS_W = std::make_unique<std::vector<Vector3<T>>>();
  grad_eS_W->reserve(grad_eS_S.size());
  for (const auto& grad_eSi_S : grad_eS_S) {
    grad_eS_W->emplace_back(X_WS.rotation() * grad_eSi_S);
  }

  // The contact surface is documented as having the normals pointing *out* of
  // the second surface and into the first. This mesh intersection creates a
  // surface mesh with normals pointing out of the rigid surface, so we make
  // sure the ids are ordered so that the rigid is the second id.
  return std::make_unique<ContactSurface<T>>(id_S, id_R, std::move(surface_SR),
                                             std::move(e_SR),
                                             std::move(grad_eS_W), nullptr);
}

template class SurfaceVolumeIntersector<double>;
template class SurfaceVolumeIntersector<AutoDiffXd>;

template std::unique_ptr<ContactSurface<double>>
ComputeContactSurfaceFromSoftVolumeRigidSurface(
    const GeometryId id_S, const VolumeMeshField<double, double>& field_S,
    const Bvh<VolumeMesh<double>>& bvh_S,
    const math::RigidTransform<double>& X_WS, const GeometryId id_R,
    const SurfaceMesh<double>& mesh_R, const Bvh<SurfaceMesh<double>>& bvh_R,
    const math::RigidTransform<double>& X_WR);

template std::unique_ptr<ContactSurface<AutoDiffXd>>
ComputeContactSurfaceFromSoftVolumeRigidSurface(
    const GeometryId id_S,
    const VolumeMeshField<AutoDiffXd, AutoDiffXd>& field_S,
    const Bvh<VolumeMesh<AutoDiffXd>>& bvh_S,
    const math::RigidTransform<AutoDiffXd>& X_WS, const GeometryId id_R,
    const SurfaceMesh<AutoDiffXd>& mesh_R,
    const Bvh<SurfaceMesh<AutoDiffXd>>& bvh_R,
    const math::RigidTransform<AutoDiffXd>& X_WR);

// NOTE: This is a short-term hack to allow ProximityEngine to compile when
// invoking this method. There are currently a host of issues preventing us from
// doing contact surface computation with AutoDiffXd. This curtails those
// issues for now by short-circuiting the functionality. (See the note on the
// templated version of this function.)
std::unique_ptr<ContactSurface<AutoDiffXd>>
ComputeContactSurfaceFromSoftVolumeRigidSurface(
    const GeometryId, const VolumeMeshField<double, double>&,
    const Bvh<VolumeMesh<double>>&, const math::RigidTransform<AutoDiffXd>&,
    const GeometryId, const SurfaceMesh<double>&,
    const Bvh<SurfaceMesh<double>>&, const math::RigidTransform<AutoDiffXd>&) {
  throw std::logic_error(
      "AutoDiff-valued ContactSurface calculation between meshes is not"
      "currently supported");
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake

