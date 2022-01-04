#include "drake/geometry/proximity/mesh_intersection.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/proximity/bvh.h"
#include "drake/geometry/proximity/contact_surface_utility.h"
#include "drake/geometry/proximity/posed_half_space.h"
#include "drake/geometry/proximity/triangle_surface_mesh.h"
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
template <typename MeshType>
Vector3<typename MeshType::ScalarType>
SurfaceVolumeIntersector<MeshType>::CalcIntersection(
    const Vector3<T>& p_FA, const Vector3<T>& p_FB,
    const PosedHalfSpace<double>& H_F) {
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
  //  sites. Also, the precision that works depends on the size of the triangles
  //  in play. Generally, as long as the triangles are no larger than unit size
  //  this should be fine.
  // TODO(SeanCurtis-TRI): This probably shouldn't be executed in release
  //  builds. This type of test is best suited for a unit test (do we trust this
  //  math or not?)
  // Verify that the intersection point is on the plane of the half space.
  using std::abs;
  DRAKE_DEMAND(abs(H_F.CalcSignedDistance(convert_to_double(intersection))) <
               kEps);
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

template <typename MeshType>
void SurfaceVolumeIntersector<MeshType>::ClipPolygonByHalfSpace(
    const std::vector<Vector3<T>>& input_vertices_F,
    const PosedHalfSpace<double>& H_F,
    std::vector<Vector3<T>>* output_vertices_F) {
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
    // Again, avoid doing classification on T-valued quantities so we don't
    // waste computation in applying the chain rule.
    const bool current_contained =
        H_F.CalcSignedDistance(convert_to_double(current)) <= 0;
    const bool previous_contained =
        H_F.CalcSignedDistance(convert_to_double(previous)) <= 0;
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

template <typename MeshType>
void SurfaceVolumeIntersector<MeshType>::RemoveDuplicateVertices(
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
    // We don't want to do this *classification* problem with generic T; it
    // would be prohibitively expensive to propagate calculations to derivatives
    // just to throw the work away.
    return (convert_to_double(p) - convert_to_double(q)).squaredNorm() <
           kEpsSquared;
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

template <typename MeshType>
const std::vector<Vector3<typename MeshType::ScalarType>>&
SurfaceVolumeIntersector<MeshType>::ClipTriangleByTetrahedron(
    int element, const VolumeMesh<double>& volume_M, int face,
    const TriangleSurfaceMesh<double>& surface_N,
    const math::RigidTransform<T>& X_MN) {
  // Although polygon_M starts out pointing to polygon_[0] that is not an
  // invariant in this function.
  std::vector<Vector3<T>>* polygon_M = &(polygon_[0]);
  // Initialize output polygon in M's frame from the triangular `face` of
  // surface_N.
  polygon_M->clear();
  for (int i = 0; i < 3; ++i) {
    const int v = surface_N.element(face).vertex(i);
    const Vector3<T>& p_NV = surface_N.vertex(v).cast<T>();
    polygon_M->push_back(X_MN * p_NV);
  }
  // Get the positions, in M's frame, of the four vertices of the tetrahedral
  // `element` of volume_M. Because we are doing this in the M frame, we can
  // leave the volume mesh's quantities as double-valued -- T-values will arise
  // as we do transformed computations below.
  Vector3<double> p_MVs[4];
  for (int i = 0; i < 4; ++i) {
    int v = volume_M.element(element).vertex(i);
    p_MVs[i] = volume_M.vertex(v);
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
    const Vector3<double>& p_MA = p_MVs[face_vertex[0]];
    const Vector3<double>& p_MB = p_MVs[face_vertex[1]];
    const Vector3<double>& p_MC = p_MVs[face_vertex[2]];
    // We'll allow the PosedHalfSpace to normalize our vector.
    const Vector3<double> normal_M = (p_MB - p_MA).cross(p_MC - p_MA);
    PosedHalfSpace<double> half_space_M(normal_M, p_MA);
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

template <typename MeshType>
bool SurfaceVolumeIntersector<MeshType>::IsFaceNormalAlongPressureGradient(
    const VolumeMeshFieldLinear<double, double>& volume_field_M,
    const TriangleSurfaceMesh<double>& surface_N,
    const math::RigidTransform<double>& X_MN,
    int tet_index, int tri_index) {
  const Vector3<double> grad_p_M = volume_field_M.EvaluateGradient(tet_index);
  // TODO(SeanCurtis-TRI) If we pass in an *unnormalized* vector, we can
  //  reduce the number of square roots. Currently, we produce the *unit*
  //  gradient normal n̂ and the triangle face normal f̂ (we normalize this vector
  //  to correct for rounding error that creeps in from rotations). We use the
  //  unit vectors so that we can compute cos(θ) = f̂⋅n̂. However, we could
  //  compute the following instead:
  //
  //      cos(θ) = f⃗/|f⃗|⋅n⃗/|n⃗|
  //      cos(θ) = f⃗⋅n⃗/(|f⃗||n⃗|)
  //      cos(θ) = f⃗⋅n⃗/√(|f⃗|²|n⃗|²)
  //
  //  Two square roots become one square root.
  return IsFaceNormalInNormalDirection(grad_p_M.normalized(), surface_N,
                                       tri_index, X_MN.rotation());
}

template <typename MeshType>
template <typename MeshBuilder>
void SurfaceVolumeIntersector<MeshType>::SampleVolumeFieldOnSurface(
    const VolumeMeshFieldLinear<double, double>& volume_field_M,
    const Bvh<Obb, VolumeMesh<double>>& bvh_M,
    const TriangleSurfaceMesh<double>& surface_N,
    const Bvh<Obb, TriangleSurfaceMesh<double>>& bvh_N,
    const math::RigidTransform<T>& X_MN, MeshBuilder builder_M) {
  const VolumeMesh<double>& vol_mesh_M = volume_field_M.mesh();

  const math::RigidTransform<double>& X_MN_d = convert_to_double(X_MN);
  auto callback = [&volume_field_M, &surface_N,
                   &vol_mesh_M, &X_MN_d, &X_MN,
                   &builder_M,
                   this](int tet_index, int tri_index) -> BvttCallbackResult {
    if (!this->IsFaceNormalAlongPressureGradient(
            volume_field_M, surface_N, X_MN_d, tet_index, tri_index)) {
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
        this->ClipTriangleByTetrahedron(tet_index, vol_mesh_M, tri_index,
                                        surface_N, X_MN);

    if (polygon_vertices_M.size() < 3) return BvttCallbackResult::Continue;

    // Add the vertices to the builder (with corresponding pressure values) and
    // construct index-based polygon representation.
    polygon_vertex_indices_.clear();
    for (const auto& p_MV : polygon_vertices_M) {
      polygon_vertex_indices_.push_back(builder_M.AddVertex(
          p_MV, volume_field_M.EvaluateCartesian(tet_index, p_MV)));
    }

    const Vector3<T> nhat_M =
        X_MN.rotation() * surface_N.face_normal(tri_index).template cast<T>();
    // The gradient of the pressure field in the volume mesh is *also* the
    // gradient of the pressure field on the contact surface. That's not
    // generally true, but is for the special case of compliant-rigid case.
    // So, we use it both for AddPolygon() as grad_e_MN_M and as grad_e_M when
    // we store it per-face below.
    const Vector3<T> grad_e_MN_M = volume_field_M.EvaluateGradient(tet_index);
    const int num_new_faces =
        builder_M.AddPolygon(polygon_vertex_indices_, nhat_M, grad_e_MN_M);

    // TODO(SeanCurtis-TRI) This represents a *second* iteration on the vertices
    //  of the polygon. Rolling this into the builder would allow us to reduce
    //  the loops.
    // Store the constituent pressure gradient values from the soft mesh for
    // each face in the contact surface.
    for (int i = 0; i < num_new_faces; ++i) {
      this->grad_eM_Ms_.push_back(grad_e_MN_M);
    }

    return BvttCallbackResult::Continue;
  };
  bvh_M.Collide(bvh_N, X_MN_d, callback);

  if (builder_M.num_faces() == 0) return;

  std::tie(mesh_M_, field_M_) = builder_M.MakeMeshAndField();
}

template <typename T>
std::unique_ptr<ContactSurface<T>>
ComputeContactSurfaceFromSoftVolumeRigidSurface(
    const GeometryId id_S, const VolumeMeshFieldLinear<double, double>& field_S,
    const Bvh<Obb, VolumeMesh<double>>& bvh_S,
    const math::RigidTransform<T>& X_WS, const GeometryId id_R,
    const TriangleSurfaceMesh<double>& mesh_R,
    const Bvh<Obb, TriangleSurfaceMesh<double>>& bvh_R,
    const math::RigidTransform<T>& X_WR,
    HydroelasticContactRepresentation representation) {
  auto process_intersection =
      [&X_WS, id_S,
       id_R](auto&& intersector_in) -> std::unique_ptr<ContactSurface<T>> {
    if (intersector_in.has_intersection()) {
      // TODO(DamrongGuoy): Compute the mesh and field with the quantities
      //  expressed in World frame by construction so that we can delete these
      //  two transforming methods.

      // Transform the mesh from the S frame to the world frame. This entails:
      //  1. Transforming the surface's vertices.
      //  2. Allowing the LinearField e_SR a chance to re-express its cached
      //     gradient values.
      //  3. Re-expressing the gradients of the soft-mesh's pressure field
      //    (grad_eS_S) in the world frame (grad_eS_W).
      intersector_in.mutable_mesh().TransformVertices(X_WS);
      intersector_in.mutable_field().Transform(X_WS);
      std::vector<Vector3<T>>& grad_eS_S = intersector_in.mutable_grad_eM_M();
      for (auto& grad_eSi_S : grad_eS_S) {
        grad_eSi_S = X_WS.rotation() * grad_eSi_S;
      }
      // The contact surface is documented as having the normals pointing *out*
      // of the second surface and into the first. This mesh intersection
      // creates a surface mesh with normals pointing out of the rigid surface,
      // so we make sure the ids are ordered so that the rigid is the second id.
      return std::make_unique<ContactSurface<T>>(
          id_S, id_R, intersector_in.release_mesh(),
          intersector_in.release_field(),
          std::make_unique<std::vector<Vector3<T>>>(std::move(grad_eS_S)),
          nullptr);
    }
    return nullptr;
  };

  // Compute the transformation from the rigid frame to the soft frame.
  const math::RigidTransform<T> X_SR = X_WS.InvertAndCompose(X_WR);

  if (representation == HydroelasticContactRepresentation::kTriangle) {
    SurfaceVolumeIntersector<TriangleSurfaceMesh<T>> intersector;
    intersector.SampleVolumeFieldOnSurface(field_S, bvh_S, mesh_R, bvh_R, X_SR,
                                           TriMeshBuilder<T>());
    return process_intersection(intersector);
  } else {
    // Polygon.
    SurfaceVolumeIntersector<PolygonSurfaceMesh<T>> intersector;
    intersector.SampleVolumeFieldOnSurface(field_S, bvh_S, mesh_R, bvh_R, X_SR,
                                           PolyMeshBuilder<T>());
    return process_intersection(intersector);
  }
}

template class SurfaceVolumeIntersector<TriangleSurfaceMesh<double>>;
template class SurfaceVolumeIntersector<TriangleSurfaceMesh<AutoDiffXd>>;
template class SurfaceVolumeIntersector<PolygonSurfaceMesh<double>>;
template class SurfaceVolumeIntersector<PolygonSurfaceMesh<AutoDiffXd>>;

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS((
    &ComputeContactSurfaceFromSoftVolumeRigidSurface<T>
))

}  // namespace internal
}  // namespace geometry
}  // namespace drake
