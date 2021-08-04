#include "drake/geometry/proximity/distance_to_shape_callback.h"

#include <algorithm>
#include <limits>
#include <utility>

#include "drake/common/default_scalars.h"
#include "drake/geometry/proximity/distance_to_point_callback.h"
#include "drake/math/rotation_matrix.h"

namespace drake {
namespace geometry {
namespace internal {
namespace shape_distance {

template <typename T>
void DistancePairGeometry<T>::operator()(const fcl::Sphered& sphere_A,
                                         const fcl::Sphered& sphere_B) {
  SphereShapeDistance(sphere_A, sphere_B);
}

template <typename T>
void DistancePairGeometry<T>::operator()(const fcl::Sphered& sphere_A,
                                         const fcl::Boxd& box_B) {
  SphereShapeDistance(sphere_A, box_B);
}

template <typename T>
void DistancePairGeometry<T>::operator()(const fcl::Sphered& sphere_A,
                                         const fcl::Cylinderd& cylinder_B) {
  SphereShapeDistance(sphere_A, cylinder_B);
}

template <typename T>
void DistancePairGeometry<T>::operator()(const fcl::Sphered& sphere_A,
                                         const fcl::Halfspaced& halfspace_B) {
  SphereShapeDistance(sphere_A, halfspace_B);
}

template <typename T>
void DistancePairGeometry<T>::operator()(const fcl::Sphered& sphere_A,
                                         const fcl::Capsuled& capsule_B) {
  SphereShapeDistance(sphere_A, capsule_B);
}

template <typename T>
template <typename FclShape>
void DistancePairGeometry<T>::SphereShapeDistance(const fcl::Sphered& sphere_A,
                                                  const FclShape& shape_B) {
  const SignedDistanceToPoint<T> shape_B_to_point_Ao =
      point_distance::DistanceToPoint<T>(id_B_, X_WB_,
                                         X_WA_.translation())(shape_B);
  result_->id_A = id_A_;
  result_->id_B = id_B_;
  result_->distance = shape_B_to_point_Ao.distance - sphere_A.radius;
  // p_BCb is the witness point on ∂B measured and expressed in B.
  result_->p_BCb = shape_B_to_point_Ao.p_GN;
  result_->nhat_BA_W = shape_B_to_point_Ao.grad_W;
  // p_ACa is the witness point on ∂A measured and expressed in A.
  const math::RotationMatrix<T> R_AW = X_WA_.rotation().transpose();
  result_->p_ACa = -sphere_A.radius * (R_AW * shape_B_to_point_Ao.grad_W);
}

template <>
void CalcDistanceFallback<double>(const fcl::CollisionObjectd& a,
                                  const fcl::CollisionObjectd& b,
                                  const fcl::DistanceRequestd& request,
                                  SignedDistancePair<double>* pair_data) {
  fcl::DistanceResultd result;
  fcl::distance(&a, &b, request, result);

  pair_data->id_A = EncodedData(a).id();
  pair_data->id_B = EncodedData(b).id();

  pair_data->distance = result.min_distance;

  // Setting the witness points.
  const Eigen::Vector3d& p_WCa = result.nearest_points[0];
  pair_data->p_ACa = a.getTransform().inverse() * p_WCa;
  const Eigen::Vector3d& p_WCb = result.nearest_points[1];
  pair_data->p_BCb = b.getTransform().inverse() * p_WCb;

  // Setting the normal.
  // TODO(DamrongGuoy): We should set the tolerance through SceneGraph for
  //  determining whether the two geometries are touching or not. For now, we
  //  use this number.
  const double kEps = 1e-14;
  const double kNan = std::numeric_limits<double>::quiet_NaN();

  // Returns NaN in nhat when min_distance is 0 or almost 0.
  // TODO(DamrongGuoy): In the future, we should return nhat_BA_W as the
  //  outward face normal when the two objects are touching.
  if (std::abs(result.min_distance) < kEps) {
    pair_data->nhat_BA_W = Eigen::Vector3d(kNan, kNan, kNan);
  } else {
    pair_data->nhat_BA_W = (p_WCa - p_WCb) / result.min_distance;
  }
}

bool RequiresFallback(const fcl::CollisionObjectd& a,
                      const fcl::CollisionObjectd& b) {
  /* In the current ecosystem, we only have high-fidelity geometric code for
   Sphere-X (for *some* X). So, the conditions requiring the fallback is
   a) neither is a sphere, or b) one is a sphere, the other is the wrong X. */
  if (a.collisionGeometry()->getNodeType() != fcl::GEOM_SPHERE &&
      b.collisionGeometry()->getNodeType() != fcl::GEOM_SPHERE) {
    return true;
  }
  const fcl::CollisionGeometryd* other =
      a.collisionGeometry()->getNodeType() == fcl::GEOM_SPHERE
          ? b.collisionGeometry().get()
          : a.collisionGeometry().get();
  // Box, capsule, cylinder, half space, and cylinder don't required fallback.
  // Ellipsoid, convex do. Other fcl node types aren't currently used. Note
  // that the convex type is used to represent drake::geometry::Mesh.
  return other->getNodeType() == fcl::GEOM_ELLIPSOID ||
         other->getNodeType() == fcl::GEOM_CONVEX;
}

template <typename T>
void ComputeNarrowPhaseDistance(const fcl::CollisionObjectd& a,
                                const math::RigidTransform<T>& X_WA,
                                const fcl::CollisionObjectd& b,
                                const math::RigidTransform<T>& X_WB,
                                const fcl::DistanceRequestd& request,
                                SignedDistancePair<T>* result) {
  DRAKE_DEMAND(result != nullptr);

  if (RequiresFallback(a, b)) {
    CalcDistanceFallback<T>(a, b, request, result);
    return;
  }

  // If no fallback is necessary, one of these two *must* be a sphere.
  const bool a_is_sphere =
      a.collisionGeometry()->getNodeType() == fcl::GEOM_SPHERE;
  DRAKE_ASSERT(a_is_sphere ||
               b.collisionGeometry()->getNodeType() == fcl::GEOM_SPHERE);
  // We write `s` for the sphere object and `o` for the other object. We
  // assign either (a,b) or (b,a) to (s,o) depending on whether `a` is a
  // sphere or not. Therefore, we only need the helper DistancePairGeometry
  // that takes (sphere, other) but not (other, sphere).  This scheme helps us
  // keep the code compact; however, we might have to re-order the result
  // afterwards.
  const fcl::CollisionObjectd& s = a_is_sphere ? a : b;
  const fcl::CollisionObjectd& o = a_is_sphere ? b : a;
  const fcl::CollisionGeometryd* s_geometry = s.collisionGeometry().get();
  const fcl::CollisionGeometryd* o_geometry = o.collisionGeometry().get();
  const math::RigidTransform<T>& X_WS(a_is_sphere ? X_WA : X_WB);
  const math::RigidTransform<T>& X_WO(a_is_sphere ? X_WB : X_WA);
  const auto id_S = EncodedData(s).id();
  const auto id_O = EncodedData(o).id();
  DistancePairGeometry<T> distance_pair(id_S, id_O, X_WS, X_WO, result);
  const auto& sphere_S = *static_cast<const fcl::Sphered*>(s_geometry);
  switch (o_geometry->getNodeType()) {
    case fcl::GEOM_SPHERE: {
      const auto& sphere_O = *static_cast<const fcl::Sphered*>(o_geometry);
      distance_pair(sphere_S, sphere_O);
      break;
    }
    case fcl::GEOM_BOX: {
      const auto& box_O = *static_cast<const fcl::Boxd*>(o_geometry);
      distance_pair(sphere_S, box_O);
      break;
    }
    case fcl::GEOM_CYLINDER: {
      const auto& cylinder_O = *static_cast<const fcl::Cylinderd*>(o_geometry);
      distance_pair(sphere_S, cylinder_O);
      break;
    }
    case fcl::GEOM_HALFSPACE: {
      const auto& halfspace_O =
          *static_cast<const fcl::Halfspaced*>(o_geometry);
      distance_pair(sphere_S, halfspace_O);
      break;
    }
    case fcl::GEOM_CAPSULE: {
      const auto& capsule_O =
          *static_cast<const fcl::Capsuled*>(o_geometry);
      distance_pair(sphere_S, capsule_O);
      break;
    }
    default: {
      // The only way to reach this is for the RequresFallback() method to be
      // out of sync with this code -- that would be a bug.
      DRAKE_UNREACHABLE();
    }
  }
  // If needed, re-order the result for (s,o) back to the result for (a,b).
  if (!a_is_sphere) {
    result->SwapAAndB();
  }
}

bool ScalarSupport<double>::is_supported(fcl::NODE_TYPE node1,
                                         fcl::NODE_TYPE node2) {
  // Doubles (via its fallback) can support anything *except*
  // halfspace-X (where X is not sphere).
  // We use FCL's GJK/EPA fallback in those geometries we haven't explicitly
  // supported. However, FCL doesn't support: half spaces, planes, triangles,
  // or octtrees in that workflow. We need to give intelligent feedback rather
  // than the segfault otherwise produced.
  // NOTE: Currently this only tests for halfspace (because it is an otherwise
  // supported geometry type in SceneGraph. When meshes, planes, and/or
  // octrees are supported, this error would have to be modified.
  // TODO(SeanCurtis-TRI): Remove this test when FCL/Drake supports signed
  // distance queries for halfspaces (see issue #10905). Also see FCL issue
  // https://github.com/flexible-collision-library/fcl/issues/383.
  return (node1 != fcl::GEOM_HALFSPACE || node2 == fcl::GEOM_SPHERE) &&
         (node2 != fcl::GEOM_HALFSPACE || node1 == fcl::GEOM_SPHERE);
}

template <typename T>
bool Callback(fcl::CollisionObjectd* object_A_ptr,
              fcl::CollisionObjectd* object_B_ptr, void* callback_data,
              // NOLINTNEXTLINE
              double& max_distance) {
  auto& data = *static_cast<CallbackData<T>*>(callback_data);

  // Three things:
  //   1. We repeatedly set max_distance in each call to the callback because we
  //   can't initialize it. The cost is negligible but maximizes any culling
  //   benefit.
  //   2. Due to how FCL is implemented, passing a value <= 0 will cause results
  //   to be omitted because the bounding box test only considers *separating*
  //   distance and doesn't do any work if the distance between bounding boxes
  //   is zero.
  //   3. We pass in a number smaller than the typical epsilon because typically
  //   computation tolerances are greater than or equal to epsilon() and we
  //   don't want this value to trip those tolerances. This is safe because the
  //   bounding box test in which this is used doesn't produce a code via
  //   calculation; it is a perfect, hard-coded zero.
  const double kEps = std::numeric_limits<double>::epsilon() / 10;
  max_distance = std::max(data.max_distance, kEps);

  const EncodedData encoding_a(*object_A_ptr);
  const EncodedData encoding_b(*object_B_ptr);

  const bool can_collide = data.collision_filter.CanCollideWith(
      encoding_a.id(), encoding_b.id());

  if (can_collide) {
    // Throw if the geometry-pair isn't supported.
    if (ScalarSupport<T>::is_supported(
            object_A_ptr->collisionGeometry()->getNodeType(),
            object_B_ptr->collisionGeometry()->getNodeType())) {
      // We want to pass object_A and object_B to the narrowphase distance in a
      // specific order. This way the broadphase distance is free to give us
      // either (A,B) or (B,A), but the narrowphase distance will always receive
      // the result in a consistent order.
      const GeometryId orig_id_A = encoding_a.id();
      const GeometryId orig_id_B = encoding_b.id();
      const bool swap_AB = (orig_id_B < orig_id_A);

      // NOTE: Although this function *takes* pointers to non-const objects to
      // satisfy the fcl api, it should not exploit the non-constness to modify
      // the collision objects. We ensure this by a reference to a const version
      // and not directly use the provided pointers afterwards.
      const fcl::CollisionObjectd& fcl_object_A =
          *(swap_AB ? object_B_ptr : object_A_ptr);
      const fcl::CollisionObjectd& fcl_object_B =
          *(swap_AB ? object_A_ptr : object_B_ptr);

      const GeometryId id_A = swap_AB ? encoding_b.id() : encoding_a.id();
      const GeometryId id_B = swap_AB ? encoding_a.id() : encoding_b.id();

      SignedDistancePair<T> signed_pair;
      ComputeNarrowPhaseDistance(fcl_object_A, data.X_WGs.at(id_A),
                                 fcl_object_B, data.X_WGs.at(id_B),
                                 data.request, &signed_pair);
      if (ExtractDoubleOrThrow(signed_pair.distance) <= data.max_distance) {
        data.nearest_pairs.emplace_back(std::move(signed_pair));
      }
    } else {
      throw std::logic_error(
          fmt::format("Signed distance queries between shapes '{}' and '{}' "
                      "are not supported for scalar type {}",
                      GetGeometryName(*object_A_ptr),
                      GetGeometryName(*object_B_ptr), NiceTypeName::Get<T>()));
    }
  }
  // Returning true would tell the broadphase manager to terminate early. Since
  // we want to find all the signed distance present in the model's current
  // configuration, we return false.
  return false;
}

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS((
    &ComputeNarrowPhaseDistance<T>,
    &Callback<T>
))

}  // namespace shape_distance
}  // namespace internal
}  // namespace geometry
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::geometry::internal::shape_distance::DistancePairGeometry)
