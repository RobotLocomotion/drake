#include "drake/geometry/proximity/distance_to_shape_callback.h"

#include <algorithm>
#include <limits>
#include <utility>

#include "drake/common/default_scalars.h"
#include "drake/geometry/proximity/collision_backend.h"
#include "drake/geometry/proximity/distance_to_point_callback.h"
#include "drake/geometry/proximity/distance_to_shape_touching.h"
#include "drake/math/rotation_matrix.h"

namespace drake {
namespace geometry {
namespace internal {
namespace shape_distance {

template <typename T, typename Backend>
void DistancePairGeometry<T, Backend>::operator()(
    const typename Backend::Sphere& sphere_A,
    const typename Backend::Sphere& sphere_B) {
  SphereShapeDistance(sphere_A, sphere_B);
}

template <typename T, typename Backend>
void DistancePairGeometry<T, Backend>::operator()(
    const typename Backend::Sphere& sphere_A,
    const typename Backend::Box& box_B) {
  SphereShapeDistance(sphere_A, box_B);
}

template <typename T, typename Backend>
void DistancePairGeometry<T, Backend>::operator()(
    const typename Backend::Sphere& sphere_A,
    const typename Backend::Cylinder& cylinder_B) {
  SphereShapeDistance(sphere_A, cylinder_B);
}

template <typename T, typename Backend>
void DistancePairGeometry<T, Backend>::operator()(
    const typename Backend::Sphere& sphere_A,
    const typename Backend::Halfspace& halfspace_B) {
  SphereShapeDistance(sphere_A, halfspace_B);
}

template <typename T, typename Backend>
void DistancePairGeometry<T, Backend>::operator()(
    const typename Backend::Sphere& sphere_A,
    const typename Backend::Capsule& capsule_B) {
  SphereShapeDistance(sphere_A, capsule_B);
}

template <typename T, typename Backend>
template <typename FclShape>
void DistancePairGeometry<T, Backend>::SphereShapeDistance(
    const typename Backend::Sphere& sphere_A, const FclShape& shape_B) {
  const SignedDistanceToPoint<T> shape_B_to_point_Ao =
      point_distance::DistanceToPoint<T, Backend>(id_B_, X_WB_,
                                                  X_WA_.translation())(shape_B);
  result_->id_A = id_A_;
  result_->id_B = id_B_;
  result_->distance = shape_B_to_point_Ao.distance - Backend::radius(sphere_A);
  // p_BCb is the witness point on ∂B measured and expressed in B.
  result_->p_BCb = shape_B_to_point_Ao.p_GN;
  result_->nhat_BA_W = shape_B_to_point_Ao.grad_W;
  // p_ACa is the witness point on ∂A measured and expressed in A.
  const math::RotationMatrix<T> R_AW = X_WA_.rotation().transpose();
  result_->p_ACa =
      -Backend::radius(sphere_A) * (R_AW * shape_B_to_point_Ao.grad_W);
}

template <typename T, typename Backend>
void CalcDistanceFallback(const typename Backend::Object& a,
                          const math::RigidTransform<T>& X_WA,
                          const typename Backend::Object& b,
                          const math::RigidTransform<T>& X_WB,
                          const typename Backend::DistanceRequest& request,
                          SignedDistancePair<T>* pair_data) {
  if constexpr (!std::is_same_v<T, double>) {
    unused(X_WA, X_WB, request, pair_data);
    throw std::logic_error(fmt::format(
        "Signed distance queries between shapes '{}' and '{}' "
        "are not supported for scalar type {}. See the documentation for "
        "QueryObject::ComputeSignedDistancePairwiseClosestPoints() for the "
        "full status of supported geometries.",
        GetGeometryName<Backend>(a), GetGeometryName<Backend>(b),
        NiceTypeName::Get<T>()));
  } else {
    typename Backend::DistanceResult result;
    Backend::Distance(a.collisionGeometry().get(), a.getTransform(),
                      b.collisionGeometry().get(), b.getTransform(), request,
                      &result);

    pair_data->id_A = EncodedData(a).id();
    pair_data->id_B = EncodedData(b).id();

    pair_data->distance = result.min_distance;

    // Setting the witness points.
    const Eigen::Vector3d& p_WCa = result.nearest_points[0];
    pair_data->p_ACa = X_WA.inverse() * p_WCa;
    const Eigen::Vector3d& p_WCb = result.nearest_points[1];
    pair_data->p_BCb = X_WB.inverse() * p_WCb;

    // Setting the normal.
    // TODO(DamrongGuoy): We should set the tolerance through SceneGraph for
    //  determining whether the two geometries are touching or not. For now, we
    //  use this number.
    const double kEps = 1e-14;

    if (std::abs(result.min_distance) < kEps) {
      pair_data->nhat_BA_W = CalcGradientWhenTouching<Backend>(
          a, X_WA, b, X_WB, pair_data->p_ACa, pair_data->p_BCb);
    } else {
      pair_data->nhat_BA_W = (p_WCa - p_WCb) / result.min_distance;
    }
  }
}

template <typename Backend>
bool RequiresFallback(const typename Backend::Object& a,
                      const typename Backend::Object& b) {
  /* In the current ecosystem, we only have high-fidelity geometric code for
   Sphere-X (for *some* X). So, the conditions requiring the fallback is
   a) neither is a sphere, or b) one is a sphere, the other is the wrong X. */
  const ShapeKind a_kind =
      Backend::Classify(a.collisionGeometry()->getNodeType());
  const ShapeKind b_kind =
      Backend::Classify(b.collisionGeometry()->getNodeType());
  if (a_kind != ShapeKind::kSphere && b_kind != ShapeKind::kSphere) {
    return true;
  }
  const ShapeKind other_kind = a_kind == ShapeKind::kSphere ? b_kind : a_kind;
  // Box, capsule, cylinder, half space, and cylinder don't required fallback.
  // Ellipsoid, convex do. Other shape kinds aren't currently used. Note
  // that the convex kind is used to represent drake::geometry::Mesh.
  return other_kind == ShapeKind::kEllipsoid ||
         other_kind == ShapeKind::kConvex;
}

template <typename T, typename Backend>
void ComputeNarrowPhaseDistance(
    const typename Backend::Object& a, const math::RigidTransform<T>& X_WA,
    const typename Backend::Object& b, const math::RigidTransform<T>& X_WB,
    const typename Backend::DistanceRequest& request,
    SignedDistancePair<T>* result) {
  DRAKE_DEMAND(result != nullptr);

  if (RequiresFallback<Backend>(a, b)) {
    CalcDistanceFallback<T, Backend>(a, X_WA, b, X_WB, request, result);
    return;
  }

  const ShapeKind a_kind =
      Backend::Classify(a.collisionGeometry()->getNodeType());
  const ShapeKind b_kind =
      Backend::Classify(b.collisionGeometry()->getNodeType());
  // If no fallback is necessary, one of these two *must* be a sphere.
  const bool a_is_sphere = a_kind == ShapeKind::kSphere;
  DRAKE_ASSERT(a_is_sphere || b_kind == ShapeKind::kSphere);
  // We write `s` for the sphere object and `o` for the other object. We
  // assign either (a,b) or (b,a) to (s,o) depending on whether `a` is a
  // sphere or not. Therefore, we only need the helper DistancePairGeometry
  // that takes (sphere, other) but not (other, sphere).  This scheme helps us
  // keep the code compact; however, we might have to re-order the result
  // afterwards.
  const typename Backend::Object& s = a_is_sphere ? a : b;
  const typename Backend::Object& o = a_is_sphere ? b : a;
  const typename Backend::Geometry* s_geometry = s.collisionGeometry().get();
  const typename Backend::Geometry* o_geometry = o.collisionGeometry().get();
  const math::RigidTransform<T>& X_WS(a_is_sphere ? X_WA : X_WB);
  const math::RigidTransform<T>& X_WO(a_is_sphere ? X_WB : X_WA);
  const auto id_S = EncodedData(s).id();
  const auto id_O = EncodedData(o).id();
  DistancePairGeometry<T, Backend> distance_pair(id_S, id_O, X_WS, X_WO,
                                                 result);
  const auto& sphere_S =
      *static_cast<const typename Backend::Sphere*>(s_geometry);
  switch (a_is_sphere ? b_kind : a_kind) {
    case ShapeKind::kSphere: {
      const auto& sphere_O =
          *static_cast<const typename Backend::Sphere*>(o_geometry);
      distance_pair(sphere_S, sphere_O);
      break;
    }
    case ShapeKind::kBox: {
      const auto& box_O =
          *static_cast<const typename Backend::Box*>(o_geometry);
      distance_pair(sphere_S, box_O);
      break;
    }
    case ShapeKind::kCylinder: {
      const auto& cylinder_O =
          *static_cast<const typename Backend::Cylinder*>(o_geometry);
      distance_pair(sphere_S, cylinder_O);
      break;
    }
    case ShapeKind::kHalfspace: {
      const auto& halfspace_O =
          *static_cast<const typename Backend::Halfspace*>(o_geometry);
      distance_pair(sphere_S, halfspace_O);
      break;
    }
    case ShapeKind::kCapsule: {
      const auto& capsule_O =
          *static_cast<const typename Backend::Capsule*>(o_geometry);
      distance_pair(sphere_S, capsule_O);
      break;
    }
    case ShapeKind::kConvex:
    case ShapeKind::kEllipsoid:
    case ShapeKind::kUnsupported: {
      // The only way to reach this is for the RequiresFallback() method to be
      // out of sync with this code -- that would be a bug.
      DRAKE_UNREACHABLE();
    }
  }
  // If needed, re-order the result for (s,o) back to the result for (a,b).
  if (!a_is_sphere) {
    result->SwapAAndB();
  }
}

bool ScalarSupport<double>::is_supported(ShapeKind kind1, ShapeKind kind2) {
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
  return (kind1 != ShapeKind::kHalfspace || kind2 == ShapeKind::kSphere) &&
         (kind2 != ShapeKind::kHalfspace || kind1 == ShapeKind::kSphere);
}

template <typename T, typename Backend>
bool Callback(typename Backend::Object* object_A_ptr,
              typename Backend::Object* object_B_ptr, void* callback_data,
              // NOLINTNEXTLINE
              typename Backend::Scalar& max_distance) {
  auto& data = *static_cast<CallbackData<T, Backend>*>(callback_data);

  // Three things:
  //   1. We repeatedly set max_distance in each call to the callback because we
  //   can't initialize it. The cost is negligible but maximizes any culling
  //   benefit.
  //   2. Due to how the broadphase is implemented, passing a value <= 0 will
  //   cause results
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

  const bool can_collide =
      data.collision_filter == nullptr ||
      data.collision_filter->CanCollideWith(encoding_a.id(), encoding_b.id());

  if (can_collide) {
    // Throw if the geometry-pair isn't supported.
    if (ScalarSupport<T>::is_supported(
            Backend::Classify(object_A_ptr->collisionGeometry()->getNodeType()),
            Backend::Classify(
                object_B_ptr->collisionGeometry()->getNodeType()))) {
      // We want to pass object_A and object_B to the narrowphase distance in a
      // specific order. This way the broadphase distance is free to give us
      // either (A,B) or (B,A), but the narrowphase distance will always receive
      // the result in a consistent order.
      const GeometryId orig_id_A = encoding_a.id();
      const GeometryId orig_id_B = encoding_b.id();
      const bool swap_AB = (orig_id_B < orig_id_A);

      // NOTE: Although this function *takes* pointers to non-const objects to
      // satisfy the backend api, it should not exploit the non-constness to
      // modify the collision objects. We ensure this by a reference to a const
      // version and not directly use the provided pointers afterwards.
      const typename Backend::Object& object_A =
          *(swap_AB ? object_B_ptr : object_A_ptr);
      const typename Backend::Object& object_B =
          *(swap_AB ? object_A_ptr : object_B_ptr);

      const GeometryId id_A = swap_AB ? encoding_b.id() : encoding_a.id();
      const GeometryId id_B = swap_AB ? encoding_a.id() : encoding_b.id();

      SignedDistancePair<T> signed_pair;
      ComputeNarrowPhaseDistance<T, Backend>(object_A, data.X_WGs.at(id_A),
                                             object_B, data.X_WGs.at(id_B),
                                             data.request, &signed_pair);
      if (ExtractDoubleOrThrow(signed_pair.distance) <= data.max_distance) {
        data.nearest_pairs.emplace_back(std::move(signed_pair));
      }
    } else {
      throw std::logic_error(fmt::format(
          "Signed distance queries between shapes '{}' and '{}' "
          "are not supported for scalar type {}. See the documentation for "
          "QueryObject::ComputeSignedDistancePairwiseClosestPoints() for the "
          "full status of supported geometries.",
          GetGeometryName<Backend>(*object_A_ptr),
          GetGeometryName<Backend>(*object_B_ptr), NiceTypeName::Get<T>()));
    }
  }
  // Returning true would tell the broadphase manager to terminate early. Since
  // we want to find all the signed distance present in the model's current
  // configuration, we return false.
  return false;
}

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_SCALARS_AND_BACKENDS(
    (&ComputeNarrowPhaseDistance<T, Backend>, &Callback<T, Backend>,
     &CalcDistanceFallback<T, Backend>));

}  // namespace shape_distance
}  // namespace internal
}  // namespace geometry
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_SCALARS_AND_BACKENDS(
    class ::drake::geometry::internal::shape_distance::DistancePairGeometry);
