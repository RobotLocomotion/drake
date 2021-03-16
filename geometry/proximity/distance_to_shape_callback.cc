#include "drake/geometry/proximity/distance_to_shape_callback.h"

#include <algorithm>
#include <limits>
#include <utility>

namespace drake {
namespace geometry {
namespace internal {
namespace shape_distance {

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
  //  outward face normal when the two objects are touching and set
  //  is_nhat_BA_W_unique to true.
  if (std::abs(result.min_distance) < kEps) {
    pair_data->nhat_BA_W = Eigen::Vector3d(kNan, kNan, kNan);
    pair_data->is_nhat_BA_W_unique = false;
  } else {
    pair_data->nhat_BA_W = (p_WCa - p_WCb) / result.min_distance;
    pair_data->is_nhat_BA_W_unique = true;
  }
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
      encoding_a.encoding(), encoding_b.encoding());

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

template bool Callback<double>(fcl::CollisionObjectd*, fcl::CollisionObjectd*,
                               void*, double&);
template bool Callback<AutoDiffXd>(fcl::CollisionObjectd*,
                                   fcl::CollisionObjectd*, void*, double&);

}  // namespace shape_distance
}  // namespace internal
}  // namespace geometry
}  // namespace drake
