#include "drake/geometry/query_object.h"

#include "drake/common/default_scalars.h"

namespace drake {
namespace geometry {

template <typename T>
std::vector<PenetrationAsPointPair<double>>
QueryObject<T>::ComputePointPairPenetration() const {
  ThrowIfNotCallable();

  FullPoseUpdate();
  const GeometryState<T>& state = geometry_state();
  return state.ComputePointPairPenetration();
}

template <typename T>
std::vector<SortedPair<GeometryId>> QueryObject<T>::FindCollisionCandidates()
    const {
  ThrowIfNotCallable();
  // TODO(amcastro-tri): Modify this when the cache system is in place.
  FullPoseUpdate();
  const GeometryState<T>& state = geometry_state();
  return state.FindCollisionCandidates();
}

template <typename T>
bool QueryObject<T>::HasCollisions() const {
  ThrowIfNotCallable();

  FullPoseUpdate();
  const GeometryState<T>& state = geometry_state();
  return state.HasCollisions();
}

template <typename T>
std::vector<ContactSurface<T>>
QueryObject<T>::ComputeContactSurfaces() const {
  ThrowIfNotCallable();

  FullPoseUpdate();
  const GeometryState<T>& state = geometry_state();
  return state.ComputeContactSurfaces();
}

template <typename T>
void QueryObject<T>::ComputeContactSurfacesWithFallback(
    std::vector<ContactSurface<T>>* surfaces,
    std::vector<PenetrationAsPointPair<double>>* point_pairs) const {
  DRAKE_DEMAND(surfaces);
  DRAKE_DEMAND(point_pairs);

  ThrowIfNotCallable();

  FullPoseUpdate();
  const GeometryState<T>& state = geometry_state();
  state.ComputeContactSurfacesWithFallback(surfaces, point_pairs);
}

template <typename T>
std::vector<SignedDistancePair<T>>
QueryObject<T>::ComputeSignedDistancePairwiseClosestPoints(
    const double max_distance) const {
  ThrowIfNotCallable();

  FullPoseUpdate();
  const GeometryState<T>& state = geometry_state();
  return state.ComputeSignedDistancePairwiseClosestPoints(max_distance);
}

template <typename T>
SignedDistancePair<T> QueryObject<T>::ComputeSignedDistancePairClosestPoints(
    GeometryId id_A, GeometryId id_B) const {
  ThrowIfNotCallable();

  FullPoseUpdate();
  const GeometryState<T>& state = geometry_state();
  return state.ComputeSignedDistancePairClosestPoints(id_A, id_B);
}

template <typename T>
std::vector<SignedDistanceToPoint<T>>
QueryObject<T>::ComputeSignedDistanceToPoint(
    const Vector3<T>& p_WQ,
    const double threshold) const {
  ThrowIfNotCallable();

  FullPoseUpdate();
  const GeometryState<T>& state = geometry_state();
  return state.ComputeSignedDistanceToPoint(p_WQ, threshold);
}

}  // namespace geometry
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::geometry::QueryObject)
