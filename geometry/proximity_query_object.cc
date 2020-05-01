#include "drake/geometry/proximity_query_object.h"

#include "drake/common/default_scalars.h"
#include "drake/geometry/geometry_state.h"

namespace drake {
namespace geometry {

template <typename T>
std::vector<PenetrationAsPointPair<double>>
ProximityQueryObject<T>::ComputePointPairPenetration() const {
  this->ValidateAndUpdate();
  const GeometryState<T>& state = this->geometry_state();
  return state.ComputePointPairPenetration();
}

template <typename T>
std::vector<SortedPair<GeometryId>>
ProximityQueryObject<T>::FindCollisionCandidates() const {
  this->ValidateAndUpdate();
  const GeometryState<T>& state = this->geometry_state();
  return state.FindCollisionCandidates();
}

template <typename T>
bool ProximityQueryObject<T>::HasCollisions() const {
  this->ValidateAndUpdate();
  const GeometryState<T>& state = this->geometry_state();
  return state.HasCollisions();
}

template <typename T>
std::vector<ContactSurface<T>> ProximityQueryObject<T>::ComputeContactSurfaces()
    const {
  this->ValidateAndUpdate();
  const GeometryState<T>& state = this->geometry_state();
  return state.ComputeContactSurfaces();
}

template <typename T>
void ProximityQueryObject<T>::ComputeContactSurfacesWithFallback(
    std::vector<ContactSurface<T>>* surfaces,
    std::vector<PenetrationAsPointPair<double>>* point_pairs) const {
  DRAKE_DEMAND(surfaces);
  DRAKE_DEMAND(point_pairs);
  this->ValidateAndUpdate();
  const GeometryState<T>& state = this->geometry_state();
  state.ComputeContactSurfacesWithFallback(surfaces, point_pairs);
}

template <typename T>
std::vector<SignedDistancePair<T>>
ProximityQueryObject<T>::ComputeSignedDistancePairwiseClosestPoints(
    const double max_distance) const {
  this->ValidateAndUpdate();
  const GeometryState<T>& state = this->geometry_state();
  return state.ComputeSignedDistancePairwiseClosestPoints(max_distance);
}

template <typename T>
SignedDistancePair<T>
ProximityQueryObject<T>::ComputeSignedDistancePairClosestPoints(
    GeometryId id_A, GeometryId id_B) const {
  this->ValidateAndUpdate();
  const GeometryState<T>& state = this->geometry_state();
  return state.ComputeSignedDistancePairClosestPoints(id_A, id_B);
}

template <typename T>
std::vector<SignedDistanceToPoint<T>>
ProximityQueryObject<T>::ComputeSignedDistanceToPoint(
    const Vector3<T>& p_WQ, const double threshold) const {
  this->ValidateAndUpdate();
  const GeometryState<T>& state = this->geometry_state();
  return state.ComputeSignedDistanceToPoint(p_WQ, threshold);
}

}  // namespace geometry
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::geometry::ProximityQueryObject)
