#include "drake/automotive/maliput/api/lane.h"

namespace drake {
namespace maliput {
namespace api {

// These instantiations must match the API documentation in lane.h.
template<>
LanePositionT<double> Lane::ToLanePositionT<double>(
    const GeoPositionT<double>& geo_pos,
    GeoPositionT<double>* nearest_point,
    double* distance) const {
  return DoToLanePosition(geo_pos, nearest_point, distance);
}

template<>
LanePositionT<AutoDiffXd> Lane::ToLanePositionT<AutoDiffXd>(
    const GeoPositionT<AutoDiffXd>& geo_pos,
    GeoPositionT<AutoDiffXd>* nearest_point,
    AutoDiffXd* distance) const {
  // Fail fast if geo_pos contains derivatives of inconsistent sizes.
  const Eigen::VectorXd deriv = geo_pos.x().derivatives();
  DRAKE_THROW_UNLESS(deriv.size() == geo_pos.y().derivatives().size());
  DRAKE_THROW_UNLESS(deriv.size() == geo_pos.z().derivatives().size());

  LanePositionT<AutoDiffXd> result =
      DoToLanePositionAutoDiff(geo_pos, nearest_point, distance);

  // If the partial derivatives of result, nearest_point and distance are not of
  // the same dimension as those in geo_pos, pad them with zeros of the same
  // dismension as those in geo_pos.
  Eigen::internal::make_coherent(result.s().derivatives(), deriv);
  Eigen::internal::make_coherent(result.r().derivatives(), deriv);
  Eigen::internal::make_coherent(result.h().derivatives(), deriv);
  if (nearest_point != nullptr) {
    const Vector3<AutoDiffXd>& xyz = nearest_point->xyz();
    Eigen::internal::make_coherent(xyz.x().derivatives(), deriv);
    Eigen::internal::make_coherent(xyz.y().derivatives(), deriv);
    Eigen::internal::make_coherent(xyz.z().derivatives(), deriv);
    nearest_point->set_xyz(xyz);
  }
  if (distance != nullptr) {
    Eigen::internal::make_coherent(distance->derivatives(), deriv);
  }
  return result;
}

}  // namespace api
}  // namespace maliput
}  // namespace drake
