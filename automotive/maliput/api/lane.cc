#include "drake/automotive/maliput/api/lane.h"

namespace drake {
namespace maliput {
namespace api {

// These instantiations must match the API documentation in lane.h.
template<>
GeoPositionT<double> Lane::ToGeoPositionT<double>(
    const LanePositionT<double>& lane_pos) const {
  return DoToGeoPosition(lane_pos);
}

template<>
GeoPositionT<AutoDiffXd> Lane::ToGeoPositionT<AutoDiffXd>(
    const LanePositionT<AutoDiffXd>& lane_pos) const {
  // Fail fast if lane_pos contains derivatives of inconsistent sizes.
  const Eigen::VectorXd deriv = lane_pos.s().derivatives();
  DRAKE_THROW_UNLESS(deriv.size() == lane_pos.r().derivatives().size());
  DRAKE_THROW_UNLESS(deriv.size() == lane_pos.h().derivatives().size());

  return DoToGeoPositionAutoDiff(lane_pos);
}

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
  Vector3<AutoDiffXd> srh = result.srh();
  Eigen::internal::make_coherent(srh.x().derivatives(), deriv);
  Eigen::internal::make_coherent(srh.y().derivatives(), deriv);
  Eigen::internal::make_coherent(srh.z().derivatives(), deriv);
  result.set_srh(srh);
  if (nearest_point != nullptr) {
    Vector3<AutoDiffXd> xyz = nearest_point->xyz();
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
