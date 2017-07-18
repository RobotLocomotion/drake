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
  return DoToLanePosition(geo_pos, nearest_point, distance);
}

}  // namespace api
}  // namespace maliput
}  // namespace drake
