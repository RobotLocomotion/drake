#include "drake/automotive/maliput/api/lane.h"

namespace drake {
namespace maliput {
namespace api {

// These instantiations must match the API documentation in lane.h.
template LanePositionT<double> Lane::ToLanePositionT<double>(
    const GeoPositionT<double>& geo_position,
    GeoPositionT<double>* nearest_point,
    double* distance) const;
template LanePositionT<AutoDiffXd> Lane::ToLanePositionT<AutoDiffXd>(
    const GeoPositionT<AutoDiffXd>& geo_position,
    GeoPositionT<AutoDiffXd>* nearest_point,
    AutoDiffXd* distance) const;

}  // namespace api
}  // namespace maliput
}  // namespace drake
