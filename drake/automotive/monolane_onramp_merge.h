#pragma once

#include <cmath>
#include <iostream>

#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/monolane/builder.h"

namespace drake {
namespace maliput {
namespace monolane {

const double& kDefaultLaneWidth = 2.;
const double& kDefaultDriveableWidth = 4.;

template <typename T>
class MonolaneOnrampMerge {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MonolaneOnrampMerge)

  MonolaneOnrampMerge(const T& lane_length);

  MonolaneOnrampMerge(const T& lane_length,
                      const T& lane_width,
                      const T& driveable_width);

  void Build(const T& lane_length);

  std::unique_ptr<const api::RoadGeometry> own_road_geometry() {
    return std::move(rg_);
  };

 private:
  api::RBounds lane_bounds_{-kDefaultLaneWidth / 2., kDefaultLaneWidth / 2.};
  api::RBounds driveable_bounds_{-kDefaultDriveableWidth / 2.,
        kDefaultDriveableWidth / 2.};
  std::unique_ptr<const api::RoadGeometry> rg_;
};

}  // namespace monolane
}  // namespace maliput
}  // namespace drake
