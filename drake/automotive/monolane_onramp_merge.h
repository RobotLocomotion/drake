#pragma once

#include <cmath>
#include <iostream>

#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/monolane/builder.h"
#include "drake/automotive/maliput/monolane/road_geometry.h"
#include "drake/automotive/maliput/monolane/road_section_builder.h"

namespace drake {
namespace automotive {

namespace mono = maliput::monolane;

/// MonolaneOnrampMerge creates a road geometry representative of a lane-merge
/// scenario.
template <typename T>
class MonolaneOnrampMerge {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MonolaneOnrampMerge)

  MonolaneOnrampMerge() {
    b_.reset(new mono::Builder(road_.lane_bounds, road_.driveable_bounds,
                               kLinearTolerance_, kAngularTolerance_));
    BuildOnramp();
  }

  MonolaneOnrampMerge(const mono::RoadCharacteristics& rc) : road_(rc) {
    b_.reset(new mono::Builder(road_.lane_bounds, road_.driveable_bounds,
                               kLinearTolerance_, kAngularTolerance_));
    BuildOnramp();
  };

  std::unique_ptr<const maliput::api::RoadGeometry> own_road_geometry() {
    return std::move(rg_);
  };

 private:
  void BuildOnramp();
  const double kLinearTolerance_ = 0.01;
  const double kAngularTolerance_ = 0.01 * M_PI;

  const mono::RoadCharacteristics road_{};
  std::unique_ptr<const maliput::api::RoadGeometry> rg_;
  std::unique_ptr<mono::Builder> b_;
};

}  // namespace automotive
}  // namespace drake
