#pragma once

#include <cmath>
#include <iostream>
#include <memory>
#include <utility>

#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/monolane/builder.h"
#include "drake/automotive/maliput/monolane/road_geometry.h"
#include "drake/automotive/maliput/monolane/road_section_builder.h"

namespace drake {
namespace automotive {

namespace mono = maliput::monolane;

/// MonolaneOnrampMerge contains an example lane-merge scenario expressed as a
/// maliput monolane road geometry.  The intent of this class is to enable easy
/// creation and modification of road geometries for simulating/analyzing
/// such scenarios.
template <typename T>
class MonolaneOnrampMerge {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MonolaneOnrampMerge)

  /// Constructor for the example.  Optionally, the user may supply @p rc, a
  /// RoadCharacteristics structure that aggregates the road boundary data.
  MonolaneOnrampMerge() {
    b_.reset(new mono::Builder(road_.lane_bounds, road_.driveable_bounds,
                               kLinearTolerance_, kAngularTolerance_));
    BuildOnramp();
  }

  explicit MonolaneOnrampMerge(const maliput::monolane::RoadCharacteristics& rc)
      : road_(rc) {
    b_.reset(new mono::Builder(road_.lane_bounds, road_.driveable_bounds,
                               kLinearTolerance_, kAngularTolerance_));
    BuildOnramp();
  }

  /// Produces the resultant RoadGeometry, relinquishing ownership.
  std::unique_ptr<const maliput::api::RoadGeometry> own_road_geometry() {
    return std::move(rg_);
  }

 private:
  /// Implements the onramp example.
  void BuildOnramp();

  /// Tolerances for monolane's Builder.
  const double kLinearTolerance_ = 0.01;
  const double kAngularTolerance_ = 0.01 * M_PI;

  const mono::RoadCharacteristics road_{};
  std::unique_ptr<const maliput::api::RoadGeometry> rg_;
  std::unique_ptr<mono::Builder> b_;
};

}  // namespace automotive
}  // namespace drake
