#pragma once

#include <cmath>
#include <memory>

#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/monolane/builder.h"

namespace drake {
namespace automotive {

namespace mono = maliput::monolane;

/// RoadCharacteristics computes and stores characteristics of a road network;
/// i.e. bounds on the lane width and driveable width. Default settings are
/// taken if no others are specified.
struct RoadCharacteristics {
  /// Constructor for using default road geometries.
  RoadCharacteristics() = default;

  /// Constructor for custom road geometries.
  RoadCharacteristics(const double lw, const double dw)
      : lane_width(lw), driveable_width(dw) {}

  // Default settings.
  const double lane_width{4.};
  const double driveable_width{8.};

  const maliput::api::RBounds lane_bounds{-lane_width / 2., lane_width / 2.};
  const maliput::api::RBounds driveable_bounds{-driveable_width / 2.,
                                               driveable_width / 2.};
};

/// MonolaneOnrampMerge contains an example lane-merge scenario expressed as a
/// maliput monolane road geometry.  The intent of this class is to enable easy
/// creation and modification of road geometries for simulating/analyzing
/// such scenarios.
///
/// Implements the following onramp example, where each road lane is composed
/// of sequences of linear and arc primitives:
///
/// <pre>
///           pre-merge      post-merge
///             road           road
///        |------>-------+------>-------|
///                      /
///                     /
///            onramp  /
///                   ^
///                   |
///                   |
///                   -
/// </pre>
template <typename T>
class MonolaneOnrampMerge {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MonolaneOnrampMerge)

  /// Constructor for the example.  The user supplies @p rc, a
  /// RoadCharacteristics structure that aggregates the road boundary data.
  explicit MonolaneOnrampMerge(const RoadCharacteristics& rc) {
    rb_.reset(new mono::Builder(rc.lane_bounds, rc.driveable_bounds,
                                kLinearTolerance_, kAngularTolerance_));
  }

  /// Constructor for the example, using default RoadCharacteristics settings.
  MonolaneOnrampMerge() : MonolaneOnrampMerge(RoadCharacteristics{}) {}

  /// Implements the onramp example.
  std::unique_ptr<const maliput::api::RoadGeometry> BuildOnramp();

 private:

  /// Tolerances for monolane's Builder.
  const double kLinearTolerance_ = 0.01;
  const double kAngularTolerance_ = 0.01 * M_PI;

  std::unique_ptr<mono::Builder> rb_;
};

}  // namespace automotive
}  // namespace drake
