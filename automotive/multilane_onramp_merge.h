#pragma once

#include <cmath>
#include <memory>

#include "drake/automotive/deprecated.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/multilane/builder.h"
#include "drake/automotive/maliput/multilane/road_curve.h"

namespace drake {
namespace automotive {

/// MultilaneRoadCharacteristics computes and stores characteristics of a
/// multilane road network; i.e. bounds on the lane width and shoulders width.
/// Default settings are taken if no others are specified.
struct DRAKE_DEPRECATED_AUTOMOTIVE
    MultilaneRoadCharacteristics {
  /// Constructor for using default road geometries.
  MultilaneRoadCharacteristics() = default;

  /// Constructor for custom road geometries.
  ///
  /// @param lw Lane's width.
  /// @param lshoulder The left shoulder width.
  /// @param rshoulder The right shoulder width.
  /// @param lnumber The number of lanes.
  MultilaneRoadCharacteristics(double lw, double lshoulder, double rshoulder,
                               int lnumber)
      : lane_width(lw),
        left_shoulder(lshoulder),
        right_shoulder(rshoulder),
        lane_number(lnumber) {}

  const double lane_width{4.};
  const double left_shoulder{2.};
  const double right_shoulder{2.};
  const int lane_number{1};

  const maliput::api::HBounds elevation_bounds{0., 5.2};
};

/// MultilaneOnrampMerge contains an example lane-merge scenario expressed as a
/// maliput mulitilane road geometry.  The intent of this class is to enable
/// easy creation and modification of road geometries for simulating/analyzing
/// such scenarios.
///
/// Implements the following onramp example, where each road section is composed
/// of sequences of linear and arc primitives:
///
/// <pre>
///           pre-merge           post-merge
///             road                 road
///        |------>------------+------>-------|
///        |------>------------+------>-------|
///                           /+------>-------|
///                          //
///                         //
///                        //   onramp
///                       ^^
///                       ||
///                       ||
///                       __
/// </pre>
///
/// The number of lanes of each road depends on the properties the
/// MultilaneRoadCharacteristics. `post` roads will have the full number of
/// lanes. `pre` and `onramp` roads will have half plus one lanes (note the
/// integer division) and will be placed to the left and right sides of `post`
/// road respectively. When the full lane number is even, two lanes from `pre`
/// and `onramp` will overlap. Otherwise, only one lane will overlap.
///
/// Note that this factory sets some constants to the `multilane::Builder` when
/// creating the RoadGeometry. Linear and angular tolerances, the scale length
/// and the ComputationPolicy are set to appropriate values to build this
/// RoadGeometry.
class DRAKE_DEPRECATED_AUTOMOTIVE
    MultilaneOnrampMerge {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MultilaneOnrampMerge)

  /// Constructor for the factory.
  ///
  /// @param rc A structure that aggregates the road boundary data.
  explicit MultilaneOnrampMerge(const MultilaneRoadCharacteristics& rc)
      : rc_(rc) {}

  /// Constructor for the example, using default MultilaneRoadCharacteristics
  /// settings.
  MultilaneOnrampMerge()
      : MultilaneOnrampMerge(MultilaneRoadCharacteristics{}) {}

  /// @return A std::unique_ptr<const maliput::api::RoadGeometry> with the
  /// onramp example.
  std::unique_ptr<const maliput::api::RoadGeometry> BuildOnramp() const;

 private:
  // Tolerances and properties for multilane's Builder.
  const double linear_tolerance_{0.01};
  const double angular_tolerance_{0.01 * M_PI};
  const double scale_length_{1.};
  const maliput::multilane::ComputationPolicy computation_policy_{
      maliput::multilane::ComputationPolicy::kPreferSpeed};
  // Road characteristics.
  const MultilaneRoadCharacteristics rc_;
};

}  // namespace automotive
}  // namespace drake
