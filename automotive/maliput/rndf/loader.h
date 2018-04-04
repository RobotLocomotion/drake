#pragma once

#include <memory>
#include <string>

#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/common/drake_copyable.h"

namespace drake {
namespace maliput {
namespace rndf {

/// Holds common api::RoadGeometry characteristics needed to construct one.
struct RoadCharacteristics {
  RoadCharacteristics() = default;

  /// Constructor for custom geometry characteristics.
  /// @param default_width_in The default lane width in meters, used when either
  /// no lane width or zero lane width was specified in the RNDF (as lane widths
  /// are an optional non negative quantity in the RNDF format).
  /// @param linear_tolerance_in The linear tolerance for lane geometries, in
  /// radians.
  /// @param angular_tolerance_in The angular tolerance for lane geometries, in
  /// radians.
  explicit RoadCharacteristics(double default_width_in,
                               double linear_tolerance_in,
                               double angular_tolerance_in)
      : default_width(default_width_in),
        linear_tolerance(linear_tolerance_in),
        angular_tolerance(angular_tolerance_in) {}

  /// Default width for RNDF Lanes, in meters.
  double default_width{4.};
  /// Linear tolerance for RNDF RoadGeometry, in radians.
  double linear_tolerance{0.01};
  /// Angular tolerance for RNDF RoadGeometry, in radians.
  double angular_tolerance{0.01 * M_PI};
};

/// Loads a given RNDF at @p filepath and builds an equivalent
/// api::RoadGeometry with the given @p road_characteristics.
///
/// RNDF waypoints are given in UTM (latitude / longitude) coordinates.
/// In the resulting api::RoadGeometry, they are mapped to ENU (Cartesian)
/// coordinates whose origin coincides with the location of waypoint '1.1.1'.
/// Note that due to the planar nature of the underlying api::RoadGeometry
/// implementation, the elevation coordinate will be forced to 0.
///
/// @param filepath The RNDF path.
/// @param road_characteristics The common geometrical aspects to comply with
/// when building the api::RoadGeometry.
/// @return The built api::RoadGeometry.
/// @throw std::runtime_error When the given file is not a valid RNDF.
/// @throw std::runtime_error When the given RNDF doesn't have at least
/// a single lane segment with one or more waypoints. RNDFs containing
/// only zones are not supported.
std::unique_ptr<const api::RoadGeometry> LoadFile(
    const std::string& filepath,
    const RoadCharacteristics& road_characteristics);

/// Loads a given RNDF at @p filepath and builds an equivalent
/// api::RoadGeometry.using default RoadCharacteristics.
///
/// This is an overloaded function provided for convenience. See
/// LoadFile(const std::string&, const RoadCharacteristics&).
std::unique_ptr<const api::RoadGeometry> LoadFile(
    const std::string& filepath);

}  // namespace rndf
}  // namespace maliput
}  // namespace drake
