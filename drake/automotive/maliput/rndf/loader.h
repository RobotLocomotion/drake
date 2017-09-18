#pragma once

#include <memory>
#include <string>

#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/common/drake_copyable.h"

namespace drake {
namespace maliput {
namespace rndf {

/// Common RoadGeometry characteristics needed to construct one.
struct RoadCharacteristics {
  RoadCharacteristics() = default;

  /// Constructor for custom road geometries.
  /// @param default_width The default lane width.
  /// @param linear_tolerance The linear tolerance for lane geometries.
  /// @param angular_tolerance The angular tolerance for lane geometries.
  explicit RoadCharacteristics(double _default_width, double _linear_tolerance,
                               double _angular_tolerance)
      : default_width(_default_width),
        linear_tolerance(_linear_tolerance),
        angular_tolerance(_angular_tolerance) {}

  /// Default width for RNDF Lanes.
  double default_width{4.};
  /// Linear tolerance for RNDF RoadGeometry.
  double linear_tolerance{0.01};
  /// Angular tolerance for RNDF RoadGeometry.
  double angular_tolerance{0.01 * M_PI};
};

/// A class to load an RNDF map into a Maliput RoadGeometry.
///
/// As RNDF waypoint locations are given in UTM (latitude/longitude)
/// coordinates, these are mapped to ENU (cartesian) coordinates
/// whose origin coincides with the location of the '1.1.1' waypoint. Note
/// that due to the essentially planar nature of the underlying RoadGeometry
/// implementation, the elevation coordinate will be forced to 0.
///
/// Additional customization on the resulting road geometry can be performed
/// by instantiating the Loader with specific RoadCharacteristics.
class Loader {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Loader);

  /// Default constructor, using default RoadCharacteristics.
  Loader() = default;

  /// Constructs a Loader instance, using the given @p road_characteristics.
  explicit Loader(const RoadCharacteristics& road_characteristics)
      : road_characteristics_(road_characteristics) {}

  /// Loads a given RNDF at @p filepath and builds an equivalent RoadGeometry.
  ///
  /// @param filepath The RNDF path.
  /// @return The built api::RoadGeometry.
  /// @throw std::runtime_error When the given file is not a valid RNDF.
  /// @throw std::runtime_error When the given RNDF doesn't have at least
  /// a single lane segment with one or more waypoints. Zones only RNDFs are
  /// not supported.
  std::unique_ptr<const api::RoadGeometry> LoadFile(
      const std::string& filepath) const;

 private:
  const RoadCharacteristics road_characteristics_{};
};

}  // namespace rndf
}  // namespace maliput
}  // namespace drake
