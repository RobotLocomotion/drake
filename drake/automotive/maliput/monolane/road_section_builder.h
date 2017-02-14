#pragma once

#include <cmath>
#include <iostream>

#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/monolane/builder.h"
#include "drake/automotive/maliput/monolane/road_geometry.h"

namespace drake {
namespace maliput {
namespace monolane {

enum ArcDirection { kCW, kCCW };

/// RoadCharacteristics computes and stores characteristics of a road network;
/// i.e. bounds on the lane width and driveable width. Default settings are
/// taken if no others are specified.
struct RoadCharacteristics {
  /// Constructor for using default road geometries.
  RoadCharacteristics() = default;

  /// Constructor for custom-set road geometries.
  RoadCharacteristics(const double lw, const double dw)
      : lane_width(lw), driveable_width(dw) {}

  // Default parameters.
  const double lane_width{4.};
  const double driveable_width{8.};

  const api::RBounds lane_bounds{-lane_width / 2., lane_width / 2.};
  const api::RBounds driveable_bounds{-driveable_width / 2.,
                                      driveable_width / 2.};
};

/// RoadSectionBuilder is a convenience utility used on top of Builder that
/// allows a user to pave an unbranching lane from some specified starting
/// configuration in a freehand manner.  Two types of monolane primitives are
/// supported: straight-lane or arc-lane.  RoadSectionBuilder offers compact
/// semantics for each added primitive; the user specifies the primitive's
/// length and, if an arc lane, the arc's radius and direction.  Optionally, the
/// user is free to specify elevation/superelevation at the boundary condition
/// of the primitive.  The user may choose to reverse the direction of the
/// entire section of road.

/// If the @p starting_config is not provided, the road begins at the origin
/// of the world.  RoadSectionBuilder takes owership of the @p builder upon
/// construction, releasing back to the caller when Finalize is invoked.
template <typename T>
class RoadSectionBuilder {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RoadSectionBuilder)

  /// @p builder is the monolane Builder into which we want to construct a new
  /// road section, @p is_reversed is a boolean indicating that the road will be
  /// constructed in reverse order, and @p starting_config is the datum for the
  /// new road section.
  RoadSectionBuilder(std::unique_ptr<Builder> builder, const bool& is_reversed)
      : is_reversed_(is_reversed), b_(std::move(builder)) {}

  RoadSectionBuilder(std::unique_ptr<Builder> builder, const bool& is_reversed,
                     Endpoint starting_config)
      : is_reversed_(is_reversed), last_endpoint_(starting_config),
        b_(std::move(builder)) {}

  /// Adds an arc segment, with some specified @p arc_length, @p arc_radius, @p
  /// arc_direction, and EndpointZ specifier, @p end_z, of lane
  /// elevation/superelevation at the end of the primitive.
  void AddArcPrimitive(const T& arc_length, const T& arc_radius,
                     const ArcDirection& arc_direction, const EndpointZ& end_z);

  /// Vanilla version of AddArcSegment assuming a flat lane primitive.
  void AddArcPrimitive(const T& arc_length, const T& arc_radius,
                     const ArcDirection& arc_direction);

  /// Adds a linear segment, with some specified @p length and EndpointZ
  /// specifier, @p end_z, of lane elevation/superelevation at the end of the
  /// primitive.
  void AddLinearPrimitive(const T& length, const EndpointZ& end_z);

  /// Vanilla version of AddLinearSegment assuming a flat lane primitive.
  void AddLinearPrimitive(const T& length);

  /// Gets the Endpoint at the final end position for the road.
  const Endpoint& get_last_endpoint() { return last_endpoint_; }

  /// Returns ownership of the Builder back to the caller.
  std::unique_ptr<Builder> Finalize() { return std::move(b_); }

 private:
  const bool is_reversed_{false};
  const EndpointXy origin_xy_{0., 0., 0.};
  const EndpointZ flat_z_{0., 0., 0., 0.};

  Endpoint last_endpoint_{origin_xy_, flat_z_};
  int id_{0};

  std::unique_ptr<Builder> b_;
};

}  // namespace monolane
}  // namespace maliput
}  // namespace drake
