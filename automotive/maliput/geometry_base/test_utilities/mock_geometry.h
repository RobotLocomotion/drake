#pragma once

#include "drake/automotive/deprecated.h"
#include "drake/automotive/maliput/geometry_base/branch_point.h"
#include "drake/automotive/maliput/geometry_base/junction.h"
#include "drake/automotive/maliput/geometry_base/lane.h"
#include "drake/automotive/maliput/geometry_base/road_geometry.h"
#include "drake/automotive/maliput/geometry_base/segment.h"

namespace drake {
namespace maliput {
namespace geometry_base {
namespace test {

/// @file
/// Mock concrete implementation of maliput geometry API, useful for tests.
///
/// The classes in this file are concrete implementations of the
/// maliput geometry API, built on top of the @ref
/// drake::maliput::geometry_base "geometry_base" base classes.  The
/// only difference between these classes and @ref
/// drake::maliput::geometry_base "geometry_base" is that all the
/// remaining pure virtual methods in @ref
/// drake::maliput::geometry_base "geometry_base" (i.e., the methods
/// involving actual lane-frame and world-frame geometry) have been
/// provided with implementations that simply throw `std::exception`.
/// These "Mock" classes do provide sufficient functionality to
/// exercise the geometry API's object graph.
///
/// All virtual methods are overridable (i.e., none are marked `final`).
/// Test implementors may re-implement methods as they see fit.

/// Mock api::RoadGeometry implementation; see mock_geometry.h.
class DRAKE_DEPRECATED_AUTOMOTIVE
    MockRoadGeometry : public geometry_base::RoadGeometry {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MockRoadGeometry);

  /// Constructs an empty MockRoadGeometry with the specified tolerances.
  ///
  /// @param id the ID of the RoadGeometry
  /// @param linear_tolerance the linear tolerance
  /// @param angular_tolerance the angular tolerance
  ///
  /// @throws std::exception if either `linear_tolerance` or
  ///         `angular_tolerance` or `scale_length` is non-positive.
  MockRoadGeometry(const api::RoadGeometryId& id,
                   double linear_tolerance,
                   double angular_tolerance,
                   double scale_length)
      : geometry_base::RoadGeometry(id, linear_tolerance, angular_tolerance,
                                    scale_length) {}

 private:
  api::RoadPosition DoToRoadPosition(const api::GeoPosition& geo_position,
                                     const api::RoadPosition* hint,
                                     api::GeoPosition* nearest_position,
                                     double* distance) const override;
};


/// Mock api::BranchPoint implementation; see mock_geometry.h.
class DRAKE_DEPRECATED_AUTOMOTIVE
    MockBranchPoint : public geometry_base::BranchPoint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MockBranchPoint);

  /// Constructs a partially-initialized MockBranchPoint.
  ///
  /// @param id the ID
  ///
  /// See geometry_base::BranchPoint for discussion on initialization.
  explicit MockBranchPoint(const api::BranchPointId& id)
      : geometry_base::BranchPoint(id) {}
};


/// Mock api::Junction implementation; see mock_geometry.h.
class DRAKE_DEPRECATED_AUTOMOTIVE
    MockJunction : public geometry_base::Junction {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MockJunction);

  /// Constructs a partially-initialized MockJunction.
  ///
  /// @param id the ID
  ///
  /// See geometry_base::Junction for discussion on initialization.
  explicit MockJunction(const api::JunctionId& id)
      : geometry_base::Junction(id) {}
};


/// Mock api::Segment implementation; see mock_geometry.h.
class DRAKE_DEPRECATED_AUTOMOTIVE
    MockSegment : public geometry_base::Segment {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MockSegment);

  /// Constructs a partially-initialized MockSegment.
  ///
  /// @param id the ID
  ///
  /// See geometry_base::Segment for discussion on initialization.
  explicit MockSegment(const api::SegmentId& id)
      : geometry_base::Segment(id) {}
};


/// Mock api::Lane implementation; see mock_geometry.h.
class DRAKE_DEPRECATED_AUTOMOTIVE
    MockLane : public geometry_base::Lane {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MockLane);

  /// Constructs a partially-initialized MockLane.
  ///
  /// @param id the ID
  ///
  /// See geometry_base::Lane for discussion on initialization.
  explicit MockLane(const api::LaneId& id) : geometry_base::Lane(id) {}

 private:
  double do_length() const override;
  api::RBounds do_lane_bounds(double) const override;
  api::RBounds do_driveable_bounds(double) const override;
  api::HBounds do_elevation_bounds(double, double) const override;
  api::GeoPosition DoToGeoPosition(
      const api::LanePosition& lane_pos) const override;
  api::Rotation DoGetOrientation(
      const api::LanePosition& lane_pos) const override;
  api::LanePosition DoEvalMotionDerivatives(
      const api::LanePosition& position,
      const api::IsoLaneVelocity& velocity) const override;
  api::LanePosition DoToLanePosition(
      const api::GeoPosition& geo_position,
      api::GeoPosition* nearest_position, double* distance) const override;
};


}  // namespace test
}  // namespace geometry_base
}  // namespace maliput
}  // namespace drake
