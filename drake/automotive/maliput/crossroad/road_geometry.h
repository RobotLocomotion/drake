#pragma once

#include <limits>
#include <memory>
#include <vector>

#include "drake/automotive/maliput/api/branch_point.h"
#include "drake/automotive/maliput/api/junction.h"
#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/crossroad/branch_point.h"
#include "drake/automotive/maliput/crossroad/junction.h"
#include "drake/common/drake_copyable.h"

namespace drake {
namespace maliput {
namespace crossroad{

/// Crossroad's implementation of api::RoadGeometry.
///
/// To understand the characteristics of the geometry, consult the
/// crossroad::Segment and crossroad::Lane detailed class overview docs.
class RoadGeometry final : public api::RoadGeometry {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RoadGeometry)

  /// Constructs a crossroad RoadGeometry.
  ///
  /// @param[in] id The ID of this RoadGeometry. This can be any user-selectable
  /// value.
  ///
  /// @param[in] num_lanes The number of lanes. This must be greater than zero.
  ///
  /// @param[in] length The length of the crossroad.
  ///
  /// @param[in] lane_width The width of each lane.
  ///
  /// @param[in] shoulder_width The width of the shoulders on each side of the
  /// road.
  ///
  /// @param[in] linear_tolerance The tolerance guaranteed for linear
  /// measurements (positions).
  ///
  /// @param[in] angular_tolerance The tolerance guaranteed for angular
  /// measurements (orientations).
  ///
  RoadGeometry(const api::RoadGeometryId& id,
              int num_horizontal_lanes,
               int num_vertical_lanes,
               double length,
               double lane_width,
               double shoulder_width,
               double linear_tolerance =
                   std::numeric_limits<double>::epsilon(),
               double angular_tolerance =
                   std::numeric_limits<double>::epsilon());

  ~RoadGeometry() final = default;

  const Junction* junction(int index) const { return do_junction(index); }


 private:
  const api::RoadGeometryId do_id() const final { return id_; }

  int do_num_junctions() const final { return 1; }

  const Junction* do_junction(int index) const final;

  int do_num_branch_points() const final;

  const api::BranchPoint* do_branch_point(int index) const final;

  api::RoadPosition DoToRoadPosition(
      const api::GeoPosition& geo_position,
      const api::RoadPosition* hint,
      api::GeoPosition* nearest_position,
      double* distance) const final;

  double do_linear_tolerance() const final { return linear_tolerance_; }

  double do_angular_tolerance() const final { return angular_tolerance_; }

  // Returns true iff `geo_pos` is "on" the crossroad. It is on the crossroad iff
  // `geo_pos.x` and `geo_pos.y` fall within the crossroad's driveable region.
  // bool IsGeoPositionOnCrossroad(const api::GeoPosition& geo_pos) const;


  // Returns the index of the lane on which the provided `geo_pos` resides. This
  // method requires that the provided `geo_pos` be on the crossroad as determined
  // by IsGeoPositionOnCrossroad().
  // int GetLaneIndex(const api::GeoPosition& geo_pos) const;

  const api::RoadGeometryId id_;
  const double linear_tolerance_{};
  const double angular_tolerance_{};
  const Junction junction_;
};

}  // namespace crossroad
}  // namespace maliput
}  // namespace drake
