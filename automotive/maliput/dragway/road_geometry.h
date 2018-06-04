#pragma once

#include <limits>
#include <memory>
#include <vector>

#include "drake/automotive/maliput/api/basic_id_index.h"
#include "drake/automotive/maliput/api/branch_point.h"
#include "drake/automotive/maliput/api/junction.h"
#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/dragway/branch_point.h"
#include "drake/automotive/maliput/dragway/junction.h"
#include "drake/common/drake_copyable.h"

namespace drake {
namespace maliput {
namespace dragway {

/// Dragway's implementation of api::RoadGeometry.
///
/// To understand the characteristics of the geometry, consult the
/// dragway::Segment and dragway::Lane detailed class overview docs.
class RoadGeometry final : public api::RoadGeometry {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RoadGeometry)

  /// Constructs a dragway RoadGeometry.
  ///
  /// @param[in] id The ID of this RoadGeometry. This can be any user-selectable
  /// value.
  ///
  /// @param[in] num_lanes The number of lanes. This must be greater than zero.
  ///
  /// @param[in] length The length of the dragway.
  ///
  /// @param[in] lane_width The width of each lane.
  ///
  /// @param[in] shoulder_width The width of the shoulders on each side of the
  /// road.
  ///
  /// @param[in] maximum_height The maximum height above the road surface
  /// modelled by the RoadGeometry.
  ///
  /// @param[in] linear_tolerance The tolerance guaranteed for linear
  /// measurements (positions).
  ///
  /// @param[in] angular_tolerance The tolerance guaranteed for angular
  /// measurements (orientations).
  ///
  RoadGeometry(const api::RoadGeometryId& id,
               int num_lanes,
               double length,
               double lane_width,
               double shoulder_width,
               double maximum_height,
               double linear_tolerance,
               double angular_tolerance);

  ~RoadGeometry() final = default;

 private:
  const api::RoadGeometryId do_id() const final { return id_; }

  int do_num_junctions() const final { return 1; }

  const api::Junction* do_junction(int index) const final;

  int do_num_branch_points() const final;

  const api::BranchPoint* do_branch_point(int index) const final;

  const IdIndex& DoById() const override { return id_index_; }

  api::RoadPosition DoToRoadPosition(
      const api::GeoPosition& geo_position,
      const api::RoadPosition* hint,
      api::GeoPosition* nearest_position,
      double* distance) const final;

  double do_linear_tolerance() const final { return linear_tolerance_; }

  double do_angular_tolerance() const final { return angular_tolerance_; }

  // Returns true iff `geo_pos` is "on" the dragway. It is on the dragway iff
  // `geo_pos.x` and `geo_pos.y` fall within the dragway's driveable region.
  bool IsGeoPositionOnDragway(const api::GeoPosition& geo_pos) const;

  // Returns the index of the lane on which the provided `geo_pos` resides. This
  // method requires that the provided `geo_pos` be on the dragway as determined
  // by IsGeoPositionOnDragway().
  int GetLaneIndex(const api::GeoPosition& geo_pos) const;

  const api::RoadGeometryId id_;
  const double linear_tolerance_{};
  const double angular_tolerance_{};
  const Junction junction_;
  api::BasicIdIndex id_index_;
};

}  // namespace dragway
}  // namespace maliput
}  // namespace drake
