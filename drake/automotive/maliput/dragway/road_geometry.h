#pragma once

#include <limits>
#include <memory>
#include <vector>

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
class RoadGeometry : public api::RoadGeometry {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RoadGeometry)

  /// Constructs a dragway RoadGeometry.
  ///
  /// @param[in] id The ID of this RoadGeometry. This can be any user-selectable
  /// value.
  ///
  /// @param[in] num_southbound_lanes The number of southbound lanes.
  ///
  /// @param[in] num_northbound_lanes The number of northbound lanes.
  ///
  /// @param[in] length The total length of the road.
  ///
  /// @param[in] lane_bounds The bounds of each lane.
  ///
  /// @param[in] drivable_bounds The drivable bounds of each lane.
  ///
  RoadGeometry(const api::RoadGeometryId& id,
               int num_southbound_lanes,
               int num_northbound_lanes,
               double length,
               const api::RBounds& lane_bounds,
               const api::RBounds& driveable_bounds,
               double linear_tolerance =
                   std::numeric_limits<double>::epsilon(),
               double angular_tolerance =
                   std::numeric_limits<double>::epsilon());

  ~RoadGeometry() override = default;

  double length() const { return length_; }

  const Junction* junction() const { return &junction_; }

 private:
  const api::RoadGeometryId do_id() const final { return id_; }

  int do_num_junctions() const final { return 1; }

  const api::Junction* do_junction(int index) const final;

  int do_num_branch_points() const final { return branch_points_.size(); }

  const api::BranchPoint* do_branch_point(int index) const final;

  api::RoadPosition DoToRoadPosition(
      const api::GeoPosition& geo_pos,
      const api::RoadPosition& hint) const final;

  double do_linear_tolerance() const final { return linear_tolerance_; }

  double do_angular_tolerance() const final { return angular_tolerance_; }

  const api::RoadGeometryId id_;
  const double length_{};
  const double linear_tolerance_{};
  const double angular_tolerance_{};
  Junction junction_;
  std::vector<std::unique_ptr<BranchPoint>> branch_points_;
};

}  // namespace dragway
}  // namespace maliput
}  // namespace drake
