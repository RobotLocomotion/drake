#pragma once

#include <memory>
#include <vector>

#include "drake/automotive/maliput/api/branch_point.h"
#include "drake/automotive/maliput/api/junction.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/monolane/branch_point.h"
#include "drake/automotive/maliput/monolane/junction.h"
#include "drake/common/drake_copyable.h"

namespace drake {
namespace maliput {
namespace monolane {

/// A simple api::RoadGeometry implementation that only supports a single
/// lane per segment.  Use the Builder interface to actually assemble
/// a sensible road network.
class RoadGeometry : public api::RoadGeometry {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RoadGeometry)

  /// Constructs an empty RoadGeometry with the specified tolerances.
  RoadGeometry(const api::RoadGeometryId& id,
               const double linear_tolerance,
               const double angular_tolerance)
      : id_(id),
        linear_tolerance_(linear_tolerance),
        angular_tolerance_(angular_tolerance) {}

  /// Creates and adds a new Junction with the specified @p id.
  Junction* NewJunction(api::JunctionId id);

  /// Creates and adds a new BranchPoint with the specified @p id.
  BranchPoint* NewBranchPoint(api::BranchPointId id);

  ~RoadGeometry() override = default;

 private:
  const api::RoadGeometryId do_id() const override { return id_; }

  int do_num_junctions() const override { return junctions_.size(); }

  const api::Junction* do_junction(int index) const override;

  int do_num_branch_points() const override { return branch_points_.size(); }

  const api::BranchPoint* do_branch_point(int index) const override;

  // Returns a RoadPosition for a lane containing the provided `geo_position`.
  // If there is no containing lane, the position is returned for the lane
  // closest to the centerline curve.  If `hint` is non-null, then the search is
  // restricted to the `hint->lane` and lanes adjacent to `hint->lane`.
  api::RoadPosition DoToRoadPosition(
      const api::GeoPosition& geo_position,
      const api::RoadPosition* hint,
      api::GeoPosition* nearest_position,
      double* distance) const override;

  double do_linear_tolerance() const override { return linear_tolerance_; }

  double do_angular_tolerance() const override { return angular_tolerance_; }

  api::RoadGeometryId id_;
  double linear_tolerance_{};
  double angular_tolerance_{};
  std::vector<std::unique_ptr<Junction>> junctions_;
  std::vector<std::unique_ptr<BranchPoint>> branch_points_;
};

}  // namespace monolane
}  // namespace maliput
}  // namespace drake
