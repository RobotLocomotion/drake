#pragma once

#include <memory>
#include <vector>

#include "drake/automotive/maliput/api/basic_id_index.h"
#include "drake/automotive/maliput/api/branch_point.h"
#include "drake/automotive/maliput/api/junction.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/multilane/branch_point.h"
#include "drake/automotive/maliput/multilane/junction.h"
#include "drake/common/drake_copyable.h"

namespace drake {
namespace maliput {
namespace multilane {

/// A simple api::RoadGeometry implementation that only supports a single
/// lane per segment.  Use the Builder interface to actually assemble
/// a sensible road network.
class RoadGeometry : public api::RoadGeometry {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RoadGeometry)

  /// Constructs an empty RoadGeometry with the specified tolerances and
  /// scale-length.
  RoadGeometry(const api::RoadGeometryId& id,
               double linear_tolerance,
               double angular_tolerance,
               double scale_length)
      : id_(id),
        linear_tolerance_(linear_tolerance),
        angular_tolerance_(angular_tolerance),
        scale_length_(scale_length) {}

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

  const IdIndex& DoById() const override { return id_index_; }

  // Returns a RoadPosition for a lane containing the provided `geo_position`.
  // Either if there is or not a containing lane, the position is returned for
  // the lane whose centerline curve is closest to `geo_position`. In other
  // words, for the lane whose LanePosition makes the r coordinate be the
  // smallest. If `hint` is non-null, then the search is restricted to the
  // `hint->lane` and lanes adjacent to `hint->lane`.
  // TODO(agalbachicar) Take into account `h` coordinate to return by minimum
  //                    `h` and then minimum `r`.
  api::RoadPosition DoToRoadPosition(
      const api::GeoPosition& geo_position,
      const api::RoadPosition* hint,
      api::GeoPosition* nearest_position,
      double* distance) const override;

  double do_linear_tolerance() const override { return linear_tolerance_; }

  double do_angular_tolerance() const override { return angular_tolerance_; }

  // TODO(maddog@tri.global)  scale_length should really be kept consistent
  //                          with the geometry of the curves themselves,
  //                          perhaps even derived from the curves directly.
  double do_scale_length() const override { return scale_length_; }

  api::RoadGeometryId id_;
  double linear_tolerance_{};
  double angular_tolerance_{};
  double scale_length_{};
  std::vector<std::unique_ptr<Junction>> junctions_;
  std::vector<std::unique_ptr<BranchPoint>> branch_points_;
  api::BasicIdIndex id_index_;
};

}  // namespace multilane
}  // namespace maliput
}  // namespace drake
