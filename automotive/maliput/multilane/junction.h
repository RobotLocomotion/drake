#pragma once

#include <memory>
#include <vector>

#include "drake/automotive/maliput/api/junction.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/api/segment.h"
#include "drake/automotive/maliput/multilane/road_curve.h"
#include "drake/automotive/maliput/multilane/segment.h"
#include "drake/common/drake_copyable.h"

namespace drake {
namespace maliput {
namespace multilane {

/// An api::Junction implementation.
class Junction : public api::Junction {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Junction)

  /// Constructs an empty Junction.
  ///
  /// @p road_geometry must remain valid for the lifetime of this class,
  /// and must refer to the RoadGeometry which will contain the newly
  /// constructed Junction instance.
  Junction(const api::JunctionId& id, api::RoadGeometry* road_geometry)
      : id_(id), road_geometry_(road_geometry) {}

  /// Creates and adds a new Segment.
  /// @param id Segment's ID.
  /// @param road_curve Reference trajectory over the Segment's surface.
  /// @param r_min Lateral distance to the minimum extent of road_curve's curve
  /// from where Segment's surface starts.
  /// @param r_max Lateral distance to the maximum extent of road_curve's curve
  /// from where Segment's surface ends.
  /// @param elevation_bounds The height bounds over the segment' surface.
  /// @return A Segment object.
  Segment* NewSegment(const api::SegmentId& id,
                      std::unique_ptr<RoadCurve> road_curve, double r_min,
                      double r_max, const api::HBounds& elevation_bounds);

  ~Junction() override = default;

 private:
  const api::JunctionId do_id() const override { return id_; }

  const api::RoadGeometry* do_road_geometry() const override;

  int do_num_segments() const override { return segments_.size(); }

  const api::Segment* do_segment(int index) const override {
    return segments_[index].get();
  }

  api::JunctionId id_;
  api::RoadGeometry* road_geometry_{};
  std::vector<std::unique_ptr<Segment>> segments_;
};

}  // namespace multilane
}  // namespace maliput
}  // namespace drake
