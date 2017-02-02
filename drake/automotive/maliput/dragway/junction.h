#pragma once

#include <memory>
#include <vector>

#include "drake/automotive/maliput/api/junction.h"
#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/api/segment.h"
#include "drake/automotive/maliput/dragway/segment.h"
#include "drake/common/drake_copyable.h"

namespace drake {
namespace maliput {
namespace dragway {

class RoadGeometry;

/// Dragway's implementation of api::Junction.
class Junction : public api::Junction {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Junction)

  /// Constructs a Junction with a single Segment.
  ///
  /// @p road_geometry must remain valid for the lifetime of this class,
  /// and must refer to the RoadGeometry which will contain this newly
  /// constructed Junction instance.
  Junction(RoadGeometry* road_geometry,
      int num_negative_x_bound_lanes,
      int num_positive_x_bound_lanes,
      double length,
      const api::RBounds& lane_bounds,
      const api::RBounds& driveable_bounds);

  ~Junction() override = default;

  double length() const { return length_; }

  const Segment* segment() const { return &segment_; }

 private:
  const api::JunctionId do_id() const override { return id_; }

  const api::RoadGeometry* do_road_geometry() const override;

  int do_num_segments() const override { return 1; }

  const api::Segment* do_segment(int index) const override {
    DRAKE_DEMAND(index < num_segments());
    return &segment_;
  }

  api::JunctionId id_;
  const RoadGeometry* road_geometry_{};
  const double length_;
  Segment segment_;
};

}  // namespace dragway
}  // namespace maliput
}  // namespace drake
