#pragma once

#include <memory>
#include <vector>

#include "drake/automotive/maliput/api/junction.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/api/segment.h"
#include "drake/automotive/maliput/monolane/segment.h"

namespace drake {
namespace maliput {
namespace monolane {

class RoadGeometry;

/// A Junction implementation.
class Junction : public api::Junction {
 public:
  /// Constructs an empty Junction.
  Junction(const api::JunctionId& id, RoadGeometry* rg)
      : id_(id), road_geometry_(rg) {}

  /// Creates and adds a new Segment with the specified @p id.
  Segment* NewSegment(api::SegmentId id);

  virtual ~Junction() {}

 private:
  const api::JunctionId do_id() const override { return id_; }

  const api::RoadGeometry* do_road_geometry() const override;

  int do_num_segments() const override { return segments_.size(); }

  const api::Segment* do_segment(int index) const override {
    return segments_[index].get();
  }

  api::JunctionId id_;
  RoadGeometry* road_geometry_{};
  std::vector<std::unique_ptr<Segment>> segments_;
};

}  // namespace monolane
}  // namespace maliput
}  // namespace drake
