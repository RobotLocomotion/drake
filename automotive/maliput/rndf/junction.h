#pragma once

#include <memory>
#include <vector>

#include "drake/automotive/maliput/api/junction.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/api/segment.h"
#include "drake/automotive/maliput/rndf/segment.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"

namespace drake {
namespace maliput {
namespace rndf {

/// An api::Junction implementation for RNDF.
class Junction : public api::Junction {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Junction)

  /// Constructs an empty Junction.
  ///
  /// @param id is the ID of the Junction.
  /// @param road_geometry must remain valid for the lifetime of this class,
  /// and must refer to the RoadGeometry which will contain the newly
  /// constructed Junction instance.
  Junction(const api::JunctionId& id, api::RoadGeometry* road_geometry)
      : id_(id), road_geometry_(road_geometry) {}

  /// Creates and adds a new Segment with the specified @p id.
  ///
  /// @param id is the ID of the segment.
  Segment* NewSegment(api::SegmentId id);

  ~Junction() override = default;

 private:
  api::JunctionId do_id() const override { return id_; }

  const api::RoadGeometry* do_road_geometry() const override {
    return road_geometry_;
  }

  int do_num_segments() const override { return segments_.size(); }

  // Throws an exception in case the index is not in between zero and the
  // number of segments.
  const api::Segment* do_segment(int index) const override {
    DRAKE_THROW_UNLESS(index >= 0 &&
                       index < static_cast<int>(segments_.size()));
    return segments_[index].get();
  }

  api::JunctionId id_;
  api::RoadGeometry* road_geometry_{};
  std::vector<std::unique_ptr<Segment>> segments_;
};

}  // namespace rndf
}  // namespace maliput
}  // namespace drake
