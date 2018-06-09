#pragma once

#include <functional>
#include <memory>
#include <vector>

#include "drake/automotive/maliput/api/junction.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/api/segment.h"
#include "drake/automotive/maliput/monolane/segment.h"
#include "drake/common/drake_copyable.h"

namespace drake {
namespace maliput {
namespace monolane {

/// An api::Junction implementation.
class Junction : public api::Junction {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Junction);

  /// Constructs an empty Junction.
  ///
  /// @p road_geometry must remain valid for the lifetime of this class,
  /// and must refer to the RoadGeometry which will contain the newly
  /// constructed Junction instance.
  /// @p register_segment and @p register_lane will be called on any new
  /// Segment or Lane instances created as children of the Junction.
  Junction(const api::JunctionId& id, const api::RoadGeometry* road_geometry,
           const std::function<void(const api::Segment*)>& register_segment,
           const std::function<void(const api::Lane*)>& register_lane)
      : id_(id), road_geometry_(road_geometry),
        register_segment_(register_segment),
        register_lane_(register_lane) {}

  /// Creates and adds a new Segment with the specified @p id.
  Segment* NewSegment(api::SegmentId id);

  ~Junction() override = default;

 private:
  const api::JunctionId do_id() const override { return id_; }

  const api::RoadGeometry* do_road_geometry() const override;

  int do_num_segments() const override { return segments_.size(); }

  const api::Segment* do_segment(int index) const override {
    return segments_[index].get();
  }

  api::JunctionId id_;
  const api::RoadGeometry* road_geometry_{};
  std::function<void(const api::Segment*)> register_segment_;
  std::function<void(const api::Lane*)> register_lane_;
  std::vector<std::unique_ptr<Segment>> segments_;
};

}  // namespace monolane
}  // namespace maliput
}  // namespace drake
