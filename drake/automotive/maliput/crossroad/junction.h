#pragma once
  
#include <memory>
#include <vector>

#include "drake/automotive/maliput/api/junction.h"
#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/api/segment.h"
#include "drake/automotive/maliput/crossroad/segment.h"
#include "drake/common/drake_copyable.h"

namespace drake {
namespace maliput {
namespace crossroad{

class RoadGeometry;

/// Dragway's implementation of api::Junction.
class Junction final : public api::Junction {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Junction)
  /// Constructs a Junction with a two Segments (one horizontal and one
  /// vertical.
  ///
  /// @p road_geometry must remain valid for the lifetime of this class,
  /// and must refer to the RoadGeometry which will contain this newly
  /// constructed Junction instance.
  Junction(RoadGeometry* road_geometry,
      int num_horizontal_lanes,
      int num_vertical_lanes,
      double length,
      double lane_width,
      double shoulder_width);

  ~Junction() final = default;

  /// Returns the number of Segments contained in this Junction.
  ///
  /// Return value is non-negative.
  int num_segments() const { return do_num_segments(); }

 private:
  const api::JunctionId do_id() const final { return id_; }

  const api::RoadGeometry* do_road_geometry() const final;

  int do_num_segments() const final { return 2; }

  const api::Segment* do_segment(int index) const final;

  const api::JunctionId id_;

  const RoadGeometry* const road_geometry_{};

  std::vector<std::unique_ptr<Segment>> segments_;

};

}  // namespace crossroad
}  // namespace maliput
}  // namespace drake
