#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "drake/automotive/maliput/api/junction.h"
#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/automotive/maliput/api/segment.h"
#include "drake/automotive/maliput/dragway/lane.h"
#include "drake/common/drake_copyable.h"

namespace drake {
namespace maliput {
namespace dragway {

class Junction;

/**
  Dragway's implementation of api::Segment. It contains multiple straight
  lanes. For the lane semantics, see the class descriptions of Lane.

  The following ASCII art shows how N lanes are arranged in a segment.

  <pre>

              lane_bounds ---     X         -------- lane index 1
                            |     ^         |
     lane index n ---       |     |         |   --- lane index 0
                    |       |     |         |   |
                    V     |<->|   |         V   V
                -------------------------------------
                | | : | : | : | : | : | : | : | : | |
                | | : | : | : | : | : | : | : | : | |
                | | : | : | : | : | : | : | : | : | |
                | | : | : | : | : | : | : | : | : | |
                | | : | : | : | : | : | : | : | : | |
                | | : | : | : | : | : | : | : | : | |
  Y <-----------------------------o----------------------------->
                ^                 |             ^   ^
                |                 |             |   |
              y_max               |             |  y_min
                                  |             |
                                  V             --- y offset of lane 0

                |<--------------------------------->|
                              road_width

  </pre>

   Note that lane indices increase to the left, which matches the fact that
   within a Lane, `r` increases to the left.
*/
class Segment final : public api::Segment {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Segment)

  /// Constructs a new dragway Segment.
  ///
  /// @param[in] junction The junction to which this Segment belongs.
  ///
  /// @param[in] num_lanes The number of lanes in the segment.
  ///
  /// @param[in] length The length of the dragway.
  ///
  /// @param[in] lane_width The width of each lane.
  ///
  /// @param[in] shoulder_width The width of the shoulders on each side of the
  /// road.
  ///
  /// @param[in] maximum_height The maximum height above the road surface.
  /// modelled by the RoadGeometry.
  Segment(Junction* junction,
      int num_lanes,
      double length,
      double lane_width,
      double shoulder_width,
      double maximum_height);

  ~Segment() final = default;

 private:
  api::SegmentId do_id() const final { return id_; }

  const api::Junction* do_junction() const final;

  int do_num_lanes() const final { return static_cast<int>(lanes_.size()); }

  const api::Lane* do_lane(int index) const final;

  const api::SegmentId id_;
  const Junction* junction_{};
  std::vector<std::unique_ptr<Lane>> lanes_;
};

}  // namespace dragway
}  // namespace maliput
}  // namespace drake
