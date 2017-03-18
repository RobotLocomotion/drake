#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "drake/automotive/maliput/api/junction.h"
#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/automotive/maliput/api/segment.h"
#include "drake/automotive/maliput/crossroad/lane.h"
#include "drake/common/drake_copyable.h"

namespace drake {
namespace maliput {
namespace crossroad {

class Junction;

/**
  Crossroad's implementation of api::Segment. It contains multiple straight
  lanes. For the lane semantics, see the class descriptions of Lane.

  The following ASCII art shows how N lanes are arranged in a segment. For the
  horizontal segment, its lanes are parallel to the X-axis in the world frame,
  and for the vertical segment, the Y-axis.

  <pre>

              lane_bounds ---     X/(Y)      -------- lane index 1
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
  Y <-----------------------------o-----------------------------> (X)
                ^                 |             ^   ^
                |                 |             |   |
              r_max               |             |  r_min
                                  |             |
                                  V             --- r_offset of lane 0

                |<--------------------------------->|
                              road_width

  </pre>

   Note that lane index starts from the right-most (relative to the
   corresponding axis), and increases to the left, which matches the fact that
   within a Lane, `r` increases to the left.
**/
class Segment final : public api::Segment {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Segment)

  /// Constructs a new crossroad Segment.
  ///
  /// @param[in] index The index of the segment with its Junction.

  /// @param[in] junction The junction to which this Segment belongs.
  ///
  /// @param[in] num_lanes The number of lanes in the segment.
  ///
  /// @param[in] length The length of the crossroad.
  ///
  /// @param[in] lane_width The width of each lane.
  ///
  /// @param[in] shoulder_width The width of the shoulders on each side of the
  /// road.
  Segment(Junction* junction, int index, int num_lanes, double length,
          double lane_width, double shoulder_width, api::SegmentId segment_id);

  ~Segment() final = default;

  /// Returns the index of this Segment within the Junction which owns it.
  int index() const { return do_index(); }

 private:
  const int index_{};  // The index of this segment within a Junction.

  int do_index() const { return index_; }

  const api::SegmentId do_id() const final { return id_; }

  const api::Junction* do_junction() const final;

  int do_num_lanes() const final { return static_cast<int>(lanes_.size()); }

  const api::Lane* do_lane(int index) const final;

  const api::SegmentId id_;
  const Junction* junction_{};
  std::vector<std::unique_ptr<Lane>> lanes_;
};

}  // namespace crossroad
}  // namespace maliput
}  // namespace drake
