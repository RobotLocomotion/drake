#include "drake/automotive/maliput/rndf/spline_lane.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <tuple>

#include "drake/common/drake_assert.h"

namespace drake {
namespace maliput {
namespace rndf {

SplineLane::SplineLane(const api::LaneId& id, const api::Segment* segment,
                       double width, int index)
    : Lane(id, segment, width, index) {}

}  // namespace rndf
}  // namespace maliput
}  // namespace drake
