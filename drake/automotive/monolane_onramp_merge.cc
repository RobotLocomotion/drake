#include "monolane_onramp_merge.h"

#include <cmath>
#include <iostream>

#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/monolane/builder.h"

namespace drake {
namespace maliput {
namespace monolane {

template <typename T>
void RoadSection<T>::AddArcSegment(const T& arc_length, const T& arc_radius,
                                   const ArcDirection& dir,
                                   const EndpointZ& end_z) {
  DRAKE_DEMAND(arc_length < 2 * M_PI * arc_radius);
  const T& angle = dir == kCCW ? arc_length / arc_radius
      : -arc_length / arc_radius;
  const ArcOffset& arc{arc_radius, angle};

  const Connection* connection = b_->Connect(std::to_string(id_++),
                                             last_endpoint_, arc, end_z);
  last_endpoint_ = connection->end();
}

template <typename T>
void RoadSection<T>::AddArcSegment(const T& arc_length, const T& arc_radius,
                                   const ArcDirection& dir) {
  AddArcSegment(arc_length, arc_radius, dir, flat_z_);
}

template <typename T>
void RoadSection<T>::AddStraightSegment(const T& lane_length,
                                        const EndpointZ& end_z) {
  const Connection* connection = b_->Connect(std::to_string(id_++),
                                             last_endpoint_, lane_length,
                                             end_z);
  last_endpoint_ = connection->end();
}

template <typename T>
void RoadSection<T>::AddStraightSegment(const T& lane_length) {
  AddStraightSegment(lane_length, flat_z_);
}

template <typename T>
void MonolaneOnrampMerge<T>::BuildOnramp() {
  Builder b{road_.lane_bounds, road_.driveable_bounds, kLinearTolerance_,
            kAngularTolerance_};

  std::unique_ptr<RoadSection<T>>
      rs_pre(new RoadSection<T>(std::move(b_), "Pre", road_));
  rs_pre->AddArcSegment(70., 50., kCW);
  rs_pre->AddArcSegment(40., 50., kCCW);
  const Endpoint endpoint_pre = rs_pre->get_last_endpoint();
  b_ = rs_pre->Finalize();

  std::unique_ptr<RoadSection<T>>
      rs_post(new RoadSection<T>(std::move(b_), "Post", road_, endpoint_pre));
  rs_post->AddStraightSegment(50.);
  b_ = rs_post->Finalize();

  std::unique_ptr<RoadSection<T>>
      rs_onramp(new RoadSection<T>(std::move(b_), "Onramp", road_,
                                   endpoint_pre.reverse()));
  rs_onramp->AddArcSegment(50., 30, kCCW);
  rs_onramp->AddStraightSegment(100.);
  b_ = rs_onramp->Finalize();

  //b.SetDefaultBranch(
  //    left1, api::LaneEnd::kStart, left1, api::LaneEnd::kFinish);

  rg_ = b_->Build({"example"});
}

// These instantiations must match the API documentation in
// monolane_onramp_merge.h.
template class MonolaneOnrampMerge<double>;

}  // namespace monolane
}  // namespace maliput
}  // namespace drake
