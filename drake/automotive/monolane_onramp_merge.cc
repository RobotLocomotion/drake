#include "monolane_onramp_merge.h"

#include <cmath>
#include <iostream>

#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/monolane/builder.h"

namespace drake {
namespace maliput {
namespace monolane {

const double& kLinearTolerance = 0.01;
const double& kAngularTolerance = 0.01 * M_PI;

template <typename T>
MonolaneOnrampMerge<T>::MonolaneOnrampMerge(const T& lane_length,
                                            const T& lane_width,
                                            const T& driveable_width)
    : lane_bounds_(-lane_width / 2., lane_width / 2.),
      driveable_bounds_{-driveable_width / 2., driveable_width / 2.} {
  Build(lane_length);
}

template <typename T>
MonolaneOnrampMerge<T>::MonolaneOnrampMerge(const T& lane_length) {
  Build(lane_length);
}

template <typename T>
void MonolaneOnrampMerge<T>::Build(const T& lane_length) {
  Builder b{lane_bounds_, driveable_bounds_, kLinearTolerance,
        kAngularTolerance};

  const EndpointZ& kLowFlatZ{0., 0., 0., 0.};
  const EndpointZ& kMidFlatZ{3., 0., 0., 0.};
  const EndpointZ& kMidTiltLeftZ{3., 0., -0.4, 0.};
  const EndpointZ& kMidTiltRightZ{3., 0., 0.4, 0.};
  const EndpointZ& kHighFlatZ{6., 0., 0., 0.};

  const ArcOffset& kCounterClockwiseArc{lane_length, 0.75 * M_PI};  // 135deg,
                                                                    // 50m
                                                                    // radius
  const ArcOffset& kClockwiseArc{lane_length, -0.75 * M_PI};  // 135deg, 50m
                                                              // radius

  Endpoint start {{0., 0., -M_PI / 4.}, kLowFlatZ};

  auto c0 = b.Connect("0", start, lane_length, kMidFlatZ);

  auto c1 = b.Connect("1", c0->end(), kCounterClockwiseArc, kMidTiltLeftZ);
  auto c2 = b.Connect("2", c1->end(), kCounterClockwiseArc, kMidFlatZ);

  auto c3 = b.Connect("3", c2->end(), lane_length, kHighFlatZ);
  auto c4 = b.Connect("4", c3->end(), lane_length, kMidFlatZ);

  auto c5 = b.Connect("5", c4->end(), kClockwiseArc, kMidTiltRightZ);
  auto c6 = b.Connect("6", c5->end(), kClockwiseArc, kMidFlatZ);

  b.Connect("7", c6->end(), lane_length, c0->start().z());

  rg_ = b.Build({"figure-eight"});
}

// These instantiations must match the API documentation in
// monolane_onramp_merge.h.
template class MonolaneOnrampMerge<double>;

}  // namespace monolane
}  // namespace maliput
}  // namespace drake
