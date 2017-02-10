#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/monolane/builder.h"

#include <cmath>
#include <iostream>

const double kLaneBoundsLength = 2.;
const double kDriveableBoundsLength = 4.;
const double kLinearTolerance = 0.01;
const double kAngularTolerance = 0.01 * M_PI;

template <typename T>
class MonolaneOnrampMerge {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MonolaneOnrampMerge)

  const api::RBounds kLaneBounds(-kLaneBoundsLength, kLaneBoundsLength);
  const api::RBounds kDriveableBounds(-kDriveableBoundsLength,
                                      kDriveableBoundsLength);
  Builder b(kLaneBounds, kDriveableBounds, kLinearTolerance, kAngularTolerance);

  const EndpointZ kLowFlatZ(0., 0., 0., 0.);
  const EndpointZ kMidFlatZ(3., 0., 0., 0.);
  const EndpointZ kMidTiltLeftZ(3., 0., -0.4, 0.);
  const EndpointZ kMidTiltRightZ(3., 0., 0.4, 0.);
  const EndpointZ kHighFlatZ(6., 0., 0., 0.);

  const ArcOffset kCounterClockwiseArc(50., 0.75 * M_PI);  // 135deg, 50m radius
  const ArcOffset kClockwiseArc(50., -0.75 * M_PI);  // 135deg, 50m radius

  Endpoint start {{0., 0., -M_PI / 4.}, kLowFlatZ};

  auto c0 = b.Connect("0", start, 50., kMidFlatZ);

  auto c1 = b.Connect("1", c0->end(), kCounterClockwiseArc, kMidTiltLeftZ);
  auto c2 = b.Connect("2", c1->end(), kCounterClockwiseArc, kMidFlatZ);

  auto c3 = b.Connect("3", c2->end(), 50., kHighFlatZ);
  auto c4 = b.Connect("4", c3->end(), 50., kMidFlatZ);

  auto c5 = b.Connect("5", c4->end(), kClockwiseArc, kMidTiltRightZ);
  auto c6 = b.Connect("6", c5->end(), kClockwiseArc, kMidFlatZ);

  // Tweak ends to check if fuzzy-matching is working.
  Endpoint c6end = c6->end();
  c6end = Endpoint(c6end_xy, c6end_z);
  EndpointZ c0start_z = c0->start().z();
  c0start_z = EndpointZ(c0start_z.z() - kLinearTolerance * 0.5,
                        c0start_z.z_dot(),
                        c0start_z.theta(), c0start_z.theta_dot());

  b.Connect("7", c6end, 50., c0start_z);

  rg_ = b.Build({"figure-eight"});

}

}  // namespace monolane
}  // namespace maliput
}  // namespace drake
