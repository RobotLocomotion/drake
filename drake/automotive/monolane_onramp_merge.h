#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/monolane/builder.h"

#include <cmath>
#include <iostream>

template <typename T>
class MonolaneOnrampMerge {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MonolaneOnrampMerge)

  MonolaneOnrampMerge();

 private:
  std::unique_ptr<const api::RoadGeometry> rg_;
}

}  // namespace monolane
}  // namespace maliput
}  // namespace drake
