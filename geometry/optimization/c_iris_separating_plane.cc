#include "drake/geometry/optimization/c_iris_separating_plane.h"

namespace drake {
namespace geometry {
namespace optimization {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
SeparatingPlaneOrder ToPlaneOrder(int plane_degree) {
  if (plane_degree == 1) {
    return SeparatingPlaneOrder::kAffine;
  } else {
    throw std::runtime_error(fmt::format(
        "ToPlaneOrder: plane_degree={}, only support plane_degree = 1.",
        plane_degree));
  }
}

int ToPlaneDegree(SeparatingPlaneOrder plane_order) {
  switch (plane_order) {
    case SeparatingPlaneOrder::kAffine: {
      return 1;
    }
  }
  DRAKE_UNREACHABLE();
}
#pragma GCC diagnostic pop  // pop -Wdeprecated-declarations
}  // namespace optimization
}  // namespace geometry
}  // namespace drake
