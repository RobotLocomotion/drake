#pragma once

#include <vector>

// This file only exists to expose some internal methods for unit testing.  It
// should NOT be included in user code.

namespace drake {
namespace solvers {
namespace internal {

std::vector<Eigen::Vector3d> ComputeBoxEdgesAndSphereIntersection(
    const Eigen::Vector3d& bmin, const Eigen::Vector3d& bmax);

}  // internal
}  // solvers
}  // drake
