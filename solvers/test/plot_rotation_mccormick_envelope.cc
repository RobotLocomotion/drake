/* clang-format off to disable clang-format-includes */
#include "drake/solvers/test/rotation_constraint_visualization.h"
/* clang-format on */

#include "drake/common/proto/call_matlab.h"
#include "drake/solvers/rotation_constraint.h"

// Plot the McCormick Envelope on the unit sphere, to help developers to
// visualize the McCormick Envelope relaxation.

namespace drake {
namespace solvers {
namespace {
// This draws all the McCormick Envelope in the first orthant (+++).
void DrawAllMcCormickEnvelopes(int num_bins) {
  for (int i = 0; i < num_bins; ++i) {
    for (int j = 0; j < num_bins; ++j) {
      for (int k = 0; k < num_bins; ++k) {
        Eigen::Vector3d bmin(static_cast<double>(i) / num_bins,
                             static_cast<double>(j) / num_bins,
                             static_cast<double>(k) / num_bins);
        Eigen::Vector3d bmax(static_cast<double>(i + 1) / num_bins,
                             static_cast<double>(j + 1) / num_bins,
                             static_cast<double>(k + 1) / num_bins);
        if (bmin.norm() <= 1 && bmax.norm() >= 1) {
          DrawBoxSphereIntersection(bmin, bmax);
          DrawBox(bmin, bmax);
        }
      }
    }
  }
}

void DoMain() {
  using common::CallMatlab;
  for (int num_bins = 1; num_bins <= 3; ++num_bins) {
    CallMatlab("figure", num_bins);
    CallMatlab("clf");
    CallMatlab("hold", "on");
    CallMatlab("axis", "equal");
    DrawSphere();
    DrawAllMcCormickEnvelopes(num_bins);
  }
}
}  // namespace
}  // namespace solvers
}  // namespace drake

int main() {
  drake::solvers::DoMain();
  return 0;
}
