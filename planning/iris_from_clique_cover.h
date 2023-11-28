#pragma once

#include "drake/geometry/optimization/convex_set.h"
#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/geometry/optimization/iris.h"
#include "drake/planning/approximate_convex_cover_builder_base.h"
#include "drake/planning/iris_region_from_clique_builder.h"

namespace drake {
namespace planning {
using geometry::HPolyhedron;

struct IrisFromCliqueCoverOptions {
  IrisFromCliqueCoverOptions() = default;

  /**
   * The options used on internal calls to IRIS.
   */
  IrisOptions iris_options{DefaultIrisOptionsForIrisRegionFromCliqueBuilder()};

  /**
   * The amount of coverage
   */
   double coverage_termination_threshold{0.7};
};


void IrisFromCliqueCover(const ConvexSets& obstacles, const HPolyhedron& domain,
                         const IrisOptions& options,
                         std::vector<HPolyhedron>* sets);

}  // namespace planning
}  // namespace drake
