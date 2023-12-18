#include "drake/planning/iris_region_from_clique_builder.h"

#include "drake/geometry/optimization/hyperellipsoid.h"
namespace drake {
namespace planning {
using geometry::optimization::ConvexSets;
using geometry::optimization::Hyperellipsoid;
using geometry::optimization::IrisOptions;

DefaultIrisOptionsForIrisRegionFromCliqueBuilder::
    DefaultIrisOptionsForIrisRegionFromCliqueBuilder()
    : IrisOptions() {
  iteration_limit = 1;
}

IrisRegionFromCliqueBuilder::IrisRegionFromCliqueBuilder(
    const ConvexSets& obstacles, const HPolyhedron& domain,
    const IrisOptions& options, const double rank_tol_for_lowner_john_ellipse)
    : ConvexSetFromCliqueBuilderBase(),
      obstacles_{obstacles},
      domain_{domain},
      options_{options},
      rank_tol_for_lowner_john_ellipse_{rank_tol_for_lowner_john_ellipse} {}

copyable_unique_ptr<ConvexSet> IrisRegionFromCliqueBuilder::DoBuildConvexSet(
    const Eigen::Ref<const Eigen::MatrixXd>& clique_points) {
  const Hyperellipsoid starting_ellipse =
      Hyperellipsoid::MinimumVolumeCircumscribedEllipsoid(
          clique_points, rank_tol_for_lowner_john_ellipse_);
  options_.starting_ellipse = starting_ellipse;
  return copyable_unique_ptr<ConvexSet>(geometry::optimization::Iris(
      obstacles_, starting_ellipse.center(), domain_, options_));
}

}  // namespace planning
}  // namespace drake
