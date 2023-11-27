#include "drake/planning/convex_set_from_clique_builder_base.h"

namespace drake {
namespace planning {

copyable_unique_ptr<ConvexSet> ConvexSetFromCliqueBuilderBase::BuildConvexSet(
      const Eigen::Ref<const Eigen::MatrixXd>& clique_points) {
  return DoBuildConvexSet(clique_points);
}

}  // namespace planning
}  // namespace drake