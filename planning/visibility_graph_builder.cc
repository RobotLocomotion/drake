#include "drake/planning/visibility_graph_builder.h"
#include "drake/planning/visibility_graph.h"

namespace drake {
namespace planning {

VisibilityGraphBuilder::VisibilityGraphBuilder(
    std::unique_ptr<CollisionChecker>& checker, bool parallelize)
    : checker_{std::move(checker)}, parallelize_{parallelize} {}

Eigen::SparseMatrix<bool> VisibilityGraphBuilder::DoBuildAdjacencyMatrix(
    const Eigen::Ref<const Eigen::MatrixXd>& points) {
  return VisibilityGraph(*checker_, points, parallelize_);

}

}  // namespace planning
}  // namespace drake
