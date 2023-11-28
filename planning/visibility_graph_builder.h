#pragma once
#include "drake/planning/adjacency_matrix_builder_base.h"
#include "drake/planning/collision_checker.h"

#include <memory>

namespace drake {
namespace planning {


class VisibilityGraphBuilder final : AdjacencyMatrixBuilderBase {
 public:
  VisibilityGraphBuilder(std::unique_ptr<CollisionChecker>& checker,
                         bool parallelize = true);



 private:
  Eigen::SparseMatrix<bool> DoBuildAdjacencyMatrix(
      const Eigen::Ref<const Eigen::MatrixXd>& points) override;

  std::unique_ptr<CollisionChecker> checker_;
  bool parallelize_;


};

}  // namespace planning
}  // namespace drake

