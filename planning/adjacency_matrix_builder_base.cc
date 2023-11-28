#include "drake/planning/adjacency_matrix_builder_base.h"

#include "drake/common/drake_throw.h"

namespace drake {
namespace planning {
Eigen::SparseMatrix<bool> AdjacencyMatrixBuilderBase::BuildAdjacencyMatrix(
      const Eigen::Ref<const Eigen::MatrixXd>& points) const {
  DRAKE_THROW_UNLESS(!points.hasNaN());
  DRAKE_THROW_UNLESS(points.allFinite());
  return DoBuildAdjacencyMatrix(points);
}


}  // namespace planning
}  // namespace drake
