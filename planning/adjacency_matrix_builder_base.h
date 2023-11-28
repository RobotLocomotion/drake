#pragma once

#include <Eigen/Core>
#include <Eigen/Sparse>

#include "drake/common/drake_copyable.h"

namespace drake {
namespace planning {

class AdjacencyMatrixBuilderBase {
 public:
  AdjacencyMatrixBuilderBase() = default;
  /**
   * Construct the adjacency matrix of a graph whose vertices are the points in
   * points, and whose edge relations are given according to the concrete
   * implementation of this class.
   *
   * Throws an exception of any of points are infinite or nan.
   */
  Eigen::SparseMatrix<bool> BuildAdjacencyMatrix(
      const Eigen::Ref<const Eigen::MatrixXd>& points) const;

  virtual ~AdjacencyMatrixBuilderBase() {}

 protected:
  // We put the copy/move/assignment constructors as protected to avoid copy
  // slicing. The inherited final subclasses should put them in public
  // functions.
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(AdjacencyMatrixBuilderBase);

  virtual Eigen::SparseMatrix<bool> DoBuildAdjacencyMatrix(
      const Eigen::Ref<const Eigen::MatrixXd>& points) const = 0;
};

}  // namespace planning
}  // namespace drake
