#pragma once

#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/topology/forest.h"

namespace drake {
namespace multibody {
namespace internal {

/* This class is one of the cache entries in the Context. It holds the current
SystemJacobian Jv_V_WB, stored as per-tree blocks. The blocks follow the
tree ordering defined by a SpanningForest, so are in TreeIndex order.
The block for treeᵢ is a MatrixX of size 6nᵢ x mᵢ, where nᵢ is the number
of mobilized bodies in treeᵢ and mᵢ is the total number of mobilizer velocity
degrees of freedom (mobilities) in the tree. Every tree has an entry even
if it has no mobilities (in that case mᵢ=0).

Note that locking and unlocking mobilizers does not affect the contents here;
the Jacobian reflects what would happen if a velocity variable changed
regardless of whether it can currently do so.

@tparam_default_scalar */
template <typename T>
class BlockSystemJacobianCache {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(BlockSystemJacobianCache);

  // Allocates the appropriately-sized data structure for the block system
  // Jacobian, initialized to zero.
  explicit BlockSystemJacobianCache(const SpanningForest& forest);

  const std::vector<Eigen::MatrixX<T>>& block_system_jacobian() const {
    return block_system_jacobian_;
  }

  std::vector<Eigen::MatrixX<T>>& mutable_block_system_jacobian() {
    return block_system_jacobian_;
  }

  Eigen::MatrixX<T> ToFullMatrix() const;

 private:
  int total_rows_{};  // 6 * number of mobods
  int total_cols_{};  // number of mobilities

  // Indexed by TreeIndex.
  std::vector<Eigen::MatrixX<T>> block_system_jacobian_;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::BlockSystemJacobianCache);
